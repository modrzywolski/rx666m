/*
 * rx666m USB driver
 * Copyright (C) 2020 Marcin Odrzywolski <emk6@wp.pl>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/workqueue.h>

#include <linux/kernel.h>
#include <linux/time.h>

#include "rx666m_ioctl.h"

#define USB_CYPRESS_VENDOR_ID   				0x04b4
#define USB_FX3_PRODUCT_ID      				0x00f3
#define USB_RX666M_MINOR_BASE					193
#define NUM_CONCURRENT_TRANSFERS  				8
#define NUM_DATA_URB							NUM_CONCURRENT_TRANSFERS*2
#define DATA_BUFFER_SIZE						(1024*4)
#define USB_TYPE_OUT							0x40
#define USB_TYPE_IN								0xC0
#define RX666_TIMEOUT_MS						1000
#define RX666_READ_TIMEOUT_S					5
#define MAX_IOCTL_SIZE          				(2 * 1024)


#define GFP_KERNEL_TRY GFP_ATOMIC
#define ANCHORURB	1
#define DMA_EN		0
#define LOG_PERIOD 	(5*HZ)
#define LOG_EN		1

typedef struct
{
	struct rx666m_device_s *dev;

    struct workqueue_struct * wq;
    struct delayed_work stats_dumper_work;
} stats_dumper_priv_t;

typedef struct
{
	struct list_head list;

    struct urb  *urb;
    void        *addr;
    dma_addr_t   dma;
	size_t		 cnt;
	__u32		 tv_sec;
	__u32		 tv_usec;

	int			taken;
	int			taken_from;

	struct rx666m_device_s* dev;
}rx666m_data_buffer_t;

typedef struct
{
	rx666m_data_buffer_t   *avl_bufs[NUM_DATA_URB];
	rx666m_data_buffer_t   *busy_bufs[NUM_DATA_URB];
	rx666m_data_buffer_t   *rdy_bufs[NUM_DATA_URB];

}rx666m_driver_snapshot_t;

typedef struct rx666m_device_s
{
    struct usb_device    *udev;
    struct usb_interface *interface;

    int                   intnum;
    int                   disconnecting;


    int                   rx_en;
    spinlock_t            data_in_lock;
    wait_queue_head_t     data_in_wait;

    atomic_t              data_in_ready;
    atomic_t              data_in_used; //--->use
    atomic_t              data_in_progress;

#if ANCHORURB
    struct usb_anchor     data_in_anchor;
#endif
	struct list_head      data_in_bufs_avail;
	struct list_head      data_in_bufs_busy;
	struct list_head      data_in_bufs_ready;

    struct file           *reader;

	ktime_t ts_start, 	ts_end;
	size_t			read_cb_cnt;
	size_t			bad_status;
	size_t			overrun;
	size_t			submitted;
	size_t			last_read_ret;


#if LOG_EN
	stats_dumper_priv_t		stats_dumper_priv;

	struct workqueue_struct *queue;
	rx666m_driver_snapshot_t *snapshot;
#endif

}rx666m_device_t;


static void rx666m_read_cb(struct urb *urb);



static struct usb_driver rx666m_driver;

// USB PID-VID table
static struct usb_device_id rx666m_table[] = {
    { USB_DEVICE(USB_CYPRESS_VENDOR_ID, USB_FX3_PRODUCT_ID) },
    { USB_DEVICE(USB_CYPRESS_VENDOR_ID, 0x00f1) },
    { } /* Terminate entry */
};
MODULE_DEVICE_TABLE(usb, rx666m_table);





#if LOG_EN
static void dump_info(rx666m_device_t *dev);

static inline const char* status2E(int status)
{
        char *errmsg = "E???";

        switch (status) {
		case 0:
                errmsg = "OK";
                break;
        case -ENOENT:
                errmsg = "ENOENT";
                break;
        case -ECONNRESET:
                errmsg = "ECONNRESET";
                break;
        case -ENOSR:
                errmsg = "ENOSR";
                break;
        case -EPIPE:
                errmsg = "EPIPE";
                break;
        case -EOVERFLOW:
                errmsg = "EOVERFLOW";
                break;
        case -EPROTO:
                errmsg = "EPROTO";
                break;
        case -EILSEQ:
                errmsg = "EILSEQ";
                break;
        case -ETIME:
                errmsg = "ETIME";
                break;
		case -EINPROGRESS:
                errmsg = "EINPROGRESS";
                break;
        }

		return errmsg;
}



void stats_dumper_work(struct work_struct * work_struct_ptr)
{
    struct delayed_work * delayed = container_of(work_struct_ptr, struct delayed_work, work);
    stats_dumper_priv_t * priv = container_of(delayed, stats_dumper_priv_t, stats_dumper_work);

	printk(KERN_INFO "stats_dumper_work:\n");

	dump_info(priv->dev);

    queue_delayed_work(priv->wq, &priv->stats_dumper_work, LOG_PERIOD);
}

static void stats_dumper_start_workqueue(stats_dumper_priv_t *priv, rx666m_device_t *dev)
{
	priv->dev=dev;
    priv->wq = create_singlethread_workqueue("stats_dumper_st_wq");

    INIT_DELAYED_WORK(&priv->stats_dumper_work, stats_dumper_work);
    queue_delayed_work(priv->wq, &priv->stats_dumper_work, LOG_PERIOD);
}

static void stats_dumper_stop_workqueue(stats_dumper_priv_t *priv)
{
    cancel_delayed_work_sync(&priv->stats_dumper_work);
    destroy_workqueue(priv->wq);
}

static void dump_info(rx666m_device_t *dev)
{
	int i, avl_cnt=0, busy_cnt=0, rdy_cnt=0;
	struct list_head* p;
	unsigned long irq_flags;

	dev_info(&dev->interface->dev, "read_cb_cnt=%ld\n", dev->read_cb_cnt);
	dev_info(&dev->interface->dev, "bad_status=%ld\n", dev->bad_status);
	dev_info(&dev->interface->dev, "overrun=%ld\n", dev->overrun);
	dev_info(&dev->interface->dev, "submitted=%ld\n", dev->submitted);
	dev_info(&dev->interface->dev, "data_in_ready=%d/%d\n", (int)atomic_read(&dev->data_in_ready), NUM_DATA_URB);
	dev_info(&dev->interface->dev, "data_in_used=%d/%d\n", (int)atomic_read(&dev->data_in_used), NUM_DATA_URB);
	dev_info(&dev->interface->dev, "data_in_progress=%d/%d\n", (int)atomic_read(&dev->data_in_progress), NUM_CONCURRENT_TRANSFERS);
#if ANCHORURB
	dev_info(&dev->interface->dev, "usb_anchor_empty %d\n", usb_anchor_empty(&dev->data_in_anchor));
#endif
	dev_info(&dev->interface->dev, "last_read_ret=%ld\n", dev->last_read_ret);

	memset(dev->snapshot->avl_bufs, 0, sizeof(dev->snapshot->avl_bufs));
	memset(dev->snapshot->busy_bufs, 0, sizeof(dev->snapshot->busy_bufs));
	memset(dev->snapshot->rdy_bufs, 0, sizeof(dev->snapshot->rdy_bufs));

    spin_lock_irqsave(&dev->data_in_lock, irq_flags);
	list_for_each(p, &dev->data_in_bufs_avail)
	{
		dev->snapshot->avl_bufs[avl_cnt++] = list_entry(p, rx666m_data_buffer_t, list);
	}

	list_for_each(p, &dev->data_in_bufs_busy)
	{
		dev->snapshot->busy_bufs[busy_cnt++] = list_entry(p, rx666m_data_buffer_t, list);

	}

	list_for_each(p, &dev->data_in_bufs_ready)
	{
		dev->snapshot->rdy_bufs[rdy_cnt++] = list_entry(p, rx666m_data_buffer_t, list);

	}
	spin_unlock_irqrestore(&dev->data_in_lock, irq_flags);

	if(avl_cnt > NUM_DATA_URB) dev_err(&dev->interface->dev, "avl_cnt overflow\n");
	if(avl_cnt > NUM_DATA_URB) dev_err(&dev->interface->dev, "avl_cnt overflow\n");
	if(rdy_cnt > NUM_DATA_URB) dev_err(&dev->interface->dev, "rdy_cnt overflow\n");

	for(i=0;i<avl_cnt;i++)
	{
		dev_info(&dev->interface->dev, "data_in_bufs_avail=%p (addr=%p dma=%p urb=%p urb->status=%s urb->transfer_flags=%x urb->actual_length=%d, cnt=%ld taken=%d taken_from=%d)\n", 
				dev->snapshot->avl_bufs[i], 
				dev->snapshot->avl_bufs[i]->addr, 
				(void*)dev->snapshot->avl_bufs[i]->dma, 
				dev->snapshot->avl_bufs[i]->urb, 
				dev->snapshot->avl_bufs[i]->urb ? status2E(dev->snapshot->avl_bufs[i]->urb->status) : "???", 
				(int)(dev->snapshot->avl_bufs[i]->urb? dev->snapshot->avl_bufs[i]->urb->transfer_flags : -1), 
				(int)(dev->snapshot->avl_bufs[i]->urb? dev->snapshot->avl_bufs[i]->urb->actual_length : -1),
				dev->snapshot->avl_bufs[i]->cnt, 
				dev->snapshot->avl_bufs[i]->taken, dev->snapshot->avl_bufs[i]->taken_from);
	}

	for(i=0;i<busy_cnt;i++)
	{
		dev_info(&dev->interface->dev, "data_in_bufs_busy=%p (addr=%p dma=%p urb=%p urb->status=%s urb->transfer_flags=%x urb->actual_length=%d, cnt=%ld taken=%d taken_from=%d ts=%u.%u)\n",
				dev->snapshot->busy_bufs[i], 
				dev->snapshot->busy_bufs[i]->addr,
				(void*)dev->snapshot->busy_bufs[i]->dma,
				dev->snapshot->busy_bufs[i]->urb,
				dev->snapshot->busy_bufs[i]->urb ? status2E(dev->snapshot->busy_bufs[i]->urb->status) : "???",
				(int)(dev->snapshot->busy_bufs[i]->urb? dev->snapshot->busy_bufs[i]->urb->transfer_flags : -1),
				(int)(dev->snapshot->busy_bufs[i]->urb? dev->snapshot->busy_bufs[i]->urb->actual_length : -1),
				dev->snapshot->busy_bufs[i]->cnt,
				dev->snapshot->busy_bufs[i]->taken, dev->snapshot->busy_bufs[i]->taken_from,
				dev->snapshot->busy_bufs[i]->tv_sec,
				dev->snapshot->busy_bufs[i]->tv_usec);
	}

	for(i=0;i<rdy_cnt;i++)
	{
		dev_info(&dev->interface->dev, "data_in_bufs_ready=%p (addr=%p dma=%p urb=%p urb->status=%s urb->transfer_flags=%x urb->actual_length=%d, cnt=%ld taken=%d taken_from=%d)\n",
				dev->snapshot->rdy_bufs[i],
				dev->snapshot->rdy_bufs[i]->addr,
				(void*)dev->snapshot->rdy_bufs[i]->dma,
				dev->snapshot->rdy_bufs[i]->urb,
				dev->snapshot->rdy_bufs[i]->urb ? status2E(dev->snapshot->rdy_bufs[i]->urb->status) : "???",
				(int)(dev->snapshot->rdy_bufs[i]->urb? dev->snapshot->rdy_bufs[i]->urb->transfer_flags : -1),
				(int)(dev->snapshot->rdy_bufs[i]->urb? dev->snapshot->rdy_bufs[i]->urb->actual_length : -1),
				dev->snapshot->rdy_bufs[i]->cnt,
				dev->snapshot->rdy_bufs[i]->taken, dev->snapshot->rdy_bufs[i]->taken_from);
	}
}
#endif

#if 0
static void dump_periodic(rx666m_device_t *dev)
{
	s64 ts_diff ;
	dev->ts_end = ktime_get();

	ts_diff = ktime_to_ms(ktime_sub(dev->ts_end, dev->ts_start));
	if( ts_diff > 5000 )
	{
		dev->ts_start = dev->ts_end;

		dump_info(dev);
	}
}
#endif

static int rx666m_submit_urb(rx666m_device_t *dev, int taken_from)
{
    struct urb *urb;
    unsigned long irq_flags;
	rx666m_data_buffer_t *data_buf;
    int ret = 0;
	struct timespec64 now;

    for(;;)
	{
		ktime_get_ts64(&now);

		spin_lock_irqsave(&dev->data_in_lock, irq_flags);
		if (atomic_read(&dev->data_in_progress) < NUM_CONCURRENT_TRANSFERS && atomic_read(&dev->data_in_used) < NUM_DATA_URB)
		{
			if (list_empty(&dev->data_in_bufs_avail))
			{
				ret = -1;
				goto leave_spin;
			}

			data_buf = list_first_entry(&dev->data_in_bufs_avail, rx666m_data_buffer_t, list);
			list_move(&data_buf->list, &dev->data_in_bufs_busy);

			urb = data_buf->urb;

			atomic_inc(&dev->data_in_used);
			atomic_inc(&dev->data_in_progress);
			data_buf->taken=1;
			data_buf->taken_from=taken_from;
			data_buf->tv_sec = now.tv_sec;
			data_buf->tv_usec = now.tv_nsec;

			spin_unlock_irqrestore(&dev->data_in_lock, irq_flags);

			data_buf->cnt++;

			usb_fill_bulk_urb(urb, dev->udev, usb_rcvbulkpipe(dev->udev, 0x81),
					data_buf->addr, DATA_BUFFER_SIZE, rx666m_read_cb, data_buf);
					urb->transfer_flags = 0;
			#if DMA_EN
				urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
				urb->transfer_dma = data_buf->dma;
			#endif
			
			#if ANCHORURB
				usb_anchor_urb(urb, &dev->data_in_anchor);
			#endif
			ret = usb_submit_urb(urb, GFP_ATOMIC);
			if (ret)
			{
				if(ANCHORURB) usb_unanchor_urb(urb);

				spin_lock_irqsave(&dev->data_in_lock, irq_flags);	
				list_move(&data_buf->list, &dev->data_in_bufs_avail);
				data_buf->taken=0;
				atomic_dec(&dev->data_in_used);
				atomic_dec(&dev->data_in_progress);
				spin_unlock_irqrestore(&dev->data_in_lock, irq_flags);

			}
			else
			{
				dev->submitted++;
			}

			goto leave_rx;
		}
		else
		{
			//no more available urbs
			goto leave_spin;
		}
	}
leave_spin:
    spin_unlock_irqrestore(&dev->data_in_lock, irq_flags);
leave_rx:

    return ret;
}

static inline const char* translate_status(int status)
{
        char *errmsg = "Unknown";

        switch (status) {
        case -ENOENT:
                errmsg = "unlinked synchronously";
                break;
        case -ECONNRESET:
                errmsg = "unlinked asynchronously";
                break;
        case -ENOSR:
                errmsg = "Buffer error (overrun)";
                break;
        case -EPIPE:
                errmsg = "Stalled (device not responding)";
                break;
        case -EOVERFLOW:
                errmsg = "Babble (bad cable?)";
                break;
        case -EPROTO:
                errmsg = "Bit-stuff error (bad cable?)";
                break;
        case -EILSEQ:
                errmsg = "CRC/Timeout (could be anything)";
                break;
        case -ETIME:
                errmsg = "Device does not respond";
                break;
        }

		return errmsg;
}

static void rx666m_read_cb(struct urb *urb)
{
	rx666m_data_buffer_t *data_buf;
    rx666m_device_t *dev;
    unsigned long flags;
	int status;

	usb_unanchor_urb(urb);

    data_buf = (rx666m_data_buffer_t*)urb->context;
	dev = data_buf->dev;
	dev->read_cb_cnt++; //TODO: change to atomic


	switch (urb->status) {
	case 0:             /* success */
		break;
	case -ETIMEDOUT:
		dev_err_ratelimited(&dev->interface->dev, "URB timeout %d urb=%p\n", urb->status, urb);
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ENODEV:
	case -ESHUTDOWN:
		dev_err_ratelimited(&dev->interface->dev, "URB failed1 %d urb=%p, ret\n", urb->status, urb);
		break;
	default:            /* error */
		dev_err_ratelimited(&dev->interface->dev, "URB failed2 %d urb=%p\n", urb->status, urb);
		break;
	}

	if(urb->status)
		dev_err_ratelimited(&dev->interface->dev, "URB failed %d [%s]", urb->status, translate_status(urb->status));

	if(!data_buf->taken)
		dev_err_ratelimited(&dev->interface->dev, "Buffer not taken %p\n", data_buf);

    spin_lock_irqsave(&dev->data_in_lock, flags);
	data_buf->taken=0;
	if(urb->status)
	{
		list_move_tail(&data_buf->list, &dev->data_in_bufs_avail);
		atomic_dec(&dev->data_in_used);
		atomic_dec(&dev->data_in_progress);
	}
	else
	{
		list_move_tail(&data_buf->list, &dev->data_in_bufs_ready);
	    atomic_dec(&dev->data_in_progress);
	    atomic_inc(&dev->data_in_ready);
	}
	status = urb->status;
    spin_unlock_irqrestore(&dev->data_in_lock, flags);

	if(status)
	{
		dev->bad_status++;
	}

    if (dev->rx_en)
	{
#if 0
		int i;
		for(i=0;i<NUM_CONCURRENT_TRANSFERS && atomic_read(&dev->data_in_progress) < NUM_CONCURRENT_TRANSFERS ;++i)
		if(rx666m_submit_urb(dev, 1))
		{
			dev->overrun++;
			dev_err_ratelimited(&dev->interface->dev, "URB submit overrun CB\n");
		}
		else
		{
			//dev_info(&dev->interface->dev, "URB submited from CB\n");
	//dev_info(&dev->interface->dev, "data_in_ready=%d/%d\n", (int)atomic_read(&dev->data_in_ready), NUM_DATA_URB);
	//dev_info(&dev->interface->dev, "data_in_used=%d/%d\n", (int)atomic_read(&dev->data_in_used), NUM_DATA_URB);
	//dev_info(&dev->interface->dev, "data_in_progress=%d/%d\n", (int)atomic_read(&dev->data_in_progress), NUM_CONCURRENT_TRANSFERS);

		}
#endif
	}
	if(!status)
	{
    	wake_up_interruptible(&dev->data_in_wait);
	}
}

#if !ANCHORURB
static void rx666m_kill_urbs(rx666m_device_t *dev)
{
	rx666m_data_buffer_t   *busy_bufs[NUM_DATA_URB];
	unsigned long irq_flags;
	size_t i, busy_cnt=0;
	struct list_head* p;

	memset(busy_bufs, 0, sizeof(busy_bufs));

    spin_lock_irqsave(&dev->data_in_lock, irq_flags);
	list_for_each(p, &dev->data_in_bufs_busy)
	{
		busy_bufs[busy_cnt++] = list_entry(p, rx666m_data_buffer_t, list);

	}
	spin_unlock_irqrestore(&dev->data_in_lock, irq_flags);

	for(i=0;i<busy_cnt;i++)
	{
		usb_kill_urb(busy_bufs[i]->urb);
	}
}
#endif

static int rx666m_start(rx666m_device_t *dev)
{
    int i;
    void *buf;
    struct urb *urb;

	dev_info(&dev->interface->dev, "starting\n");

    dev->rx_en = 0;
    atomic_set(&dev->data_in_ready, 0);
    atomic_set(&dev->data_in_used, 0);

#if LOG_EN
	dev->snapshot = (rx666m_driver_snapshot_t*)kmalloc(sizeof(rx666m_driver_snapshot_t), GFP_KERNEL);
	if(!dev->snapshot)
	{
		dev_err(&dev->interface->dev, "Could not allocate snapshot\n");
		return -1;
	}
#endif

    for (i = 0; i < NUM_DATA_URB; i++)
	{
		rx666m_data_buffer_t *entry = (rx666m_data_buffer_t*)kzalloc(sizeof(rx666m_data_buffer_t), GFP_ATOMIC);
		if(entry == NULL)
		{
			//remove already allocated buffers
			return -1;
		}

		INIT_LIST_HEAD(&entry->list);
		list_add(&entry->list, &dev->data_in_bufs_avail);

		entry->dev = dev;

#if DMA_EN
        buf = usb_alloc_coherent(dev->udev, DATA_BUFFER_SIZE, GFP_ATOMIC, &entry->dma);
#else
		buf = kzalloc(DATA_BUFFER_SIZE, GFP_KERNEL_TRY);
#endif
        //memset(buf, 0, DATA_BUFFER_SIZE);

        if (!buf)
		{
            dev_err_ratelimited(&dev->interface->dev, "Could not allocate data IN buffer\n");
			//remove already allocated buffers
            return -1;
        }

        entry->addr = buf;

        urb = usb_alloc_urb(0, GFP_ATOMIC);
        if (!buf)
		{
            dev_err_ratelimited(&dev->interface->dev, "Could not allocate data IN URB\n");
			//remove already allocated buffers
            return -1;
        }

        entry->urb = urb;

        usb_fill_bulk_urb(urb, dev->udev, usb_rcvbulkpipe(dev->udev, 0x81),
                entry->addr, DATA_BUFFER_SIZE, rx666m_read_cb, entry);

		urb->transfer_flags = 0;
		#if DMA_EN
	        urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
        	urb->transfer_dma = entry->dma;
		#endif
    }


	#if LOG_EN
		stats_dumper_start_workqueue( &dev->stats_dumper_priv, dev );
	#endif

	dev_info(&dev->interface->dev, "started\n");

    return 0;
}

static void rx666m_stop(rx666m_device_t *dev)
{
	struct list_head *p, *q;

	rx666m_data_buffer_t *data_buf;


	#if LOG_EN
		stats_dumper_stop_workqueue( &dev->stats_dumper_priv );

        kfree(dev->snapshot);
		dev->snapshot = NULL;
	#endif
	//TODO: wait until all transfers in progress are completed

	list_for_each_safe(p, q, &dev->data_in_bufs_avail)
	{
		data_buf = list_entry(p, rx666m_data_buffer_t, list);

		#if DMA_EN
        	usb_free_coherent(dev->udev, DATA_BUFFER_SIZE, data_buf->addr, data_buf->dma);
		#else
			kfree(data_buf->addr);
		#endif

        usb_free_urb(data_buf->urb);

		list_del(p);

		kfree(data_buf);
    }

	//TODO: check until all callback has been executed
	INIT_LIST_HEAD(&dev->data_in_bufs_avail);
	INIT_LIST_HEAD(&dev->data_in_bufs_busy);
	INIT_LIST_HEAD(&dev->data_in_bufs_ready);

	dev_info(&dev->interface->dev, "stop: avail=%d busy=%d\n", list_empty(&dev->data_in_bufs_avail), list_empty(&dev->data_in_bufs_busy));
}

static int disable_rx(rx666m_device_t *dev)
{
    int ret;
	unsigned long flags;

	dev_info(&dev->interface->dev, "disabling rx\n");

    if (dev->intnum != 1)
        return -1;


    dev->rx_en = 0;

    #if ANCHORURB
		usb_kill_anchored_urbs(&dev->data_in_anchor);
	#else
		rx666m_kill_urbs(dev);
	#endif

    ret = 0;
    atomic_set(&dev->data_in_ready, 0);
    atomic_set(&dev->data_in_used, 0);
    atomic_set(&dev->data_in_progress, 0);

	spin_lock_irqsave(&dev->data_in_lock, flags);

	list_splice(&dev->data_in_bufs_busy, &dev->data_in_bufs_avail);
	list_splice(&dev->data_in_bufs_ready, &dev->data_in_bufs_avail);
    INIT_LIST_HEAD(&dev->data_in_bufs_busy);
    INIT_LIST_HEAD(&dev->data_in_bufs_ready);
	spin_unlock_irqrestore(&dev->data_in_lock, flags);


	dev_info(&dev->interface->dev, "rx disabled\n");

    return ret;
}

static int enable_rx(rx666m_device_t *dev)
{
    int ret;
    int i;
    unsigned int val;
    val = 1;

	dev_info(&dev->interface->dev, "enabling rx\n");

    if (dev->intnum != 1)
	{
        return -1;
	}

    if (dev->disconnecting)
	{
        return -ENODEV;
	}


	dev->ts_start = ktime_get();
    ret = 0;
    dev->rx_en = 1;

    for (i=0; i<NUM_CONCURRENT_TRANSFERS; i++)
	{
		dev_err(&dev->interface->dev, "Submit initial URBs (%d/%d)\n", i, NUM_CONCURRENT_TRANSFERS); /////////////////////
        if ((ret = rx666m_submit_urb(dev, 2)) < 0)
		{
            dev_err(&dev->interface->dev, "Error submitting initial RX URBs (%d/%d), error=%d\n", i, NUM_CONCURRENT_TRANSFERS, ret);
            break;
        }
    }


	dev_info(&dev->interface->dev, "rx enabled\n");

    return ret;
}

static ssize_t rx666m_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    ssize_t ret = 0;
    rx666m_device_t *dev;
    unsigned long flags;
	rx666m_data_buffer_t *data_buf;
	size_t ptr = 0;

    dev = (rx666m_device_t *)file->private_data;

    if (dev->intnum != 1)
	{
		dev_err_ratelimited(&dev->interface->dev, "read intnum\n");
        return -1;
    }

    if (dev->reader)
	{
        if (file != dev->reader)
		{
			dev_err_ratelimited(&dev->interface->dev, "read EPERM\n");
            return -EPERM;
        }
    }
	else
	{
        dev->reader = file;
	}

    if (dev->disconnecting)
	{
		dev_err_ratelimited(&dev->interface->dev, "read ENODEV\n");
        return -ENODEV;
	}

    if (!dev->rx_en)
	{
        if (enable_rx(dev))
		{
			dev_err_ratelimited(&dev->interface->dev, "read EINVAL\n");
            return -ENODEV;
        }
    }

	if (count < DATA_BUFFER_SIZE)
	{
		return -EINVAL;
	}

    for(;dev->rx_en;)
	{
#if 1
		int i;
        for(i=0;i<NUM_CONCURRENT_TRANSFERS && atomic_read(&dev->data_in_progress) < NUM_CONCURRENT_TRANSFERS ;++i)
        if (atomic_read(&dev->data_in_progress) < NUM_CONCURRENT_TRANSFERS)
		{
			rx666m_submit_urb(dev, 4);
			//dev_err_ratelimited(&dev->interface->dev, "URB submit1\n");
		}
#endif
        spin_lock_irqsave(&dev->data_in_lock, flags);
        if (atomic_read(&dev->data_in_ready) && !list_empty(&dev->data_in_bufs_ready))
		{
			//dev_info(&dev->interface->dev, "read some\n");
            atomic_dec(&dev->data_in_ready);
            atomic_dec(&dev->data_in_used);

			data_buf = list_first_entry(&dev->data_in_bufs_ready, rx666m_data_buffer_t, list);
			list_del(&data_buf->list);
            spin_unlock_irqrestore(&dev->data_in_lock, flags);

#if 0
            if (copy_to_user(&buf[ptr], data_buf->addr, DATA_BUFFER_SIZE))
			{
                ret = -EFAULT;
            }
			else
			{
				ptr += DATA_BUFFER_SIZE;
                ret = 0;
            }
#else
			ptr += DATA_BUFFER_SIZE;
			ret = 0;
#endif
			spin_lock_irqsave(&dev->data_in_lock, flags);
			list_add(&data_buf->list, &dev->data_in_bufs_avail);
			spin_unlock_irqrestore(&dev->data_in_lock, flags);

			//dev_info(&dev->interface->dev, "read some - done\n");
	//dev_info(&dev->interface->dev, "data_in_ready=%d/%d\n", (int)atomic_read(&dev->data_in_ready), NUM_DATA_URB);
	//dev_info(&dev->interface->dev, "data_in_used=%d/%d\n", (int)atomic_read(&dev->data_in_used), NUM_DATA_URB);
	//dev_info(&dev->interface->dev, "data_in_progress=%d/%d\n", (int)atomic_read(&dev->data_in_progress), NUM_CONCURRENT_TRANSFERS);

#if 0
			//if(!atomic_read(&dev->data_in_progress))
	        for(i=0;i<NUM_CONCURRENT_TRANSFERS && atomic_read(&dev->data_in_progress) < NUM_CONCURRENT_TRANSFERS ;++i)
            if (atomic_read(&dev->data_in_progress) < NUM_CONCURRENT_TRANSFERS)
			{
                rx666m_submit_urb(dev, 3);
			//	dev_err_ratelimited(&dev->interface->dev, "URB submit2\n");
			}
#endif
			if(!ret && count-ptr < DATA_BUFFER_SIZE)
			{
				ret = ptr;
				break;
			}

			if(ret < 0)
			{
				break;
			}

        }
		else
		{
            spin_unlock_irqrestore(&dev->data_in_lock, flags);
            ret = wait_event_interruptible_timeout(dev->data_in_wait, atomic_read(&dev->data_in_ready), RX666_READ_TIMEOUT_S * HZ);
            if (ret < 0)
			{
				dev_info(&dev->interface->dev, "read error - wait_event_interruptible_timeout\n");
                break;
            }
			else if (ret == 0)
			{
				dev_info(&dev->interface->dev, "read timeout\n");
                ret = -ETIMEDOUT;
                break;
            }
			else
			{
                ret = 0;
            }
        }
    }

	if(ret<0)
	{
		dev_info(&dev->interface->dev, "read error ret=%ld\n", ret);
	}

	dev->last_read_ret = ret;

    return ret;
}

static ssize_t rx666m_write(struct file *file, const char *user_buf, size_t count, loff_t *ppos)
{
    rx666m_device_t *dev;

    dev = (rx666m_device_t *)file->private_data;

	return -EFAULT;
}

long rx666m_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    rx666m_device_t *dev;
    void __user *data;
    int ret;
    int retval = -EINVAL;
	size_t index;
	uint8_t *buf;
	rx666m_ioctl_write_ram_t arg_wr_ram;

    dev = file->private_data;
    data = (void __user *)arg;

	dev_info(&dev->interface->dev, "ioctl: %d \n", cmd);

	if (_IOC_TYPE(cmd) != MYDRBASE)
		return -EINVAL;

    switch (cmd)
	{
		case RX666M_IS_BOOTLOADER_RUNNING:
			{
				uint8_t *buf = (uint8_t*)kmalloc(MAX_IOCTL_SIZE, GFP_KERNEL);
					if(!buf)
						return -ENOMEM;

				dev_info(&dev->interface->dev, "RX666M_IS_BOOTLOADER_RUNNING\n");

				retval = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
						0xA0, 0x40, 0x0000, 0x0000, buf, 1, RX666_TIMEOUT_MS);

				dev_info(&dev->interface->dev, "RX666M_IS_BOOTLOADER_RUNNING: ret=%d retval=%d\n", (int)ret, (int)retval);

				if(!retval)
					retval = buf[0];

				kfree(buf);
			}
			break;
		case RX666M_WRITE_RAM:
			retval = 0;

			dev_info(&dev->interface->dev, "RX666M_WRITE_RAM\n");

            if (copy_from_user(&arg_wr_ram, data, sizeof(rx666m_ioctl_write_ram_t))) {
                return -EFAULT;
            }

			if(arg_wr_ram.len == 0)
			{
					ret = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
							0xA0, 0x40, (uint16_t)(arg_wr_ram.address & 0xffff), (uint16_t)(arg_wr_ram.address >> 16),
							NULL, 0, RX666_TIMEOUT_MS);
					if(ret)
					{
						dev_err(&dev->interface->dev, "RX666M_WRITE_RAM retval=%d\n", ret);
						retval=-EFAULT;
						break;
					}
			}
			else
			{
				uint8_t *buf = (uint8_t*)kmalloc(MAX_IOCTL_SIZE, GFP_KERNEL);
				if(!buf)
					return -ENOMEM;

				index = 0;
				while(arg_wr_ram.len > 0)
				{
					size_t size = (arg_wr_ram.len > MAX_IOCTL_SIZE) ? MAX_IOCTL_SIZE : arg_wr_ram.len;

					dev_info(&dev->interface->dev, "RX666M_WRITE_RAM arg_wr_ram.address=%x size=%lx\n", arg_wr_ram.address, size);

					if (copy_from_user(buf, arg_wr_ram.firmware + index, size)) {
						retval=-EFAULT;
						break;
					}

					ret = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
							0xA0, 0x40, (uint16_t)(arg_wr_ram.address & 0xffff), (uint16_t)(arg_wr_ram.address >> 16),
							buf, size, RX666_TIMEOUT_MS);
					if(ret != size)
					{
						dev_err(&dev->interface->dev, "RX666M_WRITE_RAM retval=%d\n", ret);
						retval=-EFAULT;
						break;
					}

					dev_info(&dev->interface->dev, "RX666M_WRITE_RAM retval=%d arg_wr_ram.address=%x size=%lx\n", retval, arg_wr_ram.address, size);

					arg_wr_ram.len -= size;
					arg_wr_ram.address += size;
					index += size;
				}

				kfree(buf);
			}

			break;
		case RX666M_I2C_READ:
			dev_info(&dev->interface->dev, "RX666M_I2C_READ address\n");
			retval = 0;

			buf = (uint8_t*)kmalloc(MAX_IOCTL_SIZE, GFP_KERNEL);
			if(!buf)
				return -ENOMEM;

			if (copy_from_user(buf, data, sizeof(rx666m_ioctl_i2c_transfer_t)))
			{
				retval=-EFAULT;
			}

			if(!retval)
			{
				rx666m_ioctl_i2c_transfer_t *tr_data = (rx666m_ioctl_i2c_transfer_t*)buf;

				dev_info(&dev->interface->dev, "RX666M_I2C_READ address=0x%x reg=0x%x len=0x%x", (int)tr_data->address, (int)tr_data->reg, (int)tr_data->len);

				ret = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
						0xbe, 0xc0, tr_data->address, tr_data->reg,
						tr_data->data, tr_data->len, RX666_TIMEOUT_MS);
				if(ret != tr_data->len)
				{
					dev_err(&dev->interface->dev, "RX666M_I2C_READ retval=%d\n", ret);
					retval=-EFAULT;
				}
			}

			if(!retval)
			{
				if (copy_to_user(data, buf, sizeof(rx666m_ioctl_i2c_transfer_t)))
				{
					retval=-EFAULT;
				}
			}

			kfree(buf);

			break;

		case RX666M_I2C_WRITE:
			dev_info(&dev->interface->dev, "RX666M_I2C_WRITE address\n");
			retval = 0;

			buf = (uint8_t*)kmalloc(MAX_IOCTL_SIZE, GFP_KERNEL);
			if(!buf)
				return -ENOMEM;

			if (copy_from_user(buf, data, sizeof(rx666m_ioctl_i2c_transfer_t)))
			{
				retval=-EFAULT;
			}

			if(!retval)
			{
				rx666m_ioctl_i2c_transfer_t *tr_data = (rx666m_ioctl_i2c_transfer_t*)buf;

				dev_info(&dev->interface->dev, "RX666M_I2C_WRITE address=0x%x reg=0x%x len=0x%x", (int)tr_data->address, (int)tr_data->reg, (int)tr_data->len);

				ret = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
						0xba, 0x40, tr_data->address, tr_data->reg,
						tr_data->data, tr_data->len, RX666_TIMEOUT_MS);
				if(ret != tr_data->len)
				{
					dev_err(&dev->interface->dev, "RX666M_I2C_WRITE retval=%d\n", ret);
					retval=-EFAULT;
				}
			}

			if(!retval)
			{
				if (copy_to_user(data, buf, sizeof(rx666m_ioctl_i2c_transfer_t)))
				{
					retval=-EFAULT;
				}
			}

			kfree(buf);

			break;

		case RX666M_GPIO_WRITE:
			dev_info(&dev->interface->dev, "RX666M_GPIO_WRITE address\n");
			retval = 0;

			buf = (uint8_t*)kmalloc(MAX_IOCTL_SIZE, GFP_KERNEL);
			if(!buf)
				return -ENOMEM;


			buf[0]=arg;
			ret = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
					0xbc, 0x40, 0, 0,
					buf, 1, RX666_TIMEOUT_MS);
			if(ret != 1)
			{
				dev_err(&dev->interface->dev, "RX666M_GPIO_WRITE retval=%d\n", ret);
				retval=-EFAULT;
			}

			kfree(buf);

			break;

		case RX666M_START:
			dev_info(&dev->interface->dev, "RX666M_SART\n");
			retval = 0;

			buf = (uint8_t*)kmalloc(MAX_IOCTL_SIZE, GFP_KERNEL);
			if(!buf)
				return -ENOMEM;


			buf[0]=0;
			ret = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
					0xaa, 0x40, 0, 0,
					buf, 1, RX666_TIMEOUT_MS);
			if(ret<0)
			{
				dev_err(&dev->interface->dev, "RX666M_START retval=%d\n", ret);
				retval=-EFAULT;
			}
			kfree(buf);

			break;
    }

	dev_info(&dev->interface->dev, "ioctl end: %x retval=%d\n", (int)cmd, (int)retval);

    return retval;
}

static int rx666m_open(struct inode *inode, struct file *file)
{
    rx666m_device_t *dev;
    struct usb_interface *interface;
    int subminor;

	printk("open\n");

    subminor = iminor(inode);

    interface = usb_find_interface(&rx666m_driver, subminor);
    if (interface == NULL)
	{
        pr_err("%s - error, cannot find device for minor %d\n", __func__, subminor);
        return -ENODEV;
    }


    dev = usb_get_intfdata(interface);
    if (dev == NULL)
	{
        return -ENODEV;
    }

    file->private_data = dev;

	dev_info(&interface->dev, "opened\n");

    return 0;
}

static int rx666m_release(struct inode *inode, struct file *file)
{
    rx666m_device_t *dev;

	printk("rx666m release\n");

    dev = (rx666m_device_t *)file->private_data;

    if (dev->reader && dev->reader == file)
	{
        if (dev->rx_en)
		{
            disable_rx(dev);
        }
        dev->reader = NULL;
    }

	printk("rx666m released");

    return 0;
}

static char *rx666m_devnode(struct device *dev, umode_t *mode)
{
	if (! mode)
	{
		return NULL;
	}
	*mode = 0666;

	return NULL;
}

static struct file_operations rx666m_fops =
{
    .owner    =  THIS_MODULE,
    .read     =  rx666m_read,
    .write    =  rx666m_write,
    .unlocked_ioctl = rx666m_ioctl,
    .open     =  rx666m_open,
    .release  =  rx666m_release,
};

static struct usb_class_driver rx666m_class =
{
    .name       = "rx666m",
    .fops       = &rx666m_fops,
	.devnode	= rx666m_devnode,
    .minor_base = USB_RX666M_MINOR_BASE,
};

static int rx666m_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    rx666m_device_t *dev;
	struct usb_device *udev;
    int retval;

	dev_info(&interface->dev, "probing...\n");

    if (interface->cur_altsetting->desc.bInterfaceNumber != 0)
	{
        return 0;
	}

    udev = interface_to_usbdev(interface);


    dev = kzalloc(sizeof(rx666m_device_t), GFP_KERNEL_TRY);
    if (dev == NULL)
	{
        dev_err(&interface->dev, "Out of memory\n");
        goto error_oom;
    }

	INIT_LIST_HEAD(&dev->data_in_bufs_avail);
	INIT_LIST_HEAD(&dev->data_in_bufs_busy);
	INIT_LIST_HEAD(&dev->data_in_bufs_ready);

    spin_lock_init(&dev->data_in_lock);
    dev->udev = usb_get_dev(interface_to_usbdev(interface));
    dev->interface = interface;
    dev->intnum = 1;
	dev->read_cb_cnt = 0;
	dev->bad_status = 0;
	dev->overrun = 0;
    dev->disconnecting = 0;

    atomic_set(&dev->data_in_progress, 0);

    #if ANCHORURB
		init_usb_anchor(&dev->data_in_anchor);
	#endif
    init_waitqueue_head(&dev->data_in_wait);

    rx666m_start(dev);

    usb_set_intfdata(interface, dev);

    retval = usb_register_dev(interface, &rx666m_class);
    if (retval)
	{
        dev_err(&interface->dev, "Unable to get a minor device number for RX666m device\n");
        usb_set_intfdata(interface, NULL);
        return retval;
    }


    dev_info(&interface->dev, "device is now attached\n");
    return 0;

error_oom:
    return -ENOMEM;
}

static void rx666m_disconnect(struct usb_interface *interface)
{
    rx666m_device_t *dev;
    if (interface->cur_altsetting->desc.bInterfaceNumber != 0)
        return;

    dev = usb_get_intfdata(interface);

    dev->disconnecting = 1;
    dev->rx_en = 0;

    #if ANCHORURB
		usb_kill_anchored_urbs(&dev->data_in_anchor);
	#endif

    rx666m_stop(dev);

    usb_deregister_dev(interface, &rx666m_class);

    usb_set_intfdata(interface, NULL);

    usb_put_dev(dev->udev);

    dev_info(&interface->dev, "device has been disconnected\n");

    kfree(dev);
}

static struct usb_driver rx666m_driver =
{
    .name = "rx666m",
    .probe = rx666m_probe,
    .disconnect = rx666m_disconnect,
    .id_table = rx666m_table,
};

module_usb_driver(rx666m_driver);

MODULE_AUTHOR("Marcin Odrzywolski <emk6@wp.pl>");
MODULE_DESCRIPTION("RX666m USB driver");
MODULE_VERSION("v1.1");
MODULE_LICENSE("GPL");
