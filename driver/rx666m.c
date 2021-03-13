/*
 * rx666m USB driver
 * Copyright (C) 2020 Marcin Odrzywolski
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

//////////////////////////////////
#define BLADE_USB_CMD_QUERY_VERSION             0
#define USB_CYPRESS_VENDOR_ID   0x04b4
#define USB_FX3_PRODUCT_ID      0x00f3
#define USB_NUAND_BLADERF_MINOR_BASE 193
#define NUM_CONCURRENT  8
#define NUM_DATA_URB    NUM_CONCURRENT*1024*128
#define DATA_BUF_SZ     (1024*8)
#define USB_TYPE_OUT      0x40
#define BLADE_USB_TYPE_IN       0xC0
#define RX666_TIMEOUT_MS		 1000

#define MAX_WRITE_SIZE          (2 * 1024)

#   define PACK(decl_to_pack_) \
            decl_to_pack_ __attribute__((__packed__))
//#define MY_DBG_MSG dev_info
#define MY_DBG_MSG(args, ...)


#define ANCHORURB 1
#define DMA_EN 1
//////////////////////////////////

#if 0
struct data_buffer
{
    struct urb  *urb;
    void        *addr;
    dma_addr_t   dma;
    int          valid;
};
#endif

typedef struct {
	struct rx666m_device_s *dev;

    struct workqueue_struct * wq;
    struct delayed_work periodic_work;
} lcd_priv_t;


typedef struct rx666m_device_s
{
    struct usb_device    *udev;
    struct usb_interface *interface;

    int                   intnum;
    int                   disconnecting;


    int                   rx_en;
    spinlock_t            data_in_lock;		// spinlock for data_in*
    wait_queue_head_t     data_in_wait;		// event

    //unsigned int          data_in_consumer_idx;
    //unsigned int          data_in_producer_idx;

    atomic_t              data_in_ready;       // number of buffers with data unread by the usermode application [transmit index - transmit index]
    atomic_t              data_in_used;         // number of buffers that may be inflight or have unread data [transmit index - consumer index]
    atomic_t              data_in_inflight;     // number of buffers currently in the USB stack [transmit index - consumer index]
    //struct data_buffer    data_in_bufs[NUM_DATA_URB];
#if ANCHORURB
    struct usb_anchor     data_in_anchor;
#endif
	struct list_head      data_in_bufs_avail;
	struct list_head      data_in_bufs_busy;
	struct list_head      data_in_bufs_ready;

    // allow only one reader
    struct file           *reader;

	ktime_t ts_start, 	ts_end;
    int bytes;
    int debug;
	size_t			read_cb_cnt;
	size_t			bad_status;
	size_t			overrun;
	size_t			submitted;
	size_t			monotonic_cnt;
	size_t			monotonic_cnt_last_read;
	size_t			last_read_ret;


	lcd_priv_t		lcd_priv;

	struct workqueue_struct *queue;

} rx666m_device_t;

typedef struct
{
	struct list_head list;

    struct urb  *urb;
    void        *addr;
    dma_addr_t   dma;
	size_t		 cnt;
	size_t		 monotonic_cnt;
	__u32		 tv_sec;
	__u32		 tv_usec;

	int			taken;

	rx666m_device_t	*dev;
}rx666m_data_buffer_t;


static void dump_info(rx666m_device_t *dev);



///////////////////////////////////////////
#define TOGGLE_FREQUENCY 5*HZ
void lcd_periodic_work(struct work_struct * work_struct_ptr)
{
    struct delayed_work * delayed = container_of(work_struct_ptr, struct delayed_work, work);
    lcd_priv_t * priv = container_of(delayed, lcd_priv_t, periodic_work);

	printk(KERN_INFO "lcd_periodic_work!\n");

	dump_info(priv->dev);

    queue_delayed_work(priv->wq, &priv->periodic_work, TOGGLE_FREQUENCY);
}

static void lcd_start_workqueue(lcd_priv_t *priv, rx666m_device_t *dev)
{
	priv->dev=dev;
    priv->wq = create_singlethread_workqueue("lcd_periodic_st_wq");

    INIT_DELAYED_WORK(&priv->periodic_work, lcd_periodic_work);
    queue_delayed_work(priv->wq, &priv->periodic_work, TOGGLE_FREQUENCY);
}

static void lcd_stop_workqueue(lcd_priv_t *priv)
{
    cancel_delayed_work_sync(&priv->periodic_work);
    destroy_workqueue(priv->wq);
}
//////////////////////////////////////////

static void __RX666m_read_cb(struct urb *urb);



static struct usb_driver rx666m_driver;

// USB PID-VID table
static struct usb_device_id rx666m_table[] = {
    { USB_DEVICE(USB_CYPRESS_VENDOR_ID, USB_FX3_PRODUCT_ID) },
    { USB_DEVICE(USB_CYPRESS_VENDOR_ID, 0x00f1) },
    { } /* Terminate entry */
};
MODULE_DEVICE_TABLE(usb, rx666m_table);

static void dump_info(rx666m_device_t *dev)
{
#if 0
	int i, avl_cnt=0, busy_cnt=0, rdy_cnt=0;
	struct list_head* p;
	rx666m_data_buffer_t   *avl_bufs[NUM_DATA_URB];
	rx666m_data_buffer_t   *busy_bufs[NUM_DATA_URB];
	rx666m_data_buffer_t   *rdy_bufs[NUM_DATA_URB];
	unsigned long irq_flags;

	dev_info(&dev->interface->dev, "read_cb_cnt=%ld\n", dev->read_cb_cnt);
	dev_info(&dev->interface->dev, "bad_status=%ld\n", dev->bad_status);
	dev_info(&dev->interface->dev, "overrun=%ld\n", dev->overrun);
	dev_info(&dev->interface->dev, "submitted=%ld\n", dev->submitted);
	//dev_info(&dev->interface->dev, "data_in_consumer_idx=%d/%d\n", dev->data_in_consumer_idx, NUM_DATA_URB);
	//dev_info(&dev->interface->dev, "data_in_producer_idx=%d/%d\n", dev->data_in_producer_idx, NUM_DATA_URB);
	dev_info(&dev->interface->dev, "data_in_ready=%d/%d\n", (int)atomic_read(&dev->data_in_ready), NUM_DATA_URB);
	dev_info(&dev->interface->dev, "data_in_used=%d/%d\n", (int)atomic_read(&dev->data_in_used), NUM_DATA_URB);
	dev_info(&dev->interface->dev, "data_in_inflight=%d/%d\n", (int)atomic_read(&dev->data_in_inflight), NUM_CONCURRENT);
#if ANCHORURB
	dev_info(&dev->interface->dev, "usb_anchor_empty %d\n", usb_anchor_empty(&dev->data_in_anchor));
#endif
	dev_info(&dev->interface->dev, "monotonic_cnt=%ld\n", dev->monotonic_cnt);
	dev_info(&dev->interface->dev, "monotonic_cnt_last_read=%ld\n", dev->monotonic_cnt_last_read);
	dev_info(&dev->interface->dev, "last_read_ret=%ld\n", dev->last_read_ret);

	memset(avl_bufs, 0, sizeof(avl_bufs));
	memset(busy_bufs, 0, sizeof(busy_bufs));
	memset(rdy_bufs, 0, sizeof(rdy_bufs));

    spin_lock_irqsave(&dev->data_in_lock, irq_flags);
	list_for_each(p, &dev->data_in_bufs_avail)
	{
		avl_bufs[avl_cnt++] = list_entry(p, rx666m_data_buffer_t, list);
	}

	list_for_each(p, &dev->data_in_bufs_busy)
	{
		busy_bufs[busy_cnt++] = list_entry(p, rx666m_data_buffer_t, list);

	}

	list_for_each(p, &dev->data_in_bufs_ready)
	{
		rdy_bufs[rdy_cnt++] = list_entry(p, rx666m_data_buffer_t, list);

	}
	spin_unlock_irqrestore(&dev->data_in_lock, irq_flags);

	for(i=0;i<avl_cnt;i++)
		dev_info(&dev->interface->dev, "data_in_bufs_avail=%p (addr=%p dma=%p urb=%p urb->status=%d urb->transfer_flags=%x cnt=%ld taken=%d)\n", avl_bufs[i], avl_bufs[i]->addr, (void*)avl_bufs[i]->dma, avl_bufs[i]->urb, avl_bufs[i]->urb ? avl_bufs[i]->urb->status : -1, (int)(avl_bufs[i]->urb? avl_bufs[i]->urb->transfer_flags : -1), avl_bufs[i]->cnt, avl_bufs[i]->taken);

	for(i=0;i<busy_cnt;i++)
		dev_info(&dev->interface->dev, "data_in_bufs_busy=%p (addr=%p dma=%p urb=%p urb->status=%d urb->transfer_flags=%x cnt=%ld taken=%d ts=%u.%u)\n", busy_bufs[i], busy_bufs[i]->addr, (void*)busy_bufs[i]->dma, busy_bufs[i]->urb, busy_bufs[i]->urb ? busy_bufs[i]->urb->status : -1, (int)(busy_bufs[i]->urb? busy_bufs[i]->urb->transfer_flags : -1) ,busy_bufs[i]->cnt, busy_bufs[i]->taken, busy_bufs[i]->tv_sec, busy_bufs[i]->tv_usec);

	for(i=0;i<rdy_cnt;i++)
		dev_info(&dev->interface->dev, "data_in_bufs_ready=%p (addr=%p dma=%p urb=%p urb->status=%d urb->transfer_flags=%x urb->actual_length=%d, cnt=%ld taken=%d monotonic_cnt=%ld)\n", rdy_bufs[i], rdy_bufs[i]->addr, (void*)rdy_bufs[i]->dma, rdy_bufs[i]->urb, rdy_bufs[i]->urb ? rdy_bufs[i]->urb->status : -1, (int)(rdy_bufs[i]->urb? rdy_bufs[i]->urb->transfer_flags : -1), (int)(rdy_bufs[i]->urb? rdy_bufs[i]->urb->actual_length : -1), rdy_bufs[i]->cnt, rdy_bufs[i]->taken, rdy_bufs[i]->monotonic_cnt);
#endif
}

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

static int __submit_rx_urb(rx666m_device_t *dev)
{
    struct urb *urb;
    unsigned long irq_flags;
	rx666m_data_buffer_t *data_buf;
    int ret = 0;
	//struct timeval now;
	struct timespec64 now;

	//MY_DBG_MSG(&dev->interface->dev, "RX666m enter __submit_rx_urb, use buf");
	//dump_info(dev);



    for(;;)
	{
		//do_gettimeofday(&now);
		ktime_get_ts64(&now);

    spin_lock_irqsave(&dev->data_in_lock, irq_flags);
	if (atomic_read(&dev->data_in_inflight) < NUM_CONCURRENT && atomic_read(&dev->data_in_used) < NUM_DATA_URB)
	{
		if (list_empty(&dev->data_in_bufs_avail))
		{
			//printk("data_in error\n");
			ret = -1;
			goto leave_spin;
		}

		data_buf = list_first_entry(&dev->data_in_bufs_avail, rx666m_data_buffer_t, list);
		list_move(&data_buf->list, &dev->data_in_bufs_busy);

		urb = data_buf->urb;

		atomic_inc(&dev->data_in_used);
		//if(data_buf->taken)
		//	printk("buffer taken!!!!");
		data_buf->taken=1;
		data_buf->monotonic_cnt=0;
		data_buf->tv_sec = now.tv_sec;
		//data_buf->tv_usec = now.tv_usec;  //old kernel
		data_buf->tv_usec = now.tv_nsec;	//new kernel

		spin_unlock_irqrestore(&dev->data_in_lock, irq_flags);

		//MY_DBG_MSG(&dev->interface->dev, "RX666m submitting  __submit_rx_urb %p urb=%p\n", data_buf, urb);
		data_buf->cnt++;

		atomic_inc(&dev->data_in_inflight);
		#if 1

        if(1) { usb_fill_bulk_urb(urb, dev->udev, usb_rcvbulkpipe(dev->udev, 0x81),
                data_buf->addr, DATA_BUF_SZ, __RX666m_read_cb, data_buf);
				urb->transfer_flags = 0;
#if DMA_EN
        urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
        urb->transfer_dma = data_buf->dma;
#endif
		}
		#if ANCHORURB
			usb_anchor_urb(urb, &dev->data_in_anchor);
		#endif
		ret = usb_submit_urb(urb, GFP_ATOMIC);
		if (ret)
		{
			//ustaw valid na = 1, unanchor, data_in_used, data_in_producer_idx
			//dev_info(&dev->interface->dev, "RX666m urb lost\n");
			if(ANCHORURB) usb_unanchor_urb(urb);

			spin_lock_irqsave(&dev->data_in_lock, irq_flags);	
			list_move(&data_buf->list, &dev->data_in_bufs_avail);
			data_buf->taken=0;
			data_buf->monotonic_cnt=0;
			spin_unlock_irqrestore(&dev->data_in_lock, irq_flags);

			atomic_dec(&dev->data_in_inflight);
			//goto leave_rx;
		}
		else
		{
			dev->submitted++;
		}
		#endif
		//MY_DBG_MSG(&dev->interface->dev, "RX666m submitted\n");
		goto leave_rx;
	}
	else
	{
		//ret = -1;
		//printk("no more available urbs\n");
		break;
	}
	}
leave_spin:
    spin_unlock_irqrestore(&dev->data_in_lock, irq_flags);
leave_rx:

	//dump_info(dev);

    return ret;
}

#if 0
static inline void print_err_status(struct tm6000_core *dev, int packet, int status)
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
        if (packet < 0) {
                dprintk(dev, 1, "URB status %d [%s].\n",
                        status, errmsg);
        } else {
                dprintk(dev, 1, "URB packet %d, status %d [%s].\n",
                        packet, status, errmsg);
        }
}
#endif

static void __RX666m_read_cb(struct urb *urb)
{
	rx666m_data_buffer_t *data_buf;
    rx666m_device_t *dev;
    unsigned long flags;
	int status;

    //usb_unanchor_urb(urb);
    data_buf = (rx666m_data_buffer_t*)urb->context;
	dev = data_buf->dev;
	dev->read_cb_cnt++; //zamien na atomic


	switch (urb->status) {
	case 0:             /* success */
	case -ETIMEDOUT:
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ENODEV:
	case -ESHUTDOWN:
		dev_err_ratelimited(&dev->interface->dev, "URB failed1 %d urb=%p, ret\n", urb->status, urb);
		dev_err(&dev->interface->dev, "URB failed1 %d, ret\n", urb->status);

		return;
	default:            /* error */
		dev_err_ratelimited(&dev->interface->dev, "URB failed2 %d urb=%p\n", urb->status, urb);
		//dev_err(&dev->interface->dev, "URB failed2 %d\n", urb->status);
		break;
	}

	MY_DBG_MSG(&dev->interface->dev, "RX666m read cb %p\n", data_buf);
	//dump_info(dev);

	if(!data_buf->taken)
		dev_err_ratelimited(&dev->interface->dev, "buffer not taken %p\n", data_buf);;

	if(1){
    spin_lock_irqsave(&dev->data_in_lock, flags);
	data_buf->taken=0;
	if(urb->status)
	{
		list_move_tail(&data_buf->list, &dev->data_in_bufs_avail);
		atomic_dec(&dev->data_in_used);
		atomic_dec(&dev->data_in_inflight);
	}
	else
	{
		list_move_tail(&data_buf->list, &dev->data_in_bufs_ready);
	    atomic_dec(&dev->data_in_inflight);
    	dev->bytes += DATA_BUF_SZ;
		data_buf->monotonic_cnt = ++dev->monotonic_cnt;
	    atomic_inc(&dev->data_in_ready);
	}
	status = urb->status;
    spin_unlock_irqrestore(&dev->data_in_lock, flags);
	}

#if 1
	//check status here!!!!
	if(status)
	{
		//dev_info(&dev->interface->dev, "RX666m read cb - bad status: %d\n", status);
		dev->bad_status++;
	}

	//dump_info(dev);

	MY_DBG_MSG(&dev->interface->dev, "RX666m read cb - about to submit rx_en=%d status=%d\n", (int)dev->rx_en, status);
    if (dev->rx_en)
	{
#if 1
		MY_DBG_MSG(&dev->interface->dev, "RX666m read cb - __submit_rx_urb\n");
		if(__submit_rx_urb(dev))
			dev->overrun++;
#endif
	}
	if(!status)
	{
    	wake_up_interruptible(&dev->data_in_wait);
	}
#endif

	//dump_info(dev);

	MY_DBG_MSG(&dev->interface->dev, "RX666m read cb - end\n");
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

	MY_DBG_MSG(&dev->interface->dev, "RX666m starting\n");

    dev->rx_en = 0;
    atomic_set(&dev->data_in_ready, 0);
    atomic_set(&dev->data_in_used, 0);
    //dev->data_in_consumer_idx = 0;
    //dev->data_in_producer_idx = 0;

    for (i = 0; i < NUM_DATA_URB; i++)
	{
		rx666m_data_buffer_t *entry = (rx666m_data_buffer_t*)kzalloc(sizeof(rx666m_data_buffer_t), GFP_ATOMIC);  //KERNEL
		if(entry == NULL)
		{
			//remove already allocated buffers
			return -1;
		}

		INIT_LIST_HEAD(&entry->list);
		list_add(&entry->list, &dev->data_in_bufs_avail);

		entry->dev = dev;

#if DMA_EN
        buf = usb_alloc_coherent(dev->udev, DATA_BUF_SZ, GFP_ATOMIC, &entry->dma);
#else
		buf = kzalloc(DATA_BUF_SZ, GFP_KERNEL);
#endif
        memset(buf, 0, DATA_BUF_SZ);
        if (!buf)
		{
            dev_err(&dev->interface->dev, "Could not allocate data IN buffer\n");
			//remove already allocated buffers
            return -1;
        }

        entry->addr = buf;
		MY_DBG_MSG(&dev->interface->dev, "RX666m start alloc %p %p\n", entry->addr, entry->dma);

        urb = usb_alloc_urb(0, GFP_KERNEL);
        if (!buf)
		{
            dev_err(&dev->interface->dev, "Could not allocate data IN URB\n");
			//remove already allocated buffers
            return -1;
        }
		MY_DBG_MSG(&dev->interface->dev, "RX666m allocating urb %p\n", urb);

        entry->urb = urb;
        //dev->data_in_bufs[i].valid = 1;

#if 1
        usb_fill_bulk_urb(urb, dev->udev, usb_rcvbulkpipe(dev->udev, 0x81),
                entry->addr, DATA_BUF_SZ, __RX666m_read_cb, entry);

		urb->transfer_flags = 0;
 #if DMA_EN
        urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
        urb->transfer_dma = entry->dma;
 #endif
#endif
    }




	//test_wq->dev=dev;
	//dev->queue = create_singlethread_workqueue("myworkqueue");
	//queue_delayed_work(dev->queue, &test_wq->work, 4 * HZ);

	//lcd_start_workqueue( &dev->lcd_priv, dev );

	MY_DBG_MSG(&dev->interface->dev, "RX666m started\n");

	//dump_info(dev);

    return 0;
}

static void rx666m_stop(rx666m_device_t *dev)
{
	struct list_head *p, *q;

	rx666m_data_buffer_t *data_buf;

	//lcd_stop_workqueue( &dev->lcd_priv );


	//dump_info(dev);

	//tczy tu nie trzeba sie upewnic ze juz nic w tle nie leci i ew nie dac jakiegos spinlcoka?

	list_for_each_safe(p, q, &dev->data_in_bufs_avail)
	{
		data_buf = list_entry(p, rx666m_data_buffer_t, list);
		MY_DBG_MSG(&dev->interface->dev, "RX666m stop: free %p %p %p", data_buf, data_buf->addr, data_buf->dma);

#if DMA_EN
        usb_free_coherent(dev->udev, DATA_BUF_SZ, data_buf->addr, data_buf->dma);
#else

		kfree(data_buf->addr);
#endif
        usb_free_urb(data_buf->urb);

		list_del(p);

		kfree(data_buf);
    }

	//sprawdz czy wszystkie callbacki sie skonczyly
	INIT_LIST_HEAD(&dev->data_in_bufs_avail);
	INIT_LIST_HEAD(&dev->data_in_bufs_busy);
	INIT_LIST_HEAD(&dev->data_in_bufs_ready);

	MY_DBG_MSG(&dev->interface->dev, "RX666m stop: empty1=%d empty=%d\n", list_empty(&dev->data_in_bufs_avail), list_empty(&dev->data_in_bufs_busy));
}

int __rx666m_snd_cmd(rx666m_device_t *dev, int cmd, void *ptr, __u16 len);

static int disable_rx(rx666m_device_t *dev)
{
    int ret;
	unsigned long flags;

	dev_info(&dev->interface->dev, "RX666m disabling rx\n");
	//dump_info(dev);

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
    atomic_set(&dev->data_in_inflight, 0);
    //dev->data_in_consumer_idx = 0;
    //dev->data_in_producer_idx = 0;

	spin_lock_irqsave(&dev->data_in_lock, flags);
	//move all buffer to availibility list
	list_splice(&dev->data_in_bufs_busy, &dev->data_in_bufs_avail);
	list_splice(&dev->data_in_bufs_ready, &dev->data_in_bufs_avail);
    INIT_LIST_HEAD(&dev->data_in_bufs_busy);
    INIT_LIST_HEAD(&dev->data_in_bufs_ready);
	spin_unlock_irqrestore(&dev->data_in_lock, flags);


	//dump_info(dev);
	dev_info(&dev->interface->dev, "RX666m rx disabled\n");

    return ret;
}

static int enable_rx(rx666m_device_t *dev)
{
    int ret;
    int i;
    unsigned int val;
    val = 1;

	dev_info(&dev->interface->dev, "RX666m enabling rx\n");
	//dump_info(dev);

    if (dev->intnum != 1)
        return -1;

    if (dev->disconnecting)
        return -ENODEV;

/*    for (i = 0; i < NUM_DATA_URB; i++)
	{
        dev->data_in_bufs[i].valid = 1;
	}*/

	dev->ts_start = ktime_get();
    ret = 0;
    dev->rx_en = 1;

    for (i=0; i<NUM_CONCURRENT; i++)
	{
		MY_DBG_MSG(&dev->interface->dev, "RX666m enable rx - __submit_rx_urb\n");
        if ((ret = __submit_rx_urb(dev)) < 0)
		{
            dev_err(&dev->interface->dev, "Error submitting initial RX URBs (%d/%d), error=%d\n", i, NUM_CONCURRENT, ret);
            break;
        }
    }

	//dump_info(dev);

	dev_info(&dev->interface->dev, "RX666m rx enabled\n");

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

	MY_DBG_MSG(&dev->interface->dev, "RX666m read\n");

	//dump_periodic(dev);

    if (dev->intnum != 1)
	{
		MY_DBG_MSG(&dev->interface->dev, "RX666m read intnum\n");
        return -1;
    }

    if (dev->reader)
	{
        if (file != dev->reader)
		{
			MY_DBG_MSG(&dev->interface->dev, "RX666m read EPERM\n");
            return -EPERM;
        }
    }
	else
	{
        dev->reader = file;
	}

    if (dev->disconnecting)
	{
		MY_DBG_MSG(&dev->interface->dev, "RX666m read ENODEV\n");
        return -ENODEV;
	}

    if (!dev->rx_en)
	{
        if (enable_rx(dev))
		{
			MY_DBG_MSG(&dev->interface->dev, "RX666m read EINVAL\n");
            return -ENODEV;
        }
    }

	if (count < DATA_BUF_SZ)
	{
		return -EINVAL;
	}

    for(;dev->rx_en;)
	{
		MY_DBG_MSG(&dev->interface->dev, "RX666m read\n");
		
		//dump_info(dev);

        spin_lock_irqsave(&dev->data_in_lock, flags);
        if (atomic_read(&dev->data_in_ready) && !list_empty(&dev->data_in_bufs_ready))
		{
            atomic_dec(&dev->data_in_ready);
            atomic_dec(&dev->data_in_used);

            //idx = dev->data_in_consumer_idx++;
            //dev->data_in_consumer_idx &= (NUM_DATA_URB - 1);
			//list_move(&dev->data_in_bufs_ready, &dev->data_in_bufs_avail); 
			data_buf = list_first_entry(&dev->data_in_bufs_ready, rx666m_data_buffer_t, list);
			list_del(&data_buf->list);
            spin_unlock_irqrestore(&dev->data_in_lock, flags);

#if 1
            if (copy_to_user(&buf[ptr], data_buf->addr, DATA_BUF_SZ))
			{
                ret = -EFAULT;
            }
			else
			{
				ptr += DATA_BUF_SZ;
                ret = 0;
            }
#else
			ptr += DATA_BUF_SZ;
			ret = 0;
#endif

			if(data_buf->monotonic_cnt < dev->monotonic_cnt_last_read)
			{
					dev_info(&dev->interface->dev, "RX666m read - out of order read %ld vs %ld\n", data_buf->monotonic_cnt, dev->monotonic_cnt_last_read);
			}
			dev->monotonic_cnt_last_read = data_buf->monotonic_cnt;

			spin_lock_irqsave(&dev->data_in_lock, flags);
			list_add(&data_buf->list, &dev->data_in_bufs_avail);
			spin_unlock_irqrestore(&dev->data_in_lock, flags);

            //dev->data_in_bufs[idx].valid = 1; // mark this RX packet as free


            if (atomic_read(&dev->data_in_inflight) < NUM_CONCURRENT)
			{
				MY_DBG_MSG(&dev->interface->dev, "RX666m read - __submit_rx_urb more\n");
                __submit_rx_urb(dev);
			}

			if(!ret && count-ptr < DATA_BUF_SZ)
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
            ret = wait_event_interruptible_timeout(dev->data_in_wait, atomic_read(&dev->data_in_ready), 60 * HZ);
            if (ret < 0)
			{
				dev_info(&dev->interface->dev, "RX666m read error - wait_event_interruptible_timeout\n");
                break;
            }
			else if (ret == 0)
			{
				dev_info(&dev->interface->dev, "RX666m read timeout\n");
				//dump_info(dev);
                ret = -ETIMEDOUT;
                break;
            }
			else
			{
                ret = 0;
            }
        }
    }

	MY_DBG_MSG(&dev->interface->dev, "RX666m read end\n");

	if(ret<0)
	{
		dev_info(&dev->interface->dev, "RX666m read error ret=%d\n", ret);
		//dump_info(dev);
	}

	dev->last_read_ret = ret;

    return ret;
}

static ssize_t rx666m_write(struct file *file, const char *user_buf, size_t count, loff_t *ppos)
{
    rx666m_device_t *dev;

    dev = (rx666m_device_t *)file->private_data;

    //dump_periodic(dev);

	return -EFAULT;
}

#if 0
int __rx666m_rcv_cmd(rx666m_device_t *dev, int cmd, void *ptr, __u16 len) {
    int tries = 3;
    int retval;

    do {
        retval = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
                cmd, BLADE_USB_TYPE_IN, 0, 0,
                ptr, len, RX666_TIMEOUT_MS);
        if (retval < 0) {
            dev_err(&dev->interface->dev, "Error in %s calling usb_control_msg()"
                    " with error %d, %d tries left\n", __func__, retval, tries);
        }
    } while ((retval < 0) && --tries);

    return retval;
}
#endif

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

	MY_DBG_MSG(&dev->interface->dev, "RX666m ioctl: %d \n", cmd);

	if (_IOC_TYPE(cmd) != MYDRBASE) return -EINVAL;

	MY_DBG_MSG(&dev->interface->dev, "RX666m ioctl OK: %d \n", cmd);

    switch (cmd) {
		case RX666M_IS_BOOTLOADER_RUNNING:
			{
				uint8_t *buf = (uint8_t*)kmalloc(MAX_WRITE_SIZE, GFP_KERNEL);
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
				uint8_t *buf = (uint8_t*)kmalloc(MAX_WRITE_SIZE, GFP_KERNEL);
				if(!buf)
					return -ENOMEM;

				index = 0;
				while(arg_wr_ram.len > 0)
				{
					size_t size = (arg_wr_ram.len > MAX_WRITE_SIZE) ? MAX_WRITE_SIZE : arg_wr_ram.len;

					MY_DBG_MSG(&dev->interface->dev, "RX666M_WRITE_RAM arg_wr_ram.address=%x size=%lx\n", arg_wr_ram.address, size);

					if (copy_from_user(buf, arg_wr_ram.firmware + index, size)) {
						retval=-EFAULT;
						break;
					}

					MY_DBG_MSG(&dev->interface->dev, "[0]=%x\n", *(uint32_t*)buf);
					ret = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
							0xA0, 0x40, (uint16_t)(arg_wr_ram.address & 0xffff), (uint16_t)(arg_wr_ram.address >> 16),
							buf, size, RX666_TIMEOUT_MS);
					if(ret != size)
					{
						dev_err(&dev->interface->dev, "RX666M_WRITE_RAM retval=%d\n", ret);
						retval=-EFAULT;
						break;
					}

					MY_DBG_MSG(&dev->interface->dev, "RX666M_WRITE_RAM retval=%d arg_wr_ram.address=%x size=%lx\n", retval, arg_wr_ram.address, size);

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

			buf = (uint8_t*)kmalloc(MAX_WRITE_SIZE, GFP_KERNEL);
			if(!buf)
				return -ENOMEM;

			if (copy_from_user(buf, data, sizeof(rx666m_ioctl_i2c_transfer_t)))
			{
				retval=-EFAULT;
			}

			if(!retval)
			{
				rx666m_ioctl_i2c_transfer_t *tr_data = (rx666m_ioctl_i2c_transfer_t*)buf;

				MY_DBG_MSG(&dev->interface->dev, "RX666M_I2C_READ address=0x%x reg=0x%x len=0x%x", (int)tr_data->address, (int)tr_data->reg, (int)tr_data->len);

				ret = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
						0xbe, 0xc0, tr_data->address, tr_data->reg,
						tr_data->data, tr_data->len, RX666_TIMEOUT_MS);
				if(ret != tr_data->len)
				{
					dev_err(&dev->interface->dev, "RX666M_I2C_READ retval=%d\n", ret);
					retval=-EFAULT;
				}
				else
				{
					MY_DBG_MSG(&dev->interface->dev, "RX666M_I2C_READ [0]=0x%x\n", (int)tr_data->data[0]);
					MY_DBG_MSG(&dev->interface->dev, "RX666M_I2C_READ [1]=0x%x\n", (int)tr_data->data[1]);
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
			MY_DBG_MSG(&dev->interface->dev, "RX666M_I2C_WRITE address\n");
			retval = 0;

			buf = (uint8_t*)kmalloc(MAX_WRITE_SIZE, GFP_KERNEL);
			if(!buf)
				return -ENOMEM;

			if (copy_from_user(buf, data, sizeof(rx666m_ioctl_i2c_transfer_t)))
			{
				retval=-EFAULT;
			}

			if(!retval)
			{
				rx666m_ioctl_i2c_transfer_t *tr_data = (rx666m_ioctl_i2c_transfer_t*)buf;

				MY_DBG_MSG(&dev->interface->dev, "RX666M_I2C_WRITE address=0x%x reg=0x%x len=0x%x", (int)tr_data->address, (int)tr_data->reg, (int)tr_data->len);

				ret = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
						0xba, 0x40, tr_data->address, tr_data->reg,
						tr_data->data, tr_data->len, RX666_TIMEOUT_MS);
				if(ret != tr_data->len)
				{
					dev_err(&dev->interface->dev, "RX666M_I2C_WRITE retval=%d\n", ret);
					retval=-EFAULT;
				}
				else
				{
					MY_DBG_MSG(&dev->interface->dev, "RX666M_I2C_WRITE [0]=0x%x\n", (int)tr_data->data[0]);
					MY_DBG_MSG(&dev->interface->dev, "RX666M_I2C_WRITE [1]=0x%x\n", (int)tr_data->data[1]);
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
			MY_DBG_MSG(&dev->interface->dev, "RX666M_GPIO_WRITE address\n");
			retval = 0;

			buf = (uint8_t*)kmalloc(MAX_WRITE_SIZE, GFP_KERNEL);
			if(!buf)
				return -ENOMEM;


			buf[0]=arg;
			ret = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
					0xbc, 0x40, 0, 0,
					buf, 1, RX666_TIMEOUT_MS);
			if(ret != 1)
			{
				printk(KERN_ERR "RX666M_GPIO_WRITE retval=%d\n", ret);
				retval=-EFAULT;
			}

			kfree(buf);

			break;

		case RX666M_START:
			MY_DBG_MSG(&dev->interface->dev, "RX666M_SART\n");
			retval = 0;

			buf = (uint8_t*)kmalloc(MAX_WRITE_SIZE, GFP_KERNEL);
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

	MY_DBG_MSG(&dev->interface->dev, "RX666m ioctl end: %d retval=%d\n", (int)cmd, (int)retval);

    return retval;
}

static int rx666m_open(struct inode *inode, struct file *file)
{
    rx666m_device_t *dev;
    struct usb_interface *interface;
    int subminor;

	printk("RX666m open\n");

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

	dev_info(&interface->dev, "RX666m opened\n");

    return 0;
}

static int rx666m_release(struct inode *inode, struct file *file)
{
    rx666m_device_t *dev;

	printk("RX666m release\n");

    dev = (rx666m_device_t *)file->private_data;

    if (dev->reader && dev->reader == file)
	{
        if (dev->rx_en)
		{
            disable_rx(dev);
        }
        dev->reader = NULL;
    }

	printk("RX666m released");

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
    .minor_base = USB_NUAND_BLADERF_MINOR_BASE,
};

static int rx666m_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    rx666m_device_t *dev;
	struct usb_device *udev;
    int retval;

	MY_DBG_MSG(&interface->dev, "RX666m probing...\n");

    if (interface->cur_altsetting->desc.bInterfaceNumber != 0)
	{
        return 0;
	}


	//test_wq = kmalloc(sizeof(*test_wq), GFP_KERNEL);


    udev = interface_to_usbdev(interface);


	dev_info(&interface->dev, "RX666m probing bNumConfigurations=%d\n",  (int)udev->descriptor.bNumConfigurations);
        /* A multi-config CPiA2 camera? */
    //    if (udev->descriptor.bNumConfigurations != 1)
    //            return -ENODEV;

//	if(r=usb_set_configuration(udev, 1))
//	{
//		MY_DBG_MSG(&interface->dev, "RX666m probing... usb_set_configuration r=%d\n", r);
//	}

    dev = kzalloc(sizeof(rx666m_device_t), GFP_KERNEL);
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
    dev->bytes = 0;
    dev->debug = 0;
	dev->read_cb_cnt = 0;
	dev->bad_status = 0;
	dev->overrun = 0;
    dev->disconnecting = 0;
	dev->monotonic_cnt = 0;
	dev->monotonic_cnt_last_read = 0;

    atomic_set(&dev->data_in_inflight, 0);

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


    MY_DBG_MSG(&interface->dev, "RX666m device is now attached2\n");
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

    MY_DBG_MSG(&interface->dev, "RX666m device has been disconnected\n");

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
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL");
