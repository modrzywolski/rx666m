obj-m = rx666m.o
MY_CFLAGS += -g 
ccflags-y += ${MY_CFLAGS}
CC += ${MY_CFLAGS}

KVERSION = $(shell uname -r)
all: ioctl
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean

ioctl: ioctl.o
	${CC} ${CFLAGS} -o $@ ioctl.c 

run: all
	-sudo rmmod rx666m
	-sudo insmod rx666m.ko
	-sudo ./ioctl /dev/rx666m* 0
	-sudo chmod a+rw /dev/rx666m*
	#-sudo ./ioctl /dev/rx666m* 1
	#sudo timeout 5 cat /dev/rx666* | pv > /dev/null
	#-sudo rmmod rx666m
	#-dmesg | tail -n20

