obj-m = rx666m.o
#EXTRA_CFLAGS = -Wall -g -I$(PWD)/../common/
ccflags-y += -g -I$(PWD)/../common/

KVERSION = $(shell uname -r)
all:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean


