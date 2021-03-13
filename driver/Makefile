obj-m = rx666m.o
MY_CFLAGS += -g 
ccflags-y += ${MY_CFLAGS}
CC += ${MY_CFLAGS}

KVERSION = $(shell uname -r)
all:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean


