obj-m = lan884x.o
lan884x-objs := micrel.o

KVERSION = $(shell uname -r)
all:
	        make -C /lib/modules/$(KVERSION)/build M=$(PWD) modules
clean:
	        make -C /lib/modules/$(KVERSION)/build M=$(PWD) clean
