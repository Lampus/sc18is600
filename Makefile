ifneq ($(KERNELRELEASE),)
obj-m := i2c-sc18is600.o
else
#KDIR := /usr/src/linux-headers-`uname -r`
#ARCH := ARM
#CROSS_COMPILE := /opt/arm-2010q1/bin/arm-none-linux-gnueabi-
KDIR := /media/stuff/StarterKit/new_src/linux-2.6.39.2_st3
all:
	$(MAKE) -C $(KDIR) M=`pwd` modules
endif
