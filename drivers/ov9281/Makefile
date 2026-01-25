# SPDX-License-Identifier: GPL-2.0-only
# Makefile for OV9281 camera driver

obj-$(CONFIG_VIDEO_OV9281) += ov9281.o

# For out-of-tree build
ifneq ($(KERNELRELEASE),)
obj-m := ov9281.o
else
KDIR ?= /lib/modules/$(shell uname -r)/build

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

endif
