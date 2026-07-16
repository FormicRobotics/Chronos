# SPDX-License-Identifier: GPL-2.0-only
# Makefile for the OV9281 camera driver.
#
# NOT part of the Chronos build: on HW_CHRONOS_R1 the sensors are on
# FPGA-private SCCB buses and configured by the FPGA; this module must not
# be loaded there.  It only builds when CONFIG_VIDEO_OV9281 is explicitly
# set (in-tree) or when this directory is built directly (out-of-tree),
# and is deliberately excluded from the top-level drivers/Makefile.

obj-$(CONFIG_VIDEO_OV9281) += ov9281.o

# For out-of-tree build (only when invoked in this directory on purpose)
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
