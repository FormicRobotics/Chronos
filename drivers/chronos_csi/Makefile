# SPDX-License-Identifier: GPL-2.0-only
# Makefile for Chronos CSI driver

obj-$(CONFIG_VIDEO_CHRONOS_CSI) += chronos_csi.o

# For out-of-tree build
ifneq ($(KERNELRELEASE),)
obj-m := chronos_csi.o
else
KDIR ?= /lib/modules/$(shell uname -r)/build

# For Jetson, use L4T kernel source
ifdef TEGRA_KERNEL_OUT
KDIR := $(TEGRA_KERNEL_OUT)
endif

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

endif
