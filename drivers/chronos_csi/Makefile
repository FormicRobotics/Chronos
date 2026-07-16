# SPDX-License-Identifier: GPL-2.0-only
# Makefile for the Chronos CSI bridge subdevice
#
# The bridge uses symbols exported by chronos_fpga.ko; build that module
# first so its Module.symvers is available (handled automatically by the
# top-level drivers/Makefile, or pass KBUILD_EXTRA_SYMBOLS manually).

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

KBUILD_EXTRA_SYMBOLS ?= $(PWD)/../chronos_fpga/Module.symvers

default:
	$(MAKE) -C $(KDIR) M=$(PWD) KBUILD_EXTRA_SYMBOLS=$(KBUILD_EXTRA_SYMBOLS) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

endif
