# SPDX-License-Identifier: GPL-2.0-only
# Makefile for the Chronos FPGA control driver

obj-$(CONFIG_CHRONOS_FPGA) += chronos_fpga.o

# For out-of-tree build
ifneq ($(KERNELRELEASE),)
obj-m := chronos_fpga.o
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
