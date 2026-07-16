# SPDX-License-Identifier: GPL-2.0-only
# Makefile for the ICM-42688-P IMU driver.
#
# NOT part of the Chronos R1 build: the IMU is wired to the FPGA only and
# the Jetson cannot reach it (see README.md).  Deliberately excluded from
# the top-level drivers/Makefile; kept for a future hardware revision.

obj-$(CONFIG_IIO_ICM42688_CHRONOS) += icm42688_chronos.o

# Out-of-tree build for Jetson kernels.
ifneq ($(KERNELRELEASE),)
obj-m := icm42688_chronos.o
else
KDIR ?= /lib/modules/$(shell uname -r)/build

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

endif
