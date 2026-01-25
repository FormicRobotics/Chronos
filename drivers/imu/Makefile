# SPDX-License-Identifier: GPL-2.0-only
# Makefile for BMI088 IMU driver

obj-$(CONFIG_IIO_BMI088_CHRONOS) += bmi088_chronos.o

# For out-of-tree build
ifneq ($(KERNELRELEASE),)
obj-m := bmi088_chronos.o
else
KDIR ?= /lib/modules/$(shell uname -r)/build

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

endif
