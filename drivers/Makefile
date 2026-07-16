# SPDX-License-Identifier: GPL-2.0-only
# Top-level Makefile for the Chronos Jetson-side kernel modules.
#
# Builds only the modules that are actually used on HW_CHRONOS_R1:
#   chronos_fpga - FPGA control interface (I2C 0x3C)
#   chronos_csi  - aggregated CSI-2 bridge subdevice (needs chronos_fpga
#                  symbols, so it is built second with its Module.symvers)
#
# ov9281/ and imu/ are deliberately NOT built: the sensors and the IMU are
# FPGA-attached and unreachable from the Jetson (see the READMEs and the
# top-of-file comments in those directories).
#
# Usage:
#   make KDIR=/path/to/jetson/kernel/build [CROSS_COMPILE=aarch64-linux-gnu- ARCH=arm64]

KDIR ?= /lib/modules/$(shell uname -r)/build

# For Jetson, use L4T kernel source
ifdef TEGRA_KERNEL_OUT
KDIR := $(TEGRA_KERNEL_OUT)
endif

all: chronos_fpga chronos_csi

chronos_fpga:
	$(MAKE) -C $(KDIR) M=$(CURDIR)/chronos_fpga modules

chronos_csi: chronos_fpga
	$(MAKE) -C $(KDIR) M=$(CURDIR)/chronos_csi \
		KBUILD_EXTRA_SYMBOLS=$(CURDIR)/chronos_fpga/Module.symvers modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR)/chronos_fpga clean
	$(MAKE) -C $(KDIR) M=$(CURDIR)/chronos_csi clean

install:
	$(MAKE) -C $(KDIR) M=$(CURDIR)/chronos_fpga modules_install
	$(MAKE) -C $(KDIR) M=$(CURDIR)/chronos_csi modules_install

.PHONY: all chronos_fpga chronos_csi clean install
