# 🐧 Chronos Linux Drivers

<p align="center">
  <img src="https://img.shields.io/badge/Kernel-5.10+-yellow?style=flat-square" alt="Kernel">
  <img src="https://img.shields.io/badge/Platform-Jetson%20Orin%20NX-76B900?style=flat-square" alt="Platform">
  <img src="https://img.shields.io/badge/License-GPL%20v2-blue?style=flat-square" alt="License">
</p>

## Overview

Linux kernel drivers for the Chronos multi-camera synchronization system on NVIDIA Jetson platforms.

## 📁 Directory Structure

```
drivers/
├── ov9281/                     # OV9281 Camera Sensor Driver
│   ├── ov9281.c                # V4L2 subdevice driver
│   ├── Makefile
│   └── Kconfig
│
├── chronos_csi/                # Chronos CSI Platform Driver
│   ├── chronos_csi.c           # VC demux + video devices
│   └── Makefile
│
├── imu/                        # BMI088 IMU Driver
│   ├── bmi088_chronos.c        # IIO driver with sync
│   └── Makefile
│
└── chronos-orin-nx.dts         # Device Tree Overlay
```

## 🔌 Driver Overview

### OV9281 Sensor Driver

V4L2 subdevice driver for the OmniVision OV9281 global shutter sensor.

**Features:**
- I2C register access
- Exposure & gain control
- External trigger mode (FSIN)
- Power management
- 1280×800 @ 120fps

**Device Path:** `/dev/v4l-subdev*`

### Chronos CSI Driver

Platform driver for multi-camera management and virtual channel demultiplexing.

**Features:**
- Creates `/dev/video0-3` for each camera
- DMA-BUF buffer management
- FPGA communication via I2C
- Frame rate control

**Device Paths:** `/dev/video0`, `/dev/video1`, `/dev/video2`, `/dev/video3`

### BMI088 IMU Driver

IIO subsystem driver for synchronized IMU capture.

**Features:**
- SPI communication @ 10MHz
- Interrupt-driven sampling
- Synchronized trigger
- Accelerometer + gyroscope

**Sysfs Path:** `/sys/bus/iio/devices/iio:device*`

## 🛠️ Building

### Prerequisites

```bash
# Install kernel headers (on Jetson)
sudo apt install nvidia-l4t-kernel-headers

# Or set up cross-compilation environment
export CROSS_COMPILE=aarch64-linux-gnu-
export ARCH=arm64
export KDIR=/path/to/jetson/kernel/source
```

### Build All Drivers

```bash
cd drivers
make all
```

### Install

```bash
sudo make install
sudo depmod -a
```

## 📦 Loading Drivers

### Manual Loading

```bash
# Load in correct order
sudo modprobe ov9281
sudo modprobe chronos_csi
sudo modprobe bmi088_chronos

# Verify
lsmod | grep -E "ov9281|chronos|bmi088"
```

### Auto-load at Boot

```bash
# Add to /etc/modules-load.d/chronos.conf
echo "ov9281" | sudo tee /etc/modules-load.d/chronos.conf
echo "chronos_csi" | sudo tee -a /etc/modules-load.d/chronos.conf
echo "bmi088_chronos" | sudo tee -a /etc/modules-load.d/chronos.conf
```

## 🌳 Device Tree

### Compile Overlay

```bash
dtc -I dts -O dtb -o chronos-orin-nx.dtbo chronos-orin-nx.dts
```

### Install Overlay

```bash
sudo cp chronos-orin-nx.dtbo /boot/

# Add to /boot/extlinux/extlinux.conf:
# FDT /boot/chronos-orin-nx.dtbo
```

## 🔍 Debugging

### Check Driver Status

```bash
# Kernel messages
dmesg | grep -E "ov9281|chronos|bmi088"

# Video devices
v4l2-ctl --list-devices

# Camera capabilities
v4l2-ctl -d /dev/video0 --all

# FPGA communication
i2cget -y 2 0x3c 0xf0  # Read version
i2cget -y 2 0x3c 0x20  # Read status
```

### Common Issues

| Issue | Solution |
|-------|----------|
| No `/dev/video*` | Check DTS overlay, verify CSI connection |
| I2C errors | Check I2C bus number, verify wiring |
| No frames | Enable trigger: `i2cset -y 2 0x3c 0x00 0x01` |
| IMU not found | Check SPI bus, verify chip select |

## 📋 Module Parameters

### ov9281

| Parameter | Default | Description |
|-----------|---------|-------------|
| `debug` | 0 | Enable debug messages |

### chronos_csi

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_cameras` | 4 | Number of cameras |
| `fpga_addr` | 0x3c | FPGA I2C address |

## 📜 License

All drivers are licensed under GPL v2.
