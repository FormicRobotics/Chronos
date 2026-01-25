# Chronos User Guide

## Introduction

Chronos is a multi-camera MIPI synchronization system designed for synchronized capture from four OV9281 global shutter cameras on the NVIDIA Jetson Orin NX platform.

## Hardware Setup

### Components

1. **Chronos PCB** - Custom board with CrossLink-NX FPGA
2. **4x OV9281 Camera Modules** (KLT-Z6MF-OV9281 V3.0)
3. **FPC Cables** (10-15cm, 15-pin)
4. **Jetson Orin NX** development kit
5. **Power supply** (5V, 3A minimum)

### Connection Diagram

```
                    ┌─────────────────────────┐
                    │      Chronos PCB        │
                    │                         │
    Camera 0 ──────▶│ CAM0                    │
    Camera 1 ──────▶│ CAM1                    │
    Camera 2 ──────▶│ CAM2           CSI-OUT ─┼──▶ Jetson CSI
    Camera 3 ──────▶│ CAM3                    │
                    │                         │
         IMU ──────▶│ SPI (IMU)       I2C ───┼──▶ Jetson I2C
                    │                         │
                    │ PWR ───────────────────┼──▶ 5V Power
                    └─────────────────────────┘
```

### Installation Steps

1. **Mount cameras** in desired positions
2. **Connect FPC cables** to Chronos PCB (CAM0-CAM3)
3. **Connect CSI cable** from Chronos to Jetson (2-lane CSI port)
4. **Connect I2C cable** for FPGA/sensor configuration
5. **Connect SPI cable** to IMU (if using)
6. **Apply power** (5V)

## Software Installation

### Prerequisites

- JetPack 5.x or later
- CUDA Toolkit
- CMake 3.18+
- GCC/G++ toolchain

### Build Instructions

```bash
# Clone repository
git clone https://github.com/your-org/chronos.git
cd chronos

# Build kernel modules
cd drivers
make all
sudo make install

# Build application
cd ../app
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# Load modules
sudo modprobe ov9281
sudo modprobe chronos_csi
sudo modprobe bmi088_chronos
```

### Device Tree Installation

```bash
# Compile overlay
dtc -I dts -O dtb -o chronos-orin-nx.dtbo drivers/chronos-orin-nx.dts

# Install overlay
sudo cp chronos-orin-nx.dtbo /boot/

# Add to extlinux.conf
sudo nano /boot/extlinux/extlinux.conf
# Add: FDT /boot/chronos-orin-nx.dtbo

# Reboot
sudo reboot
```

## Running the Demo Application

### Basic Usage

```bash
# Run demo with default settings (30 fps)
chronos_demo

# Run at 120 fps
chronos_demo --rate 120

# Run with custom exposure
chronos_demo --rate 60 --exposure 2000

# Run headless (no display)
chronos_demo --headless --duration 60
```

### Command Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `-r, --rate <fps>` | Frame rate (1-120) | 30 |
| `-e, --exposure <us>` | Exposure time (µs) | 5000 |
| `-g, --gain <dB>` | Analog gain | 0.0 |
| `-d, --duration <sec>` | Run duration (0=infinite) | 0 |
| `-n, --no-imu` | Disable IMU | enabled |
| `-H, --headless` | No display | disabled |

### Expected Output

```
Chronos Demo - 4-Camera Synchronized Capture
=============================================
Frame Rate: 60 fps
Exposure: 5000 us
Gain: 0.0 dB

Press Ctrl+C to stop...

FPS: 60.0  Sync: 1234  
  IMU: Accel[  0.02,  -0.01,   9.81] m/s²  Gyro[  0.00,   0.00,   0.00] rad/s
  Frame Sync: Cam0:1234 Cam1:1234 Cam2:1234 Cam3:1234 [SYNC OK]
```

## Synchronization Testing

### Running Sync Test

```bash
# Run sync validation
sync_test -n 1000 -r 120

# Verbose output
sync_test -n 500 -r 60 -v
```

### Expected Results

```
=== SYNCHRONIZATION TEST RESULTS ===

Inter-Camera Skew (all 4 cameras):
  Samples:    1000
  Mean:       42.123 us
  Std Dev:    8.456 us
  Min:        28.100 us
  Max:        89.200 us
  99th %ile:  76.400 us
  RESULT:     PASS (requirement: <100 us)

Frame Interval (frame-to-frame timing):
  Expected:   8333.333 us
  Mean:       8334.125 us
  Std Dev:    12.345 us
  Min:        8310.000 us
  Max:        8360.000 us
  Error:      0.01%
  RESULT:     PASS (requirement: <1% error)

=== OVERALL: ALL TESTS PASSED ===
```

## Troubleshooting

### Common Issues

#### No Video Devices

**Symptom**: `/dev/video*` devices not created

**Solutions**:
1. Check device tree overlay is loaded
2. Verify CSI cable connection
3. Check kernel module is loaded: `lsmod | grep chronos`
4. Check dmesg for errors: `dmesg | grep chronos`

#### Sync Failures

**Symptom**: "SYNC FAIL" messages

**Solutions**:
1. Reduce frame rate
2. Check all camera connections
3. Verify FPGA trigger is enabled
4. Check for I2C communication errors

#### CUDA Errors

**Symptom**: `CHRONOS_ERROR_CUDA`

**Solutions**:
1. Ensure CUDA is properly installed
2. Check GPU memory availability: `nvidia-smi`
3. Reduce buffer count if memory limited

#### Low Frame Rate

**Symptom**: Actual FPS lower than configured

**Solutions**:
1. Reduce exposure time
2. Check for buffer overruns in stats
3. Ensure no other processes using CSI/VI

### Debug Commands

```bash
# Check loaded modules
lsmod | grep -E "ov9281|chronos|bmi088"

# View kernel messages
dmesg | tail -50

# Check video devices
v4l2-ctl --list-devices

# Query camera capabilities
v4l2-ctl -d /dev/video0 --all

# Check FPGA status (via I2C)
i2cget -y 2 0x3c 0x20  # Status register
i2cget -y 2 0x3c 0xf0  # Firmware version

# Monitor IMU
cat /sys/bus/iio/devices/iio:device0/in_accel_x_raw
```

## Performance Tuning

### Optimizing for Low Latency

1. Use callback mode instead of polling
2. Keep exposure time short
3. Use triple buffering (default)
4. Process on GPU immediately

### Optimizing for High Throughput

1. Use maximum frame rate (120 fps)
2. Ensure adequate GPU memory
3. Use batch CUDA processing
4. Monitor buffer overruns

### Power Optimization

1. Reduce frame rate when possible
2. Disable unused cameras
3. Use shorter exposure times

## Calibration

### Camera Synchronization Calibration

The FPGA supports per-camera trigger delay adjustment:

```bash
# Set camera 1 delay (in clock cycles)
i2cset -y 2 0x3c 0x11 0x10

# Verify
i2cget -y 2 0x3c 0x11
```

### IMU Alignment

The IMU mount matrix can be configured in the device tree to match physical orientation.

## Contact and Support

For technical support:
- Email: support@chronos-project.com
- GitHub Issues: https://github.com/your-org/chronos/issues
