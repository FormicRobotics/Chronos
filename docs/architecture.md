# Chronos System Architecture

## Overview

Chronos is a multi-camera MIPI synchronization system designed for the NVIDIA Jetson Orin NX platform. It synchronizes four OV9281 global shutter cameras and an IMU, delivering synchronized frames directly to GPU memory for computer vision applications.

## System Block Diagram

```
┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│  OV9281 #0  │  │  OV9281 #1  │  │  OV9281 #2  │  │  OV9281 #3  │
│  (Camera)   │  │  (Camera)   │  │  (Camera)   │  │  (Camera)   │
└──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘
       │ CSI-2          │ CSI-2          │ CSI-2          │ CSI-2
       │ 2-lane         │ 2-lane         │ 2-lane         │ 2-lane
       ▼                ▼                ▼                ▼
┌──────────────────────────────────────────────────────────────────┐
│                    CrossLink-NX FPGA                              │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐          │
│  │ CSI-2 RX │  │ CSI-2 RX │  │ CSI-2 RX │  │ CSI-2 RX │          │
│  │   VC0    │  │   VC1    │  │   VC2    │  │   VC3    │          │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘          │
│       │             │             │             │                 │
│       └─────────────┴──────┬──────┴─────────────┘                 │
│                            │                                       │
│                    ┌───────▼───────┐      ┌─────────────────┐     │
│                    │ Frame Buffer  │      │ Trigger Gen     │     │
│                    │   (SRAM)      │      │ (PLL-based)     │     │
│                    └───────┬───────┘      └────────┬────────┘     │
│                            │                       │               │
│                    ┌───────▼───────┐               │               │
│                    │  TX Arbiter   │◄──────────────┘               │
│                    │ (VC Tagging)  │                               │
│                    └───────┬───────┘                               │
│                            │                                       │
│                    ┌───────▼───────┐                               │
│                    │   CSI-2 TX    │                               │
│                    │   (2-lane)    │                               │
│                    └───────┬───────┘                               │
└────────────────────────────┼───────────────────────────────────────┘
                             │ CSI-2 2-lane
                             │ (4 Virtual Channels)
                             ▼
┌──────────────────────────────────────────────────────────────────┐
│                     Jetson Orin NX                                │
│                                                                   │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────┐  │
│  │   NVCSI     │───▶│ VI (Video   │───▶│ Chronos CSI Driver  │  │
│  │  (D-PHY)    │    │   Input)    │    │ (VC Demux)          │  │
│  └─────────────┘    └─────────────┘    └─────────┬───────────┘  │
│                                                   │              │
│        ┌──────────────────────────────────────────┼───────────┐  │
│        │                                          ▼           │  │
│        │  ┌───────────┐  ┌───────────┐  ┌───────────────────┐│  │
│        │  │/dev/video0│  │/dev/video1│  │   NvBuffer Pool   ││  │
│        │  │/dev/video1│  │/dev/video2│  │   (DMA-capable)   ││  │
│        │  │/dev/video2│  │/dev/video3│  └─────────┬─────────┘│  │
│        │  │/dev/video3│  │           │            │          │  │
│        │  └───────────┘  └───────────┘            ▼          │  │
│        │                               ┌───────────────────┐ │  │
│        │                               │ CUDA-EGL Interop  │ │  │
│        │                               │ (Zero-Copy)       │ │  │
│        │                               └─────────┬─────────┘ │  │
│        │                                         ▼           │  │
│        │                               ┌───────────────────┐ │  │
│        │                               │    GPU Memory     │ │  │
│        │                               │ (CUDA Processing) │ │  │
│        │                               └───────────────────┘ │  │
│        └─────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘

Trigger Signal Distribution:
┌─────────────────────────────────────────────────────────────────┐
│                           FPGA                                   │
│  ┌──────────────┐                                               │
│  │   PLL-based  │──────┬──────┬──────┬──────┬──────────────────▶│ FSIN
│  │   Trigger    │      │      │      │      │                    │ to Cam0
│  │   Generator  │      │      │      │      │                    │
│  └──────────────┘      │      │      │      └──────────────────▶│ FSIN
│                        │      │      │                           │ to Cam1
│                        │      │      └─────────────────────────▶│ FSIN
│                        │      │                                  │ to Cam2
│                        │      └────────────────────────────────▶│ FSIN
│                        │                                         │ to Cam3
│                        └───────────────────────────────────────▶│ INT
│                                                                  │ to IMU
└──────────────────────────────────────────────────────────────────┘
```

## Component Details

### 1. FPGA Firmware (Lattice CrossLink-NX)

**Purpose**: Aggregate 4 camera streams into a single 2-lane MIPI CSI-2 output using virtual channel multiplexing.

**Key Modules**:
- `csi2_rx.sv` - 4x CSI-2 receivers with D-PHY interface
- `frame_buffer.sv` - SRAM-based line buffers for stream aggregation
- `tx_arbiter.sv` - Round-robin arbitration with VC tagging
- `csi2_tx.sv` - CSI-2 transmitter with packet reconstruction
- `trigger_generator.sv` - PLL-based precision trigger (<100ns skew)
- `config_regs.sv` - I2C-accessible configuration registers

**Data Flow**:
1. Each camera sends 2-lane MIPI CSI-2 to its dedicated RX
2. Packets are parsed and stored in line buffers
3. TX arbiter reads from buffers in round-robin order
4. Virtual channel tag is added (Camera N → VC N)
5. Single 2-lane output stream sent to Jetson

### 2. Linux Kernel Drivers

**OV9281 Driver** (`drivers/ov9281/ov9281.c`):
- V4L2 subdevice driver for sensor control
- Exposure and gain adjustment via I2C
- External trigger mode (FSIN) support
- Power management

**Chronos CSI Driver** (`drivers/chronos_csi/chronos_csi.c`):
- Platform driver for multi-camera management
- Virtual channel demultiplexing
- Creates `/dev/video0-3` for each camera
- DMA-BUF buffer management

**BMI088 IMU Driver** (`drivers/imu/bmi088_chronos.c`):
- IIO subsystem driver
- SPI communication at 10MHz
- Synchronized interrupt handling
- Timestamp correlation with camera frames

### 3. Zero-Copy GPU Memory Pipeline

**NvBuffer Integration**:
- Hardware-accelerated buffer management
- DMA-capable memory allocation
- Direct path from VI to GPU memory

**CUDA-EGL Interop**:
- EGLImage creation from DMA-BUF
- CUDA external memory import
- Zero CPU copy path

**Triple Buffering**:
- Prevents frame drops under load
- Producer-consumer synchronization
- Automatic buffer recycling

## Synchronization Architecture

### Trigger Distribution
- FPGA PLL generates master clock
- Configurable frame rate (1-120 fps)
- Sub-100ns skew between all outputs
- Simultaneous trigger to all cameras and IMU

### Timestamp Correlation
1. FPGA sends trigger pulse
2. Cameras capture on rising edge (FSIN)
3. IMU samples on same edge (interrupt)
4. All devices share same timestamp reference
5. Software verifies sync by comparing timestamps

## Memory Map

### FPGA Register Map (I2C Address 0x3C)

| Address | Name | Description |
|---------|------|-------------|
| 0x00 | CTRL | Control register (enable, reset) |
| 0x01 | FRAME_RATE | Target frame rate (1-120) |
| 0x02-03 | PULSE_WIDTH | Trigger pulse width |
| 0x04 | CAM_ENABLE | Camera enable mask |
| 0x10-13 | TRIG_DELAY | Per-camera trigger delay |
| 0x20 | STATUS | PLL lock, sync status |
| 0x21 | ERROR | Error flags |
| 0x30-37 | FRAME_CNT | Per-camera frame counters |
| 0xF0 | VERSION | Firmware version |
| 0xFE-FF | DEVICE_ID | Device identification |

## Performance Specifications

| Parameter | Specification |
|-----------|--------------|
| Frame Rate | Up to 120 fps |
| Resolution | 1280 x 800 per camera |
| Bit Depth | 10-bit (RAW10) |
| Sync Accuracy | < 100 ns inter-camera skew |
| Latency | < 5 ms capture-to-GPU |
| Throughput | ~1.2 Gbps aggregate |
| Power | ~2.5W total board power |
