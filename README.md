<p align="center">
  <img src="https://img.shields.io/badge/Platform-Jetson%20Orin%20NX-76B900?style=for-the-badge&logo=nvidia" alt="Platform">
  <img src="https://img.shields.io/badge/FPGA-CrossLink--NX-ED1C24?style=for-the-badge&logo=lattice" alt="FPGA">
  <img src="https://img.shields.io/badge/Cameras-4x%20OV9281-00599C?style=for-the-badge" alt="Cameras">
  <img src="https://img.shields.io/badge/Sync-<%3C100ns-00C853?style=for-the-badge" alt="Sync">
</p>

<h1 align="center">⏱️ Chronos</h1>

<p align="center">
  <strong>Multi-Camera MIPI Synchronization System</strong><br>
  <em>Sub-microsecond synchronized capture for computer vision</em>
</p>

<p align="center">
  <a href="#-features">Features</a> •
  <a href="#-architecture">Architecture</a> •
  <a href="#-quick-start">Quick Start</a> •
  <a href="#-documentation">Documentation</a> •
  <a href="#-hardware">Hardware</a>
</p>

---

## ✨ Features

<table>
<tr>
<td width="50%">

### 🎯 Precision Synchronization
- **< 100ns** inter-camera trigger skew
- Hardware-triggered global shutter capture
- PLL-based timing generation
- Per-camera delay calibration

</td>
<td width="50%">

### 🚀 Zero-Copy GPU Pipeline
- Direct DMA to CUDA memory
- NvBuffer + EGL interop
- Triple-buffered capture
- No CPU memcpy overhead

</td>
</tr>
<tr>
<td width="50%">

### 📷 4-Camera Aggregation
- Single CSI-2 port for all cameras
- FPGA virtual channel multiplexing
- 1280×800 @ 120fps per camera
- 10-bit RAW global shutter

</td>
<td width="50%">

### 🔄 Integrated IMU
- BMI088 6-axis IMU
- Synchronized with cameras
- SPI @ 10MHz
- Timestamp correlation

</td>
</tr>
</table>

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           CHRONOS SYSTEM                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐                       │
│   │  CAM 0  │ │  CAM 1  │ │  CAM 2  │ │  CAM 3  │     4× OV9281         │
│   │ OV9281  │ │ OV9281  │ │ OV9281  │ │ OV9281  │     Global Shutter    │
│   └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘                       │
│        │ CSI-2     │           │           │                             │
│        └───────────┴─────┬─────┴───────────┘                             │
│                          ▼                                               │
│   ┌──────────────────────────────────────────────┐                      │
│   │           CrossLink-NX FPGA                   │     Virtual Channel  │
│   │  ┌────────┐  ┌────────┐  ┌────────────────┐  │     Aggregation      │
│   │  │ 4× RX  │→ │ Buffer │→ │ TX + VC Tag    │  │                      │
│   │  └────────┘  └────────┘  └───────┬────────┘  │                      │
│   │  ┌─────────────────────────────┐ │           │                      │
│   │  │    Trigger Generator (PLL)  │─┼──────────────→ FSIN + IMU        │
│   │  └─────────────────────────────┘ │           │                      │
│   └──────────────────────────────────┼───────────┘                      │
│                                      │ Single 2-lane CSI-2              │
│                                      ▼                                   │
│   ┌──────────────────────────────────────────────────────────────┐      │
│   │                    Jetson Orin NX                             │      │
│   │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────────────┐  │      │
│   │  │  NVCSI  │→ │   VI    │→ │ Driver  │→ │   GPU Memory    │  │      │
│   │  │  D-PHY  │  │  DMA    │  │ Demux   │  │   (Zero-Copy)   │  │      │
│   │  └─────────┘  └─────────┘  └─────────┘  └─────────────────┘  │      │
│   └──────────────────────────────────────────────────────────────┘      │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 🚀 Quick Start

### Prerequisites

- NVIDIA Jetson Orin NX with JetPack 5.x
- Chronos PCB (CrossLink-NX FPGA board)
- 4× OV9281 camera modules
- CUDA Toolkit, CMake 3.18+

### Build & Install

```bash
# Clone the repository
git clone https://github.com/your-org/chronos.git
cd chronos

# Build kernel drivers
cd drivers
make all
sudo make install

# Build user-space applications
cd ../app
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### Run Demo

```bash
# Load drivers
sudo modprobe ov9281
sudo modprobe chronos_csi

# Run synchronized capture demo
chronos_demo --rate 60 --exposure 5000

# Run synchronization validation
sync_test -n 1000 -r 120
```

### Example Output

```
Chronos Demo - 4-Camera Synchronized Capture
=============================================
Frame Rate: 60 fps
Exposure: 5000 us

FPS: 60.0  Sync: 1234
  IMU: Accel[  0.02,  -0.01,   9.81] m/s²  Gyro[  0.00,   0.00,   0.00] rad/s
  Frame Sync: Cam0:1234 Cam1:1234 Cam2:1234 Cam3:1234 [SYNC OK]
```

---

## 📁 Project Structure

```
chronos/
├── 📂 fpga/                    # FPGA Firmware
│   ├── 📂 rtl/                 # SystemVerilog sources
│   │   ├── chronos_top.sv      # Top-level module
│   │   ├── csi2_rx.sv          # CSI-2 receivers (×4)
│   │   ├── csi2_tx.sv          # CSI-2 transmitter
│   │   ├── frame_buffer.sv     # SRAM line buffers
│   │   ├── tx_arbiter.sv       # VC multiplexer
│   │   ├── trigger_generator.sv # Sync pulse generator
│   │   └── config_regs.sv      # I2C register bank
│   └── 📂 constraints/         # Timing/pin constraints
│
├── 📂 drivers/                 # Linux Kernel Modules
│   ├── 📂 ov9281/              # OV9281 V4L2 driver
│   ├── 📂 chronos_csi/         # CSI demux driver
│   ├── 📂 imu/                 # BMI088 IIO driver
│   └── chronos-orin-nx.dts     # Device tree overlay
│
├── 📂 app/                     # User-Space Applications
│   └── 📂 src/
│       ├── chronos_capture.h   # Capture library API
│       ├── chronos_capture.c   # Library implementation
│       ├── demo_app.cpp        # Demo application
│       └── sync_test.cpp       # Sync validation tool
│
└── 📂 docs/                    # Documentation
    ├── architecture.md         # System design
    ├── api_reference.md        # API documentation
    └── user_guide.md           # User manual
```

---

## 📖 Documentation

| Document | Description |
|----------|-------------|
| [📐 Architecture](docs/architecture.md) | System design, block diagrams, data flow |
| [📚 API Reference](docs/api_reference.md) | Complete C API documentation |
| [📘 User Guide](docs/user_guide.md) | Installation, usage, troubleshooting |
| [🤝 Contributing](CONTRIBUTING.md) | Development guidelines |

---

## 🔧 Hardware Specifications

| Component | Specification |
|-----------|---------------|
| **FPGA** | Lattice CrossLink-NX (LIFCL-40) |
| **Cameras** | 4× OV9281 (1280×800, global shutter) |
| **Interface** | MIPI CSI-2, 2-lane @ 800 Mbps/lane |
| **Output** | Single 2-lane CSI-2 (4 virtual channels) |
| **Sync Accuracy** | < 100 ns trigger skew |
| **Frame Rate** | 1–120 fps (configurable) |
| **IMU** | BMI088 / ICM-42688-P (6-axis) |
| **Power** | ~2.5W total |
| **Temperature** | -10°C to +55°C (commercial) |

---

## 📊 Performance

<table>
<tr>
<th>Metric</th>
<th>Value</th>
<th>Notes</th>
</tr>
<tr>
<td>Inter-camera sync</td>
<td><strong>< 100 ns</strong></td>
<td>99th percentile</td>
</tr>
<tr>
<td>Capture-to-GPU latency</td>
<td><strong>< 5 ms</strong></td>
<td>Zero-copy path</td>
</tr>
<tr>
<td>Max frame rate</td>
<td><strong>120 fps</strong></td>
<td>All 4 cameras</td>
</tr>
<tr>
<td>Aggregate throughput</td>
<td><strong>1.2 Gbps</strong></td>
<td>4× RAW10 streams</td>
</tr>
<tr>
<td>Frame drop rate</td>
<td><strong>< 0.01%</strong></td>
<td>Triple-buffered</td>
</tr>
</table>

---

## 🛠️ API Example

```c
#include <chronos_capture.h>

int main() {
    // Initialize
    chronos_init();
    
    // Configure synchronized capture
    chronos_config_t config = {
        .frame_rate = 60,
        .exposure_us = 5000,
        .external_trigger = true,
        .enable_imu = true,
    };
    chronos_configure(&config);
    
    // Capture loop
    chronos_start_capture();
    while (running) {
        chronos_sync_frame_set_t frames;
        
        if (chronos_get_frame_set(&frames, 1000) == CHRONOS_OK) {
            // Get CUDA pointer (zero-copy!)
            void *gpu_ptr;
            chronos_get_cuda_ptr(&frames.frames[0], &gpu_ptr);
            
            // Run your CUDA kernels here...
            
            chronos_release_frame_set(&frames);
        }
    }
    
    chronos_shutdown();
    return 0;
}
```

---

## 📜 License

| Component | License |
|-----------|---------|
| FPGA Firmware | Proprietary |
| Linux Drivers | GPL v2 |
| User Applications | MIT |

---

## 👥 Authors

**Umit Kayacik** — *Senior Electronics Engineer*

---

<p align="center">
  <em>Built with ❤️ for computer vision applications</em>
</p>
