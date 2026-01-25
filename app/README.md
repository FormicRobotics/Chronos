# 📱 Chronos Applications

<p align="center">
  <img src="https://img.shields.io/badge/Language-C%2FC%2B%2B-00599C?style=flat-square" alt="Language">
  <img src="https://img.shields.io/badge/CUDA-11.4+-76B900?style=flat-square" alt="CUDA">
  <img src="https://img.shields.io/badge/Build-CMake-064F8C?style=flat-square" alt="Build">
</p>

## Overview

User-space capture library and demo applications for the Chronos multi-camera system.

## 📁 Contents

```
app/
├── src/
│   ├── chronos_capture.h       # Library API header
│   ├── chronos_capture.c       # Library implementation
│   ├── demo_app.cpp            # 4-camera demo with IMU
│   └── sync_test.cpp           # Synchronization validator
│
└── CMakeLists.txt              # Build configuration
```

## 🏗️ Building

### Prerequisites

- CMake 3.18+
- CUDA Toolkit 11.4+
- JetPack 5.x (for NvBuffer API)

### Build Steps

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Install

```bash
sudo make install
```

## 📚 Library API

### Quick Example

```c
#include <chronos_capture.h>

// Initialize
chronos_init();

// Configure
chronos_config_t config = {
    .frame_rate = 60,
    .exposure_us = 5000,
    .external_trigger = true,
};
chronos_configure(&config);

// Capture
chronos_start_capture();
while (running) {
    chronos_sync_frame_set_t frames;
    if (chronos_get_frame_set(&frames, 1000) == CHRONOS_OK) {
        // Access CUDA pointer (zero-copy)
        void *gpu;
        chronos_get_cuda_ptr(&frames.frames[0], &gpu);
        
        // Process with CUDA...
        
        chronos_release_frame_set(&frames);
    }
}
chronos_shutdown();
```

### Key Functions

| Function | Description |
|----------|-------------|
| `chronos_init()` | Initialize capture system |
| `chronos_configure()` | Set capture parameters |
| `chronos_start_capture()` | Begin synchronized capture |
| `chronos_get_frame_set()` | Get next frame set (blocking) |
| `chronos_get_cuda_ptr()` | Get GPU pointer (zero-copy) |
| `chronos_release_frame_set()` | Return frames to pool |
| `chronos_shutdown()` | Cleanup and shutdown |

## 🎮 Demo Application

### Usage

```bash
chronos_demo [options]

Options:
  -r, --rate <fps>      Frame rate (1-120, default: 30)
  -e, --exposure <us>   Exposure time (default: 5000)
  -g, --gain <dB>       Analog gain (default: 0.0)
  -d, --duration <sec>  Run duration (0=infinite)
  -n, --no-imu          Disable IMU display
  -H, --headless        No display output
  -h, --help            Show help
```

### Examples

```bash
# Basic 60fps capture
chronos_demo --rate 60

# High-speed capture
chronos_demo --rate 120 --exposure 2000

# Headless benchmark
chronos_demo --rate 120 --headless --duration 60
```

## 🧪 Sync Test

### Usage

```bash
sync_test [options]

Options:
  -n <count>    Number of frames to test (default: 1000)
  -r <fps>      Frame rate (default: 120)
  -v            Verbose output
  -h            Show help
```

### Example Output

```
=== SYNCHRONIZATION TEST RESULTS ===

Inter-Camera Skew (all 4 cameras):
  Mean:       42.1 us
  Max:        89.2 us
  99th %ile:  76.4 us
  RESULT:     PASS (requirement: <100 us)

Frame Interval:
  Expected:   8333.3 us
  Mean:       8334.1 us
  Error:      0.01%
  RESULT:     PASS (requirement: <1% error)

=== OVERALL: ALL TESTS PASSED ===
```

## ⚡ Performance Tips

1. **Use Zero-Copy**: Always use `chronos_get_cuda_ptr()` instead of copying
2. **Release Promptly**: Call `chronos_release_frame_set()` ASAP to avoid stalls
3. **Batch Processing**: Process all 4 frames together in CUDA
4. **Async Mode**: Use callbacks for lowest latency
5. **Triple Buffering**: Default setting prevents drops

## 📊 Benchmarks

| Metric | Value |
|--------|-------|
| Capture latency | < 5 ms |
| CUDA mapping | < 100 µs |
| Max throughput | 1.2 Gbps |
| CPU overhead | < 5% |
