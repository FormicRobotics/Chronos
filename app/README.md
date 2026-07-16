# 📱 Chronos Applications

<p align="center">
  <img src="https://img.shields.io/badge/Language-C%2FC%2B%2B-00599C?style=flat-square" alt="Language">
  <img src="https://img.shields.io/badge/CUDA-optional-76B900?style=flat-square" alt="CUDA">
  <img src="https://img.shields.io/badge/Build-CMake-064F8C?style=flat-square" alt="Build">
</p>

## Overview

User-space capture library and demo applications for the Chronos multi-camera system.

Capture uses plain V4L2 MMAP buffers: every delivered frame exposes a CPU
pointer directly into the driver's buffer, and buffers are only re-queued when
the consumer releases the frame set (so held frames are never overwritten).
An optional CUDA path copies frames into device memory on demand
(`chronos_get_cuda_ptr()`); there is no EGL/NvBuffer dependency.

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
- JetPack 5.x or 6.x (V4L2 camera stack)
- CUDA Toolkit (optional — enables the GPU copy path and the demo app)

### Build Steps

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

CUDA is controlled by the `CHRONOS_WITH_CUDA` option (default `ON`; it is
disabled automatically when no CUDA compiler is found). Without CUDA only
the library and `sync_test` are built — `chronos_demo` needs CUDA for its
processing kernels.

```bash
cmake -DCHRONOS_WITH_CUDA=OFF ..
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
    .frame_rate = 30,
    .exposure_us = 5000,
    .external_trigger = true,
};
chronos_configure(&config);

// Capture
chronos_start_capture();
while (running) {
    chronos_sync_frame_set_t frames;
    if (chronos_get_frame_set(&frames, 1000) == CHRONOS_OK) {
        // CPU access (mmap'd V4L2 buffer, valid until release)
        uint16_t *pixels = frames.frames[0].data;

        // Optional GPU access (copies the frame to device memory)
        void *gpu;
        chronos_get_cuda_ptr(&frames.frames[0], &gpu);

        // Process...

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
| `chronos_start_capture()` | Begin synchronized capture (enables FSIN trigger) |
| `chronos_get_frame_set()` | Get next frame set (blocking) |
| `chronos_get_cuda_ptr()` | Copy frame to GPU, get device pointer |
| `chronos_release_frame_set()` | Re-queue buffers to the drivers |
| `chronos_get_stats()` | Capture statistics + FPGA STATUS/ERROR registers |
| `chronos_shutdown()` | Cleanup and shutdown |

### Notes

- Frame data (`frames[i].data`) points into the driver's MMAP buffer; honor
  `frames[i].pitch` (bytesperline) when walking rows. The CUDA copy is
  tightly packed (pitch = width × 2 bytes).
- `buffer_count` is fixed at `chronos_init()`; passing a different value to
  `chronos_configure()` afterwards returns an error (0 keeps the current value).
- Exposure is programmed in the sensors by the FPGA; the FSIN pulse width can
  be overridden via `config.fsin_pulse_width_cycles` (0 = default 2000 cycles
  of the 192 MHz FPGA clock, ~10.4 µs).
- If the consumer never releases a frame set, capture stalls gracefully and
  drops are counted in the statistics — held frames are never corrupted.

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

When built with OpenCV and not running headless, the demo shows a live 2×2
mosaic of all four cameras.

### Examples

```bash
# Basic 30fps capture (design point)
chronos_demo --rate 30

# Headless benchmark
chronos_demo --rate 30 --headless --duration 60
```

## 🧪 Sync Test

### Usage

```bash
sync_test [options]

Options:
  -n <count>    Number of frames to test (default: 1000)
  -r <fps>      Frame rate (default: 30; 120 = stress mode)
  -v            Verbose output
  -h            Show help
```

The default 30 fps matches the system design point (4 cameras aggregated
onto a 2-lane CSI-2 link). `-r 120` is a stress mode that exceeds the
aggregate TX bandwidth with all four cameras active — expect drops.

### Example Output

```
=== SYNCHRONIZATION TEST RESULTS ===

Inter-Camera Skew (all 4 cameras):
  Mean:       42.1 us
  Max:        89.2 us
  99th %ile:  76.4 us
  RESULT:     PASS (requirement: <100 us)

Frame Interval:
  Expected:   33333.3 us
  Mean:       33334.1 us
  Error:      0.01%
  RESULT:     PASS (requirement: <1% error)

=== OVERALL: ALL TESTS PASSED ===
```

## ⚡ Usage Tips

1. **Release Promptly**: Call `chronos_release_frame_set()` ASAP — buffers are
   not re-queued until then, so holding sets stalls capture
2. **CPU First**: The CPU pointer is free; only call `chronos_get_cuda_ptr()`
   when you actually process on the GPU (it performs a host-to-device copy)
3. **Batch Processing**: Process all 4 frames together in CUDA
4. **Async Mode**: Use callbacks for lowest latency
5. **Triple Buffering**: Default setting prevents drops
