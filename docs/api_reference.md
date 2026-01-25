# Chronos API Reference

## Overview

The Chronos Capture Library provides a C API for synchronized multi-camera capture with zero-copy GPU memory access.

## Header File

```c
#include <chronos_capture.h>
```

## Initialization and Configuration

### chronos_init

Initialize the Chronos capture system.

```c
chronos_error_t chronos_init(void);
```

**Returns**: `CHRONOS_OK` on success, error code otherwise.

**Notes**:
- Must be called before any other Chronos functions
- Initializes all 4 cameras, IMU, and CUDA context
- Allocates NvBuffer pools for zero-copy operation

---

### chronos_shutdown

Shutdown the Chronos capture system.

```c
void chronos_shutdown(void);
```

**Notes**:
- Stops any active capture
- Releases all buffers and resources
- Safe to call even if not initialized

---

### chronos_configure

Configure capture parameters.

```c
chronos_error_t chronos_configure(const chronos_config_t *config);
```

**Parameters**:
- `config` - Pointer to configuration structure

**Returns**: `CHRONOS_OK` on success, error code otherwise.

**Configuration Structure**:
```c
typedef struct {
    uint32_t frame_rate;        // Target frame rate (1-120)
    uint32_t exposure_us;       // Exposure time in microseconds
    float gain_db;              // Analog gain in dB
    bool external_trigger;      // Use external trigger mode
    bool enable_imu;            // Enable IMU capture
    uint32_t buffer_count;      // Number of buffers (default 3)
} chronos_config_t;
```

---

### chronos_get_config

Get current configuration.

```c
chronos_error_t chronos_get_config(chronos_config_t *config);
```

## Capture Control

### chronos_start_capture

Start capturing frames.

```c
chronos_error_t chronos_start_capture(void);
```

**Returns**: `CHRONOS_OK` on success, error code otherwise.

**Notes**:
- Enables FPGA trigger
- Starts streaming on all cameras
- Launches capture thread

---

### chronos_stop_capture

Stop capturing frames.

```c
chronos_error_t chronos_stop_capture(void);
```

---

### chronos_is_capturing

Check if capture is running.

```c
bool chronos_is_capturing(void);
```

## Frame Acquisition

### chronos_get_frame_set

Get next synchronized frame set (blocking).

```c
chronos_error_t chronos_get_frame_set(
    chronos_sync_frame_set_t *frame_set,
    int timeout_ms
);
```

**Parameters**:
- `frame_set` - Pointer to frame set structure to fill
- `timeout_ms` - Timeout in milliseconds (-1 for infinite)

**Returns**: `CHRONOS_OK` on success, `CHRONOS_ERROR_TIMEOUT` on timeout.

**Frame Set Structure**:
```c
typedef struct {
    chronos_frame_t frames[CHRONOS_NUM_CAMERAS];
    chronos_imu_data_t imu;
    uint64_t sync_timestamp_ns;
    uint32_t sync_sequence;
    bool complete;              // All frames received
} chronos_sync_frame_set_t;
```

---

### chronos_release_frame_set

Release a frame set back to the buffer pool.

```c
chronos_error_t chronos_release_frame_set(
    chronos_sync_frame_set_t *frame_set
);
```

**Notes**:
- Must be called after processing each frame set
- Returns buffers to the pool for reuse

---

### chronos_set_frame_callback

Register callback for asynchronous frame delivery.

```c
chronos_error_t chronos_set_frame_callback(
    chronos_frame_callback_t callback,
    void *user_data
);
```

**Callback Type**:
```c
typedef void (*chronos_frame_callback_t)(
    chronos_sync_frame_set_t *frame_set,
    void *user_data
);
```

## CUDA Integration

### chronos_get_cuda_ptr

Get CUDA pointer for a frame (zero-copy).

```c
chronos_error_t chronos_get_cuda_ptr(
    chronos_frame_t *frame,
    void **cuda_ptr
);
```

**Parameters**:
- `frame` - Frame to get CUDA pointer for
- `cuda_ptr` - Pointer to store CUDA device pointer

**Notes**:
- Maps the frame to CUDA if not already mapped
- Returns device pointer for use in CUDA kernels
- Zero-copy - no CPU memcpy involved

---

### chronos_cuda_map / chronos_cuda_unmap

Explicit CUDA mapping control.

```c
chronos_error_t chronos_cuda_map(chronos_frame_t *frame);
chronos_error_t chronos_cuda_unmap(chronos_frame_t *frame);
```

## IMU Access

### chronos_get_imu_data

Get latest IMU data.

```c
chronos_error_t chronos_get_imu_data(chronos_imu_data_t *imu_data);
```

**IMU Data Structure**:
```c
typedef struct {
    uint64_t timestamp_ns;      // Synchronized timestamp
    float accel[3];             // Accelerometer (m/s^2)
    float gyro[3];              // Gyroscope (rad/s)
    float temp;                 // Temperature (Celsius)
    uint64_t sync_count;        // Sync pulse count
} chronos_imu_data_t;
```

## Statistics

### chronos_get_stats

Get capture statistics.

```c
chronos_error_t chronos_get_stats(chronos_stats_t *stats);
```

**Statistics Structure**:
```c
typedef struct {
    uint64_t frames_captured[CHRONOS_NUM_CAMERAS];
    uint64_t frames_dropped[CHRONOS_NUM_CAMERAS];
    uint64_t sync_errors;
    uint64_t buffer_overruns;
    float avg_latency_us;
    float max_sync_skew_us;
    uint32_t current_fps;
} chronos_stats_t;
```

---

### chronos_reset_stats

Reset statistics counters.

```c
void chronos_reset_stats(void);
```

## Error Handling

### Error Codes

```c
typedef enum {
    CHRONOS_OK = 0,
    CHRONOS_ERROR_INIT = -1,
    CHRONOS_ERROR_CAPTURE = -2,
    CHRONOS_ERROR_MEMORY = -3,
    CHRONOS_ERROR_CUDA = -4,
    CHRONOS_ERROR_TIMEOUT = -5,
    CHRONOS_ERROR_SYNC = -6,
    CHRONOS_ERROR_PARAM = -7,
} chronos_error_t;
```

### chronos_strerror

Get error string for error code.

```c
const char *chronos_strerror(chronos_error_t error);
```

## Example Usage

### Basic Capture Loop

```c
#include <chronos_capture.h>

int main() {
    // Initialize
    if (chronos_init() != CHRONOS_OK) {
        return 1;
    }
    
    // Configure
    chronos_config_t config = {
        .frame_rate = 60,
        .exposure_us = 5000,
        .gain_db = 0.0f,
        .external_trigger = true,
        .enable_imu = true,
        .buffer_count = 3
    };
    chronos_configure(&config);
    
    // Start capture
    chronos_start_capture();
    
    // Capture loop
    for (int i = 0; i < 100; i++) {
        chronos_sync_frame_set_t frame_set;
        
        if (chronos_get_frame_set(&frame_set, 1000) == CHRONOS_OK) {
            // Process frames
            for (int cam = 0; cam < CHRONOS_NUM_CAMERAS; cam++) {
                void *cuda_ptr;
                chronos_get_cuda_ptr(&frame_set.frames[cam], &cuda_ptr);
                // Use cuda_ptr in CUDA kernel...
            }
            
            // Release frame set
            chronos_release_frame_set(&frame_set);
        }
    }
    
    // Cleanup
    chronos_stop_capture();
    chronos_shutdown();
    
    return 0;
}
```

### Asynchronous Capture with Callback

```c
void frame_callback(chronos_sync_frame_set_t *frame_set, void *user_data) {
    // Process frame_set...
    
    // IMPORTANT: Release frame set
    chronos_release_frame_set(frame_set);
}

int main() {
    chronos_init();
    chronos_set_frame_callback(frame_callback, NULL);
    chronos_start_capture();
    
    // Do other work while frames arrive...
    sleep(10);
    
    chronos_stop_capture();
    chronos_shutdown();
    return 0;
}
```
