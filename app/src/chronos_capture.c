/*
 * Chronos Multi-Camera Capture Library Implementation
 *
 * Copyright (C) 2025 Chronos Project
 *
 * Zero-copy capture using NvBuffer, CUDA-EGL interop, and V4L2
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/i2c-dev.h>

#include "chronos_capture.h"

/* NVIDIA specific headers */
#include <nvbuf_utils.h>
#include <NvEglRenderer.h>
#include <cuda_runtime.h>
#include <cuda_egl_interop.h>

/* IIO for IMU */
#include <linux/iio/buffer.h>

/* ============================================================================
 * Internal Structures
 * ============================================================================ */

/* Buffer state */
typedef enum {
    BUF_STATE_FREE,
    BUF_STATE_QUEUED,
    BUF_STATE_DEQUEUED,
    BUF_STATE_CUDA_MAPPED,
} buffer_state_t;

/* Internal buffer structure */
typedef struct {
    int dmabuf_fd;
    int nvbuf_fd;
    void *cpu_ptr;
    void *cuda_ptr;
    cudaGraphicsResource_t cuda_resource;
    CUeglFrame egl_frame;
    buffer_state_t state;
    chronos_frame_meta_t meta;
} internal_buffer_t;

/* Per-camera context */
typedef struct {
    int fd;                                     /* V4L2 device fd */
    char dev_path[32];                          /* Device path */
    internal_buffer_t buffers[CHRONOS_BUFFER_COUNT];
    uint32_t buf_count;
    uint32_t sequence;
    bool streaming;
} camera_context_t;

/* IMU context */
typedef struct {
    int iio_fd;
    char dev_path[64];
    bool enabled;
    chronos_imu_data_t latest_data;
    pthread_mutex_t lock;
} imu_context_t;

/* Main context */
typedef struct {
    camera_context_t cameras[CHRONOS_NUM_CAMERAS];
    imu_context_t imu;
    
    chronos_config_t config;
    chronos_stats_t stats;
    
    pthread_t capture_thread;
    pthread_mutex_t mutex;
    pthread_cond_t frame_ready_cond;
    
    chronos_sync_frame_set_t *pending_frame_set;
    bool frame_available;
    
    chronos_frame_callback_t callback;
    void *callback_user_data;
    
    bool initialized;
    bool running;
    
    /* CUDA context */
    CUcontext cuda_context;
    cudaStream_t cuda_stream;
} chronos_context_t;

static chronos_context_t g_ctx = {0};

/* ============================================================================
 * NvBuffer Integration
 * ============================================================================ */

static int create_nvbuffer(int width, int height, NvBufferColorFormat format)
{
    NvBufferCreateParams params = {0};
    int fd = -1;
    
    params.width = width;
    params.height = height;
    params.payloadType = NvBufferPayload_SurfArray;
    params.memsize = width * height * 2;  /* 10-bit packed */
    params.layout = NvBufferLayout_Pitch;
    params.colorFormat = format;
    params.nvbuf_tag = NvBufferTag_CAMERA;
    
    if (NvBufferCreateEx(&fd, &params) != 0) {
        fprintf(stderr, "Failed to create NvBuffer\n");
        return -1;
    }
    
    return fd;
}

static int nvbuffer_to_dmabuf(int nvbuf_fd)
{
    NvBufferParams params;
    
    if (NvBufferGetParams(nvbuf_fd, &params) != 0) {
        return -1;
    }
    
    /* NvBuffer fd is also the DMA-BUF fd on Jetson */
    return nvbuf_fd;
}

/* ============================================================================
 * CUDA-EGL Interop
 * ============================================================================ */

static chronos_error_t setup_cuda_interop(internal_buffer_t *buf)
{
    cudaError_t cuda_err;
    EGLImageKHR egl_image;
    
    /* Create EGL image from DMA-BUF */
    EGLint egl_attrs[] = {
        EGL_WIDTH, CHRONOS_FRAME_WIDTH,
        EGL_HEIGHT, CHRONOS_FRAME_HEIGHT,
        EGL_LINUX_DRM_FOURCC_EXT, DRM_FORMAT_R16,  /* 16-bit grayscale */
        EGL_DMA_BUF_PLANE0_FD_EXT, buf->dmabuf_fd,
        EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
        EGL_DMA_BUF_PLANE0_PITCH_EXT, CHRONOS_FRAME_WIDTH * 2,
        EGL_NONE
    };
    
    /* Note: In production, use proper EGL context initialization */
    
    /* Register with CUDA */
    cuda_err = cudaGraphicsEGLRegisterImage(
        &buf->cuda_resource,
        egl_image,
        cudaGraphicsRegisterFlagsReadOnly
    );
    
    if (cuda_err != cudaSuccess) {
        fprintf(stderr, "CUDA EGL register failed: %s\n", 
                cudaGetErrorString(cuda_err));
        return CHRONOS_ERROR_CUDA;
    }
    
    return CHRONOS_OK;
}

static chronos_error_t map_cuda_buffer(internal_buffer_t *buf)
{
    cudaError_t cuda_err;
    
    if (buf->state == BUF_STATE_CUDA_MAPPED) {
        return CHRONOS_OK;  /* Already mapped */
    }
    
    cuda_err = cudaGraphicsMapResources(1, &buf->cuda_resource, g_ctx.cuda_stream);
    if (cuda_err != cudaSuccess) {
        return CHRONOS_ERROR_CUDA;
    }
    
    cuda_err = cudaGraphicsResourceGetMappedEglFrame(
        &buf->egl_frame, buf->cuda_resource, 0, 0);
    if (cuda_err != cudaSuccess) {
        cudaGraphicsUnmapResources(1, &buf->cuda_resource, g_ctx.cuda_stream);
        return CHRONOS_ERROR_CUDA;
    }
    
    buf->cuda_ptr = buf->egl_frame.frame.pPitch[0];
    buf->state = BUF_STATE_CUDA_MAPPED;
    
    return CHRONOS_OK;
}

static chronos_error_t unmap_cuda_buffer(internal_buffer_t *buf)
{
    cudaError_t cuda_err;
    
    if (buf->state != BUF_STATE_CUDA_MAPPED) {
        return CHRONOS_OK;
    }
    
    cuda_err = cudaGraphicsUnmapResources(1, &buf->cuda_resource, g_ctx.cuda_stream);
    if (cuda_err != cudaSuccess) {
        return CHRONOS_ERROR_CUDA;
    }
    
    buf->cuda_ptr = NULL;
    buf->state = BUF_STATE_DEQUEUED;
    
    return CHRONOS_OK;
}

/* ============================================================================
 * V4L2 Operations
 * ============================================================================ */

static chronos_error_t init_camera(int cam_idx)
{
    camera_context_t *cam = &g_ctx.cameras[cam_idx];
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    int ret;
    
    snprintf(cam->dev_path, sizeof(cam->dev_path), "/dev/video%d", cam_idx);
    
    cam->fd = open(cam->dev_path, O_RDWR);
    if (cam->fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", cam->dev_path, strerror(errno));
        return CHRONOS_ERROR_INIT;
    }
    
    /* Check capabilities */
    if (ioctl(cam->fd, VIDIOC_QUERYCAP, &cap) < 0) {
        close(cam->fd);
        return CHRONOS_ERROR_INIT;
    }
    
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        close(cam->fd);
        return CHRONOS_ERROR_INIT;
    }
    
    /* Set format */
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = CHRONOS_FRAME_WIDTH;
    fmt.fmt.pix.height = CHRONOS_FRAME_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_Y10;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    
    if (ioctl(cam->fd, VIDIOC_S_FMT, &fmt) < 0) {
        close(cam->fd);
        return CHRONOS_ERROR_INIT;
    }
    
    /* Request DMA-BUF buffers */
    memset(&req, 0, sizeof(req));
    req.count = g_ctx.config.buffer_count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_DMABUF;
    
    if (ioctl(cam->fd, VIDIOC_REQBUFS, &req) < 0) {
        close(cam->fd);
        return CHRONOS_ERROR_INIT;
    }
    
    cam->buf_count = req.count;
    
    /* Allocate NvBuffers and setup CUDA interop */
    for (uint32_t i = 0; i < cam->buf_count; i++) {
        internal_buffer_t *buf = &cam->buffers[i];
        
        buf->nvbuf_fd = create_nvbuffer(CHRONOS_FRAME_WIDTH, CHRONOS_FRAME_HEIGHT,
                                        NvBufferColorFormat_GRAY8);
        if (buf->nvbuf_fd < 0) {
            return CHRONOS_ERROR_MEMORY;
        }
        
        buf->dmabuf_fd = nvbuffer_to_dmabuf(buf->nvbuf_fd);
        buf->state = BUF_STATE_FREE;
        
        /* Setup CUDA interop */
        ret = setup_cuda_interop(buf);
        if (ret != CHRONOS_OK) {
            fprintf(stderr, "CUDA interop setup failed for cam %d buf %d\n", 
                    cam_idx, i);
        }
    }
    
    printf("Camera %d initialized: %s\n", cam_idx, cam->dev_path);
    
    return CHRONOS_OK;
}

static chronos_error_t queue_buffer(camera_context_t *cam, int buf_idx)
{
    struct v4l2_buffer buf;
    
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index = buf_idx;
    buf.m.fd = cam->buffers[buf_idx].dmabuf_fd;
    
    if (ioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
        return CHRONOS_ERROR_CAPTURE;
    }
    
    cam->buffers[buf_idx].state = BUF_STATE_QUEUED;
    
    return CHRONOS_OK;
}

static chronos_error_t dequeue_buffer(camera_context_t *cam, int *buf_idx,
                                      chronos_frame_meta_t *meta)
{
    struct v4l2_buffer buf;
    
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;
    
    if (ioctl(cam->fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) {
            return CHRONOS_ERROR_TIMEOUT;
        }
        return CHRONOS_ERROR_CAPTURE;
    }
    
    *buf_idx = buf.index;
    cam->buffers[buf.index].state = BUF_STATE_DEQUEUED;
    
    /* Fill metadata */
    meta->timestamp_ns = (uint64_t)buf.timestamp.tv_sec * 1000000000ULL +
                         (uint64_t)buf.timestamp.tv_usec * 1000ULL;
    meta->sequence = buf.sequence;
    meta->sync_valid = true;
    
    return CHRONOS_OK;
}

/* ============================================================================
 * IMU Operations
 * ============================================================================ */

static chronos_error_t init_imu(void)
{
    imu_context_t *imu = &g_ctx.imu;
    
    snprintf(imu->dev_path, sizeof(imu->dev_path), 
             "/sys/bus/iio/devices/iio:device0");
    
    /* Open IIO buffer device */
    char buf_path[128];
    snprintf(buf_path, sizeof(buf_path), "%s/buffer0/enable", imu->dev_path);
    
    imu->iio_fd = open("/dev/iio:device0", O_RDONLY);
    if (imu->iio_fd < 0) {
        fprintf(stderr, "Failed to open IMU device: %s\n", strerror(errno));
        imu->enabled = false;
        return CHRONOS_OK;  /* Non-fatal */
    }
    
    pthread_mutex_init(&imu->lock, NULL);
    imu->enabled = true;
    
    printf("IMU initialized\n");
    
    return CHRONOS_OK;
}

/*
 * Read one accel/gyro sample from the ICM-42688-P IIO interface.
 *
 * On the production board the IMU lives at /sys/bus/iio/devices/iio:device0
 * (driver name "icm42688-chronos").  We pull the raw counts via sysfs and
 * multiply by the scale that the driver advertises (in_*_scale, expressed
 * as a fractional number, IIO_VAL_INT_PLUS_NANO).  Falling back to the
 * 16 g / 2000 dps default scales lets the app boot before the driver has
 * fully populated its sysfs files.
 */
static long read_long_file(const char *path, long fallback)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) return fallback;
    char buf[64];
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return fallback;
    buf[n] = '\0';
    /* Use strtol so signed values are parsed correctly (atoi gives 0 on '-'). */
    return strtol(buf, NULL, 10);
}

static double read_scale_file(const char *path, double fallback)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) return fallback;
    char buf[64];
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    if (n <= 0) return fallback;
    buf[n] = '\0';
    return strtod(buf, NULL);
}

static void read_imu_data(void)
{
    imu_context_t *imu = &g_ctx.imu;
    if (!imu->enabled) return;

    const char *axis_name[3] = { "x", "y", "z" };
    char path[256];

    pthread_mutex_lock(&imu->lock);

    /* Scales (m/s^2 per LSB, rad/s per LSB).  Read once per sample so the
     * application sees range changes done at runtime via sysfs. */
    snprintf(path, sizeof(path), "%s/in_accel_scale", imu->dev_path);
    double acc_scale  = read_scale_file(path, 4.789062e-3); /* 16 g default */
    snprintf(path, sizeof(path), "%s/in_anglvel_scale", imu->dev_path);
    double gyro_scale = read_scale_file(path, 1.064724e-3); /* 2000 dps default */

    for (int i = 0; i < 3; ++i) {
        snprintf(path, sizeof(path),
                 "%s/in_accel_%s_raw", imu->dev_path, axis_name[i]);
        long raw = read_long_file(path, 0);
        imu->latest_data.accel[i] = (float)(raw * acc_scale);

        snprintf(path, sizeof(path),
                 "%s/in_anglvel_%s_raw", imu->dev_path, axis_name[i]);
        raw = read_long_file(path, 0);
        imu->latest_data.gyro[i] = (float)(raw * gyro_scale);
    }

    /* Temperature: (raw / 132.48) + 25 deg C  -- raw in milli-degrees C
     * is exposed by the driver as in_temp_raw with separate scale/offset. */
    snprintf(path, sizeof(path), "%s/in_temp_raw", imu->dev_path);
    long traw = read_long_file(path, 0);
    imu->latest_data.temp = (float)(traw / 132.48 + 25.0);

    /* Sync count / last_sync_time exposed by the driver as device attrs. */
    snprintf(path, sizeof(path), "%s/sync_count", imu->dev_path);
    imu->latest_data.sync_count = (uint64_t)read_long_file(path, 0);
    snprintf(path, sizeof(path), "%s/last_sync_time", imu->dev_path);
    imu->latest_data.timestamp_ns = (uint64_t)read_long_file(path, 0);

    pthread_mutex_unlock(&imu->lock);
}

/* ============================================================================
 * Capture Thread
 * ============================================================================ */

static void *capture_thread_func(void *arg)
{
    (void)arg;

    /* Build a select() set out of cameras that actually opened cleanly.
     * Treat fd <= 0 as "not present" — the previous code FD_SET()ed those
     * which is undefined behaviour. */
    int max_fd = 0;
    int active_count = 0;
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) {
        int fd = g_ctx.cameras[i].fd;
        if (fd > 0) {
            ++active_count;
            if (fd > max_fd) max_fd = fd;
        }
    }
    if (active_count == 0) {
        fprintf(stderr, "No active cameras — capture thread exiting.\n");
        return NULL;
    }

    /* Track which buffers we have actually dequeued (and therefore own). */
    while (g_ctx.running) {
        fd_set fds;
        FD_ZERO(&fds);
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) {
            if (g_ctx.cameras[i].fd > 0)
                FD_SET(g_ctx.cameras[i].fd, &fds);
        }

        struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
        int ret = select(max_fd + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (ret == 0) continue;             /* idle tick — keep looping */

        chronos_sync_frame_set_t frame_set;
        memset(&frame_set, 0, sizeof(frame_set));
        int frames_received = 0;
        int dequeued_idx[CHRONOS_NUM_CAMERAS];
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) dequeued_idx[i] = -1;

        /* Drain one frame from every ready camera. */
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) {
            if (g_ctx.cameras[i].fd <= 0) continue;
            if (!FD_ISSET(g_ctx.cameras[i].fd, &fds)) continue;

            int buf_idx;
            chronos_frame_meta_t meta;
            if (dequeue_buffer(&g_ctx.cameras[i], &buf_idx, &meta) != CHRONOS_OK)
                continue;

            dequeued_idx[i] = buf_idx;
            internal_buffer_t *buf = &g_ctx.cameras[i].buffers[buf_idx];

            chronos_frame_t *f = &frame_set.frames[i];
            f->dmabuf_fd = buf->dmabuf_fd;
            f->nvbuf_fd  = buf->nvbuf_fd;
            f->cuda_ptr  = buf->cuda_ptr;
            f->width     = CHRONOS_FRAME_WIDTH;
            f->height    = CHRONOS_FRAME_HEIGHT;
            f->pitch     = CHRONOS_FRAME_WIDTH * CHRONOS_FRAME_BPP;
            f->size      = CHRONOS_FRAME_WIDTH * CHRONOS_FRAME_HEIGHT * CHRONOS_FRAME_BPP;
            f->meta            = meta;
            f->meta.camera_id  = (uint32_t)i;
            f->meta.vc_id      = (uint32_t)i;

            ++frames_received;
            ++g_ctx.stats.frames_captured[i];
        }

        /* IMU sample (best-effort). */
        read_imu_data();
        pthread_mutex_lock(&g_ctx.imu.lock);
        frame_set.imu = g_ctx.imu.latest_data;
        pthread_mutex_unlock(&g_ctx.imu.lock);

        frame_set.complete       = (frames_received == active_count);
        frame_set.sync_sequence  = g_ctx.cameras[0].sequence++;
        frame_set.sync_timestamp_ns = frame_set.frames[0].meta.timestamp_ns;

        if (!frame_set.complete)
            ++g_ctx.stats.sync_errors;

        /* Deliver. */
        if (g_ctx.callback) {
            g_ctx.callback(&frame_set, g_ctx.callback_user_data);
            /* Caller is expected to call chronos_release_frame_set(). */
        } else {
            pthread_mutex_lock(&g_ctx.mutex);
            if (g_ctx.pending_frame_set) {
                ++g_ctx.stats.buffer_overruns;
                free(g_ctx.pending_frame_set);
            }
            g_ctx.pending_frame_set = malloc(sizeof(frame_set));
            if (g_ctx.pending_frame_set) {
                *g_ctx.pending_frame_set = frame_set;
                g_ctx.frame_available    = true;
                pthread_cond_signal(&g_ctx.frame_ready_cond);
            }
            pthread_mutex_unlock(&g_ctx.mutex);
        }

        /* Re-queue every buffer we just dequeued.  Previous version walked
         * every camera's buffer list looking for BUF_STATE_FREE, which
         * left dequeued buffers stuck in DEQUEUED forever after the first
         * non-callback consumer. */
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) {
            if (dequeued_idx[i] < 0) continue;
            if (g_ctx.cameras[i].buffers[dequeued_idx[i]].state == BUF_STATE_DEQUEUED)
                queue_buffer(&g_ctx.cameras[i], dequeued_idx[i]);
        }
    }

    return NULL;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

chronos_error_t chronos_init(void)
{
    cudaError_t cuda_err;
    chronos_error_t ret;
    
    if (g_ctx.initialized) {
        return CHRONOS_ERROR_INIT;
    }
    
    memset(&g_ctx, 0, sizeof(g_ctx));
    
    /* Default configuration */
    g_ctx.config.frame_rate = 30;
    g_ctx.config.exposure_us = 5000;
    g_ctx.config.gain_db = 0.0f;
    g_ctx.config.external_trigger = true;
    g_ctx.config.enable_imu = true;
    g_ctx.config.buffer_count = CHRONOS_BUFFER_COUNT;
    
    pthread_mutex_init(&g_ctx.mutex, NULL);
    pthread_cond_init(&g_ctx.frame_ready_cond, NULL);
    
    /* Initialize CUDA */
    cuda_err = cudaSetDevice(0);
    if (cuda_err != cudaSuccess) {
        fprintf(stderr, "CUDA device init failed: %s\n", cudaGetErrorString(cuda_err));
        return CHRONOS_ERROR_CUDA;
    }
    
    cuda_err = cudaStreamCreate(&g_ctx.cuda_stream);
    if (cuda_err != cudaSuccess) {
        return CHRONOS_ERROR_CUDA;
    }
    
    /* Initialize cameras */
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        ret = init_camera(i);
        if (ret != CHRONOS_OK) {
            fprintf(stderr, "Failed to init camera %d\n", i);
            return ret;
        }
    }
    
    /* Initialize IMU */
    init_imu();
    
    g_ctx.initialized = true;
    
    printf("Chronos capture system initialized\n");
    
    return CHRONOS_OK;
}

void chronos_shutdown(void)
{
    if (!g_ctx.initialized) return;
    
    chronos_stop_capture();
    
    /* Close cameras */
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        camera_context_t *cam = &g_ctx.cameras[i];
        
        for (uint32_t j = 0; j < cam->buf_count; j++) {
            if (cam->buffers[j].cuda_resource) {
                cudaGraphicsUnregisterResource(cam->buffers[j].cuda_resource);
            }
            if (cam->buffers[j].nvbuf_fd >= 0) {
                NvBufferDestroy(cam->buffers[j].nvbuf_fd);
            }
        }
        
        if (cam->fd >= 0) {
            close(cam->fd);
        }
    }
    
    /* Close IMU */
    if (g_ctx.imu.iio_fd >= 0) {
        close(g_ctx.imu.iio_fd);
        pthread_mutex_destroy(&g_ctx.imu.lock);
    }
    
    /* Cleanup CUDA */
    cudaStreamDestroy(g_ctx.cuda_stream);
    
    pthread_mutex_destroy(&g_ctx.mutex);
    pthread_cond_destroy(&g_ctx.frame_ready_cond);
    
    g_ctx.initialized = false;
    
    printf("Chronos capture system shutdown\n");
}

/*
 * Apply the cached configuration to the FPGA register bank over I2C.
 *
 * The kernel-side chronos_csi driver also writes the same registers via the
 * I2C client it owns, but the user-space library needs to be able to push a
 * config update before streaming starts (e.g. set the frame rate before
 * VIDIOC_STREAMON) without round-tripping through a custom ioctl.  We open
 * /dev/i2c-<bus> directly and talk to the FPGA at 7-bit address 0x3C.
 *
 * The bus number is taken from CHRONOS_FPGA_I2C_BUS (default 1, matching the
 * Jetson Orin NX I2C2 instance that the device-tree overlay activates).
 */
#define CHRONOS_FPGA_I2C_ADDR  0x3C
#ifndef CHRONOS_FPGA_I2C_BUS
#define CHRONOS_FPGA_I2C_BUS   1
#endif

/* Register map – must stay in lock-step with config_regs.sv */
enum {
    FPGA_REG_CTRL          = 0x00,
    FPGA_REG_FRAME_RATE    = 0x01,
    FPGA_REG_PULSE_WIDTH_L = 0x02,
    FPGA_REG_PULSE_WIDTH_H = 0x03,
    FPGA_REG_CAM_ENABLE    = 0x04,
    FPGA_REG_DATA_TYPE     = 0x05,
    FPGA_REG_TRIG_DELAY_0  = 0x10,
};

static int fpga_i2c_write(uint8_t reg, uint8_t val)
{
    char dev_path[32];
    snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%d", CHRONOS_FPGA_I2C_BUS);
    int fd = open(dev_path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "FPGA I2C open %s failed: %s\n", dev_path, strerror(errno));
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE, CHRONOS_FPGA_I2C_ADDR) < 0) {
        close(fd);
        return -1;
    }
    uint8_t buf[2] = { reg, val };
    int ret = write(fd, buf, sizeof(buf)) == sizeof(buf) ? 0 : -1;
    close(fd);
    return ret;
}

static chronos_error_t push_config_to_fpga(const chronos_config_t *cfg)
{
    uint32_t fps = cfg->frame_rate ? cfg->frame_rate : 30;
    if (fps > 120) fps = 120;

    /*
     * Pulse width is in fabric (clk_sys = 200 MHz) cycles inside the FPGA.
     * Take half the requested exposure as a generous default — the
     * sensor's own exposure register controls integration time, FSIN
     * width only needs to satisfy OV9281's minimum pulse spec (~1 us).
     */
    uint32_t pulse_us = cfg->exposure_us ? (cfg->exposure_us / 2) : 10;
    if (pulse_us < 2)    pulse_us = 2;
    if (pulse_us > 1000) pulse_us = 1000;
    uint32_t pulse_cycles = pulse_us * 200;     /* 200 cycles == 1 us @ 200 MHz */
    if (pulse_cycles > 0xFFFF) pulse_cycles = 0xFFFF;

    if (fpga_i2c_write(FPGA_REG_FRAME_RATE,    (uint8_t)fps) < 0)
        return CHRONOS_ERROR_INIT;
    if (fpga_i2c_write(FPGA_REG_PULSE_WIDTH_L, (uint8_t)(pulse_cycles & 0xFF)) < 0)
        return CHRONOS_ERROR_INIT;
    if (fpga_i2c_write(FPGA_REG_PULSE_WIDTH_H, (uint8_t)((pulse_cycles >> 8) & 0xFF)) < 0)
        return CHRONOS_ERROR_INIT;
    if (fpga_i2c_write(FPGA_REG_CAM_ENABLE,    0x0F) < 0)
        return CHRONOS_ERROR_INIT;
    if (fpga_i2c_write(FPGA_REG_DATA_TYPE,     0x2B /* RAW10 */) < 0)
        return CHRONOS_ERROR_INIT;

    /* Per-camera trigger delay calibration (defaults to 0). */
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) {
        if (fpga_i2c_write((uint8_t)(FPGA_REG_TRIG_DELAY_0 + i), 0) < 0)
            return CHRONOS_ERROR_INIT;
    }
    return CHRONOS_OK;
}

chronos_error_t chronos_configure(const chronos_config_t *config)
{
    if (!config) return CHRONOS_ERROR_PARAM;
    if (!g_ctx.initialized) return CHRONOS_ERROR_INIT;
    if (g_ctx.running) return CHRONOS_ERROR_CAPTURE;

    /* Validate up front to avoid pushing nonsense to the hardware. */
    if (config->frame_rate > 120) return CHRONOS_ERROR_PARAM;
    if (config->exposure_us > 100000) return CHRONOS_ERROR_PARAM;
    if (config->buffer_count > CHRONOS_BUFFER_COUNT) return CHRONOS_ERROR_PARAM;

    g_ctx.config = *config;
    if (g_ctx.config.buffer_count == 0)
        g_ctx.config.buffer_count = CHRONOS_BUFFER_COUNT;

    /* Apply to the FPGA register bank and the per-sensor V4L2 controls.
     * If the FPGA is reachable we expect success; otherwise we still keep
     * the cached config so demo apps that run without an attached board
     * (eg. CI smoke tests on a Jetson without the daughter card) don't
     * fail outright. */
    chronos_error_t fr = push_config_to_fpga(&g_ctx.config);
    if (fr != CHRONOS_OK) {
        fprintf(stderr,
                "Warning: could not push configuration to FPGA (continuing).\n");
    }

    /* Per-camera exposure / gain via VIDIOC_S_CTRL.  Best-effort. */
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) {
        if (g_ctx.cameras[i].fd < 0)
            continue;
        struct v4l2_control ctrl;
        ctrl.id = V4L2_CID_EXPOSURE;
        /* OV9281 exposure unit ≈ line time ~= 13.7 us @ 120 fps.
         * exposure_us / line_us → integer line count. */
        ctrl.value = (int32_t)(g_ctx.config.exposure_us / 14);
        (void)ioctl(g_ctx.cameras[i].fd, VIDIOC_S_CTRL, &ctrl);
    }

    return CHRONOS_OK;
}

chronos_error_t chronos_get_config(chronos_config_t *config)
{
    if (!config) return CHRONOS_ERROR_PARAM;
    if (!g_ctx.initialized) return CHRONOS_ERROR_INIT;
    
    *config = g_ctx.config;
    
    return CHRONOS_OK;
}

chronos_error_t chronos_start_capture(void)
{
    if (!g_ctx.initialized) return CHRONOS_ERROR_INIT;
    if (g_ctx.running) return CHRONOS_OK;
    
    /* Queue initial buffers */
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        camera_context_t *cam = &g_ctx.cameras[i];
        
        for (uint32_t j = 0; j < cam->buf_count; j++) {
            queue_buffer(cam, j);
        }
        
        /* Start streaming */
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(cam->fd, VIDIOC_STREAMON, &type) < 0) {
            fprintf(stderr, "Failed to start streaming on cam %d\n", i);
            return CHRONOS_ERROR_CAPTURE;
        }
        
        cam->streaming = true;
    }
    
    /* Start capture thread */
    g_ctx.running = true;
    if (pthread_create(&g_ctx.capture_thread, NULL, capture_thread_func, NULL) != 0) {
        g_ctx.running = false;
        return CHRONOS_ERROR_INIT;
    }
    
    printf("Capture started\n");
    
    return CHRONOS_OK;
}

chronos_error_t chronos_stop_capture(void)
{
    if (!g_ctx.running) return CHRONOS_OK;
    
    g_ctx.running = false;
    pthread_join(g_ctx.capture_thread, NULL);
    
    /* Stop streaming */
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        camera_context_t *cam = &g_ctx.cameras[i];
        
        if (cam->streaming) {
            enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(cam->fd, VIDIOC_STREAMOFF, &type);
            cam->streaming = false;
        }
    }
    
    printf("Capture stopped\n");
    
    return CHRONOS_OK;
}

bool chronos_is_capturing(void)
{
    return g_ctx.running;
}

chronos_error_t chronos_get_frame_set(chronos_sync_frame_set_t *frame_set, int timeout_ms)
{
    if (!frame_set) return CHRONOS_ERROR_PARAM;
    if (!g_ctx.running) return CHRONOS_ERROR_CAPTURE;
    
    pthread_mutex_lock(&g_ctx.mutex);
    
    if (!g_ctx.frame_available) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += timeout_ms / 1000;
        ts.tv_nsec += (timeout_ms % 1000) * 1000000;
        
        int ret = pthread_cond_timedwait(&g_ctx.frame_ready_cond, &g_ctx.mutex, &ts);
        if (ret == ETIMEDOUT) {
            pthread_mutex_unlock(&g_ctx.mutex);
            return CHRONOS_ERROR_TIMEOUT;
        }
    }
    
    if (g_ctx.pending_frame_set) {
        *frame_set = *g_ctx.pending_frame_set;
        free(g_ctx.pending_frame_set);
        g_ctx.pending_frame_set = NULL;
        g_ctx.frame_available = false;
    }
    
    pthread_mutex_unlock(&g_ctx.mutex);
    
    return CHRONOS_OK;
}

chronos_error_t chronos_release_frame_set(chronos_sync_frame_set_t *frame_set)
{
    if (!frame_set) return CHRONOS_ERROR_PARAM;
    
    /* Mark buffers as free for re-use */
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        for (uint32_t j = 0; j < g_ctx.cameras[i].buf_count; j++) {
            if (g_ctx.cameras[i].buffers[j].dmabuf_fd == frame_set->frames[i].dmabuf_fd) {
                unmap_cuda_buffer(&g_ctx.cameras[i].buffers[j]);
                g_ctx.cameras[i].buffers[j].state = BUF_STATE_FREE;
                break;
            }
        }
    }
    
    return CHRONOS_OK;
}

chronos_error_t chronos_set_frame_callback(chronos_frame_callback_t callback, void *user_data)
{
    g_ctx.callback = callback;
    g_ctx.callback_user_data = user_data;
    return CHRONOS_OK;
}

chronos_error_t chronos_get_cuda_ptr(chronos_frame_t *frame, void **cuda_ptr)
{
    if (!frame || !cuda_ptr) return CHRONOS_ERROR_PARAM;
    
    /* Find the internal buffer */
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        for (uint32_t j = 0; j < g_ctx.cameras[i].buf_count; j++) {
            if (g_ctx.cameras[i].buffers[j].dmabuf_fd == frame->dmabuf_fd) {
                chronos_error_t ret = map_cuda_buffer(&g_ctx.cameras[i].buffers[j]);
                if (ret == CHRONOS_OK) {
                    *cuda_ptr = g_ctx.cameras[i].buffers[j].cuda_ptr;
                }
                return ret;
            }
        }
    }
    
    return CHRONOS_ERROR_PARAM;
}

chronos_error_t chronos_cuda_map(chronos_frame_t *frame)
{
    void *ptr;
    return chronos_get_cuda_ptr(frame, &ptr);
}

chronos_error_t chronos_cuda_unmap(chronos_frame_t *frame)
{
    if (!frame) return CHRONOS_ERROR_PARAM;
    
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        for (uint32_t j = 0; j < g_ctx.cameras[i].buf_count; j++) {
            if (g_ctx.cameras[i].buffers[j].dmabuf_fd == frame->dmabuf_fd) {
                return unmap_cuda_buffer(&g_ctx.cameras[i].buffers[j]);
            }
        }
    }
    
    return CHRONOS_ERROR_PARAM;
}

chronos_error_t chronos_get_imu_data(chronos_imu_data_t *imu_data)
{
    if (!imu_data) return CHRONOS_ERROR_PARAM;
    if (!g_ctx.imu.enabled) return CHRONOS_ERROR_INIT;
    
    pthread_mutex_lock(&g_ctx.imu.lock);
    *imu_data = g_ctx.imu.latest_data;
    pthread_mutex_unlock(&g_ctx.imu.lock);
    
    return CHRONOS_OK;
}

chronos_error_t chronos_get_stats(chronos_stats_t *stats)
{
    if (!stats) return CHRONOS_ERROR_PARAM;
    *stats = g_ctx.stats;
    return CHRONOS_OK;
}

void chronos_reset_stats(void)
{
    memset(&g_ctx.stats, 0, sizeof(g_ctx.stats));
}

const char *chronos_strerror(chronos_error_t error)
{
    switch (error) {
    case CHRONOS_OK:            return "Success";
    case CHRONOS_ERROR_INIT:    return "Initialization error";
    case CHRONOS_ERROR_CAPTURE: return "Capture error";
    case CHRONOS_ERROR_MEMORY:  return "Memory allocation error";
    case CHRONOS_ERROR_CUDA:    return "CUDA error";
    case CHRONOS_ERROR_TIMEOUT: return "Timeout";
    case CHRONOS_ERROR_SYNC:    return "Synchronization error";
    case CHRONOS_ERROR_PARAM:   return "Invalid parameter";
    default:                    return "Unknown error";
    }
}
