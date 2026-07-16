/*
 * Chronos Multi-Camera Capture Library Implementation
 *
 * Copyright (C) 2025 Chronos Project
 *
 * V4L2 MMAP capture with an optional CUDA device-copy path.
 *
 * Frames are captured into driver-owned MMAP buffers and the CPU pointer is
 * handed to the consumer directly.  Buffers are NOT re-queued until the
 * consumer calls chronos_release_frame_set(), so delivered frame data can
 * never be overwritten behind the consumer's back.  When built with
 * CHRONOS_WITH_CUDA, chronos_get_cuda_ptr() copies the frame into a
 * per-buffer cudaMalloc'd allocation (tight pitch) on demand.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>
#include <linux/i2c-dev.h>

#include "chronos_capture.h"

#ifdef CHRONOS_WITH_CUDA
#include <cuda_runtime_api.h>
#endif

/* ============================================================================
 * Internal Structures
 * ============================================================================ */

/* Buffer state */
typedef enum {
    BUF_STATE_FREE,
    BUF_STATE_QUEUED,
    BUF_STATE_DEQUEUED,
} buffer_state_t;

/* Internal buffer structure */
typedef struct {
    void *cpu_ptr;              /* mmap()ed V4L2 buffer */
    size_t length;              /* mmap length */
#ifdef CHRONOS_WITH_CUDA
    void *cuda_ptr;             /* device-side copy destination */
    bool cuda_valid;            /* device copy is current for this frame */
#endif
    buffer_state_t state;
} internal_buffer_t;

/* Per-camera context */
typedef struct {
    int fd;                                     /* V4L2 device fd */
    char dev_path[32];                          /* Device path */
    internal_buffer_t buffers[CHRONOS_BUFFER_COUNT];
    uint32_t buf_count;
    uint32_t bytesperline;                      /* negotiated line pitch */
    uint32_t sizeimage;                         /* negotiated frame size */
    uint32_t last_sequence;                     /* for drop detection */
    bool have_sequence;
    bool has_gain_ctrl;                         /* driver exposes V4L2_CID_GAIN */
    int32_t gain_min;
    int32_t gain_max;
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

/*
 * In-flight frame set bookkeeping.  Each delivered frame set records which
 * buffer indices it borrowed from each camera; chronos_release_frame_set()
 * looks the entry up and re-queues those buffers.  Worst case every buffer
 * of every camera sits in its own (incomplete) set.
 */
#define CHRONOS_MAX_INFLIGHT_SETS (CHRONOS_NUM_CAMERAS * CHRONOS_BUFFER_COUNT)

typedef struct {
    bool valid;
    uint32_t sync_sequence;
    int buf_idx[CHRONOS_NUM_CAMERAS];           /* -1 = no frame from that cam */
} inflight_set_t;

/* Main context */
typedef struct {
    camera_context_t cameras[CHRONOS_NUM_CAMERAS];
    imu_context_t imu;

    chronos_config_t config;
    chronos_stats_t stats;

    pthread_t capture_thread;
    pthread_mutex_t mutex;
    pthread_cond_t frame_ready_cond;
    pthread_mutex_t i2c_lock;

    chronos_sync_frame_set_t *pending_frame_set;
    bool frame_available;

    inflight_set_t inflight[CHRONOS_MAX_INFLIGHT_SETS];

    chronos_frame_callback_t callback;
    void *callback_user_data;

    bool initialized;
    bool running;

    /* Persistent FPGA control link (7-bit 0x3C on /dev/i2c-N) */
    int fpga_i2c_fd;

    /* Cached FPGA sync status (refreshed ~1 Hz by the capture thread) */
    bool fpga_status_known;
    bool fpga_sync_ok;

    /* Frame-rate measurement window */
    uint64_t fps_window_start_ns;
    uint32_t fps_window_frames;

#ifdef CHRONOS_WITH_CUDA
    bool cuda_enabled;
#endif
} chronos_context_t;

static chronos_context_t g_ctx = { .fpga_i2c_fd = -1 };

static uint64_t monotonic_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* ============================================================================
 * FPGA Control (I2C)
 * ============================================================================ */

/*
 * The FPGA register bank lives at 7-bit address 0x3C.  The bus fd is opened
 * once at chronos_init() (with I2C_SLAVE already set) and kept for the whole
 * session; read and write helpers share it under i2c_lock.
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
    FPGA_REG_STATUS        = 0x20,
    FPGA_REG_ERROR         = 0x21,
    FPGA_REG_VERSION       = 0xF0,
    FPGA_REG_ID_L          = 0xFE,
    FPGA_REG_ID_H          = 0xFF,
};

/* CTRL register bits */
#define FPGA_CTRL_TRIGGER_ENABLE  0x01

/* Expected identity values */
#define FPGA_VERSION_EXPECTED  0x10
#define FPGA_ID_L_EXPECTED     0x05
#define FPGA_ID_H_EXPECTED     0xC4

static int fpga_i2c_open(void)
{
    char dev_path[32];

    if (g_ctx.fpga_i2c_fd >= 0)
        return 0;

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
    g_ctx.fpga_i2c_fd = fd;
    return 0;
}

static void fpga_i2c_close(void)
{
    if (g_ctx.fpga_i2c_fd >= 0) {
        close(g_ctx.fpga_i2c_fd);
        g_ctx.fpga_i2c_fd = -1;
    }
}

static int fpga_i2c_write(uint8_t reg, uint8_t val)
{
    if (g_ctx.fpga_i2c_fd < 0)
        return -1;

    uint8_t buf[2] = { reg, val };
    pthread_mutex_lock(&g_ctx.i2c_lock);
    int ret = write(g_ctx.fpga_i2c_fd, buf, sizeof(buf)) == sizeof(buf) ? 0 : -1;
    pthread_mutex_unlock(&g_ctx.i2c_lock);
    return ret;
}

static int fpga_i2c_read(uint8_t reg, uint8_t *val)
{
    if (g_ctx.fpga_i2c_fd < 0)
        return -1;

    int ret = -1;
    pthread_mutex_lock(&g_ctx.i2c_lock);
    if (write(g_ctx.fpga_i2c_fd, &reg, 1) == 1 &&
        read(g_ctx.fpga_i2c_fd, val, 1) == 1) {
        ret = 0;
    }
    pthread_mutex_unlock(&g_ctx.i2c_lock);
    return ret;
}

/* ============================================================================
 * V4L2 Operations
 * ============================================================================ */

static void cleanup_camera(int cam_idx)
{
    camera_context_t *cam = &g_ctx.cameras[cam_idx];

    for (uint32_t j = 0; j < cam->buf_count; j++) {
        internal_buffer_t *buf = &cam->buffers[j];
        if (buf->cpu_ptr) {
            munmap(buf->cpu_ptr, buf->length);
            buf->cpu_ptr = NULL;
        }
#ifdef CHRONOS_WITH_CUDA
        if (buf->cuda_ptr) {
            cudaFree(buf->cuda_ptr);
            buf->cuda_ptr = NULL;
        }
#endif
    }
    cam->buf_count = 0;

    if (cam->fd > 0) {
        close(cam->fd);
        cam->fd = -1;
    }
}

static chronos_error_t init_camera(int cam_idx)
{
    camera_context_t *cam = &g_ctx.cameras[cam_idx];
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    struct v4l2_queryctrl qc;

    snprintf(cam->dev_path, sizeof(cam->dev_path), "/dev/video%d", cam_idx);

    cam->fd = open(cam->dev_path, O_RDWR);
    if (cam->fd < 0) {
        fprintf(stderr, "Failed to open %s: %s\n", cam->dev_path, strerror(errno));
        return CHRONOS_ERROR_INIT;
    }

    /* Check capabilities */
    if (ioctl(cam->fd, VIDIOC_QUERYCAP, &cap) < 0) {
        return CHRONOS_ERROR_INIT;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
        !(cap.capabilities & V4L2_CAP_STREAMING)) {
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
        return CHRONOS_ERROR_INIT;
    }

    /* VIDIOC_S_FMT returns the negotiated layout in the same struct —
     * remember the real line pitch (may include padding). */
    cam->bytesperline = fmt.fmt.pix.bytesperline;
    cam->sizeimage    = fmt.fmt.pix.sizeimage;
    if (cam->bytesperline == 0)
        cam->bytesperline = CHRONOS_FRAME_WIDTH * CHRONOS_FRAME_BPP;
    if (cam->sizeimage == 0)
        cam->sizeimage = cam->bytesperline * CHRONOS_FRAME_HEIGHT;

    /* Request MMAP buffers */
    memset(&req, 0, sizeof(req));
    req.count = g_ctx.config.buffer_count;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(cam->fd, VIDIOC_REQBUFS, &req) < 0) {
        return CHRONOS_ERROR_INIT;
    }
    if (req.count == 0) {
        return CHRONOS_ERROR_INIT;
    }

    cam->buf_count = req.count;

    /* Map buffers into user space */
    for (uint32_t i = 0; i < cam->buf_count; i++) {
        internal_buffer_t *buf = &cam->buffers[i];
        struct v4l2_buffer vbuf;

        memset(&vbuf, 0, sizeof(vbuf));
        vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vbuf.memory = V4L2_MEMORY_MMAP;
        vbuf.index = i;

        if (ioctl(cam->fd, VIDIOC_QUERYBUF, &vbuf) < 0) {
            return CHRONOS_ERROR_INIT;
        }

        buf->length = vbuf.length;
        buf->cpu_ptr = mmap(NULL, vbuf.length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, cam->fd, vbuf.m.offset);
        if (buf->cpu_ptr == MAP_FAILED) {
            buf->cpu_ptr = NULL;
            return CHRONOS_ERROR_MEMORY;
        }
        buf->state = BUF_STATE_FREE;

#ifdef CHRONOS_WITH_CUDA
        if (g_ctx.cuda_enabled) {
            /* Tight-pitch device destination for the on-demand copy. */
            size_t dev_size = (size_t)CHRONOS_FRAME_WIDTH *
                              CHRONOS_FRAME_HEIGHT * CHRONOS_FRAME_BPP;
            if (cudaMalloc(&buf->cuda_ptr, dev_size) != cudaSuccess) {
                fprintf(stderr, "cudaMalloc failed — CUDA path disabled\n");
                buf->cuda_ptr = NULL;
                g_ctx.cuda_enabled = false;
            }
        }
#endif
    }

    /* Gain is only applied if the driver actually exposes V4L2_CID_GAIN. */
    memset(&qc, 0, sizeof(qc));
    qc.id = V4L2_CID_GAIN;
    if (ioctl(cam->fd, VIDIOC_QUERYCTRL, &qc) == 0 &&
        !(qc.flags & V4L2_CTRL_FLAG_DISABLED)) {
        cam->has_gain_ctrl = true;
        cam->gain_min = qc.minimum;
        cam->gain_max = qc.maximum;
    }

    printf("Camera %d initialized: %s (pitch %u, image %u bytes)\n",
           cam_idx, cam->dev_path, cam->bytesperline, cam->sizeimage);

    return CHRONOS_OK;
}

static chronos_error_t queue_buffer(camera_context_t *cam, int buf_idx)
{
    struct v4l2_buffer buf;

    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = buf_idx;

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
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(cam->fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) {
            return CHRONOS_ERROR_TIMEOUT;
        }
        return CHRONOS_ERROR_CAPTURE;
    }

    *buf_idx = buf.index;
    cam->buffers[buf.index].state = BUF_STATE_DEQUEUED;
#ifdef CHRONOS_WITH_CUDA
    cam->buffers[buf.index].cuda_valid = false;
#endif

    /* Fill metadata; sync_valid is decided per-set by the capture thread. */
    memset(meta, 0, sizeof(*meta));
    meta->timestamp_ns = (uint64_t)buf.timestamp.tv_sec * 1000000000ULL +
                         (uint64_t)buf.timestamp.tv_usec * 1000ULL;
    meta->sequence = buf.sequence;

    return CHRONOS_OK;
}

/* ============================================================================
 * In-flight Frame Set Tracking
 * ============================================================================ */

/* All helpers below must be called with g_ctx.mutex held. */

static bool register_inflight_locked(const chronos_sync_frame_set_t *set,
                                     const int *dequeued_idx)
{
    for (int s = 0; s < CHRONOS_MAX_INFLIGHT_SETS; ++s) {
        inflight_set_t *e = &g_ctx.inflight[s];
        if (e->valid) continue;
        e->valid = true;
        e->sync_sequence = set->sync_sequence;
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i)
            e->buf_idx[i] = dequeued_idx[i];
        return true;
    }
    return false;
}

static inflight_set_t *find_inflight_locked(const chronos_sync_frame_set_t *set)
{
    for (int s = 0; s < CHRONOS_MAX_INFLIGHT_SETS; ++s) {
        inflight_set_t *e = &g_ctx.inflight[s];
        if (!e->valid || e->sync_sequence != set->sync_sequence)
            continue;
        /* Sequence numbers of different cameras can collide; disambiguate
         * by matching the recorded buffers against the set's data pointers. */
        bool match = true;
        for (int i = 0; i < CHRONOS_NUM_CAMERAS && match; ++i) {
            if (e->buf_idx[i] < 0) {
                if (set->frames[i].data != NULL)
                    match = false;
            } else if (g_ctx.cameras[i].buffers[e->buf_idx[i]].cpu_ptr !=
                       set->frames[i].data) {
                match = false;
            }
        }
        if (match)
            return e;
    }
    return NULL;
}

static void requeue_inflight_locked(inflight_set_t *entry)
{
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) {
        int idx = entry->buf_idx[i];
        if (idx < 0) continue;
        camera_context_t *cam = &g_ctx.cameras[i];
        if (cam->buffers[idx].state != BUF_STATE_DEQUEUED) continue;
        if (cam->streaming)
            queue_buffer(cam, idx);
        else
            cam->buffers[idx].state = BUF_STATE_FREE;
    }
    entry->valid = false;
}

/* ============================================================================
 * IMU Operations
 * ============================================================================ */

static chronos_error_t init_imu(void)
{
    imu_context_t *imu = &g_ctx.imu;

    /* Lock is initialized unconditionally so shutdown can always destroy it. */
    pthread_mutex_init(&imu->lock, NULL);

    snprintf(imu->dev_path, sizeof(imu->dev_path),
             "/sys/bus/iio/devices/iio:device0");

    imu->iio_fd = open("/dev/iio:device0", O_RDONLY);
    if (imu->iio_fd < 0) {
        fprintf(stderr, "Failed to open IMU device: %s\n", strerror(errno));
        imu->enabled = false;
        return CHRONOS_OK;  /* Non-fatal */
    }

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

static void refresh_fpga_sync_status(uint8_t active_mask)
{
    uint8_t status;
    if (fpga_i2c_read(FPGA_REG_STATUS, &status) == 0) {
        g_ctx.fpga_status_known = true;
        /* Bits 7:4 = per-camera sync status. */
        g_ctx.fpga_sync_ok = (((status >> 4) & active_mask) == active_mask);
    } else {
        g_ctx.fpga_status_known = false;
    }
}

static void *capture_thread_func(void *arg)
{
    (void)arg;

    /* Build a select() set out of cameras that actually opened cleanly.
     * Treat fd <= 0 as "not present" — the previous code FD_SET()ed those
     * which is undefined behaviour. */
    int max_fd = 0;
    int active_count = 0;
    uint8_t active_mask = 0;
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) {
        int fd = g_ctx.cameras[i].fd;
        if (fd > 0) {
            ++active_count;
            active_mask |= (uint8_t)(1u << i);
            if (fd > max_fd) max_fd = fd;
        }
    }
    if (active_count == 0) {
        fprintf(stderr, "No active cameras — capture thread exiting.\n");
        return NULL;
    }

    uint32_t fps = g_ctx.config.frame_rate ? g_ctx.config.frame_rate : 30;
    uint64_t half_period_ns = 500000000ULL / fps;

    refresh_fpga_sync_status(active_mask);

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
        uint64_t min_ts = UINT64_MAX;
        uint64_t max_ts = 0;
        int first_cam = -1;

        /* Drain one frame from every ready camera. */
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) {
            camera_context_t *cam = &g_ctx.cameras[i];
            if (cam->fd <= 0) continue;
            if (!FD_ISSET(cam->fd, &fds)) continue;

            int buf_idx;
            chronos_frame_meta_t meta;
            if (dequeue_buffer(cam, &buf_idx, &meta) != CHRONOS_OK)
                continue;

            /* Drop detection via gaps in the driver's sequence counter. */
            if (cam->have_sequence) {
                uint32_t expected = cam->last_sequence + 1;
                if (meta.sequence != expected)
                    g_ctx.stats.frames_dropped[i] +=
                        (uint32_t)(meta.sequence - expected);
            }
            cam->last_sequence = meta.sequence;
            cam->have_sequence = true;

            dequeued_idx[i] = buf_idx;
            internal_buffer_t *buf = &cam->buffers[buf_idx];

            chronos_frame_t *f = &frame_set.frames[i];
            f->data      = buf->cpu_ptr;
            f->cuda_ptr  = NULL;    /* filled by chronos_get_cuda_ptr() */
            f->width     = CHRONOS_FRAME_WIDTH;
            f->height    = CHRONOS_FRAME_HEIGHT;
            f->pitch     = cam->bytesperline;
            f->size      = cam->sizeimage;
            f->meta            = meta;
            f->meta.camera_id  = (uint32_t)i;
            f->meta.vc_id      = (uint32_t)i;
            f->meta.exposure_us = g_ctx.config.exposure_us;
            f->meta.gain_db     = g_ctx.config.gain_db;

            if (meta.timestamp_ns < min_ts) min_ts = meta.timestamp_ns;
            if (meta.timestamp_ns > max_ts) max_ts = meta.timestamp_ns;
            if (first_cam < 0) first_cam = i;

            ++frames_received;
            ++g_ctx.stats.frames_captured[i];
        }

        if (frames_received == 0)
            continue;

        /* IMU sample (best-effort, skipped entirely when disabled so it
         * adds no per-frame latency). */
        if (g_ctx.config.enable_imu && g_ctx.imu.enabled) {
            read_imu_data();
            pthread_mutex_lock(&g_ctx.imu.lock);
            frame_set.imu = g_ctx.imu.latest_data;
            pthread_mutex_unlock(&g_ctx.imu.lock);
        }

        /* Per-set sync validity: every timestamp within half a frame
         * period, ANDed with the FPGA's cam_sync_status when readable. */
        uint64_t spread_ns = max_ts - min_ts;
        bool sync_ok = (spread_ns <= half_period_ns) &&
                       (!g_ctx.fpga_status_known || g_ctx.fpga_sync_ok);
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) {
            if (dequeued_idx[i] >= 0)
                frame_set.frames[i].meta.sync_valid = sync_ok;
        }

        frame_set.complete = (frames_received == active_count);
        /* Sequence comes from the V4L2 driver (camera 0 preferred). */
        frame_set.sync_sequence = (dequeued_idx[0] >= 0)
                                  ? frame_set.frames[0].meta.sequence
                                  : frame_set.frames[first_cam].meta.sequence;
        frame_set.sync_timestamp_ns = min_ts;

        if (!frame_set.complete)
            ++g_ctx.stats.sync_errors;

        /* Stats: skew / latency / measured frame rate. */
        float skew_us = (float)(spread_ns / 1000.0);
        if (skew_us > g_ctx.stats.max_sync_skew_us)
            g_ctx.stats.max_sync_skew_us = skew_us;

        uint64_t now_ns = monotonic_ns();
        float latency_us =
            (float)((now_ns > min_ts ? now_ns - min_ts : 0) / 1000.0);
        g_ctx.stats.avg_latency_us = (g_ctx.stats.avg_latency_us > 0.0f)
            ? g_ctx.stats.avg_latency_us * 0.9f + latency_us * 0.1f
            : latency_us;

        ++g_ctx.fps_window_frames;
        if (g_ctx.fps_window_start_ns == 0)
            g_ctx.fps_window_start_ns = now_ns;
        uint64_t window_ns = now_ns - g_ctx.fps_window_start_ns;
        if (window_ns >= 1000000000ULL) {
            g_ctx.stats.current_fps = (uint32_t)
                (((uint64_t)g_ctx.fps_window_frames * 1000000000ULL +
                  window_ns / 2) / window_ns);
            g_ctx.fps_window_frames = 0;
            g_ctx.fps_window_start_ns = now_ns;
            refresh_fpga_sync_status(active_mask);
        }

        /* Deliver.  The dequeued buffers now belong to the consumer;
         * chronos_release_frame_set() re-queues them.  Nothing is
         * re-queued here, so the consumer can never observe a buffer
         * being overwritten while it still holds the frame set. */
        pthread_mutex_lock(&g_ctx.mutex);
        if (!register_inflight_locked(&frame_set, dequeued_idx)) {
            /* Consumer is sitting on every buffer — drop this set
             * gracefully instead of corrupting held frames. */
            ++g_ctx.stats.buffer_overruns;
            for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) {
                if (dequeued_idx[i] >= 0)
                    queue_buffer(&g_ctx.cameras[i], dequeued_idx[i]);
            }
            pthread_mutex_unlock(&g_ctx.mutex);
            continue;
        }

        if (g_ctx.callback) {
            pthread_mutex_unlock(&g_ctx.mutex);
            g_ctx.callback(&frame_set, g_ctx.callback_user_data);
            /* Callback must call chronos_release_frame_set(). */
        } else {
            if (g_ctx.pending_frame_set) {
                /* Consumer too slow — reclaim the undelivered set's
                 * buffers before dropping it. */
                inflight_set_t *old =
                    find_inflight_locked(g_ctx.pending_frame_set);
                if (old)
                    requeue_inflight_locked(old);
                free(g_ctx.pending_frame_set);
                g_ctx.pending_frame_set = NULL;
                ++g_ctx.stats.buffer_overruns;
            }
            g_ctx.pending_frame_set = malloc(sizeof(frame_set));
            if (g_ctx.pending_frame_set) {
                *g_ctx.pending_frame_set = frame_set;
                g_ctx.frame_available    = true;
                pthread_cond_signal(&g_ctx.frame_ready_cond);
            } else {
                /* Allocation failed — reclaim immediately. */
                inflight_set_t *self = find_inflight_locked(&frame_set);
                if (self)
                    requeue_inflight_locked(self);
            }
            pthread_mutex_unlock(&g_ctx.mutex);
        }
    }

    return NULL;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

chronos_error_t chronos_init(void)
{
    chronos_error_t ret;

    if (g_ctx.initialized) {
        return CHRONOS_ERROR_INIT;
    }

    memset(&g_ctx, 0, sizeof(g_ctx));

    g_ctx.fpga_i2c_fd = -1;
    g_ctx.imu.iio_fd = -1;
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        g_ctx.cameras[i].fd = -1;
    }

    /* Default configuration */
    g_ctx.config.frame_rate = 30;
    g_ctx.config.exposure_us = 5000;
    g_ctx.config.gain_db = 0.0f;
    g_ctx.config.external_trigger = true;
    g_ctx.config.enable_imu = true;
    g_ctx.config.buffer_count = CHRONOS_BUFFER_COUNT;
    g_ctx.config.fsin_pulse_width_cycles = 0;   /* 0 = default (2000 cycles) */

    pthread_mutex_init(&g_ctx.mutex, NULL);
    pthread_cond_init(&g_ctx.frame_ready_cond, NULL);
    pthread_mutex_init(&g_ctx.i2c_lock, NULL);

#ifdef CHRONOS_WITH_CUDA
    /* CUDA is optional: failure only disables the device-copy path. */
    if (cudaSetDevice(0) == cudaSuccess) {
        g_ctx.cuda_enabled = true;
    } else {
        fprintf(stderr, "CUDA unavailable — device-copy path disabled\n");
        g_ctx.cuda_enabled = false;
    }
#endif

    /* FPGA control link (non-fatal if absent) and identity check. */
    if (fpga_i2c_open() == 0) {
        uint8_t version = 0, id_l = 0, id_h = 0;
        if (fpga_i2c_read(FPGA_REG_VERSION, &version) == 0 &&
            fpga_i2c_read(FPGA_REG_ID_L, &id_l) == 0 &&
            fpga_i2c_read(FPGA_REG_ID_H, &id_h) == 0) {
            if (version != FPGA_VERSION_EXPECTED ||
                id_l != FPGA_ID_L_EXPECTED ||
                id_h != FPGA_ID_H_EXPECTED) {
                fprintf(stderr,
                        "Warning: unexpected FPGA identity "
                        "(VERSION=0x%02X ID=0x%02X%02X, expected "
                        "VERSION=0x10 ID=0xC405) — continuing anyway.\n",
                        version, id_h, id_l);
            }
        } else {
            fprintf(stderr, "Warning: could not read FPGA ID registers.\n");
        }
    } else {
        fprintf(stderr,
                "Warning: FPGA I2C unreachable — FPGA control disabled.\n");
    }

    /* Initialize cameras */
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        ret = init_camera(i);
        if (ret != CHRONOS_OK) {
            fprintf(stderr, "Failed to init camera %d\n", i);
            /* Unwind everything initialized so far — no leaked fds,
             * mappings or device allocations on partial failure. */
            for (int j = 0; j <= i; j++) {
                cleanup_camera(j);
            }
            fpga_i2c_close();
            pthread_mutex_destroy(&g_ctx.mutex);
            pthread_cond_destroy(&g_ctx.frame_ready_cond);
            pthread_mutex_destroy(&g_ctx.i2c_lock);
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

    /* Close cameras (unmaps buffers, frees CUDA copies) */
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        cleanup_camera(i);
    }

    /* Close IMU */
    if (g_ctx.imu.iio_fd >= 0) {
        close(g_ctx.imu.iio_fd);
        g_ctx.imu.iio_fd = -1;
    }
    pthread_mutex_destroy(&g_ctx.imu.lock);

    fpga_i2c_close();

    pthread_mutex_destroy(&g_ctx.mutex);
    pthread_cond_destroy(&g_ctx.frame_ready_cond);
    pthread_mutex_destroy(&g_ctx.i2c_lock);

    g_ctx.initialized = false;

    printf("Chronos capture system shutdown\n");
}

/*
 * Apply the cached configuration to the FPGA register bank over I2C.
 *
 * The user-space library pushes a config update before streaming starts
 * (e.g. set the frame rate before VIDIOC_STREAMON) using the persistent
 * bus fd opened at chronos_init().
 */
static chronos_error_t push_config_to_fpga(const chronos_config_t *cfg)
{
    uint32_t fps = cfg->frame_rate ? cfg->frame_rate : 30;
    if (fps > 120) fps = 120;

    /*
     * FSIN pulse width in fabric (clk_sys = 192 MHz) cycles.  The sensor's
     * own exposure register controls integration time — the pulse only
     * needs to satisfy OV9281's minimum width spec, so a fixed default of
     * 2000 cycles (~10.4 us) is used unless the caller overrides it via
     * fsin_pulse_width_cycles.
     */
    uint32_t pulse_cycles = cfg->fsin_pulse_width_cycles
                            ? cfg->fsin_pulse_width_cycles : 2000;
    if (pulse_cycles > 0xFFFF) pulse_cycles = 0xFFFF;

    /* Enable exactly the cameras that opened successfully. */
    uint8_t cam_mask = 0;
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; ++i) {
        if (g_ctx.cameras[i].fd > 0)
            cam_mask |= (uint8_t)(1u << i);
    }

    if (fpga_i2c_write(FPGA_REG_FRAME_RATE,    (uint8_t)fps) < 0)
        return CHRONOS_ERROR_INIT;
    if (fpga_i2c_write(FPGA_REG_PULSE_WIDTH_L, (uint8_t)(pulse_cycles & 0xFF)) < 0)
        return CHRONOS_ERROR_INIT;
    if (fpga_i2c_write(FPGA_REG_PULSE_WIDTH_H, (uint8_t)((pulse_cycles >> 8) & 0xFF)) < 0)
        return CHRONOS_ERROR_INIT;
    if (fpga_i2c_write(FPGA_REG_CAM_ENABLE,    cam_mask) < 0)
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

    /* Buffers are allocated (and mmap()ed) once at init — the count cannot
     * change afterwards.  0 means "keep the current value". */
    if (config->buffer_count != 0 &&
        config->buffer_count != g_ctx.config.buffer_count) {
        return CHRONOS_ERROR_PARAM;
    }

    uint32_t buffer_count = g_ctx.config.buffer_count;
    g_ctx.config = *config;
    g_ctx.config.buffer_count = buffer_count;

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
        camera_context_t *cam = &g_ctx.cameras[i];
        if (cam->fd <= 0)
            continue;
        struct v4l2_control ctrl;
        ctrl.id = V4L2_CID_EXPOSURE;
        /* OV9281 exposure unit ≈ line time ~= 13.7 us @ 120 fps.
         * exposure_us / line_us → integer line count. */
        ctrl.value = (int32_t)(g_ctx.config.exposure_us / 14);
        (void)ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl);

        /* Gain: only when the driver exposes V4L2_CID_GAIN (queried at
         * init); skipped silently otherwise.  Control units are driver-
         * defined, so clamp the requested dB value into the advertised
         * range as a best-effort mapping. */
        if (cam->has_gain_ctrl) {
            int32_t v = (int32_t)(g_ctx.config.gain_db + 0.5f);
            if (v < cam->gain_min) v = cam->gain_min;
            if (v > cam->gain_max) v = cam->gain_max;
            ctrl.id = V4L2_CID_GAIN;
            ctrl.value = v;
            (void)ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl);
        }
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

/* Roll back a partially-started capture: trigger off, STREAMOFF every
 * streaming camera, buffer states reset.  Shared by the start_capture error
 * paths and stop_capture so a failed start never leaves hardware running. */
static void teardown_streams(void)
{
    if (fpga_i2c_write(FPGA_REG_CTRL, 0x00) < 0 &&
        g_ctx.config.external_trigger) {
        fprintf(stderr, "Warning: could not disable the FPGA trigger.\n");
    }

    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        camera_context_t *cam = &g_ctx.cameras[i];
        if (cam->fd <= 0) continue;

        /* STREAMOFF unconditionally: it also flushes buffers that were
         * QBUF'd before a failed STREAMON (cam->streaming still false). */
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(cam->fd, VIDIOC_STREAMOFF, &type);
        cam->streaming = false;

        for (uint32_t j = 0; j < cam->buf_count; j++) {
            cam->buffers[j].state = BUF_STATE_FREE;
        }
    }
}

chronos_error_t chronos_start_capture(void)
{
    if (!g_ctx.initialized) return CHRONOS_ERROR_INIT;
    if (g_ctx.running) return CHRONOS_OK;

    /* Fresh run: no in-flight sets, fresh sequence/fps tracking. */
    for (int s = 0; s < CHRONOS_MAX_INFLIGHT_SETS; ++s) {
        g_ctx.inflight[s].valid = false;
    }
    g_ctx.fps_window_start_ns = 0;
    g_ctx.fps_window_frames = 0;

    /* Queue initial buffers */
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        camera_context_t *cam = &g_ctx.cameras[i];
        if (cam->fd <= 0) continue;

        cam->have_sequence = false;

        for (uint32_t j = 0; j < cam->buf_count; j++) {
            cam->buffers[j].state = BUF_STATE_FREE;
            if (queue_buffer(cam, j) != CHRONOS_OK) {
                fprintf(stderr, "Failed to queue buffer %u on cam %d\n", j, i);
                teardown_streams();
                return CHRONOS_ERROR_CAPTURE;
            }
        }

        /* Start streaming */
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(cam->fd, VIDIOC_STREAMON, &type) < 0) {
            fprintf(stderr, "Failed to start streaming on cam %d\n", i);
            teardown_streams();
            return CHRONOS_ERROR_CAPTURE;
        }

        cam->streaming = true;
    }

    /* Gate the FPGA FSIN trigger (CTRL bit0).  The remaining bits are
     * reserved and bit1 (soft reset) is self-clearing, so a plain write
     * is safe — no read-modify-write needed. */
    uint8_t ctrl = g_ctx.config.external_trigger ? FPGA_CTRL_TRIGGER_ENABLE : 0x00;
    if (fpga_i2c_write(FPGA_REG_CTRL, ctrl) < 0 &&
        g_ctx.config.external_trigger) {
        fprintf(stderr, "Warning: could not enable the FPGA trigger.\n");
    }

    /* Start capture thread */
    g_ctx.running = true;
    if (pthread_create(&g_ctx.capture_thread, NULL, capture_thread_func, NULL) != 0) {
        g_ctx.running = false;
        teardown_streams();
        return CHRONOS_ERROR_INIT;
    }

    printf("Capture started\n");

    return CHRONOS_OK;
}

chronos_error_t chronos_stop_capture(void)
{
    bool any_streaming = false;
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        if (g_ctx.cameras[i].streaming) any_streaming = true;
    }
    /* Tear down whenever anything is live, not only when the capture thread
     * runs - a failed start_capture must still be cleanable. */
    if (!g_ctx.running && !any_streaming) return CHRONOS_OK;

    if (g_ctx.running) {
        g_ctx.running = false;
        pthread_join(g_ctx.capture_thread, NULL);
    }

    /* Trigger off first (no new frames), then STREAMOFF + buffer reset. */
    teardown_streams();

    /* Drop any undelivered frame set and forget in-flight bookkeeping. */
    pthread_mutex_lock(&g_ctx.mutex);
    if (g_ctx.pending_frame_set) {
        free(g_ctx.pending_frame_set);
        g_ctx.pending_frame_set = NULL;
    }
    g_ctx.frame_available = false;
    for (int s = 0; s < CHRONOS_MAX_INFLIGHT_SETS; ++s) {
        g_ctx.inflight[s].valid = false;
    }
    pthread_cond_broadcast(&g_ctx.frame_ready_cond);
    pthread_mutex_unlock(&g_ctx.mutex);

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

    struct timespec deadline;
    if (timeout_ms >= 0) {
        clock_gettime(CLOCK_REALTIME, &deadline);
        deadline.tv_sec += timeout_ms / 1000;
        deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
        if (deadline.tv_nsec >= 1000000000L) {
            deadline.tv_sec += 1;
            deadline.tv_nsec -= 1000000000L;
        }
    }

    /* Loop to survive spurious wakeups — only return once a frame set is
     * really pending (or the wait genuinely timed out). */
    while (!(g_ctx.frame_available && g_ctx.pending_frame_set)) {
        int ret;
        if (timeout_ms < 0) {
            ret = pthread_cond_wait(&g_ctx.frame_ready_cond, &g_ctx.mutex);
        } else {
            ret = pthread_cond_timedwait(&g_ctx.frame_ready_cond,
                                         &g_ctx.mutex, &deadline);
        }
        if (ret == ETIMEDOUT) {
            pthread_mutex_unlock(&g_ctx.mutex);
            return CHRONOS_ERROR_TIMEOUT;
        }
        if (!g_ctx.running) {
            pthread_mutex_unlock(&g_ctx.mutex);
            return CHRONOS_ERROR_CAPTURE;
        }
    }

    *frame_set = *g_ctx.pending_frame_set;
    free(g_ctx.pending_frame_set);
    g_ctx.pending_frame_set = NULL;
    g_ctx.frame_available = false;

    pthread_mutex_unlock(&g_ctx.mutex);

    return CHRONOS_OK;
}

chronos_error_t chronos_release_frame_set(chronos_sync_frame_set_t *frame_set)
{
    if (!frame_set) return CHRONOS_ERROR_PARAM;

    pthread_mutex_lock(&g_ctx.mutex);
    inflight_set_t *entry = find_inflight_locked(frame_set);
    if (entry) {
        requeue_inflight_locked(entry);
    }
    pthread_mutex_unlock(&g_ctx.mutex);

    /* No entry means the buffers were already reclaimed (e.g. by
     * chronos_stop_capture()) — nothing left to do. */
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
#ifdef CHRONOS_WITH_CUDA
    if (!frame || !cuda_ptr) return CHRONOS_ERROR_PARAM;
    if (!g_ctx.cuda_enabled) return CHRONOS_ERROR_CUDA;
    if (!frame->data || frame->meta.camera_id >= CHRONOS_NUM_CAMERAS)
        return CHRONOS_ERROR_PARAM;

    /* Find the internal buffer backing this frame */
    camera_context_t *cam = &g_ctx.cameras[frame->meta.camera_id];
    for (uint32_t j = 0; j < cam->buf_count; j++) {
        internal_buffer_t *buf = &cam->buffers[j];
        if (buf->cpu_ptr != frame->data) continue;
        if (!buf->cuda_ptr) return CHRONOS_ERROR_CUDA;

        if (!buf->cuda_valid) {
            /* Upload honoring the V4L2 line pitch; the device copy is
             * tightly packed (pitch = width * 2 bytes). */
            cudaError_t err = cudaMemcpy2D(
                buf->cuda_ptr,
                (size_t)frame->width * CHRONOS_FRAME_BPP,
                buf->cpu_ptr,
                frame->pitch,
                (size_t)frame->width * CHRONOS_FRAME_BPP,
                frame->height,
                cudaMemcpyHostToDevice);
            if (err != cudaSuccess) {
                fprintf(stderr, "CUDA frame upload failed: %s\n",
                        cudaGetErrorString(err));
                return CHRONOS_ERROR_CUDA;
            }
            buf->cuda_valid = true;
        }

        frame->cuda_ptr = buf->cuda_ptr;
        *cuda_ptr = buf->cuda_ptr;
        return CHRONOS_OK;
    }

    return CHRONOS_ERROR_PARAM;
#else
    (void)frame;
    (void)cuda_ptr;
    return CHRONOS_ERROR_CUDA;
#endif
}

chronos_error_t chronos_cuda_map(chronos_frame_t *frame)
{
    void *ptr;
    return chronos_get_cuda_ptr(frame, &ptr);
}

chronos_error_t chronos_cuda_unmap(chronos_frame_t *frame)
{
    if (!frame) return CHRONOS_ERROR_PARAM;

    /* The device copy stays allocated for buffer reuse; just drop the
     * cached pointer.  It is refreshed on the next chronos_get_cuda_ptr(). */
    frame->cuda_ptr = NULL;

    return CHRONOS_OK;
}

chronos_error_t chronos_get_imu_data(chronos_imu_data_t *imu_data)
{
    if (!imu_data) return CHRONOS_ERROR_PARAM;
    if (!g_ctx.imu.enabled) return CHRONOS_ERROR_INIT;

    /* When per-frame sampling is off (config.enable_imu = false) the
     * capture thread never refreshes latest_data — sample on demand so
     * this standalone accessor still returns fresh values. */
    if (!g_ctx.config.enable_imu)
        read_imu_data();

    pthread_mutex_lock(&g_ctx.imu.lock);
    *imu_data = g_ctx.imu.latest_data;
    pthread_mutex_unlock(&g_ctx.imu.lock);

    return CHRONOS_OK;
}

chronos_error_t chronos_get_stats(chronos_stats_t *stats)
{
    if (!stats) return CHRONOS_ERROR_PARAM;

    *stats = g_ctx.stats;

    /* Live FPGA health registers (0 when the I2C link is unavailable). */
    uint8_t v;
    stats->fpga_status = (fpga_i2c_read(FPGA_REG_STATUS, &v) == 0) ? v : 0;
    stats->fpga_error  = (fpga_i2c_read(FPGA_REG_ERROR,  &v) == 0) ? v : 0;

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
