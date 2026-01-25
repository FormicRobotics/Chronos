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

static void read_imu_data(void)
{
    imu_context_t *imu = &g_ctx.imu;
    
    if (!imu->enabled) return;
    
    /* Read from IIO sysfs */
    char path[256];
    char buf[64];
    int fd;
    
    pthread_mutex_lock(&imu->lock);
    
    /* Read accelerometer */
    for (int i = 0; i < 3; i++) {
        const char *axis[] = {"x", "y", "z"};
        snprintf(path, sizeof(path), "%s/in_accel_%s_raw", imu->dev_path, axis[i]);
        fd = open(path, O_RDONLY);
        if (fd >= 0) {
            if (read(fd, buf, sizeof(buf)) > 0) {
                int raw = atoi(buf);
                /* Convert to m/s^2 (scale factor depends on range) */
                imu->latest_data.accel[i] = raw * 0.000598f * 9.80665f;
            }
            close(fd);
        }
    }
    
    /* Read gyroscope */
    for (int i = 0; i < 3; i++) {
        const char *axis[] = {"x", "y", "z"};
        snprintf(path, sizeof(path), "%s/in_anglvel_%s_raw", imu->dev_path, axis[i]);
        fd = open(path, O_RDONLY);
        if (fd >= 0) {
            if (read(fd, buf, sizeof(buf)) > 0) {
                int raw = atoi(buf);
                /* Convert to rad/s */
                imu->latest_data.gyro[i] = raw * 0.001065f;
            }
            close(fd);
        }
    }
    
    imu->latest_data.timestamp_ns = 0;  /* TODO: Read from sync attribute */
    
    pthread_mutex_unlock(&imu->lock);
}

/* ============================================================================
 * Capture Thread
 * ============================================================================ */

static void *capture_thread_func(void *arg)
{
    (void)arg;
    
    fd_set fds;
    struct timeval tv;
    int max_fd = 0;
    
    /* Find max fd */
    for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
        if (g_ctx.cameras[i].fd > max_fd) {
            max_fd = g_ctx.cameras[i].fd;
        }
    }
    
    while (g_ctx.running) {
        FD_ZERO(&fds);
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
            FD_SET(g_ctx.cameras[i].fd, &fds);
        }
        
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        
        int ret = select(max_fd + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }
        
        if (ret == 0) continue;  /* Timeout */
        
        /* Dequeue frames from all ready cameras */
        chronos_sync_frame_set_t frame_set = {0};
        int frames_received = 0;
        
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
            if (FD_ISSET(g_ctx.cameras[i].fd, &fds)) {
                int buf_idx;
                chronos_frame_meta_t meta;
                
                if (dequeue_buffer(&g_ctx.cameras[i], &buf_idx, &meta) == CHRONOS_OK) {
                    internal_buffer_t *buf = &g_ctx.cameras[i].buffers[buf_idx];
                    
                    frame_set.frames[i].dmabuf_fd = buf->dmabuf_fd;
                    frame_set.frames[i].nvbuf_fd = buf->nvbuf_fd;
                    frame_set.frames[i].cuda_ptr = buf->cuda_ptr;
                    frame_set.frames[i].width = CHRONOS_FRAME_WIDTH;
                    frame_set.frames[i].height = CHRONOS_FRAME_HEIGHT;
                    frame_set.frames[i].pitch = CHRONOS_FRAME_WIDTH * CHRONOS_FRAME_BPP;
                    frame_set.frames[i].size = CHRONOS_FRAME_WIDTH * CHRONOS_FRAME_HEIGHT * CHRONOS_FRAME_BPP;
                    frame_set.frames[i].meta = meta;
                    frame_set.frames[i].meta.camera_id = i;
                    frame_set.frames[i].meta.vc_id = i;
                    
                    frames_received++;
                    g_ctx.stats.frames_captured[i]++;
                }
            }
        }
        
        /* Read IMU data */
        read_imu_data();
        pthread_mutex_lock(&g_ctx.imu.lock);
        frame_set.imu = g_ctx.imu.latest_data;
        pthread_mutex_unlock(&g_ctx.imu.lock);
        
        frame_set.complete = (frames_received == CHRONOS_NUM_CAMERAS);
        frame_set.sync_sequence = g_ctx.cameras[0].sequence++;
        
        /* Deliver frame set */
        if (g_ctx.callback) {
            g_ctx.callback(&frame_set, g_ctx.callback_user_data);
        } else {
            pthread_mutex_lock(&g_ctx.mutex);
            if (g_ctx.pending_frame_set) {
                /* Overwrite old frame set */
                g_ctx.stats.buffer_overruns++;
            }
            g_ctx.pending_frame_set = malloc(sizeof(frame_set));
            if (g_ctx.pending_frame_set) {
                *g_ctx.pending_frame_set = frame_set;
                g_ctx.frame_available = true;
                pthread_cond_signal(&g_ctx.frame_ready_cond);
            }
            pthread_mutex_unlock(&g_ctx.mutex);
        }
        
        /* Re-queue buffers */
        for (int i = 0; i < CHRONOS_NUM_CAMERAS; i++) {
            /* Queue next available buffer */
            for (uint32_t j = 0; j < g_ctx.cameras[i].buf_count; j++) {
                if (g_ctx.cameras[i].buffers[j].state == BUF_STATE_FREE) {
                    queue_buffer(&g_ctx.cameras[i], j);
                    break;
                }
            }
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

chronos_error_t chronos_configure(const chronos_config_t *config)
{
    if (!config) return CHRONOS_ERROR_PARAM;
    if (!g_ctx.initialized) return CHRONOS_ERROR_INIT;
    if (g_ctx.running) return CHRONOS_ERROR_CAPTURE;
    
    g_ctx.config = *config;
    
    /* TODO: Apply configuration to FPGA and sensors */
    
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
