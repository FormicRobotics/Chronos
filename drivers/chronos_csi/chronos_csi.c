// SPDX-License-Identifier: GPL-2.0-only
/*
 * Chronos Multi-Camera CSI Driver
 *
 * Copyright (C) 2025 Chronos Project
 *
 * This driver interfaces with the NVIDIA NVCSI subsystem and
 * demultiplexes virtual channel streams to separate video devices.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>

#define CHRONOS_DRIVER_NAME     "chronos-csi"
#define CHRONOS_MAX_CAMERAS     4
#define CHRONOS_BUF_COUNT       4

/* Virtual Channel IDs */
#define VC_CAM0     0
#define VC_CAM1     1
#define VC_CAM2     2
#define VC_CAM3     3

/* Frame format */
#define CHRONOS_WIDTH       1280
#define CHRONOS_HEIGHT      800
#define CHRONOS_BPP         2       /* 10-bit packed to 16-bit */
#define CHRONOS_FRAME_SIZE  (CHRONOS_WIDTH * CHRONOS_HEIGHT * CHRONOS_BPP)

/* Chronos FPGA I2C configuration */
#define CHRONOS_FPGA_I2C_ADDR   0x3C

/* FPGA Register offsets */
#define FPGA_REG_CTRL           0x00
#define FPGA_REG_FRAME_RATE     0x01
#define FPGA_REG_STATUS         0x20
#define FPGA_REG_ERROR          0x21

struct chronos_buffer {
    struct vb2_v4l2_buffer vb;
    struct list_head list;
    dma_addr_t dma_addr;
};

struct chronos_channel {
    struct video_device vdev;
    struct vb2_queue vb2_queue;
    struct v4l2_pix_format pix_fmt;
    
    struct mutex lock;
    spinlock_t qlock;
    
    struct list_head buf_list;
    struct chronos_buffer *active_buf;
    
    unsigned int sequence;
    bool streaming;
    
    int vc_id;  /* Virtual Channel ID */
    struct chronos_dev *parent;
};

struct chronos_dev {
    struct device *dev;
    struct platform_device *pdev;
    
    struct v4l2_device v4l2_dev;
    struct media_device mdev;
    
    /* I2C client for FPGA communication */
    struct i2c_client *fpga_client;
    
    /* Per-camera channels */
    struct chronos_channel channels[CHRONOS_MAX_CAMERAS];
    
    /* Clocks */
    struct clk *clk_csi;
    
    /* Statistics */
    u64 frame_count[CHRONOS_MAX_CAMERAS];
    u64 error_count;
    
    /* Synchronization tracking */
    ktime_t last_trigger_time;
    bool sync_active;
};

/* ============================================================================
 * FPGA Communication
 * ============================================================================ */

static int chronos_fpga_read_reg(struct chronos_dev *cdev, u8 reg, u8 *val)
{
    struct i2c_msg msgs[2];
    int ret;
    
    if (!cdev->fpga_client)
        return -ENODEV;
    
    msgs[0].addr = cdev->fpga_client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;
    
    msgs[1].addr = cdev->fpga_client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = val;
    
    ret = i2c_transfer(cdev->fpga_client->adapter, msgs, 2);
    return (ret == 2) ? 0 : -EIO;
}

static int chronos_fpga_write_reg(struct chronos_dev *cdev, u8 reg, u8 val)
{
    u8 buf[2] = { reg, val };
    int ret;
    
    if (!cdev->fpga_client)
        return -ENODEV;
    
    ret = i2c_master_send(cdev->fpga_client, buf, 2);
    return (ret == 2) ? 0 : -EIO;
}

static int chronos_fpga_set_trigger_enable(struct chronos_dev *cdev, bool enable)
{
    u8 ctrl;
    int ret;
    
    ret = chronos_fpga_read_reg(cdev, FPGA_REG_CTRL, &ctrl);
    if (ret)
        return ret;
    
    if (enable)
        ctrl |= 0x01;
    else
        ctrl &= ~0x01;
    
    return chronos_fpga_write_reg(cdev, FPGA_REG_CTRL, ctrl);
}

static int chronos_fpga_set_frame_rate(struct chronos_dev *cdev, u8 fps)
{
    return chronos_fpga_write_reg(cdev, FPGA_REG_FRAME_RATE, fps);
}

/* ============================================================================
 * Video Buffer Operations
 * ============================================================================ */

static int chronos_queue_setup(struct vb2_queue *vq,
                               unsigned int *nbuffers,
                               unsigned int *nplanes,
                               unsigned int sizes[],
                               struct device *alloc_devs[])
{
    struct chronos_channel *ch = vb2_get_drv_priv(vq);
    
    if (*nplanes) {
        if (sizes[0] < CHRONOS_FRAME_SIZE)
            return -EINVAL;
        return 0;
    }
    
    *nplanes = 1;
    sizes[0] = CHRONOS_FRAME_SIZE;
    
    if (*nbuffers < CHRONOS_BUF_COUNT)
        *nbuffers = CHRONOS_BUF_COUNT;
    
    dev_dbg(ch->parent->dev, "VC%d: Queue setup: %d buffers, size %u\n",
            ch->vc_id, *nbuffers, sizes[0]);
    
    return 0;
}

static int chronos_buf_prepare(struct vb2_buffer *vb)
{
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct chronos_channel *ch = vb2_get_drv_priv(vb->vb2_queue);
    struct chronos_buffer *buf = container_of(vbuf, struct chronos_buffer, vb);
    
    if (vb2_plane_size(vb, 0) < CHRONOS_FRAME_SIZE) {
        dev_err(ch->parent->dev, "Buffer too small: %lu < %u\n",
                vb2_plane_size(vb, 0), CHRONOS_FRAME_SIZE);
        return -EINVAL;
    }
    
    vb2_set_plane_payload(vb, 0, CHRONOS_FRAME_SIZE);
    
    buf->dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
    
    return 0;
}

static void chronos_buf_queue(struct vb2_buffer *vb)
{
    struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
    struct chronos_channel *ch = vb2_get_drv_priv(vb->vb2_queue);
    struct chronos_buffer *buf = container_of(vbuf, struct chronos_buffer, vb);
    unsigned long flags;
    
    spin_lock_irqsave(&ch->qlock, flags);
    list_add_tail(&buf->list, &ch->buf_list);
    spin_unlock_irqrestore(&ch->qlock, flags);
}

static int chronos_start_streaming(struct vb2_queue *vq, unsigned int count)
{
    struct chronos_channel *ch = vb2_get_drv_priv(vq);
    struct chronos_dev *cdev = ch->parent;
    int ret;
    
    dev_info(cdev->dev, "VC%d: Start streaming\n", ch->vc_id);
    
    ch->sequence = 0;
    ch->streaming = true;
    
    /* Enable FPGA trigger if this is the first channel to stream */
    if (ch->vc_id == 0) {
        ret = chronos_fpga_set_trigger_enable(cdev, true);
        if (ret) {
            dev_err(cdev->dev, "Failed to enable trigger\n");
            ch->streaming = false;
            return ret;
        }
        cdev->sync_active = true;
    }
    
    return 0;
}

static void chronos_stop_streaming(struct vb2_queue *vq)
{
    struct chronos_channel *ch = vb2_get_drv_priv(vq);
    struct chronos_dev *cdev = ch->parent;
    struct chronos_buffer *buf, *tmp;
    unsigned long flags;
    
    dev_info(cdev->dev, "VC%d: Stop streaming\n", ch->vc_id);
    
    ch->streaming = false;
    
    /* Disable trigger if last channel */
    if (ch->vc_id == 0) {
        chronos_fpga_set_trigger_enable(cdev, false);
        cdev->sync_active = false;
    }
    
    /* Return all buffers */
    spin_lock_irqsave(&ch->qlock, flags);
    
    if (ch->active_buf) {
        vb2_buffer_done(&ch->active_buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
        ch->active_buf = NULL;
    }
    
    list_for_each_entry_safe(buf, tmp, &ch->buf_list, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    
    spin_unlock_irqrestore(&ch->qlock, flags);
}

static const struct vb2_ops chronos_vb2_ops = {
    .queue_setup     = chronos_queue_setup,
    .buf_prepare     = chronos_buf_prepare,
    .buf_queue       = chronos_buf_queue,
    .start_streaming = chronos_start_streaming,
    .stop_streaming  = chronos_stop_streaming,
    .wait_prepare    = vb2_ops_wait_prepare,
    .wait_finish     = vb2_ops_wait_finish,
};

/* ============================================================================
 * V4L2 IOCTLs
 * ============================================================================ */

static int chronos_querycap(struct file *file, void *priv,
                            struct v4l2_capability *cap)
{
    struct chronos_channel *ch = video_drvdata(file);
    
    strscpy(cap->driver, CHRONOS_DRIVER_NAME, sizeof(cap->driver));
    snprintf(cap->card, sizeof(cap->card), "Chronos Camera %d", ch->vc_id);
    snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:chronos-csi.%d",
             ch->vc_id);
    
    return 0;
}

static int chronos_enum_fmt(struct file *file, void *priv,
                            struct v4l2_fmtdesc *f)
{
    if (f->index > 0)
        return -EINVAL;
    
    f->pixelformat = V4L2_PIX_FMT_Y10;
    strscpy(f->description, "10-bit Greyscale", sizeof(f->description));
    
    return 0;
}

static int chronos_g_fmt(struct file *file, void *priv,
                         struct v4l2_format *f)
{
    struct chronos_channel *ch = video_drvdata(file);
    
    f->fmt.pix = ch->pix_fmt;
    
    return 0;
}

static int chronos_s_fmt(struct file *file, void *priv,
                         struct v4l2_format *f)
{
    struct chronos_channel *ch = video_drvdata(file);
    
    /* Fixed format - just return current */
    f->fmt.pix = ch->pix_fmt;
    
    return 0;
}

static int chronos_try_fmt(struct file *file, void *priv,
                           struct v4l2_format *f)
{
    f->fmt.pix.width = CHRONOS_WIDTH;
    f->fmt.pix.height = CHRONOS_HEIGHT;
    f->fmt.pix.pixelformat = V4L2_PIX_FMT_Y10;
    f->fmt.pix.field = V4L2_FIELD_NONE;
    f->fmt.pix.bytesperline = CHRONOS_WIDTH * CHRONOS_BPP;
    f->fmt.pix.sizeimage = CHRONOS_FRAME_SIZE;
    f->fmt.pix.colorspace = V4L2_COLORSPACE_RAW;
    
    return 0;
}

static int chronos_enum_input(struct file *file, void *priv,
                              struct v4l2_input *inp)
{
    struct chronos_channel *ch = video_drvdata(file);
    
    if (inp->index > 0)
        return -EINVAL;
    
    snprintf(inp->name, sizeof(inp->name), "Camera %d", ch->vc_id);
    inp->type = V4L2_INPUT_TYPE_CAMERA;
    inp->std = V4L2_STD_UNKNOWN;
    
    return 0;
}

static int chronos_g_input(struct file *file, void *priv, unsigned int *i)
{
    *i = 0;
    return 0;
}

static int chronos_s_input(struct file *file, void *priv, unsigned int i)
{
    return (i == 0) ? 0 : -EINVAL;
}

static int chronos_g_parm(struct file *file, void *priv,
                          struct v4l2_streamparm *sp)
{
    if (sp->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;
    
    sp->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    sp->parm.capture.timeperframe.numerator = 1;
    sp->parm.capture.timeperframe.denominator = 120;  /* 120fps max */
    sp->parm.capture.readbuffers = CHRONOS_BUF_COUNT;
    
    return 0;
}

static int chronos_s_parm(struct file *file, void *priv,
                          struct v4l2_streamparm *sp)
{
    struct chronos_channel *ch = video_drvdata(file);
    struct chronos_dev *cdev = ch->parent;
    u32 fps;
    
    if (sp->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;
    
    /* Calculate requested FPS */
    if (sp->parm.capture.timeperframe.numerator == 0)
        fps = 30;
    else
        fps = sp->parm.capture.timeperframe.denominator /
              sp->parm.capture.timeperframe.numerator;
    
    /* Clamp to valid range */
    if (fps < 1) fps = 1;
    if (fps > 120) fps = 120;
    
    /* Set in FPGA */
    chronos_fpga_set_frame_rate(cdev, fps);
    
    sp->parm.capture.timeperframe.numerator = 1;
    sp->parm.capture.timeperframe.denominator = fps;
    
    return 0;
}

static const struct v4l2_ioctl_ops chronos_ioctl_ops = {
    .vidioc_querycap         = chronos_querycap,
    .vidioc_enum_fmt_vid_cap = chronos_enum_fmt,
    .vidioc_g_fmt_vid_cap    = chronos_g_fmt,
    .vidioc_s_fmt_vid_cap    = chronos_s_fmt,
    .vidioc_try_fmt_vid_cap  = chronos_try_fmt,
    .vidioc_enum_input       = chronos_enum_input,
    .vidioc_g_input          = chronos_g_input,
    .vidioc_s_input          = chronos_s_input,
    .vidioc_g_parm           = chronos_g_parm,
    .vidioc_s_parm           = chronos_s_parm,
    .vidioc_reqbufs          = vb2_ioctl_reqbufs,
    .vidioc_querybuf         = vb2_ioctl_querybuf,
    .vidioc_qbuf             = vb2_ioctl_qbuf,
    .vidioc_dqbuf            = vb2_ioctl_dqbuf,
    .vidioc_expbuf           = vb2_ioctl_expbuf,
    .vidioc_streamon         = vb2_ioctl_streamon,
    .vidioc_streamoff        = vb2_ioctl_streamoff,
    .vidioc_subscribe_event  = v4l2_ctrl_subscribe_event,
    .vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* ============================================================================
 * File Operations
 * ============================================================================ */

static int chronos_open(struct file *file)
{
    struct chronos_channel *ch = video_drvdata(file);
    int ret;
    
    ret = v4l2_fh_open(file);
    if (ret)
        return ret;
    
    dev_dbg(ch->parent->dev, "VC%d: Device opened\n", ch->vc_id);
    
    return 0;
}

static int chronos_release(struct file *file)
{
    struct chronos_channel *ch = video_drvdata(file);
    
    dev_dbg(ch->parent->dev, "VC%d: Device closed\n", ch->vc_id);
    
    return vb2_fop_release(file);
}

static const struct v4l2_file_operations chronos_fops = {
    .owner          = THIS_MODULE,
    .open           = chronos_open,
    .release        = chronos_release,
    .unlocked_ioctl = video_ioctl2,
    .read           = vb2_fop_read,
    .mmap           = vb2_fop_mmap,
    .poll           = vb2_fop_poll,
};

/* ============================================================================
 * Channel Initialization
 * ============================================================================ */

static int chronos_init_channel(struct chronos_dev *cdev, int ch_num)
{
    struct chronos_channel *ch = &cdev->channels[ch_num];
    struct vb2_queue *vq = &ch->vb2_queue;
    int ret;
    
    ch->vc_id = ch_num;
    ch->parent = cdev;
    ch->streaming = false;
    
    mutex_init(&ch->lock);
    spin_lock_init(&ch->qlock);
    INIT_LIST_HEAD(&ch->buf_list);
    
    /* Set default format */
    ch->pix_fmt.width = CHRONOS_WIDTH;
    ch->pix_fmt.height = CHRONOS_HEIGHT;
    ch->pix_fmt.pixelformat = V4L2_PIX_FMT_Y10;
    ch->pix_fmt.field = V4L2_FIELD_NONE;
    ch->pix_fmt.bytesperline = CHRONOS_WIDTH * CHRONOS_BPP;
    ch->pix_fmt.sizeimage = CHRONOS_FRAME_SIZE;
    ch->pix_fmt.colorspace = V4L2_COLORSPACE_RAW;
    
    /* Initialize vb2 queue */
    vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vq->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
    vq->dev = cdev->dev;
    vq->drv_priv = ch;
    vq->buf_struct_size = sizeof(struct chronos_buffer);
    vq->ops = &chronos_vb2_ops;
    vq->mem_ops = &vb2_dma_contig_memops;
    vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    vq->min_buffers_needed = 2;
    vq->lock = &ch->lock;
    vq->gfp_flags = GFP_DMA32;
    
    ret = vb2_queue_init(vq);
    if (ret) {
        dev_err(cdev->dev, "Failed to init vb2 queue for VC%d\n", ch_num);
        return ret;
    }
    
    /* Initialize video device */
    snprintf(ch->vdev.name, sizeof(ch->vdev.name), "chronos-cam%d", ch_num);
    ch->vdev.fops = &chronos_fops;
    ch->vdev.ioctl_ops = &chronos_ioctl_ops;
    ch->vdev.release = video_device_release_empty;
    ch->vdev.v4l2_dev = &cdev->v4l2_dev;
    ch->vdev.queue = vq;
    ch->vdev.lock = &ch->lock;
    ch->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
                           V4L2_CAP_READWRITE;
    
    video_set_drvdata(&ch->vdev, ch);
    
    ret = video_register_device(&ch->vdev, VFL_TYPE_VIDEO, -1);
    if (ret) {
        dev_err(cdev->dev, "Failed to register video device for VC%d\n", ch_num);
        return ret;
    }
    
    dev_info(cdev->dev, "Registered /dev/video%d for Camera %d (VC%d)\n",
             ch->vdev.num, ch_num, ch_num);
    
    return 0;
}

static void chronos_cleanup_channel(struct chronos_channel *ch)
{
    if (video_is_registered(&ch->vdev))
        video_unregister_device(&ch->vdev);
    
    mutex_destroy(&ch->lock);
}

/* ============================================================================
 * Platform Driver
 * ============================================================================ */

static int chronos_probe(struct platform_device *pdev)
{
    struct chronos_dev *cdev;
    struct device *dev = &pdev->dev;
    int ret, i;
    
    dev_info(dev, "Chronos Multi-Camera CSI driver probing\n");
    
    cdev = devm_kzalloc(dev, sizeof(*cdev), GFP_KERNEL);
    if (!cdev)
        return -ENOMEM;
    
    cdev->dev = dev;
    cdev->pdev = pdev;
    platform_set_drvdata(pdev, cdev);
    
    /* Get clocks */
    cdev->clk_csi = devm_clk_get(dev, "csi");
    if (IS_ERR(cdev->clk_csi)) {
        dev_warn(dev, "CSI clock not found, using default\n");
        cdev->clk_csi = NULL;
    }
    
    /* Register V4L2 device */
    ret = v4l2_device_register(dev, &cdev->v4l2_dev);
    if (ret) {
        dev_err(dev, "Failed to register V4L2 device\n");
        return ret;
    }
    
    /* Initialize channels */
    for (i = 0; i < CHRONOS_MAX_CAMERAS; i++) {
        ret = chronos_init_channel(cdev, i);
        if (ret) {
            dev_err(dev, "Failed to init channel %d\n", i);
            goto err_cleanup;
        }
    }
    
    dev_info(dev, "Chronos CSI driver initialized with %d cameras\n",
             CHRONOS_MAX_CAMERAS);
    
    return 0;

err_cleanup:
    for (i--; i >= 0; i--)
        chronos_cleanup_channel(&cdev->channels[i]);
    v4l2_device_unregister(&cdev->v4l2_dev);
    
    return ret;
}

static int chronos_remove(struct platform_device *pdev)
{
    struct chronos_dev *cdev = platform_get_drvdata(pdev);
    int i;
    
    for (i = 0; i < CHRONOS_MAX_CAMERAS; i++)
        chronos_cleanup_channel(&cdev->channels[i]);
    
    v4l2_device_unregister(&cdev->v4l2_dev);
    
    dev_info(&pdev->dev, "Chronos CSI driver removed\n");
    
    return 0;
}

static const struct of_device_id chronos_of_match[] = {
    { .compatible = "chronos,multi-camera-csi" },
    { }
};
MODULE_DEVICE_TABLE(of, chronos_of_match);

static struct platform_driver chronos_driver = {
    .probe  = chronos_probe,
    .remove = chronos_remove,
    .driver = {
        .name = CHRONOS_DRIVER_NAME,
        .of_match_table = chronos_of_match,
    },
};

module_platform_driver(chronos_driver);

MODULE_AUTHOR("Chronos Project");
MODULE_DESCRIPTION("Chronos Multi-Camera MIPI CSI Driver");
MODULE_LICENSE("GPL v2");
