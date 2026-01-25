// SPDX-License-Identifier: GPL-2.0-only
/*
 * OmniVision OV9281 CMOS Image Sensor Driver
 *
 * Copyright (C) 2025 Chronos Project
 * Author: Chronos Development Team
 *
 * This driver provides V4L2 subdevice support for the OmniVision OV9281
 * global shutter image sensor. The OV9281 is a 1MP (1280x800) monochrome
 * sensor commonly used for machine vision, depth sensing, and applications
 * requiring distortion-free capture of moving objects.
 *
 * Key Features:
 *   - 1280x800 resolution with 3µm pixel size
 *   - Global shutter for motion-free imaging
 *   - MIPI CSI-2 interface (1 or 2 lanes)
 *   - Up to 120fps at full resolution
 *   - External trigger mode via FSIN pin
 *   - RAW8/RAW10 output formats
 *
 * Hardware Connections:
 *   - I2C for register access (address 0x60)
 *   - MIPI CSI-2 for image data
 *   - FSIN pin for external trigger
 *   - Power supplies: AVDD (2.8V), DOVDD (1.8V), DVDD (1.2V)
 *
 * Synchronization:
 *   For multi-camera sync, enable external trigger mode and connect
 *   the FSIN pins to a common trigger source. All cameras will capture
 *   on the rising edge of the trigger signal.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/*
 * =============================================================================
 * Register Definitions
 * =============================================================================
 * The OV9281 uses 16-bit register addresses with 8-bit values.
 * Registers are grouped by function.
 */

/* Chip Identification */
#define OV9281_REG_CHIP_ID_H        0x300A  /* Chip ID high byte */
#define OV9281_REG_CHIP_ID_L        0x300B  /* Chip ID low byte */
#define OV9281_CHIP_ID              0x9281  /* Expected chip ID */

/* System Control */
#define OV9281_REG_MODE_SELECT      0x0100  /* Streaming mode control */
#define OV9281_MODE_STANDBY         0x00    /* Standby (low power) */
#define OV9281_MODE_STREAMING       0x01    /* Streaming (active) */

#define OV9281_REG_SOFT_RESET       0x0103  /* Software reset */
#define OV9281_SOFT_RESET           0x01    /* Write to trigger reset */

/* Timing Control (frame rate and blanking) */
#define OV9281_REG_VTS_H            0x380E  /* Vertical total size (high) */
#define OV9281_REG_VTS_L            0x380F  /* Vertical total size (low) */
#define OV9281_REG_HTS_H            0x380C  /* Horizontal total size (high) */
#define OV9281_REG_HTS_L            0x380D  /* Horizontal total size (low) */

/* Exposure Control (16-bit exposure in line units) */
#define OV9281_REG_EXPOSURE_H       0x3500  /* Exposure [19:16] */
#define OV9281_REG_EXPOSURE_M       0x3501  /* Exposure [15:8] */
#define OV9281_REG_EXPOSURE_L       0x3502  /* Exposure [7:0] */

/* Gain Control (analog gain) */
#define OV9281_REG_GAIN_H           0x3508  /* Gain [10:8] */
#define OV9281_REG_GAIN_L           0x3509  /* Gain [7:0] */

/* External Trigger Mode (FSIN) */
#define OV9281_REG_TIMING_CTRL      0x3823  /* Timing format control */
#define OV9281_REG_STROBE_CTRL      0x3B00  /* Strobe control */
#define OV9281_REG_FSIN_CTRL        0x3826  /* FSIN control */

/* Test Pattern */
#define OV9281_REG_TEST_PATTERN     0x5080  /* Test pattern control */

/*
 * =============================================================================
 * Default Values and Limits
 * =============================================================================
 */

/* Pixel rate: 400 Mpix/s (2 lanes @ 800 Mbps, 10-bit) */
#define OV9281_PIXEL_RATE           400000000ULL

/* Input clock: 24MHz XVCLK */
#define OV9281_XVCLK_FREQ           24000000

/* MIPI link frequency: 400 MHz */
#define OV9281_LINK_FREQ            400000000ULL

/* Default resolution */
#define OV9281_DEFAULT_WIDTH        1280
#define OV9281_DEFAULT_HEIGHT       800

/* Exposure limits (in line units, 1 line ≈ 13.7µs at 120fps) */
#define OV9281_EXPOSURE_MIN         4       /* Minimum: ~55µs */
#define OV9281_EXPOSURE_MAX         0xFFF0  /* Maximum: ~894ms */
#define OV9281_EXPOSURE_DEFAULT     0x0100  /* Default: ~3.5ms */

/* Gain limits (1x to 15.5x analog gain) */
#define OV9281_GAIN_MIN             0x10    /* 1x gain */
#define OV9281_GAIN_MAX             0xF8    /* 15.5x gain */
#define OV9281_GAIN_DEFAULT         0x10    /* Default: 1x */

/*
 * =============================================================================
 * Power Supply Definitions
 * =============================================================================
 * The OV9281 requires three power rails with specific sequencing.
 */

static const char * const ov9281_supply_names[] = {
    "avdd",     /* Analog power 2.8V - powers analog circuits */
    "dovdd",    /* Digital I/O power 1.8V - powers I/O and MIPI */
    "dvdd",     /* Digital core power 1.2V - powers digital logic */
};

#define OV9281_NUM_SUPPLIES ARRAY_SIZE(ov9281_supply_names)

/*
 * =============================================================================
 * Mode Definitions
 * =============================================================================
 * Currently supports single mode: 1280x800 @ up to 120fps
 * Additional modes (binning, cropping) can be added here.
 */

struct ov9281_mode {
    u32 width;              /* Output width in pixels */
    u32 height;             /* Output height in pixels */
    u32 hts;                /* Horizontal total size */
    u32 vts;                /* Vertical total size (affects frame rate) */
    const struct reg_sequence *reg_list;    /* Mode-specific registers */
    unsigned int num_regs;                  /* Number of registers */
};

/*
 * =============================================================================
 * Driver Instance Structure
 * =============================================================================
 * Main structure holding all state for one sensor instance.
 */

struct ov9281 {
    /* Device references */
    struct i2c_client *client;      /* I2C client for register access */
    struct v4l2_subdev sd;          /* V4L2 subdevice */
    struct media_pad pad;           /* Media pad (source) */
    
    /* Hardware resources */
    struct clk *xvclk;              /* External clock (24MHz) */
    struct gpio_desc *reset_gpio;   /* Reset GPIO (active low) */
    struct gpio_desc *pwdn_gpio;    /* Power down GPIO (active low) */
    struct regulator_bulk_data supplies[OV9281_NUM_SUPPLIES];
    
    /* V4L2 controls */
    struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_ctrl *pixel_rate;   /* Read-only pixel rate */
    struct v4l2_ctrl *link_freq;    /* Read-only link frequency */
    struct v4l2_ctrl *exposure;     /* Exposure time control */
    struct v4l2_ctrl *gain;         /* Analog gain control */
    struct v4l2_ctrl *hblank;       /* Horizontal blanking (read-only) */
    struct v4l2_ctrl *vblank;       /* Vertical blanking (read-only) */
    struct v4l2_ctrl *test_pattern; /* Test pattern selection */
    struct v4l2_ctrl *trigger_mode; /* External trigger enable */
    
    /* State */
    struct mutex mutex;             /* Protects streaming and mode */
    bool streaming;                 /* Currently streaming */
    
    const struct ov9281_mode *cur_mode;  /* Active mode */
};

/*
 * =============================================================================
 * Register Sequence Structure
 * =============================================================================
 */

struct reg_sequence {
    u16 reg;    /* Register address */
    u8 val;     /* Register value */
};

/*
 * 1280x800 @ 120fps mode register settings
 *
 * This configuration sets up:
 * - Full resolution output (1280x800)
 * - 2-lane MIPI at 800 Mbps/lane
 * - RAW10 output format
 * - 120fps frame rate (VTS=910)
 * - Manual exposure/gain mode
 */
static const struct reg_sequence ov9281_1280x800_regs[] = {
    /* Software reset and standby */
    {0x0103, 0x01},  /* Software reset */
    {0x0100, 0x00},  /* Enter standby mode */
    
    /*
     * PLL Configuration
     * Input: 24MHz XVCLK
     * Output: 800 Mbps/lane MIPI clock
     */
    {0x0300, 0x00},
    {0x0301, 0x00},
    {0x0302, 0x32},  /* PLL multiplier */
    {0x0303, 0x00},
    {0x030A, 0x00},
    {0x030B, 0x02},
    {0x030C, 0x00},
    {0x030D, 0x50},
    
    /* System and I/O control */
    {0x3001, 0x00},
    {0x3004, 0x00},
    {0x3005, 0x00},
    {0x3006, 0x04},
    {0x3011, 0x0A},
    {0x3013, 0x18},
    
    /* MIPI interface: 2-lane mode */
    {0x3016, 0x1C},
    {0x3017, 0x00},
    {0x3018, 0x32},  /* 2 data lanes */
    {0x301A, 0xF0},
    
    /*
     * Image window settings
     * Active area: 1280x800
     * No cropping or binning
     */
    {0x3800, 0x00},  /* X start high */
    {0x3801, 0x00},  /* X start low */
    {0x3802, 0x00},  /* Y start high */
    {0x3803, 0x00},  /* Y start low */
    {0x3804, 0x05},  /* X end high (1295) */
    {0x3805, 0x0F},  /* X end low */
    {0x3806, 0x03},  /* Y end high (815) */
    {0x3807, 0x2F},  /* Y end low */
    {0x3808, 0x05},  /* Output width high (1280) */
    {0x3809, 0x00},  /* Output width low */
    {0x380A, 0x03},  /* Output height high (800) */
    {0x380B, 0x20},  /* Output height low */
    
    /*
     * Timing configuration
     * HTS = 728 (horizontal total)
     * VTS = 910 (vertical total)
     * Frame rate = 400MHz / (728 * 910) ≈ 120fps
     */
    {0x380C, 0x02},  /* HTS high */
    {0x380D, 0xD8},  /* HTS low (728) */
    {0x380E, 0x03},  /* VTS high */
    {0x380F, 0x8E},  /* VTS low (910) */
    
    /* Windowing offsets */
    {0x3810, 0x00},
    {0x3811, 0x08},
    {0x3812, 0x00},
    {0x3813, 0x08},
    
    /* Subsampling: no binning (1:1) */
    {0x3814, 0x11},
    {0x3815, 0x11},
    
    /* Black level calibration */
    {0x4000, 0xC3},
    {0x4001, 0x40},
    {0x4002, 0x00},
    {0x4003, 0x40},
    {0x4004, 0x00},
    {0x4005, 0x00},
    
    /* MIPI timing */
    {0x4800, 0x24},
    {0x4837, 0x14},  /* MIPI global timing */
    
    /* ISP control: minimal processing */
    {0x5000, 0x01},
    {0x5001, 0x00},
    {0x5002, 0x00},
    
    /* AEC/AGC: manual mode */
    {0x3503, 0x03},  /* Manual exposure and gain */
    
    /* Default exposure: ~3.5ms */
    {0x3500, 0x00},
    {0x3501, 0x01},
    {0x3502, 0x00},
    
    /* Default gain: 1x */
    {0x3508, 0x00},
    {0x3509, 0x10},
};

static const struct ov9281_mode ov9281_modes[] = {
    {
        .width = 1280,
        .height = 800,
        .hts = 728,
        .vts = 910,
        .reg_list = ov9281_1280x800_regs,
        .num_regs = ARRAY_SIZE(ov9281_1280x800_regs),
    },
};

/* MIPI link frequency options */
static const s64 link_freq_menu_items[] = {
    OV9281_LINK_FREQ,
};

/* Test pattern menu options */
static const char * const test_pattern_menu[] = {
    "Disabled",
    "Color Bar",
    "Color Bar FADE",
    "Random Data",
    "Square",
};

/* Trigger mode menu options */
static const char * const trigger_mode_menu[] = {
    "Free Running",             /* Internal timing */
    "External Trigger (FSIN)",  /* Sync to external pulse */
};

/*
 * =============================================================================
 * Helper Functions
 * =============================================================================
 */

/**
 * to_ov9281 - Get ov9281 structure from V4L2 subdev
 * @sd: V4L2 subdevice pointer
 *
 * Returns pointer to the enclosing ov9281 structure.
 */
static inline struct ov9281 *to_ov9281(struct v4l2_subdev *sd)
{
    return container_of(sd, struct ov9281, sd);
}

/*
 * =============================================================================
 * I2C Register Access
 * =============================================================================
 * The OV9281 uses 16-bit addresses and 8-bit data values.
 */

/**
 * ov9281_read_reg - Read a single register
 * @ov9281: Sensor instance
 * @reg: Register address (16-bit)
 * @val: Pointer to store read value
 *
 * Returns 0 on success, negative error code on failure.
 */
static int ov9281_read_reg(struct ov9281 *ov9281, u16 reg, u8 *val)
{
    struct i2c_client *client = ov9281->client;
    struct i2c_msg msgs[2];
    u8 addr_buf[2];
    int ret;
    
    /* First message: write register address */
    addr_buf[0] = (reg >> 8) & 0xFF;
    addr_buf[1] = reg & 0xFF;
    
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;          /* Write */
    msgs[0].len = 2;
    msgs[0].buf = addr_buf;
    
    /* Second message: read data */
    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;   /* Read */
    msgs[1].len = 1;
    msgs[1].buf = val;
    
    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret != 2) {
        dev_err(&client->dev, "I2C read reg 0x%04x failed: %d\n", reg, ret);
        return ret < 0 ? ret : -EIO;
    }
    
    return 0;
}

/**
 * ov9281_write_reg - Write a single register
 * @ov9281: Sensor instance
 * @reg: Register address (16-bit)
 * @val: Value to write (8-bit)
 *
 * Returns 0 on success, negative error code on failure.
 */
static int ov9281_write_reg(struct ov9281 *ov9281, u16 reg, u8 val)
{
    struct i2c_client *client = ov9281->client;
    u8 buf[3];
    int ret;
    
    buf[0] = (reg >> 8) & 0xFF;     /* Address high */
    buf[1] = reg & 0xFF;            /* Address low */
    buf[2] = val;                   /* Data */
    
    ret = i2c_master_send(client, buf, 3);
    if (ret != 3) {
        dev_err(&client->dev, "I2C write reg 0x%04x=0x%02x failed: %d\n",
                reg, val, ret);
        return ret < 0 ? ret : -EIO;
    }
    
    return 0;
}

/**
 * ov9281_write_regs - Write multiple registers from a sequence
 * @ov9281: Sensor instance
 * @regs: Array of register/value pairs
 * @num_regs: Number of entries in the array
 *
 * Writes all registers in sequence. Includes appropriate delays
 * after reset commands.
 *
 * Returns 0 on success, negative error code on first failure.
 */
static int ov9281_write_regs(struct ov9281 *ov9281,
                             const struct reg_sequence *regs,
                             unsigned int num_regs)
{
    int ret;
    unsigned int i;
    
    for (i = 0; i < num_regs; i++) {
        ret = ov9281_write_reg(ov9281, regs[i].reg, regs[i].val);
        if (ret)
            return ret;
        
        /* Software reset requires delay for sensor to reinitialize */
        if (regs[i].reg == OV9281_REG_SOFT_RESET)
            usleep_range(10000, 15000);
    }
    
    return 0;
}

/*
 * =============================================================================
 * Sensor Control Functions
 * =============================================================================
 */

/**
 * ov9281_set_exposure - Set exposure time
 * @ov9281: Sensor instance
 * @exposure: Exposure value in line units
 *
 * The exposure value is 20 bits, stored across 3 registers.
 * Actual exposure time = exposure * line_time.
 * Line time at 120fps ≈ 13.7µs.
 */
static int ov9281_set_exposure(struct ov9281 *ov9281, u32 exposure)
{
    int ret;
    
    /* Split 20-bit exposure across 3 registers */
    ret = ov9281_write_reg(ov9281, OV9281_REG_EXPOSURE_H,
                           (exposure >> 12) & 0x0F);
    if (ret)
        return ret;
    
    ret = ov9281_write_reg(ov9281, OV9281_REG_EXPOSURE_M,
                           (exposure >> 4) & 0xFF);
    if (ret)
        return ret;
    
    ret = ov9281_write_reg(ov9281, OV9281_REG_EXPOSURE_L,
                           (exposure << 4) & 0xF0);
    
    return ret;
}

/**
 * ov9281_set_gain - Set analog gain
 * @ov9281: Sensor instance
 * @gain: Gain value (0x10 = 1x, 0xF8 = 15.5x)
 *
 * Gain is 11 bits with format:
 *   gain = [integer part].[fractional part]
 *   Actual gain = gain_value / 16
 */
static int ov9281_set_gain(struct ov9281 *ov9281, u32 gain)
{
    int ret;
    
    ret = ov9281_write_reg(ov9281, OV9281_REG_GAIN_H, (gain >> 8) & 0x07);
    if (ret)
        return ret;
    
    ret = ov9281_write_reg(ov9281, OV9281_REG_GAIN_L, gain & 0xFF);
    
    return ret;
}

/**
 * ov9281_set_trigger_mode - Enable/disable external trigger
 * @ov9281: Sensor instance
 * @external: true for external trigger, false for free-running
 *
 * In external trigger mode, the sensor waits for a rising edge on
 * the FSIN pin before starting each frame capture. This enables
 * precise synchronization with other cameras and sensors.
 */
static int ov9281_set_trigger_mode(struct ov9281 *ov9281, bool external)
{
    int ret;
    
    if (external) {
        /* Enable external frame sync via FSIN pin */
        ret = ov9281_write_reg(ov9281, OV9281_REG_TIMING_CTRL, 0x01);
        if (ret)
            return ret;
        
        ret = ov9281_write_reg(ov9281, OV9281_REG_FSIN_CTRL, 0x01);
    } else {
        /* Free-running mode (internal timing) */
        ret = ov9281_write_reg(ov9281, OV9281_REG_TIMING_CTRL, 0x00);
        if (ret)
            return ret;
        
        ret = ov9281_write_reg(ov9281, OV9281_REG_FSIN_CTRL, 0x00);
    }
    
    return ret;
}

/**
 * ov9281_set_test_pattern - Enable test pattern generation
 * @ov9281: Sensor instance
 * @pattern: Pattern index (0=disabled, 1-4=patterns)
 */
static int ov9281_set_test_pattern(struct ov9281 *ov9281, int pattern)
{
    if (pattern)
        return ov9281_write_reg(ov9281, OV9281_REG_TEST_PATTERN,
                                0x80 | (pattern - 1));
    else
        return ov9281_write_reg(ov9281, OV9281_REG_TEST_PATTERN, 0x00);
}

/*
 * =============================================================================
 * V4L2 Control Operations
 * =============================================================================
 */

/**
 * ov9281_s_ctrl - Handle control value changes
 * @ctrl: Control that changed
 *
 * Called by V4L2 framework when a control value is modified.
 * Only applies changes when sensor is powered on.
 */
static int ov9281_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct ov9281 *ov9281 = container_of(ctrl->handler, struct ov9281,
                                         ctrl_handler);
    int ret = 0;
    
    /* Only apply if sensor is powered on */
    if (!pm_runtime_get_if_in_use(ov9281->sd.dev))
        return 0;
    
    switch (ctrl->id) {
    case V4L2_CID_EXPOSURE:
        ret = ov9281_set_exposure(ov9281, ctrl->val);
        break;
        
    case V4L2_CID_ANALOGUE_GAIN:
        ret = ov9281_set_gain(ov9281, ctrl->val);
        break;
        
    case V4L2_CID_TEST_PATTERN:
        ret = ov9281_set_test_pattern(ov9281, ctrl->val);
        break;
        
    default:
        break;
    }
    
    pm_runtime_put(ov9281->sd.dev);
    
    return ret;
}

static const struct v4l2_ctrl_ops ov9281_ctrl_ops = {
    .s_ctrl = ov9281_s_ctrl,
};

/*
 * =============================================================================
 * V4L2 Subdev Operations
 * =============================================================================
 */

static int ov9281_enum_mbus_code(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_state *state,
                                  struct v4l2_subdev_mbus_code_enum *code)
{
    /* Only one format supported: 10-bit monochrome */
    if (code->index > 0)
        return -EINVAL;
    
    code->code = MEDIA_BUS_FMT_Y10_1X10;
    return 0;
}

static int ov9281_enum_frame_sizes(struct v4l2_subdev *sd,
                                    struct v4l2_subdev_state *state,
                                    struct v4l2_subdev_frame_size_enum *fse)
{
    if (fse->index >= ARRAY_SIZE(ov9281_modes))
        return -EINVAL;
    
    if (fse->code != MEDIA_BUS_FMT_Y10_1X10)
        return -EINVAL;
    
    /* Only discrete sizes supported (no scaling) */
    fse->min_width = ov9281_modes[fse->index].width;
    fse->max_width = ov9281_modes[fse->index].width;
    fse->min_height = ov9281_modes[fse->index].height;
    fse->max_height = ov9281_modes[fse->index].height;
    
    return 0;
}

static int ov9281_get_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *fmt)
{
    struct ov9281 *ov9281 = to_ov9281(sd);
    const struct ov9281_mode *mode = ov9281->cur_mode;
    
    mutex_lock(&ov9281->mutex);
    
    fmt->format.width = mode->width;
    fmt->format.height = mode->height;
    fmt->format.code = MEDIA_BUS_FMT_Y10_1X10;
    fmt->format.field = V4L2_FIELD_NONE;
    fmt->format.colorspace = V4L2_COLORSPACE_RAW;
    
    mutex_unlock(&ov9281->mutex);
    
    return 0;
}

static int ov9281_set_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *fmt)
{
    struct ov9281 *ov9281 = to_ov9281(sd);
    const struct ov9281_mode *mode;
    
    /* Only one mode currently - always select it */
    mode = &ov9281_modes[0];
    
    mutex_lock(&ov9281->mutex);
    
    /* Fill in the actual format (may differ from requested) */
    fmt->format.width = mode->width;
    fmt->format.height = mode->height;
    fmt->format.code = MEDIA_BUS_FMT_Y10_1X10;
    fmt->format.field = V4L2_FIELD_NONE;
    fmt->format.colorspace = V4L2_COLORSPACE_RAW;
    
    /* Apply mode if this is the active format */
    if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
        ov9281->cur_mode = mode;
    
    mutex_unlock(&ov9281->mutex);
    
    return 0;
}

/**
 * ov9281_start_streaming - Begin streaming
 * @ov9281: Sensor instance
 *
 * Configures the sensor for the current mode and starts streaming.
 * Called when userspace issues VIDIOC_STREAMON.
 */
static int ov9281_start_streaming(struct ov9281 *ov9281)
{
    int ret;
    
    /* Write mode register settings */
    ret = ov9281_write_regs(ov9281, ov9281->cur_mode->reg_list,
                            ov9281->cur_mode->num_regs);
    if (ret) {
        dev_err(&ov9281->client->dev, "Failed to write mode registers\n");
        return ret;
    }
    
    /* Apply current control values */
    ret = __v4l2_ctrl_handler_setup(&ov9281->ctrl_handler);
    if (ret) {
        dev_err(&ov9281->client->dev, "Failed to apply controls\n");
        return ret;
    }
    
    /* Enable external trigger if configured */
    if (ov9281->trigger_mode->val == 1) {
        ret = ov9281_set_trigger_mode(ov9281, true);
        if (ret) {
            dev_err(&ov9281->client->dev, "Failed to set trigger mode\n");
            return ret;
        }
        dev_info(&ov9281->client->dev, "External trigger mode enabled\n");
    }
    
    /* Start streaming */
    ret = ov9281_write_reg(ov9281, OV9281_REG_MODE_SELECT, OV9281_MODE_STREAMING);
    if (ret)
        dev_err(&ov9281->client->dev, "Failed to start streaming\n");
    
    return ret;
}

/**
 * ov9281_stop_streaming - Stop streaming
 * @ov9281: Sensor instance
 *
 * Stops the sensor and returns to standby mode.
 * Called when userspace issues VIDIOC_STREAMOFF.
 */
static int ov9281_stop_streaming(struct ov9281 *ov9281)
{
    return ov9281_write_reg(ov9281, OV9281_REG_MODE_SELECT, OV9281_MODE_STANDBY);
}

static int ov9281_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct ov9281 *ov9281 = to_ov9281(sd);
    int ret;
    
    mutex_lock(&ov9281->mutex);
    
    if (enable) {
        ret = pm_runtime_resume_and_get(sd->dev);
        if (ret < 0)
            goto unlock;
        
        ret = ov9281_start_streaming(ov9281);
        if (ret) {
            pm_runtime_put(sd->dev);
            goto unlock;
        }
    } else {
        ov9281_stop_streaming(ov9281);
        pm_runtime_put(sd->dev);
        ret = 0;
    }
    
    ov9281->streaming = enable;
    
unlock:
    mutex_unlock(&ov9281->mutex);
    return ret;
}

static const struct v4l2_subdev_video_ops ov9281_video_ops = {
    .s_stream = ov9281_s_stream,
};

static const struct v4l2_subdev_pad_ops ov9281_pad_ops = {
    .enum_mbus_code = ov9281_enum_mbus_code,
    .enum_frame_size = ov9281_enum_frame_sizes,
    .get_fmt = ov9281_get_fmt,
    .set_fmt = ov9281_set_fmt,
};

static const struct v4l2_subdev_ops ov9281_subdev_ops = {
    .video = &ov9281_video_ops,
    .pad = &ov9281_pad_ops,
};

/*
 * =============================================================================
 * Power Management
 * =============================================================================
 */

/**
 * ov9281_power_on - Power on the sensor
 * @dev: Device pointer
 *
 * Sequence: Enable regulators → Enable clock → Deassert reset
 * Timing follows OV9281 datasheet requirements.
 */
static int ov9281_power_on(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct ov9281 *ov9281 = to_ov9281(sd);
    int ret;
    
    /* Enable power supplies */
    ret = regulator_bulk_enable(OV9281_NUM_SUPPLIES, ov9281->supplies);
    if (ret) {
        dev_err(dev, "Failed to enable regulators\n");
        return ret;
    }
    
    /* Enable input clock */
    ret = clk_prepare_enable(ov9281->xvclk);
    if (ret) {
        dev_err(dev, "Failed to enable clock\n");
        regulator_bulk_disable(OV9281_NUM_SUPPLIES, ov9281->supplies);
        return ret;
    }
    
    /* Deassert reset (active low) */
    if (ov9281->reset_gpio)
        gpiod_set_value_cansleep(ov9281->reset_gpio, 0);
    
    /* Deassert power down (active low) */
    if (ov9281->pwdn_gpio)
        gpiod_set_value_cansleep(ov9281->pwdn_gpio, 0);
    
    /* Wait for sensor to initialize (10-15ms typical) */
    usleep_range(10000, 15000);
    
    return 0;
}

/**
 * ov9281_power_off - Power off the sensor
 * @dev: Device pointer
 *
 * Sequence: Assert reset → Disable clock → Disable regulators
 */
static int ov9281_power_off(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct ov9281 *ov9281 = to_ov9281(sd);
    
    /* Assert power down */
    if (ov9281->pwdn_gpio)
        gpiod_set_value_cansleep(ov9281->pwdn_gpio, 1);
    
    /* Assert reset */
    if (ov9281->reset_gpio)
        gpiod_set_value_cansleep(ov9281->reset_gpio, 1);
    
    /* Disable clock */
    clk_disable_unprepare(ov9281->xvclk);
    
    /* Disable power supplies */
    regulator_bulk_disable(OV9281_NUM_SUPPLIES, ov9281->supplies);
    
    return 0;
}

static const struct dev_pm_ops ov9281_pm_ops = {
    SET_RUNTIME_PM_OPS(ov9281_power_off, ov9281_power_on, NULL)
};

/*
 * =============================================================================
 * Control Initialization
 * =============================================================================
 */

static int ov9281_init_controls(struct ov9281 *ov9281)
{
    struct v4l2_ctrl_handler *handler = &ov9281->ctrl_handler;
    const struct ov9281_mode *mode = ov9281->cur_mode;
    s64 pixel_rate;
    int ret;
    
    ret = v4l2_ctrl_handler_init(handler, 10);
    if (ret)
        return ret;
    
    handler->lock = &ov9281->mutex;
    
    /* Read-only informational controls */
    pixel_rate = OV9281_PIXEL_RATE;
    ov9281->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
                                            pixel_rate, pixel_rate, 1, pixel_rate);
    if (ov9281->pixel_rate)
        ov9281->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
    
    ov9281->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
                                                ARRAY_SIZE(link_freq_menu_items) - 1,
                                                0, link_freq_menu_items);
    if (ov9281->link_freq)
        ov9281->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
    
    /* Exposure control */
    ov9281->exposure = v4l2_ctrl_new_std(handler, &ov9281_ctrl_ops,
                                          V4L2_CID_EXPOSURE,
                                          OV9281_EXPOSURE_MIN,
                                          OV9281_EXPOSURE_MAX, 1,
                                          OV9281_EXPOSURE_DEFAULT);
    
    /* Analog gain control */
    ov9281->gain = v4l2_ctrl_new_std(handler, &ov9281_ctrl_ops,
                                      V4L2_CID_ANALOGUE_GAIN,
                                      OV9281_GAIN_MIN,
                                      OV9281_GAIN_MAX, 1,
                                      OV9281_GAIN_DEFAULT);
    
    /* Blanking controls (read-only, fixed for this mode) */
    ov9281->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
                                        mode->hts - mode->width,
                                        mode->hts - mode->width, 1,
                                        mode->hts - mode->width);
    if (ov9281->hblank)
        ov9281->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
    
    ov9281->vblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_VBLANK,
                                        mode->vts - mode->height,
                                        mode->vts - mode->height, 1,
                                        mode->vts - mode->height);
    if (ov9281->vblank)
        ov9281->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
    
    /* Test pattern control */
    ov9281->test_pattern = v4l2_ctrl_new_std_menu_items(handler, &ov9281_ctrl_ops,
                                                         V4L2_CID_TEST_PATTERN,
                                                         ARRAY_SIZE(test_pattern_menu) - 1,
                                                         0, 0, test_pattern_menu);
    
    /* Trigger mode control (custom, uses existing control ID) */
    ov9281->trigger_mode = v4l2_ctrl_new_std_menu_items(handler, &ov9281_ctrl_ops,
                                                         V4L2_CID_FLASH_STROBE_SOURCE,
                                                         ARRAY_SIZE(trigger_mode_menu) - 1,
                                                         0, 0, trigger_mode_menu);
    
    if (handler->error) {
        ret = handler->error;
        v4l2_ctrl_handler_free(handler);
        return ret;
    }
    
    ov9281->sd.ctrl_handler = handler;
    
    return 0;
}

/*
 * =============================================================================
 * Chip ID Verification
 * =============================================================================
 */

/**
 * ov9281_check_chip_id - Verify sensor chip ID
 * @ov9281: Sensor instance
 *
 * Reads the chip ID register and verifies it matches expected value.
 * This confirms correct I2C communication and sensor identification.
 */
static int ov9281_check_chip_id(struct ov9281 *ov9281)
{
    struct i2c_client *client = ov9281->client;
    u8 id_h, id_l;
    u16 chip_id;
    int ret;
    
    ret = ov9281_read_reg(ov9281, OV9281_REG_CHIP_ID_H, &id_h);
    if (ret)
        return ret;
    
    ret = ov9281_read_reg(ov9281, OV9281_REG_CHIP_ID_L, &id_l);
    if (ret)
        return ret;
    
    chip_id = (id_h << 8) | id_l;
    
    if (chip_id != OV9281_CHIP_ID) {
        dev_err(&client->dev,
                "Unexpected chip ID 0x%04x (expected 0x%04x)\n",
                chip_id, OV9281_CHIP_ID);
        return -ENODEV;
    }
    
    dev_info(&client->dev, "OV9281 detected (chip ID: 0x%04x)\n", chip_id);
    
    return 0;
}

/*
 * =============================================================================
 * Probe and Remove
 * =============================================================================
 */

static int ov9281_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct ov9281 *ov9281;
    int ret;
    unsigned int i;
    
    dev_info(dev, "OV9281 sensor probe\n");
    
    /* Allocate driver instance */
    ov9281 = devm_kzalloc(dev, sizeof(*ov9281), GFP_KERNEL);
    if (!ov9281)
        return -ENOMEM;
    
    ov9281->client = client;
    ov9281->cur_mode = &ov9281_modes[0];
    
    mutex_init(&ov9281->mutex);
    
    /* Get external clock */
    ov9281->xvclk = devm_clk_get(dev, "xvclk");
    if (IS_ERR(ov9281->xvclk)) {
        dev_err(dev, "Failed to get xvclk\n");
        return PTR_ERR(ov9281->xvclk);
    }
    
    /* Get optional GPIOs */
    ov9281->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(ov9281->reset_gpio))
        return PTR_ERR(ov9281->reset_gpio);
    
    ov9281->pwdn_gpio = devm_gpiod_get_optional(dev, "pwdn", GPIOD_OUT_HIGH);
    if (IS_ERR(ov9281->pwdn_gpio))
        return PTR_ERR(ov9281->pwdn_gpio);
    
    /* Get power supplies */
    for (i = 0; i < OV9281_NUM_SUPPLIES; i++)
        ov9281->supplies[i].supply = ov9281_supply_names[i];
    
    ret = devm_regulator_bulk_get(dev, OV9281_NUM_SUPPLIES, ov9281->supplies);
    if (ret) {
        dev_err(dev, "Failed to get regulators\n");
        return ret;
    }
    
    /* Power on and verify chip ID */
    ret = ov9281_power_on(dev);
    if (ret) {
        dev_err(dev, "Failed to power on sensor\n");
        return ret;
    }
    
    ret = ov9281_check_chip_id(ov9281);
    if (ret)
        goto err_power_off;
    
    /* Initialize V4L2 controls */
    ret = ov9281_init_controls(ov9281);
    if (ret) {
        dev_err(dev, "Failed to init controls\n");
        goto err_power_off;
    }
    
    /* Initialize V4L2 subdevice */
    v4l2_i2c_subdev_init(&ov9281->sd, client, &ov9281_subdev_ops);
    ov9281->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    ov9281->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
    
    ov9281->pad.flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_pads_init(&ov9281->sd.entity, 1, &ov9281->pad);
    if (ret) {
        dev_err(dev, "Failed to init media entity\n");
        goto err_free_ctrl;
    }
    
    /* Enable runtime PM */
    pm_runtime_set_active(dev);
    pm_runtime_enable(dev);
    pm_runtime_idle(dev);
    
    /* Register V4L2 subdevice */
    ret = v4l2_async_register_subdev(&ov9281->sd);
    if (ret) {
        dev_err(dev, "Failed to register subdev\n");
        goto err_clean_entity;
    }
    
    dev_info(dev, "OV9281 sensor probed successfully\n");
    
    return 0;

err_clean_entity:
    pm_runtime_disable(dev);
    media_entity_cleanup(&ov9281->sd.entity);
err_free_ctrl:
    v4l2_ctrl_handler_free(&ov9281->ctrl_handler);
err_power_off:
    ov9281_power_off(dev);
    
    return ret;
}

static void ov9281_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct ov9281 *ov9281 = to_ov9281(sd);
    
    v4l2_async_unregister_subdev(sd);
    media_entity_cleanup(&sd->entity);
    v4l2_ctrl_handler_free(&ov9281->ctrl_handler);
    pm_runtime_disable(&client->dev);
    if (!pm_runtime_status_suspended(&client->dev))
        ov9281_power_off(&client->dev);
    pm_runtime_set_suspended(&client->dev);
    mutex_destroy(&ov9281->mutex);
}

/*
 * =============================================================================
 * Module Registration
 * =============================================================================
 */

static const struct of_device_id ov9281_of_match[] = {
    { .compatible = "ovti,ov9281" },
    { }
};
MODULE_DEVICE_TABLE(of, ov9281_of_match);

static const struct i2c_device_id ov9281_id[] = {
    { "ov9281", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ov9281_id);

static struct i2c_driver ov9281_i2c_driver = {
    .driver = {
        .name = "ov9281",
        .pm = &ov9281_pm_ops,
        .of_match_table = ov9281_of_match,
    },
    .probe = ov9281_probe,
    .remove = ov9281_remove,
    .id_table = ov9281_id,
};

module_i2c_driver(ov9281_i2c_driver);

MODULE_AUTHOR("Chronos Development Team");
MODULE_DESCRIPTION("OmniVision OV9281 CMOS Image Sensor Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
