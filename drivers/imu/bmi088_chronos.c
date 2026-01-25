// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bosch BMI088 IMU Driver for Chronos System
 *
 * Copyright (C) 2025 Chronos Project
 *
 * SPI driver for BMI088 6-axis IMU with synchronized trigger support
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/delay.h>

#define BMI088_DRIVER_NAME  "bmi088-chronos"

/* Accelerometer registers */
#define BMI088_ACC_CHIP_ID      0x00
#define BMI088_ACC_DATA         0x12
#define BMI088_ACC_CONF         0x40
#define BMI088_ACC_RANGE        0x41
#define BMI088_ACC_INT1_CONF    0x53
#define BMI088_ACC_INT1_MAP     0x56
#define BMI088_ACC_PWR_CONF     0x7C
#define BMI088_ACC_PWR_CTRL     0x7D
#define BMI088_ACC_SOFTRESET    0x7E

#define BMI088_ACC_CHIP_ID_VAL  0x1E

/* Gyroscope registers */
#define BMI088_GYR_CHIP_ID      0x00
#define BMI088_GYR_DATA         0x02
#define BMI088_GYR_RANGE        0x0F
#define BMI088_GYR_BANDWIDTH    0x10
#define BMI088_GYR_LPM1         0x11
#define BMI088_GYR_INT_CTRL     0x15
#define BMI088_GYR_INT3_4_IO    0x16
#define BMI088_GYR_INT3_4_MAP   0x18
#define BMI088_GYR_SOFTRESET    0x14

#define BMI088_GYR_CHIP_ID_VAL  0x0F

/* Configuration values */
#define BMI088_ACC_RANGE_3G     0x00
#define BMI088_ACC_RANGE_6G     0x01
#define BMI088_ACC_RANGE_12G    0x02
#define BMI088_ACC_RANGE_24G    0x03

#define BMI088_GYR_RANGE_2000   0x00
#define BMI088_GYR_RANGE_1000   0x01
#define BMI088_GYR_RANGE_500    0x02
#define BMI088_GYR_RANGE_250    0x03
#define BMI088_GYR_RANGE_125    0x04

/* SPI read flag */
#define BMI088_SPI_READ         0x80

struct bmi088_data {
    struct spi_device *spi_acc;     /* Accelerometer SPI device */
    struct spi_device *spi_gyr;     /* Gyroscope SPI device */
    struct iio_dev *indio_dev;
    struct iio_trigger *trig;
    
    struct mutex lock;
    
    /* Configuration */
    u8 acc_range;
    u8 gyr_range;
    u32 sample_rate;
    
    /* Synchronization */
    ktime_t last_sync_time;
    u64 sync_count;
    bool sync_enabled;
    
    /* Buffer for triggered reads */
    struct {
        s16 acc[3];
        s16 gyr[3];
        s64 timestamp;
    } scan __aligned(8);
};

/* IIO channel definitions */
#define BMI088_ACC_CHANNEL(axis, idx) {                             \
    .type = IIO_ACCEL,                                              \
    .modified = 1,                                                   \
    .channel2 = IIO_MOD_##axis,                                     \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                   \
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |          \
                                BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
    .scan_index = idx,                                               \
    .scan_type = {                                                   \
        .sign = 's',                                                 \
        .realbits = 16,                                              \
        .storagebits = 16,                                           \
        .endianness = IIO_LE,                                        \
    },                                                               \
}

#define BMI088_GYR_CHANNEL(axis, idx) {                             \
    .type = IIO_ANGL_VEL,                                           \
    .modified = 1,                                                   \
    .channel2 = IIO_MOD_##axis,                                     \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                   \
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |          \
                                BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
    .scan_index = idx,                                               \
    .scan_type = {                                                   \
        .sign = 's',                                                 \
        .realbits = 16,                                              \
        .storagebits = 16,                                           \
        .endianness = IIO_LE,                                        \
    },                                                               \
}

static const struct iio_chan_spec bmi088_channels[] = {
    BMI088_ACC_CHANNEL(X, 0),
    BMI088_ACC_CHANNEL(Y, 1),
    BMI088_ACC_CHANNEL(Z, 2),
    BMI088_GYR_CHANNEL(X, 3),
    BMI088_GYR_CHANNEL(Y, 4),
    BMI088_GYR_CHANNEL(Z, 5),
    IIO_CHAN_SOFT_TIMESTAMP(6),
};

/* SPI Communication */
static int bmi088_spi_read(struct spi_device *spi, u8 reg, u8 *val, int len)
{
    u8 tx_buf[2];
    struct spi_transfer xfers[2] = {
        {
            .tx_buf = tx_buf,
            .len = 1,
        },
        {
            .rx_buf = val,
            .len = len,
        },
    };
    
    tx_buf[0] = reg | BMI088_SPI_READ;
    
    return spi_sync_transfer(spi, xfers, 2);
}

static int bmi088_spi_write(struct spi_device *spi, u8 reg, u8 val)
{
    u8 buf[2] = { reg, val };
    
    return spi_write(spi, buf, 2);
}

/* Read accelerometer data */
static int bmi088_read_acc(struct bmi088_data *data, s16 *acc)
{
    u8 buf[6];
    int ret;
    
    /* Accelerometer requires dummy byte after address */
    ret = bmi088_spi_read(data->spi_acc, BMI088_ACC_DATA, buf, 6);
    if (ret)
        return ret;
    
    acc[0] = (s16)(buf[1] << 8 | buf[0]);
    acc[1] = (s16)(buf[3] << 8 | buf[2]);
    acc[2] = (s16)(buf[5] << 8 | buf[4]);
    
    return 0;
}

/* Read gyroscope data */
static int bmi088_read_gyr(struct bmi088_data *data, s16 *gyr)
{
    u8 buf[6];
    int ret;
    
    ret = bmi088_spi_read(data->spi_gyr, BMI088_GYR_DATA, buf, 6);
    if (ret)
        return ret;
    
    gyr[0] = (s16)(buf[1] << 8 | buf[0]);
    gyr[1] = (s16)(buf[3] << 8 | buf[2]);
    gyr[2] = (s16)(buf[5] << 8 | buf[4]);
    
    return 0;
}

/* Interrupt handler for synchronized trigger */
static irqreturn_t bmi088_trigger_handler(int irq, void *p)
{
    struct iio_poll_func *pf = p;
    struct iio_dev *indio_dev = pf->indio_dev;
    struct bmi088_data *data = iio_priv(indio_dev);
    int ret;
    
    mutex_lock(&data->lock);
    
    /* Read accelerometer */
    ret = bmi088_read_acc(data, data->scan.acc);
    if (ret)
        goto done;
    
    /* Read gyroscope */
    ret = bmi088_read_gyr(data, data->scan.gyr);
    if (ret)
        goto done;
    
    /* Record sync time */
    data->last_sync_time = ktime_get();
    data->sync_count++;
    
    iio_push_to_buffers_with_timestamp(indio_dev, &data->scan,
                                       ktime_to_ns(data->last_sync_time));

done:
    mutex_unlock(&data->lock);
    iio_trigger_notify_done(indio_dev->trig);
    
    return IRQ_HANDLED;
}

/* External sync interrupt handler */
static irqreturn_t bmi088_sync_irq(int irq, void *private)
{
    struct iio_dev *indio_dev = private;
    struct bmi088_data *data = iio_priv(indio_dev);
    
    if (data->sync_enabled && data->trig)
        iio_trigger_poll(data->trig);
    
    return IRQ_HANDLED;
}

/* IIO read raw */
static int bmi088_read_raw(struct iio_dev *indio_dev,
                           struct iio_chan_spec const *chan,
                           int *val, int *val2, long mask)
{
    struct bmi088_data *data = iio_priv(indio_dev);
    s16 acc[3], gyr[3];
    int ret;
    
    switch (mask) {
    case IIO_CHAN_INFO_RAW:
        mutex_lock(&data->lock);
        
        if (chan->type == IIO_ACCEL) {
            ret = bmi088_read_acc(data, acc);
            if (!ret)
                *val = acc[chan->scan_index];
        } else {
            ret = bmi088_read_gyr(data, gyr);
            if (!ret)
                *val = gyr[chan->scan_index - 3];
        }
        
        mutex_unlock(&data->lock);
        
        return ret ? ret : IIO_VAL_INT;
        
    case IIO_CHAN_INFO_SCALE:
        if (chan->type == IIO_ACCEL) {
            /* Scale based on range setting */
            switch (data->acc_range) {
            case BMI088_ACC_RANGE_3G:
                *val = 0;
                *val2 = 897;  /* 3g / 32768 * 9.80665 */
                break;
            case BMI088_ACC_RANGE_6G:
                *val = 0;
                *val2 = 1794;
                break;
            case BMI088_ACC_RANGE_12G:
                *val = 0;
                *val2 = 3589;
                break;
            case BMI088_ACC_RANGE_24G:
                *val = 0;
                *val2 = 7178;
                break;
            default:
                return -EINVAL;
            }
        } else {
            /* Gyroscope scale */
            switch (data->gyr_range) {
            case BMI088_GYR_RANGE_2000:
                *val = 0;
                *val2 = 1065;  /* 2000 dps / 32768 * pi/180 */
                break;
            case BMI088_GYR_RANGE_1000:
                *val = 0;
                *val2 = 532;
                break;
            case BMI088_GYR_RANGE_500:
                *val = 0;
                *val2 = 266;
                break;
            case BMI088_GYR_RANGE_250:
                *val = 0;
                *val2 = 133;
                break;
            case BMI088_GYR_RANGE_125:
                *val = 0;
                *val2 = 66;
                break;
            default:
                return -EINVAL;
            }
        }
        return IIO_VAL_INT_PLUS_MICRO;
        
    case IIO_CHAN_INFO_SAMP_FREQ:
        *val = data->sample_rate;
        return IIO_VAL_INT;
        
    default:
        return -EINVAL;
    }
}

/* IIO write raw */
static int bmi088_write_raw(struct iio_dev *indio_dev,
                            struct iio_chan_spec const *chan,
                            int val, int val2, long mask)
{
    struct bmi088_data *data = iio_priv(indio_dev);
    
    switch (mask) {
    case IIO_CHAN_INFO_SAMP_FREQ:
        /* Sample rate is controlled by external trigger */
        data->sample_rate = val;
        return 0;
        
    default:
        return -EINVAL;
    }
}

/* Sysfs attributes for synchronization */
static ssize_t sync_count_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
    struct bmi088_data *data = iio_priv(indio_dev);
    
    return sprintf(buf, "%llu\n", data->sync_count);
}
static IIO_DEVICE_ATTR_RO(sync_count, 0);

static ssize_t last_sync_time_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
    struct bmi088_data *data = iio_priv(indio_dev);
    
    return sprintf(buf, "%lld\n", ktime_to_ns(data->last_sync_time));
}
static IIO_DEVICE_ATTR_RO(last_sync_time, 0);

static ssize_t sync_enabled_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
    struct bmi088_data *data = iio_priv(indio_dev);
    
    return sprintf(buf, "%d\n", data->sync_enabled);
}

static ssize_t sync_enabled_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t len)
{
    struct iio_dev *indio_dev = dev_to_iio_dev(dev);
    struct bmi088_data *data = iio_priv(indio_dev);
    bool val;
    int ret;
    
    ret = kstrtobool(buf, &val);
    if (ret)
        return ret;
    
    data->sync_enabled = val;
    
    return len;
}
static IIO_DEVICE_ATTR_RW(sync_enabled, 0);

static struct attribute *bmi088_attrs[] = {
    &iio_dev_attr_sync_count.dev_attr.attr,
    &iio_dev_attr_last_sync_time.dev_attr.attr,
    &iio_dev_attr_sync_enabled.dev_attr.attr,
    NULL,
};

static const struct attribute_group bmi088_attrs_group = {
    .attrs = bmi088_attrs,
};

static const struct iio_info bmi088_info = {
    .read_raw = bmi088_read_raw,
    .write_raw = bmi088_write_raw,
    .attrs = &bmi088_attrs_group,
};

/* Initialize accelerometer */
static int bmi088_init_acc(struct bmi088_data *data)
{
    u8 chip_id;
    int ret;
    
    /* Soft reset */
    ret = bmi088_spi_write(data->spi_acc, BMI088_ACC_SOFTRESET, 0xB6);
    if (ret)
        return ret;
    
    msleep(50);
    
    /* Check chip ID */
    ret = bmi088_spi_read(data->spi_acc, BMI088_ACC_CHIP_ID, &chip_id, 1);
    if (ret)
        return ret;
    
    /* Note: BMI088 requires dummy read after address for ACC */
    ret = bmi088_spi_read(data->spi_acc, BMI088_ACC_CHIP_ID, &chip_id, 1);
    if (ret)
        return ret;
    
    if (chip_id != BMI088_ACC_CHIP_ID_VAL) {
        dev_err(&data->spi_acc->dev, "Invalid ACC chip ID: 0x%02x\n", chip_id);
        return -ENODEV;
    }
    
    /* Power on accelerometer */
    ret = bmi088_spi_write(data->spi_acc, BMI088_ACC_PWR_CONF, 0x00);
    if (ret)
        return ret;
    
    msleep(5);
    
    ret = bmi088_spi_write(data->spi_acc, BMI088_ACC_PWR_CTRL, 0x04);
    if (ret)
        return ret;
    
    msleep(50);
    
    /* Configure range (default 6g) */
    data->acc_range = BMI088_ACC_RANGE_6G;
    ret = bmi088_spi_write(data->spi_acc, BMI088_ACC_RANGE, data->acc_range);
    if (ret)
        return ret;
    
    /* Configure ODR and bandwidth */
    ret = bmi088_spi_write(data->spi_acc, BMI088_ACC_CONF, 0xAC);  /* ODR=1600Hz, BWP=normal */
    
    return ret;
}

/* Initialize gyroscope */
static int bmi088_init_gyr(struct bmi088_data *data)
{
    u8 chip_id;
    int ret;
    
    /* Soft reset */
    ret = bmi088_spi_write(data->spi_gyr, BMI088_GYR_SOFTRESET, 0xB6);
    if (ret)
        return ret;
    
    msleep(50);
    
    /* Check chip ID */
    ret = bmi088_spi_read(data->spi_gyr, BMI088_GYR_CHIP_ID, &chip_id, 1);
    if (ret)
        return ret;
    
    if (chip_id != BMI088_GYR_CHIP_ID_VAL) {
        dev_err(&data->spi_gyr->dev, "Invalid GYR chip ID: 0x%02x\n", chip_id);
        return -ENODEV;
    }
    
    /* Configure range (default 2000 dps) */
    data->gyr_range = BMI088_GYR_RANGE_2000;
    ret = bmi088_spi_write(data->spi_gyr, BMI088_GYR_RANGE, data->gyr_range);
    if (ret)
        return ret;
    
    /* Configure bandwidth (ODR=2000Hz, Filter BW=532Hz) */
    ret = bmi088_spi_write(data->spi_gyr, BMI088_GYR_BANDWIDTH, 0x01);
    if (ret)
        return ret;
    
    /* Normal mode */
    ret = bmi088_spi_write(data->spi_gyr, BMI088_GYR_LPM1, 0x00);
    
    return ret;
}

/* Probe function */
static int bmi088_probe(struct spi_device *spi)
{
    struct iio_dev *indio_dev;
    struct bmi088_data *data;
    int ret;
    int irq;
    
    dev_info(&spi->dev, "BMI088 Chronos IMU probe\n");
    
    indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*data));
    if (!indio_dev)
        return -ENOMEM;
    
    data = iio_priv(indio_dev);
    data->spi_acc = spi;
    data->spi_gyr = spi;  /* In actual design, may be separate CS */
    data->indio_dev = indio_dev;
    data->sample_rate = 120;  /* Match camera frame rate */
    data->sync_enabled = true;
    
    mutex_init(&data->lock);
    spi_set_drvdata(spi, indio_dev);
    
    /* Initialize sensors */
    ret = bmi088_init_acc(data);
    if (ret) {
        dev_err(&spi->dev, "Failed to init accelerometer\n");
        return ret;
    }
    
    ret = bmi088_init_gyr(data);
    if (ret) {
        dev_err(&spi->dev, "Failed to init gyroscope\n");
        return ret;
    }
    
    /* Setup IIO device */
    indio_dev->name = BMI088_DRIVER_NAME;
    indio_dev->info = &bmi088_info;
    indio_dev->channels = bmi088_channels;
    indio_dev->num_channels = ARRAY_SIZE(bmi088_channels);
    indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
    
    /* Setup triggered buffer */
    ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev, NULL,
                                          bmi088_trigger_handler, NULL);
    if (ret) {
        dev_err(&spi->dev, "Failed to setup triggered buffer\n");
        return ret;
    }
    
    /* Setup external sync interrupt */
    irq = spi->irq;
    if (irq > 0) {
        ret = devm_request_threaded_irq(&spi->dev, irq, NULL,
                                        bmi088_sync_irq,
                                        IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                                        "bmi088-sync", indio_dev);
        if (ret) {
            dev_err(&spi->dev, "Failed to request IRQ %d\n", irq);
            return ret;
        }
        
        dev_info(&spi->dev, "External sync IRQ %d configured\n", irq);
    }
    
    /* Register IIO device */
    ret = devm_iio_device_register(&spi->dev, indio_dev);
    if (ret) {
        dev_err(&spi->dev, "Failed to register IIO device\n");
        return ret;
    }
    
    dev_info(&spi->dev, "BMI088 Chronos IMU initialized\n");
    
    return 0;
}

static const struct spi_device_id bmi088_spi_id[] = {
    { "bmi088-chronos", 0 },
    { "bmi088", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, bmi088_spi_id);

static const struct of_device_id bmi088_of_match[] = {
    { .compatible = "bosch,bmi088" },
    { .compatible = "bosch,bmi088-chronos" },
    { }
};
MODULE_DEVICE_TABLE(of, bmi088_of_match);

static struct spi_driver bmi088_spi_driver = {
    .driver = {
        .name = BMI088_DRIVER_NAME,
        .of_match_table = bmi088_of_match,
    },
    .probe = bmi088_probe,
    .id_table = bmi088_spi_id,
};

module_spi_driver(bmi088_spi_driver);

MODULE_AUTHOR("Chronos Project");
MODULE_DESCRIPTION("Bosch BMI088 IMU Driver for Chronos System");
MODULE_LICENSE("GPL v2");
