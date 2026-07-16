// SPDX-License-Identifier: GPL-2.0-only
/*
 * TDK InvenSense ICM-42688-P IMU driver for the Chronos board.
 *
 * =============================================================================
 * WARNING: NOT USABLE ON CHRONOS R1 (HW_CHRONOS_R1) - DO NOT LOAD
 * =============================================================================
 * The ICM-42688-P (IC1) is wired ONLY to the FPGA: its SPI bus and INT1 go
 * to CrossLink-NX pins, not to the Jetson.  The Jetson has no SPI or GPIO
 * path to this device, so this Jetson-side SPI driver cannot possibly bind
 * to real hardware on this board, and chronos-orin-nx.dts contains no IMU
 * node.  (The FPGA-side SPI master is future work; a later FPGA release may
 * proxy IMU data through its I2C register file at 0x3C.)
 *
 * The driver is retained ONLY for a future hardware revision that routes
 * the IMU SPI to the Jetson, or as a reference for the eventual FPGA proxy.
 * See drivers/imu/README.md.
 * =============================================================================
 *
 * Copyright (C) 2025 Chronos Project
 *
 * Hardware notes (from HW_CHRONOS_R1 netlist, IC1):
 *   - 14-pin LGA, SPI 4-wire mode (CS / SCLK / SDI / SO).
 *   - Power: 1.8 V on VDD and VDDIO (single rail on this board).
 *   - INT1 (pin 4)  -> FPGA Y7  : data-ready / FIFO interrupt.
 *   - FSYNC (pin 9) -> driven by FPGA via U9 buffer, same edge that triggers
 *                      the cameras.  ICM-42688 tags the next sample with the
 *                      FSYNC time delta (REG_TMST_FSYNCH/L), which gives us a
 *                      hardware-accurate camera <-> IMU correlation.
 *
 * The previous revision shipped a Bosch BMI088 driver which is wrong: the
 * Bosch part uses two separate dies (ACC + GYR, two CS lines) and entirely
 * different register maps.  The board has neither.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define ICM42688_DRV_NAME           "icm42688-chronos"

/* SPI: bit 7 of the first byte = R/W (1 = read).  Max SCLK = 24 MHz. */
#define ICM42688_SPI_READ_FLAG      0x80

/* Bank 0 registers used here ----------------------------------------------- */
#define ICM42688_REG_DEVICE_CONFIG  0x11
#define ICM42688_DEVICE_CONFIG_RESET BIT(0)

#define ICM42688_REG_INT_CONFIG     0x14    /* INT1/INT2 polarity / mode */
#define ICM42688_INT1_POLARITY_HIGH BIT(0)
#define ICM42688_INT1_PUSH_PULL     BIT(1)
#define ICM42688_INT1_LATCHED       BIT(2)

#define ICM42688_REG_FIFO_CONFIG    0x16

#define ICM42688_REG_TEMP_DATA1     0x1D    /* big-endian, 16-bit */
#define ICM42688_REG_ACCEL_DATA_X1  0x1F    /* 6 bytes accel  + 6 bytes gyro */
#define ICM42688_REG_GYRO_DATA_X1   0x25
#define ICM42688_REG_TMST_FSYNCH    0x2B    /* FSYNC time-stamp delta hi */
#define ICM42688_REG_TMST_FSYNCL    0x2C    /* FSYNC time-stamp delta lo */
#define ICM42688_REG_INT_STATUS     0x2D
#define ICM42688_INT_STATUS_DATA_RDY BIT(3)

#define ICM42688_REG_PWR_MGMT0      0x4E
#define ICM42688_ACCEL_MODE_LN      0x03    /* low-noise mode */
#define ICM42688_GYRO_MODE_LN       (0x03 << 2)

#define ICM42688_REG_GYRO_CONFIG0   0x4F
#define ICM42688_REG_ACCEL_CONFIG0  0x50
#define ICM42688_REG_TMST_CONFIG    0x54
#define ICM42688_TMST_EN            BIT(0)
#define ICM42688_TMST_FSYNC_EN      BIT(1)
#define ICM42688_TMST_DELTA_EN      BIT(2)
#define ICM42688_TMST_RES_16US      BIT(3)
#define ICM42688_TMST_TO_REGS_EN    BIT(4)

#define ICM42688_REG_FSYNC_CONFIG   0x62
#define ICM42688_FSYNC_TAG_GYRO_X   (0x02 << 4) /* tag bit replaces LSB of gyro X */

#define ICM42688_REG_INT_CONFIG1    0x64
#define ICM42688_REG_INT_SOURCE0    0x65
#define ICM42688_INT_SOURCE0_UI_DRDY BIT(3)

#define ICM42688_REG_WHO_AM_I       0x75
#define ICM42688_WHOAMI             0x47

/* ODR / range selections (encoded as written into _CONFIG0 registers) ------ */
#define ICM42688_GYRO_FS_2000DPS    0x00
#define ICM42688_GYRO_FS_1000DPS    0x01
#define ICM42688_GYRO_FS_500DPS     0x02
#define ICM42688_GYRO_FS_250DPS     0x03

#define ICM42688_ACCEL_FS_16G       0x00
#define ICM42688_ACCEL_FS_8G        0x01
#define ICM42688_ACCEL_FS_4G        0x02
#define ICM42688_ACCEL_FS_2G        0x03

#define ICM42688_ODR_1KHZ           0x06    /* 1 kHz, default for both */

struct icm42688 {
	struct spi_device *spi;
	struct iio_dev    *idev;
	struct mutex       lock;

	u8 gyro_fs;
	u8 accel_fs;
	u32 odr_hz;

	bool sync_enabled;
	u64  sync_count;
	ktime_t last_sync_time;

	/* DMA-safe scan buffer for triggered reads (must be 8-byte aligned). */
	struct {
		s16 accel[3];
		s16 gyro[3];
		s16 temp;
		s64 timestamp __aligned(8);
	} scan __aligned(8);
};

/* --- low-level SPI ------------------------------------------------------- */

static int icm42688_read(struct icm42688 *st, u8 reg, u8 *buf, size_t len)
{
	u8 tx = reg | ICM42688_SPI_READ_FLAG;
	struct spi_transfer xfers[2] = {
		{ .tx_buf = &tx, .len = 1 },
		{ .rx_buf = buf, .len = len },
	};

	return spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));
}

static int icm42688_write(struct icm42688 *st, u8 reg, u8 val)
{
	u8 tx[2] = { reg & 0x7F, val };

	return spi_write(st->spi, tx, sizeof(tx));
}

static int icm42688_update_bits(struct icm42688 *st, u8 reg, u8 mask, u8 val)
{
	u8 cur;
	int ret;

	ret = icm42688_read(st, reg, &cur, 1);
	if (ret)
		return ret;

	cur = (cur & ~mask) | (val & mask);
	return icm42688_write(st, reg, cur);
}

/* --- channel definitions ------------------------------------------------- */

#define ICM42688_ACCEL_CHAN(_axis, _idx) {                          \
	.type = IIO_ACCEL,                                              \
	.modified = 1,                                                  \
	.channel2 = IIO_MOD_##_axis,                                    \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                   \
	.info_mask_shared_by_type =                                     \
		BIT(IIO_CHAN_INFO_SCALE) |                                  \
		BIT(IIO_CHAN_INFO_SAMP_FREQ),                               \
	.scan_index = (_idx),                                           \
	.scan_type = { .sign = 's', .realbits = 16,                     \
	               .storagebits = 16, .endianness = IIO_BE },       \
}

#define ICM42688_GYRO_CHAN(_axis, _idx) {                           \
	.type = IIO_ANGL_VEL,                                           \
	.modified = 1,                                                  \
	.channel2 = IIO_MOD_##_axis,                                    \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                   \
	.info_mask_shared_by_type =                                     \
		BIT(IIO_CHAN_INFO_SCALE) |                                  \
		BIT(IIO_CHAN_INFO_SAMP_FREQ),                               \
	.scan_index = (_idx),                                           \
	.scan_type = { .sign = 's', .realbits = 16,                     \
	               .storagebits = 16, .endianness = IIO_BE },       \
}

static const struct iio_chan_spec icm42688_channels[] = {
	ICM42688_ACCEL_CHAN(X, 0),
	ICM42688_ACCEL_CHAN(Y, 1),
	ICM42688_ACCEL_CHAN(Z, 2),
	ICM42688_GYRO_CHAN(X, 3),
	ICM42688_GYRO_CHAN(Y, 4),
	ICM42688_GYRO_CHAN(Z, 5),
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		                      BIT(IIO_CHAN_INFO_SCALE) |
		                      BIT(IIO_CHAN_INFO_OFFSET),
		.scan_index = 6,
		.scan_type = { .sign = 's', .realbits = 16,
		               .storagebits = 16, .endianness = IIO_BE },
	},
	IIO_CHAN_SOFT_TIMESTAMP(7),
};

/* --- scale tables -------------------------------------------------------- */

/* Accel: full-scale / 32768 * g, expressed as IIO_VAL_INT_PLUS_NANO. */
static const struct {
	int integer;
	int nano;
} icm42688_accel_scales[] = {
	[ICM42688_ACCEL_FS_16G] = { 0, 4789062 },   /* 16g  */
	[ICM42688_ACCEL_FS_8G]  = { 0, 2394531 },   /* 8g   */
	[ICM42688_ACCEL_FS_4G]  = { 0, 1197265 },   /* 4g   */
	[ICM42688_ACCEL_FS_2G]  = { 0,  598633 },   /* 2g   */
};

/* Gyro: full-scale / 32768 * pi/180, IIO_VAL_INT_PLUS_NANO */
static const struct {
	int integer;
	int nano;
} icm42688_gyro_scales[] = {
	[ICM42688_GYRO_FS_2000DPS] = { 0, 1064724 },
	[ICM42688_GYRO_FS_1000DPS] = { 0,  532362 },
	[ICM42688_GYRO_FS_500DPS]  = { 0,  266181 },
	[ICM42688_GYRO_FS_250DPS]  = { 0,  133090 },
};

/* --- IIO ops ------------------------------------------------------------- */

static int icm42688_read_raw_axis(struct icm42688 *st,
                                  const struct iio_chan_spec *ch, int *val)
{
	u8 buf[2];
	u8 reg;
	int ret;

	switch (ch->type) {
	case IIO_ACCEL:
		reg = ICM42688_REG_ACCEL_DATA_X1 + 2 * ch->scan_index;
		break;
	case IIO_ANGL_VEL:
		reg = ICM42688_REG_GYRO_DATA_X1  + 2 * (ch->scan_index - 3);
		break;
	case IIO_TEMP:
		reg = ICM42688_REG_TEMP_DATA1;
		break;
	default:
		return -EINVAL;
	}

	ret = icm42688_read(st, reg, buf, 2);
	if (ret)
		return ret;

	*val = (s16)((buf[0] << 8) | buf[1]);
	return IIO_VAL_INT;
}

static int icm42688_read_raw(struct iio_dev *idev,
                             struct iio_chan_spec const *ch,
                             int *val, int *val2, long mask)
{
	struct icm42688 *st = iio_priv(idev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ret = icm42688_read_raw_axis(st, ch, val);
		mutex_unlock(&st->lock);
		return ret;

	case IIO_CHAN_INFO_SCALE:
		switch (ch->type) {
		case IIO_ACCEL:
			*val  = icm42688_accel_scales[st->accel_fs].integer;
			*val2 = icm42688_accel_scales[st->accel_fs].nano;
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_ANGL_VEL:
			*val  = icm42688_gyro_scales[st->gyro_fs].integer;
			*val2 = icm42688_gyro_scales[st->gyro_fs].nano;
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_TEMP:
			/* T_degC = (raw/132.48) + 25; expressed in milli-deg-C */
			*val  = 1000;
			*val2 = 132480;
			return IIO_VAL_FRACTIONAL;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_OFFSET:
		if (ch->type != IIO_TEMP)
			return -EINVAL;
		/* offset_milli_C = 25000 / (1000/132480) -> express as raw counts */
		*val = 25 * 132480 / 1000;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->odr_hz;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int icm42688_write_raw(struct iio_dev *idev,
                              struct iio_chan_spec const *ch,
                              int val, int val2, long mask)
{
	struct icm42688 *st = iio_priv(idev);

	if (mask != IIO_CHAN_INFO_SAMP_FREQ)
		return -EINVAL;

	/*
	 * Sample rate is dictated by the external FSYNC pulse train coming
	 * from the FPGA; we only cache the value for reporting.  Writing the
	 * UI ODR register is still useful so the device-internal pipeline
	 * matches the external trigger rate.
	 */
	st->odr_hz = val;
	return 0;
}

/* --- triggered buffer ---------------------------------------------------- */

static irqreturn_t icm42688_trig_handler(int irq, void *priv)
{
	struct iio_poll_func *pf = priv;
	struct iio_dev *idev = pf->indio_dev;
	struct icm42688 *st  = iio_priv(idev);
	u8 raw[14];
	int ret;

	mutex_lock(&st->lock);

	/* Read TEMP_H..GYRO_Z_L in one burst (matches the channel order). */
	ret = icm42688_read(st, ICM42688_REG_TEMP_DATA1, raw, sizeof(raw));
	if (ret)
		goto done;

	st->scan.temp     = (s16)((raw[0] << 8) | raw[1]);
	st->scan.accel[0] = (s16)((raw[2] << 8) | raw[3]);
	st->scan.accel[1] = (s16)((raw[4] << 8) | raw[5]);
	st->scan.accel[2] = (s16)((raw[6] << 8) | raw[7]);
	st->scan.gyro[0]  = (s16)((raw[8]  << 8) | raw[9]);
	st->scan.gyro[1]  = (s16)((raw[10] << 8) | raw[11]);
	st->scan.gyro[2]  = (s16)((raw[12] << 8) | raw[13]);

	st->last_sync_time = ktime_get();
	st->sync_count++;

	iio_push_to_buffers_with_timestamp(idev, &st->scan,
	                                   ktime_to_ns(st->last_sync_time));
done:
	mutex_unlock(&st->lock);
	iio_trigger_notify_done(idev->trig);
	return IRQ_HANDLED;
}

/* Data-ready hand-off via the IRQ declared in the DT.  On Chronos R1 no
 * such path exists (INT1 goes to the FPGA only); this only applies to a
 * future revision where the IMU interrupt reaches the host. */
static irqreturn_t icm42688_irq(int irq, void *priv)
{
	struct iio_dev  *idev = priv;
	struct icm42688 *st   = iio_priv(idev);

	if (st->sync_enabled && idev->trig)
		iio_trigger_poll(idev->trig);

	return IRQ_HANDLED;
}

/* --- sysfs attributes ---------------------------------------------------- */

static ssize_t sync_count_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	struct icm42688 *st = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%llu\n", st->sync_count);
}
static IIO_DEVICE_ATTR_RO(sync_count, 0);

static ssize_t last_sync_time_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
	struct icm42688 *st = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%lld\n", ktime_to_ns(st->last_sync_time));
}
static IIO_DEVICE_ATTR_RO(last_sync_time, 0);

static ssize_t sync_enabled_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
	struct icm42688 *st = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%d\n", st->sync_enabled);
}

static ssize_t sync_enabled_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t len)
{
	struct icm42688 *st = iio_priv(dev_to_iio_dev(dev));
	bool v;
	int ret = kstrtobool(buf, &v);

	if (ret)
		return ret;
	st->sync_enabled = v;
	return len;
}
static IIO_DEVICE_ATTR_RW(sync_enabled, 0);

static struct attribute *icm42688_attrs[] = {
	&iio_dev_attr_sync_count.dev_attr.attr,
	&iio_dev_attr_last_sync_time.dev_attr.attr,
	&iio_dev_attr_sync_enabled.dev_attr.attr,
	NULL,
};

static const struct attribute_group icm42688_attr_group = {
	.attrs = icm42688_attrs,
};

static const struct iio_info icm42688_info = {
	.read_raw  = icm42688_read_raw,
	.write_raw = icm42688_write_raw,
	.attrs     = &icm42688_attr_group,
};

/* --- bring-up sequence --------------------------------------------------- */

static int icm42688_hw_init(struct icm42688 *st)
{
	struct device *dev = &st->spi->dev;
	u8 who;
	int ret;

	/* Soft reset, then wait at least 1 ms (datasheet 14.34). */
	ret = icm42688_write(st, ICM42688_REG_DEVICE_CONFIG,
	                     ICM42688_DEVICE_CONFIG_RESET);
	if (ret)
		return ret;
	usleep_range(2000, 3000);

	ret = icm42688_read(st, ICM42688_REG_WHO_AM_I, &who, 1);
	if (ret)
		return ret;
	if (who != ICM42688_WHOAMI) {
		dev_err(dev, "WHO_AM_I=0x%02x (expected 0x%02x): wrong IMU on this board?\n",
		        who, ICM42688_WHOAMI);
		return -ENODEV;
	}
	dev_info(dev, "ICM-42688-P detected (WHO_AM_I=0x47)\n");

	/* INT1 = active-high, push-pull, latched-until-status-read. */
	ret = icm42688_write(st, ICM42688_REG_INT_CONFIG,
	                     ICM42688_INT1_POLARITY_HIGH |
	                     ICM42688_INT1_PUSH_PULL     |
	                     ICM42688_INT1_LATCHED);
	if (ret)
		return ret;

	/* Route UI data-ready -> INT1. */
	ret = icm42688_write(st, ICM42688_REG_INT_SOURCE0,
	                     ICM42688_INT_SOURCE0_UI_DRDY);
	if (ret)
		return ret;

	/* Default ranges: ±16 g and ±2000 dps, ODR 1 kHz. */
	st->accel_fs = ICM42688_ACCEL_FS_16G;
	st->gyro_fs  = ICM42688_GYRO_FS_2000DPS;
	st->odr_hz   = 1000;

	ret = icm42688_write(st, ICM42688_REG_ACCEL_CONFIG0,
	                     (st->accel_fs << 5) | ICM42688_ODR_1KHZ);
	if (ret)
		return ret;
	ret = icm42688_write(st, ICM42688_REG_GYRO_CONFIG0,
	                     (st->gyro_fs  << 5) | ICM42688_ODR_1KHZ);
	if (ret)
		return ret;

	/*
	 * Enable timestamp + FSYNC handling:
	 *   - 16 us timestamp resolution
	 *   - record FSYNC delta in TMST_FSYNCH/L
	 *   - tag the next sample after FSYNC by toggling gyro-X LSB
	 * This is what lets the host correlate IMU samples with camera frames.
	 */
	ret = icm42688_write(st, ICM42688_REG_TMST_CONFIG,
	                     ICM42688_TMST_EN        |
	                     ICM42688_TMST_FSYNC_EN  |
	                     ICM42688_TMST_DELTA_EN  |
	                     ICM42688_TMST_RES_16US  |
	                     ICM42688_TMST_TO_REGS_EN);
	if (ret)
		return ret;
	ret = icm42688_write(st, ICM42688_REG_FSYNC_CONFIG,
	                     ICM42688_FSYNC_TAG_GYRO_X);
	if (ret)
		return ret;

	/* Power up accel + gyro in low-noise mode. */
	ret = icm42688_write(st, ICM42688_REG_PWR_MGMT0,
	                     ICM42688_ACCEL_MODE_LN | ICM42688_GYRO_MODE_LN);
	if (ret)
		return ret;
	usleep_range(45000, 50000);   /* Gyro start-up time per datasheet. */

	return 0;
}

/* --- probe / remove ------------------------------------------------------ */

static int icm42688_probe(struct spi_device *spi)
{
	struct iio_dev  *idev;
	struct icm42688 *st;
	int ret;

	if (spi->max_speed_hz > 24000000)
		spi->max_speed_hz = 24000000;
	spi->mode    = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	idev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!idev)
		return -ENOMEM;

	st = iio_priv(idev);
	st->spi  = spi;
	st->idev = idev;
	st->sync_enabled = true;
	mutex_init(&st->lock);
	spi_set_drvdata(spi, idev);

	ret = icm42688_hw_init(st);
	if (ret)
		return ret;

	idev->name         = ICM42688_DRV_NAME;
	idev->info         = &icm42688_info;
	idev->channels     = icm42688_channels;
	idev->num_channels = ARRAY_SIZE(icm42688_channels);
	idev->modes        = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;

	ret = devm_iio_triggered_buffer_setup(&spi->dev, idev, NULL,
	                                      icm42688_trig_handler, NULL);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
		                     "failed to set up triggered buffer\n");

	if (spi->irq > 0) {
		ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
		                                icm42688_irq,
		                                IRQF_TRIGGER_RISING | IRQF_ONESHOT,
		                                ICM42688_DRV_NAME, idev);
		if (ret)
			return dev_err_probe(&spi->dev, ret,
			                     "failed to request INT1 irq %d\n",
			                     spi->irq);
	} else {
		dev_warn(&spi->dev,
		         "no INT1 GPIO available, data-ready hand-off disabled\n");
	}

	ret = devm_iio_device_register(&spi->dev, idev);
	if (ret)
		return ret;

	dev_info(&spi->dev,
	         "ICM-42688-P ready @ %u Hz, FSYNC-tagged, scales: %ug / %udps\n",
	         st->odr_hz, 16 >> st->accel_fs, 2000 >> st->gyro_fs);
	return 0;
}

static const struct spi_device_id icm42688_spi_ids[] = {
	{ "icm42688",          0 },
	{ "icm42688-chronos",  0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, icm42688_spi_ids);

static const struct of_device_id icm42688_of_match[] = {
	{ .compatible = "invensense,icm42688" },
	{ .compatible = "invensense,icm42688p" },
	{ .compatible = "chronos,icm42688" },
	{ }
};
MODULE_DEVICE_TABLE(of, icm42688_of_match);

static struct spi_driver icm42688_driver = {
	.driver = {
		.name           = ICM42688_DRV_NAME,
		.of_match_table = icm42688_of_match,
	},
	.probe    = icm42688_probe,
	.id_table = icm42688_spi_ids,
};
module_spi_driver(icm42688_driver);

MODULE_AUTHOR("Chronos Project");
MODULE_DESCRIPTION("TDK InvenSense ICM-42688-P IMU driver (Chronos board)");
MODULE_LICENSE("GPL v2");
