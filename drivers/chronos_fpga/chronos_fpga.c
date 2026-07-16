// SPDX-License-Identifier: GPL-2.0-only
/*
 * Chronos FPGA control driver
 *
 * Copyright (C) 2026 Chronos Project
 *
 * The Chronos board (HW_CHRONOS_R1) aggregates four OV9281 cameras in a
 * Lattice CrossLink-NX FPGA and forwards them to the Jetson Orin NX on a
 * single 2-lane MIPI CSI-2 link using virtual channels VC0..VC3.  The FPGA
 * is the only device on this board the Jetson can control: it exposes an
 * I2C slave (7-bit 0x3C) implementing the register file described in
 * fpga/rtl/config_regs.sv.  The sensors are on FPGA-private SCCB buses and
 * are initialised by the FPGA itself; the ICM-42688-P IMU is wired to the
 * FPGA only.
 *
 * This driver:
 *   - binds to the "chronos,fpga-ctrl" node on the Jetson camera I2C bus,
 *   - verifies the ID/VERSION registers (warns but does not fail on
 *     mismatch, so bring-up with older bitstreams still works),
 *   - exposes the control/status registers through sysfs,
 *   - exports a small in-kernel API used by the chronos_csi bridge to
 *     start/stop the FSIN trigger generator when streaming starts/stops.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>

#include "chronos_fpga.h"

struct chronos_fpga {
	struct i2c_client *client;
	struct regmap *regmap;
	/* Serialises multi-register sequences (16-bit L/H pairs). */
	struct mutex lock;
};

/*
 * 0x20..0x37 (status, error, frame counters) and 0xF0..0xFF (version/ID)
 * change under hardware control and must never be served from cache.
 * CTRL (0x00) is volatile too because bit 1 (soft reset) self-clears.
 */
static const struct regmap_range chronos_volatile_ranges[] = {
	regmap_reg_range(CHRONOS_REG_CTRL, CHRONOS_REG_CTRL),
	regmap_reg_range(CHRONOS_REG_STATUS, CHRONOS_REG_FRAME_CNT_H(3)),
	regmap_reg_range(CHRONOS_REG_VERSION, CHRONOS_REG_ID_H),
};

static const struct regmap_access_table chronos_volatile_table = {
	.yes_ranges = chronos_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(chronos_volatile_ranges),
};

static const struct regmap_config chronos_fpga_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
	.volatile_table = &chronos_volatile_table,
	.cache_type = REGCACHE_RBTREE,
};

/* ---------------------------------------------------------------------------
 * In-kernel API (used by the chronos_csi bridge)
 * ------------------------------------------------------------------------- */

int chronos_fpga_trigger_enable(struct i2c_client *client, bool enable)
{
	struct chronos_fpga *priv;

	if (!client)
		return -ENODEV;

	priv = i2c_get_clientdata(client);
	if (!priv)
		return -ENODEV;

	return regmap_update_bits(priv->regmap, CHRONOS_REG_CTRL,
				  CHRONOS_CTRL_TRIG_EN,
				  enable ? CHRONOS_CTRL_TRIG_EN : 0);
}
EXPORT_SYMBOL_GPL(chronos_fpga_trigger_enable);

int chronos_fpga_set_frame_rate(struct i2c_client *client, u8 fps)
{
	struct chronos_fpga *priv;

	if (!client)
		return -ENODEV;

	priv = i2c_get_clientdata(client);
	if (!priv)
		return -ENODEV;

	if (fps < CHRONOS_FRAME_RATE_MIN || fps > CHRONOS_FRAME_RATE_MAX)
		return -EINVAL;

	return regmap_write(priv->regmap, CHRONOS_REG_FRAME_RATE, fps);
}
EXPORT_SYMBOL_GPL(chronos_fpga_set_frame_rate);

int chronos_fpga_read_status(struct i2c_client *client, u8 *status)
{
	struct chronos_fpga *priv;
	unsigned int val;
	int ret;

	if (!client || !status)
		return -EINVAL;

	priv = i2c_get_clientdata(client);
	if (!priv)
		return -ENODEV;

	ret = regmap_read(priv->regmap, CHRONOS_REG_STATUS, &val);
	if (ret)
		return ret;

	*status = val;
	return 0;
}
EXPORT_SYMBOL_GPL(chronos_fpga_read_status);

/**
 * chronos_fpga_get_client - resolve an "fpga" phandle to the bound client
 * @np: device node of the FPGA control device (chronos,fpga-ctrl)
 *
 * Returns the i2c_client with a device reference held (drop it with
 * put_device(&client->dev)), ERR_PTR(-EPROBE_DEFER) if the node exists but
 * this driver has not bound yet, or ERR_PTR(-ENODEV) on bad input.
 */
struct i2c_client *chronos_fpga_get_client(struct device_node *np)
{
	struct i2c_client *client;

	if (!np)
		return ERR_PTR(-ENODEV);

	/*
	 * of_find_i2c_device_by_node() looks the node up on the I2C bus and
	 * runs i2c_verify_client() on the match, so a non-NULL result is
	 * guaranteed to be a real client (reference taken on its device).
	 */
	client = of_find_i2c_device_by_node(np);
	if (!client)
		return ERR_PTR(-EPROBE_DEFER);

	/* Not bound (or bound to something else) yet: defer, don't crash. */
	if (!i2c_get_clientdata(client) ||
	    !client->dev.driver ||
	    strcmp(client->dev.driver->name, "chronos-fpga")) {
		put_device(&client->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	return client;
}
EXPORT_SYMBOL_GPL(chronos_fpga_get_client);

/* ---------------------------------------------------------------------------
 * sysfs
 * ------------------------------------------------------------------------- */

static ssize_t trigger_enable_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, CHRONOS_REG_CTRL, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", !!(val & CHRONOS_CTRL_TRIG_EN));
}

static ssize_t trigger_enable_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t len)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	ret = chronos_fpga_trigger_enable(priv->client, enable);
	if (ret)
		return ret;

	return len;
}
static DEVICE_ATTR_RW(trigger_enable);

static ssize_t frame_rate_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, CHRONOS_REG_FRAME_RATE, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", val);
}

static ssize_t frame_rate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	u8 fps;
	int ret;

	ret = kstrtou8(buf, 0, &fps);
	if (ret)
		return ret;

	ret = chronos_fpga_set_frame_rate(priv->client, fps);
	if (ret)
		return ret;

	return len;
}
static DEVICE_ATTR_RW(frame_rate);

static ssize_t pulse_width_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	unsigned int lo, hi;
	int ret;

	mutex_lock(&priv->lock);
	ret = regmap_read(priv->regmap, CHRONOS_REG_PULSE_WIDTH_L, &lo);
	if (!ret)
		ret = regmap_read(priv->regmap, CHRONOS_REG_PULSE_WIDTH_H, &hi);
	mutex_unlock(&priv->lock);

	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", (hi << 8) | lo);
}

static ssize_t pulse_width_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	u16 width;
	int ret;

	ret = kstrtou16(buf, 0, &width);
	if (ret)
		return ret;

	mutex_lock(&priv->lock);
	ret = regmap_write(priv->regmap, CHRONOS_REG_PULSE_WIDTH_L,
			   width & 0xFF);
	if (!ret)
		ret = regmap_write(priv->regmap, CHRONOS_REG_PULSE_WIDTH_H,
				   width >> 8);
	mutex_unlock(&priv->lock);

	if (ret)
		return ret;

	return len;
}
static DEVICE_ATTR_RW(pulse_width);

static ssize_t cam_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, CHRONOS_REG_CAM_ENABLE, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "0x%x\n", val & 0xF);
}

static ssize_t cam_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	u8 mask;
	int ret;

	ret = kstrtou8(buf, 0, &mask);
	if (ret)
		return ret;

	if (mask > 0xF)
		return -EINVAL;

	ret = regmap_write(priv->regmap, CHRONOS_REG_CAM_ENABLE, mask);
	if (ret)
		return ret;

	return len;
}
static DEVICE_ATTR_RW(cam_enable);

static ssize_t soft_reset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	bool reset;
	int ret;

	ret = kstrtobool(buf, &reset);
	if (ret)
		return ret;

	if (!reset)
		return -EINVAL;

	/* Self-clearing in hardware; no need to clear it back. */
	ret = regmap_update_bits(priv->regmap, CHRONOS_REG_CTRL,
				 CHRONOS_CTRL_SOFT_RESET,
				 CHRONOS_CTRL_SOFT_RESET);
	if (ret)
		return ret;

	return len;
}
static DEVICE_ATTR_WO(soft_reset);

static ssize_t status_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, CHRONOS_REG_STATUS, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "raw=0x%02x pll_locked=%d cam_sync=0x%lx\n",
			  val, !!(val & CHRONOS_STATUS_PLL_LOCKED),
			  (val & CHRONOS_STATUS_CAM_SYNC_MASK) >>
				CHRONOS_STATUS_CAM_SYNC_SHIFT);
}
static DEVICE_ATTR_RO(status);

static ssize_t error_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, CHRONOS_REG_ERROR, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "raw=0x%02x rx_error=0x%lx buf_overflow=0x%lx\n",
			  val, val & CHRONOS_ERROR_RX_MASK,
			  (val & CHRONOS_ERROR_OVF_MASK) >>
				CHRONOS_ERROR_OVF_SHIFT);
}
static DEVICE_ATTR_RO(error);

static ssize_t chronos_frame_count_show(struct device *dev, int cam, char *buf)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	unsigned int lo, hi;
	int ret;

	mutex_lock(&priv->lock);
	ret = regmap_read(priv->regmap, CHRONOS_REG_FRAME_CNT_L(cam), &lo);
	if (!ret)
		ret = regmap_read(priv->regmap, CHRONOS_REG_FRAME_CNT_H(cam),
				  &hi);
	mutex_unlock(&priv->lock);

	if (ret)
		return ret;

	return sysfs_emit(buf, "%u\n", (hi << 8) | lo);
}

#define CHRONOS_FRAME_COUNT_ATTR(n)					\
static ssize_t frame_count_##n##_show(struct device *dev,		\
				      struct device_attribute *attr,	\
				      char *buf)			\
{									\
	return chronos_frame_count_show(dev, n, buf);			\
}									\
static DEVICE_ATTR_RO(frame_count_##n)

CHRONOS_FRAME_COUNT_ATTR(0);
CHRONOS_FRAME_COUNT_ATTR(1);
CHRONOS_FRAME_COUNT_ATTR(2);
CHRONOS_FRAME_COUNT_ATTR(3);

static ssize_t version_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, CHRONOS_REG_VERSION, &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "0x%02x\n", val);
}
static DEVICE_ATTR_RO(version);

static ssize_t id_show(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	struct chronos_fpga *priv = dev_get_drvdata(dev);
	unsigned int lo, hi;
	int ret;

	mutex_lock(&priv->lock);
	ret = regmap_read(priv->regmap, CHRONOS_REG_ID_L, &lo);
	if (!ret)
		ret = regmap_read(priv->regmap, CHRONOS_REG_ID_H, &hi);
	mutex_unlock(&priv->lock);

	if (ret)
		return ret;

	return sysfs_emit(buf, "0x%02x%02x\n", hi, lo);
}
static DEVICE_ATTR_RO(id);

static struct attribute *chronos_fpga_attrs[] = {
	&dev_attr_trigger_enable.attr,
	&dev_attr_frame_rate.attr,
	&dev_attr_pulse_width.attr,
	&dev_attr_cam_enable.attr,
	&dev_attr_soft_reset.attr,
	&dev_attr_status.attr,
	&dev_attr_error.attr,
	&dev_attr_frame_count_0.attr,
	&dev_attr_frame_count_1.attr,
	&dev_attr_frame_count_2.attr,
	&dev_attr_frame_count_3.attr,
	&dev_attr_version.attr,
	&dev_attr_id.attr,
	NULL,
};
ATTRIBUTE_GROUPS(chronos_fpga);

/* ---------------------------------------------------------------------------
 * Probe / remove
 * ------------------------------------------------------------------------- */

static int chronos_fpga_check_id(struct chronos_fpga *priv)
{
	struct device *dev = &priv->client->dev;
	unsigned int id_l, id_h, version;
	int ret;

	ret = regmap_read(priv->regmap, CHRONOS_REG_ID_L, &id_l);
	if (ret)
		return ret;
	ret = regmap_read(priv->regmap, CHRONOS_REG_ID_H, &id_h);
	if (ret)
		return ret;
	ret = regmap_read(priv->regmap, CHRONOS_REG_VERSION, &version);
	if (ret)
		return ret;

	if (id_l != CHRONOS_ID_L_EXPECTED || id_h != CHRONOS_ID_H_EXPECTED)
		dev_warn(dev,
			 "unexpected FPGA ID 0x%02x%02x (expected 0x%02x%02x) - wrong or stale bitstream?\n",
			 id_h, id_l,
			 CHRONOS_ID_H_EXPECTED, CHRONOS_ID_L_EXPECTED);

	if (version != CHRONOS_VERSION_EXPECTED)
		dev_warn(dev, "FPGA version 0x%02x (driver written for 0x%02x)\n",
			 version, CHRONOS_VERSION_EXPECTED);

	dev_info(dev, "Chronos FPGA id=0x%02x%02x version=0x%02x\n",
		 id_h, id_l, version);

	return 0;
}

static int chronos_fpga_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct chronos_fpga *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	mutex_init(&priv->lock);

	priv->regmap = devm_regmap_init_i2c(client,
					    &chronos_fpga_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "failed to init regmap\n");

	i2c_set_clientdata(client, priv);

	/* Warn on mismatch but keep going; fail only if the bus is dead. */
	ret = chronos_fpga_check_id(priv);
	if (ret)
		return dev_err_probe(dev, ret,
				     "FPGA not responding at 0x%02x\n",
				     client->addr);

	return 0;
}

static int chronos_fpga_remove(struct i2c_client *client)
{
	/* Stop triggering on unload; best effort. */
	chronos_fpga_trigger_enable(client, false);
	return 0;
}

static const struct of_device_id chronos_fpga_of_match[] = {
	{ .compatible = "chronos,fpga-ctrl" },
	{ }
};
MODULE_DEVICE_TABLE(of, chronos_fpga_of_match);

static const struct i2c_device_id chronos_fpga_id[] = {
	{ "chronos-fpga", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, chronos_fpga_id);

static struct i2c_driver chronos_fpga_driver = {
	.driver = {
		.name = "chronos-fpga",
		.of_match_table = chronos_fpga_of_match,
		.dev_groups = chronos_fpga_groups,
	},
	.probe = chronos_fpga_probe,
	.remove = chronos_fpga_remove,
	.id_table = chronos_fpga_id,
};
module_i2c_driver(chronos_fpga_driver);

MODULE_AUTHOR("Chronos Project");
MODULE_DESCRIPTION("Chronos FPGA control interface driver");
MODULE_LICENSE("GPL v2");
