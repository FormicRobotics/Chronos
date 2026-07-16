// SPDX-License-Identifier: GPL-2.0-only
/*
 * Chronos CSI-2 bridge subdevice
 *
 * Copyright (C) 2026 Chronos Project
 *
 * On the Chronos board a Lattice CrossLink-NX FPGA aggregates four OV9281
 * cameras (1280x800 RAW10 monochrome, FSIN hardware-triggered, 30 fps
 * design point) into ONE 2-lane MIPI CSI-2 link to the Jetson Orin NX,
 * with virtual channels VC0..VC3 (VC = camera index).  The Jetson sees a
 * single CSI-2 source; it cannot reach the sensors over I2C (the FPGA owns
 * per-camera private SCCB buses and configures the sensors itself).
 *
 * This driver is an honest model of that: a single media-controller V4L2
 * SUBDEV that represents the FPGA's aggregated CSI-2 source toward NVCSI.
 * It creates NO video nodes and owns NO buffers - the Tegra VI driver
 * creates the /dev/video nodes and handles DMA.  Per-VC routing is
 * configured through the tegra-camera-platform modules and the VI channel
 * setup in the device tree (four VI channels with vc-id 0..3, all fed by
 * the same NVCSI stream); this subdev is the single source entity of that
 * graph.
 *
 * The only runtime control the bridge performs is starting/stopping the
 * FPGA's FSIN trigger generator (via the chronos_fpga in-kernel API) when
 * the first stream starts / the last stream stops.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "../chronos_fpga/chronos_fpga.h"

#define CHRONOS_BRIDGE_NAME	"chronos-csi-bridge"

/* Fixed output format of the FPGA CSI-2 TX (per virtual channel). */
#define CHRONOS_WIDTH		1280
#define CHRONOS_HEIGHT		800
#define CHRONOS_MBUS_CODE	MEDIA_BUS_FMT_Y10_1X10

/*
 * The pad-op signature changed in 5.14 (v4l2_subdev_pad_config was folded
 * into v4l2_subdev_state).  L4T r35 is 5.10, r36 is 5.15, so support both.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0)
#define chronos_sd_state	v4l2_subdev_pad_config
#else
#define chronos_sd_state	v4l2_subdev_state
#endif

struct chronos_bridge {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;		/* single source pad toward NVCSI */

	/* FPGA control client, resolved from the "fpga" phandle in probe. */
	struct i2c_client *fpga_client;

	/* Protects stream_count and the FPGA trigger writes. */
	struct mutex lock;
	int stream_count;
};

static inline struct chronos_bridge *to_bridge(struct v4l2_subdev *sd)
{
	return container_of(sd, struct chronos_bridge, sd);
}

/* ---------------------------------------------------------------------------
 * Pad operations - fixed 1280x800 Y10 source
 * ------------------------------------------------------------------------- */

static void chronos_fill_fmt(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = CHRONOS_WIDTH;
	fmt->height = CHRONOS_HEIGHT;
	fmt->code = CHRONOS_MBUS_CODE;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int chronos_enum_mbus_code(struct v4l2_subdev *sd,
				  struct chronos_sd_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = CHRONOS_MBUS_CODE;
	return 0;
}

static int chronos_get_fmt(struct v4l2_subdev *sd,
			   struct chronos_sd_state *state,
			   struct v4l2_subdev_format *fmt)
{
	chronos_fill_fmt(&fmt->format);
	return 0;
}

/*
 * The FPGA output is not configurable from the host; set_fmt always
 * returns the fixed hardware format regardless of what was requested.
 */
static int chronos_set_fmt(struct v4l2_subdev *sd,
			   struct chronos_sd_state *state,
			   struct v4l2_subdev_format *fmt)
{
	chronos_fill_fmt(&fmt->format);
	return 0;
}

static const struct v4l2_subdev_pad_ops chronos_pad_ops = {
	.enum_mbus_code = chronos_enum_mbus_code,
	.get_fmt = chronos_get_fmt,
	.set_fmt = chronos_set_fmt,
};

/* ---------------------------------------------------------------------------
 * Video operations - FSIN trigger start/stop
 * ------------------------------------------------------------------------- */

static int chronos_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct chronos_bridge *bridge = to_bridge(sd);
	int ret = 0;

	mutex_lock(&bridge->lock);

	if (enable) {
		/*
		 * The VI driver calls s_stream once per VI channel; only the
		 * first one actually starts the FPGA trigger generator.
		 */
		if (bridge->stream_count == 0) {
			ret = chronos_fpga_trigger_enable(bridge->fpga_client,
							  true);
			if (ret) {
				dev_err(bridge->dev,
					"failed to enable FPGA trigger: %d\n",
					ret);
				goto unlock;
			}
			dev_dbg(bridge->dev, "FSIN trigger enabled\n");
		}
		bridge->stream_count++;
	} else {
		if (WARN_ON(bridge->stream_count == 0))
			goto unlock;

		bridge->stream_count--;
		if (bridge->stream_count == 0) {
			ret = chronos_fpga_trigger_enable(bridge->fpga_client,
							  false);
			if (ret)
				dev_warn(bridge->dev,
					 "failed to disable FPGA trigger: %d\n",
					 ret);
			else
				dev_dbg(bridge->dev, "FSIN trigger disabled\n");
			/* Stream is stopping either way. */
			ret = 0;
		}
	}

unlock:
	mutex_unlock(&bridge->lock);
	return ret;
}

static const struct v4l2_subdev_video_ops chronos_video_ops = {
	.s_stream = chronos_s_stream,
};

static const struct v4l2_subdev_ops chronos_subdev_ops = {
	.video = &chronos_video_ops,
	.pad = &chronos_pad_ops,
};

/* ---------------------------------------------------------------------------
 * Probe / remove
 * ------------------------------------------------------------------------- */

static int chronos_bridge_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *fpga_np;
	struct chronos_bridge *bridge;
	struct i2c_client *client;
	int ret;

	bridge = devm_kzalloc(dev, sizeof(*bridge), GFP_KERNEL);
	if (!bridge)
		return -ENOMEM;

	bridge->dev = dev;
	mutex_init(&bridge->lock);
	platform_set_drvdata(pdev, bridge);

	/* Resolve the FPGA control client from the "fpga" phandle. */
	fpga_np = of_parse_phandle(dev->of_node, "fpga", 0);
	if (!fpga_np)
		return dev_err_probe(dev, -EINVAL,
				     "missing 'fpga' phandle in DT\n");

	client = chronos_fpga_get_client(fpga_np);
	of_node_put(fpga_np);
	if (IS_ERR(client))
		return dev_err_probe(dev, PTR_ERR(client),
				     "FPGA control driver not ready\n");
	bridge->fpga_client = client;

	v4l2_subdev_init(&bridge->sd, &chronos_subdev_ops);
	bridge->sd.owner = THIS_MODULE;
	bridge->sd.dev = dev;
	bridge->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(bridge->sd.name, sizeof(bridge->sd.name), "%s %s",
		 CHRONOS_BRIDGE_NAME, dev_name(dev));
	v4l2_set_subdevdata(&bridge->sd, bridge);

	/*
	 * The bridge acts as the "sensor" of the Tegra capture graph: one
	 * source pad feeding NVCSI port A.
	 */
	bridge->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	bridge->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&bridge->sd.entity, 1, &bridge->pad);
	if (ret) {
		dev_err(dev, "failed to init media entity: %d\n", ret);
		goto err_put_client;
	}

	/*
	 * Async registration: the Tegra VI/NVCSI notifier matches this
	 * subdev through the DT endpoint (bridge port -> nvcsi port@0).
	 */
	bridge->sd.fwnode = of_fwnode_handle(dev->of_node);
	ret = v4l2_async_register_subdev(&bridge->sd);
	if (ret) {
		dev_err(dev, "failed to register async subdev: %d\n", ret);
		goto err_clean_entity;
	}

	dev_info(dev,
		 "Chronos CSI bridge registered (2-lane, VC0..3, %ux%u Y10)\n",
		 CHRONOS_WIDTH, CHRONOS_HEIGHT);

	return 0;

err_clean_entity:
	media_entity_cleanup(&bridge->sd.entity);
err_put_client:
	put_device(&bridge->fpga_client->dev);
	return ret;
}

static int chronos_bridge_remove(struct platform_device *pdev)
{
	struct chronos_bridge *bridge = platform_get_drvdata(pdev);

	v4l2_async_unregister_subdev(&bridge->sd);
	media_entity_cleanup(&bridge->sd.entity);

	/* Make sure the trigger is off when the bridge goes away. */
	chronos_fpga_trigger_enable(bridge->fpga_client, false);
	put_device(&bridge->fpga_client->dev);

	return 0;
}

static const struct of_device_id chronos_bridge_of_match[] = {
	{ .compatible = "chronos,csi-bridge" },
	{ }
};
MODULE_DEVICE_TABLE(of, chronos_bridge_of_match);

static struct platform_driver chronos_bridge_driver = {
	.probe = chronos_bridge_probe,
	.remove = chronos_bridge_remove,
	.driver = {
		.name = CHRONOS_BRIDGE_NAME,
		.of_match_table = chronos_bridge_of_match,
	},
};
module_platform_driver(chronos_bridge_driver);

MODULE_AUTHOR("Chronos Project");
MODULE_DESCRIPTION("Chronos aggregated CSI-2 source bridge subdevice");
MODULE_LICENSE("GPL v2");
/* Imports symbols from chronos_fpga.ko; make modprobe load it first. */
MODULE_SOFTDEP("pre: chronos_fpga");
