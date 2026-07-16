/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Chronos FPGA control interface - shared register map and in-kernel API.
 *
 * The Lattice CrossLink-NX FPGA on the Chronos board is the ONLY device the
 * Jetson can talk to: it exposes an I2C slave at 7-bit address 0x3C whose
 * register file (fpga/rtl/config_regs.sv) controls the FSIN trigger
 * generator, per-camera enables and reports aggregation status.  The OV9281
 * sensors themselves sit on FPGA-private SCCB buses and are configured by
 * the FPGA - they are NOT reachable from the Jetson.
 *
 * Copyright (C) 2026 Chronos Project
 */

#ifndef __CHRONOS_FPGA_H__
#define __CHRONOS_FPGA_H__

#include <linux/bits.h>
#include <linux/types.h>

struct i2c_client;
struct device_node;

/*
 * Register map (must match fpga/rtl/config_regs.sv)
 */
#define CHRONOS_REG_CTRL		0x00
#define CHRONOS_CTRL_TRIG_EN		BIT(0)	/* FSIN trigger generator on */
#define CHRONOS_CTRL_SOFT_RESET		BIT(1)	/* self-clearing */

#define CHRONOS_REG_FRAME_RATE		0x01	/* fps, default 30 */
#define CHRONOS_REG_PULSE_WIDTH_L	0x02	/* FSIN pulse width, low byte */
#define CHRONOS_REG_PULSE_WIDTH_H	0x03	/* FSIN pulse width, high byte */
#define CHRONOS_REG_CAM_ENABLE		0x04	/* bits 3:0, one per camera */
#define CHRONOS_REG_DATA_TYPE		0x05	/* CSI-2 DT, 0x2B = RAW10 */

#define CHRONOS_REG_TRIG_DELAY(n)	(0x10 + (n))	/* per-camera, n = 0..3 */

#define CHRONOS_REG_STATUS		0x20
#define CHRONOS_STATUS_PLL_LOCKED	BIT(0)
#define CHRONOS_STATUS_CAM_SYNC_SHIFT	4	/* bits 7:4 = cam_sync_status */
#define CHRONOS_STATUS_CAM_SYNC_MASK	GENMASK(7, 4)

#define CHRONOS_REG_ERROR		0x21
#define CHRONOS_ERROR_RX_MASK		GENMASK(3, 0)	/* per-camera RX error */
#define CHRONOS_ERROR_OVF_SHIFT		4
#define CHRONOS_ERROR_OVF_MASK		GENMASK(7, 4)	/* per-camera overflow */

#define CHRONOS_REG_FRAME_CNT_L(n)	(0x30 + 2 * (n))	/* n = 0..3 */
#define CHRONOS_REG_FRAME_CNT_H(n)	(0x31 + 2 * (n))

#define CHRONOS_REG_VERSION		0xF0
#define CHRONOS_REG_ID_L		0xFE
#define CHRONOS_REG_ID_H		0xFF

#define CHRONOS_VERSION_EXPECTED	0x10
#define CHRONOS_ID_L_EXPECTED		0x05
#define CHRONOS_ID_H_EXPECTED		0xC4

#define CHRONOS_FRAME_RATE_MIN		1
#define CHRONOS_FRAME_RATE_MAX		120
#define CHRONOS_NUM_CAMERAS		4

/*
 * In-kernel API for consumers (chronos_csi bridge).
 *
 * All calls take the i2c_client bound to the "chronos,fpga-ctrl" node.
 * Resolve it from a DT phandle with chronos_fpga_get_client(); the returned
 * client holds a device reference that the caller must drop with
 * put_device(&client->dev) when done.
 */
int chronos_fpga_trigger_enable(struct i2c_client *client, bool enable);
int chronos_fpga_set_frame_rate(struct i2c_client *client, u8 fps);
int chronos_fpga_read_status(struct i2c_client *client, u8 *status);
struct i2c_client *chronos_fpga_get_client(struct device_node *np);

#endif /* __CHRONOS_FPGA_H__ */
