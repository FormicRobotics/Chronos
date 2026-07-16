# Chronos Linux Drivers

Jetson-side kernel modules for the Chronos multi-camera synchronization
system. Target: **NVIDIA Jetson Orin NX**, L4T r35/r36 (kernel 5.10/5.15),
Tegra camera stack. License: GPL v2.

## Architecture (what the Jetson actually sees)

A Lattice CrossLink-NX FPGA aggregates four OV9281 cameras (1280x800 RAW10
monochrome, FSIN hardware-triggered, 30 fps design point) into **one 2-lane
MIPI CSI-2 link** with virtual channels VC0..VC3 (VC = camera index).

- The Jetson **cannot** reach the sensors over I2C — the FPGA owns
  per-camera private SCCB buses and configures the sensors itself. There is
  no I2C mux.
- The ICM-42688-P IMU is wired to the FPGA only. The Jetson has no path
  to it.
- The **only** Jetson-visible control interface is the FPGA I2C slave at
  7-bit address `0x3C` (CTRL, FRAME_RATE, PULSE_WIDTH, CAM_ENABLE, STATUS,
  ERROR, per-camera FRAME_CNT, VERSION `0x10`, ID `0xC405`).

```
4x OV9281 --SCCB/CSI--> CrossLink-NX FPGA --2-lane CSI-2 (VC0..3)--> Jetson NVCSI/VI
                              ^
                              | I2C 0x3C (control only)
                           Jetson i2c2
```

## Components

```
drivers/
├── Makefile                    # Top level: builds chronos_fpga + chronos_csi
├── chronos_fpga/               # FPGA control driver (i2c, "chronos,fpga-ctrl")
│   ├── chronos_fpga.c          # regmap + sysfs + exported in-kernel API
│   ├── chronos_fpga.h          # register map + API for consumers
│   ├── Makefile / Kconfig
├── chronos_csi/                # Aggregated CSI-2 source bridge
│   ├── chronos_csi.c           # V4L2 subdev ("chronos,csi-bridge"), no video nodes
│   ├── Makefile / Kconfig
├── chronos-orin-nx.dts         # DT overlay: FPGA node, bridge, NVCSI, VI (4 VCs)
├── ov9281/                     # NOT USED on Chronos (FPGA owns the sensors)
└── imu/                        # NOT USED on Chronos (IMU is FPGA-attached)
```

### chronos_fpga

I2C driver for the FPGA control interface. Verifies ID/VERSION at probe
(warns on mismatch), exposes sysfs attributes under the device
(`/sys/bus/i2c/devices/<bus>-003c/`):

| Attribute | Access | Meaning |
|---|---|---|
| `trigger_enable` | rw | FSIN trigger generator on/off |
| `frame_rate` | rw | fps, 1..120 (design point 30) |
| `pulse_width` | rw | FSIN pulse width, 16-bit |
| `cam_enable` | rw | camera enable bitmask, bits 3:0 |
| `soft_reset` | wo | write 1 to soft-reset (self-clearing) |
| `status` | ro | decoded: pll_locked + cam_sync bits |
| `error` | ro | decoded: rx_error + buf_overflow bits |
| `frame_count_0..3` | ro | per-camera 16-bit frame counter |
| `version`, `id` | ro | bitstream version / device ID |

It also exports an in-kernel API (`chronos_fpga_trigger_enable`,
`chronos_fpga_set_frame_rate`, `chronos_fpga_read_status`,
`chronos_fpga_get_client`) used by the bridge.

### chronos_csi

A media-controller **V4L2 subdevice** — not a video-node driver. It
represents the FPGA's aggregated CSI-2 source toward NVCSI: fixed
1280x800 `MEDIA_BUS_FMT_Y10_1X10`, one source pad, async-registered so the
Tegra VI/NVCSI graph binds it through the DT endpoint. Its `s_stream`
starts/stops the FPGA FSIN trigger (first stream on / last stream off).
The Tegra VI creates the `/dev/video*` nodes and handles all DMA.

### chronos-orin-nx.dts

Overlay with the real topology: FPGA control node on `i2c2`, one bridge
node whose single source port carries four endpoints (one per VC, all on
the same physical 2-lane link into NVCSI port A), four NVCSI channels and
a VI with `num-channels = <4>` - paired strictly 1:1 (bridge endpoint N ->
NVCSI channel N -> VI port N) with `vc-id = <0..3>` on every endpoint, the
mechanism L4T uses for multi-VC GMSL. Four minimal `tegra-camera-platform`
module entries reference the bridge subdev and size the bandwidth budget.

## Building

Per directory (or use the top-level Makefile which handles the ordering):

```bash
# Cross-compilation environment (or build on the Jetson with
# nvidia-l4t-kernel-headers installed)
export CROSS_COMPILE=aarch64-linux-gnu-
export ARCH=arm64
export KDIR=/path/to/jetson/kernel/build

cd drivers
make            # builds chronos_fpga.ko, then chronos_csi.ko
sudo make install && sudo depmod -a
```

`chronos_csi` uses symbols exported by `chronos_fpga`, so the top-level
Makefile builds `chronos_fpga` first and passes its `Module.symvers` via
`KBUILD_EXTRA_SYMBOLS`.

Compile the overlay:

```bash
dtc -@ -I dts -O dtb -o chronos-orin-nx.dtbo chronos-orin-nx.dts
sudo cp chronos-orin-nx.dtbo /boot/
# then register it via /boot/extlinux/extlinux.conf (FDT/OVERLAYS) or
# /opt/nvidia/jetson-io/jetson-io.py
```

## What works vs. what needs L4T integration

Works with just these modules:

- FPGA register access, sysfs control/status, FSIN trigger start/stop.
- The bridge subdev appearing in the media graph as the CSI source.

Requires L4T-side integration (kernel DT merge on the flashed system):

- **VI/NVCSI channel setup**: the overlay's `vi`/`nvcsi` fragments must
  apply on top of the platform DT; verify the resulting channels with
  `media-ctl -p`. Multi-VC capture needs an L4T release where the VI
  honors `vc-id` (r35+).
- **Argus / nvarguscamerasrc**: needs full per-mode tables (`mode0` etc.)
  in the tegra-camera-platform modules matching the FPGA's timing. The
  entries shipped here are minimal; plain V4L2 capture does not need them.

## Bring-up sequence

```bash
# 1. Load the FPGA control driver and verify the FPGA answers
sudo modprobe chronos_fpga
cat /sys/bus/i2c/devices/2-003c/id        # expect 0xc405
cat /sys/bus/i2c/devices/2-003c/version   # expect 0x10
cat /sys/bus/i2c/devices/2-003c/status    # expect pll_locked=1

# 2. Apply the overlay (reboot) so the bridge/NVCSI/VI graph exists,
#    then load the bridge
sudo modprobe chronos_csi
media-ctl -p                              # bridge entity linked to NVCSI

# 3. Capture per virtual channel (VC n -> /dev/video n)
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1280,height=800,pixelformat=Y10 \
         --stream-mmap --stream-count=30
```

Useful raw I2C checks (bus 2 assumed):

```bash
i2cget -y 2 0x3c 0xf0   # VERSION = 0x10
i2cget -y 2 0x3c 0x20   # STATUS (bit0 = pll_locked)
i2cset -y 2 0x3c 0x00 0x01   # manually enable the FSIN trigger
```

## Not used on this board

- `ov9281/` — full sensor driver; the sensors are unreachable from the
  Jetson and FPGA-configured. Kept as reference only. Do not load.
- `imu/` — Jetson SPI driver for the ICM-42688-P; the IMU is FPGA-attached.
  Kept for a future hardware revision or FPGA proxy. Do not load.
  See `imu/README.md`.
