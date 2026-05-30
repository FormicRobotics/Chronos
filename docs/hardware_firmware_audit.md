# Chronos R1 — Hardware vs. Firmware Audit

This document is the result of a deep, schematic-driven review of the
`HW_CHRONOS_R1` board against the firmware/driver/application source tree.
It records both the **board facts** (extracted from `hardware/Chronos.net`,
the Altium netlist) and the **firmware deviations** that were found and
fixed.

The intent is that anyone bringing the board up — synth-fitting the FPGA,
flashing JetPack, or wiring host code — has a single sheet of paper that
maps every important hardware net to the corresponding piece of source.

---

## 1. Board summary (HW_CHRONOS_R1)

| Block | Reference designator | Part number | Notes |
|---|---|---|---|
| FPGA aggregator | **U3** | **Lattice LIFCL-40 BG400** (`LIFCL-40-BG400`) | 40 k LUT, 5 MIPI D-PHY pairs available, single 12 MHz reference clock route through JP2 |
| IMU | **IC1** | **TDK InvenSense ICM-42688-P** | 14-pin LGA, SPI 4-wire, 1.8 V VDD/VDDIO, FSYNC tagged |
| USB bridge | U1 | FTDI FT2232HL | Provides JTAG (ch A) + UART (ch B) toward FPGA; X1 = 12 MHz crystal |
| Config flash | U6 | Macronix MX25L12833F | 16 MiB SPI NOR for FPGA bitstream |
| Quad buffer | U5 | TI SN74LVC125A | Quad 3-state, used to fan FSIN out to four cameras |
| Dual buffer | U9 | TI SN74LVC2G125 | Fans FSIN out to IMU + external sync header |
| Cam connectors | J3..J6 | Panasonic AXT524124 | 24-pin board-to-board, one per camera |
| Jetson CSI FFC | J7 | Hirose FH12-22S-0.5SH | 22-pin, 2-lane MIPI + sideband |
| Ext-sync header | P1 | Würth 620003111 21 | 3-pin, accepts external FSIN |
| Crystals | X2..X5 | 27 MHz, ASE3-27.000 | **One per camera, drives the camera's own MCLK directly — NOT routed to the FPGA** |
| Power rails on board | — | LT3508 ×2, NCP110AMX280, NCP110AMX120, TLV73310, BD00IC0WHFV | Generates +12V, +5V, +3V3, +2V8, +1V8, +1V8 LDO, +1V2, +1V0 LDO, +1V0 FPGA core |

### Critical signal map (FPGA U3 ⇄ board)

Derived directly from the netlist `Chronos.net`.

| Function | FPGA pin | Net |
|---|---|---|
| 12 MHz reference (FT_OSCO → JP2 → FPGA) | **L13** | `12MHZ` |
| Global reset / SW4 button | **G19** | `GSRN` |
| Camera-0 MIPI clock pair | **A2 / B1** | `CAM0_CLK_P / CAM0_CLK_N` |
| Camera-0 MIPI D0 pair | **B2 / C1** | `CAM0_0_P / CAM0_0_N` |
| Camera-0 MIPI D1 pair | **A3 / B3** | `CAM0_1_P / CAM0_1_N` |
| Camera-1 MIPI clock pair | **W11 / Y11** | `CAM1_CLK_P / CAM1_CLK_N` |
| Camera-1 MIPI D0 pair | **V11 / U11** | `CAM1_0_P / CAM1_0_N` |
| Camera-1 MIPI D1 pair | **W13 / V12** | `CAM1_1_P / CAM1_1_N` |
| Camera-2 MIPI clock pair | **A8 / B8** | `CAM2_CLK_P / CAM2_CLK_N` |
| Camera-2 MIPI D0 pair | **A7 / B7** | `CAM2_0_P / CAM2_0_N` |
| Camera-2 MIPI D1 pair | **A9 / B9** | `CAM2_1_P / CAM2_1_N` |
| Camera-3 MIPI clock pair | **T13 / T14** | `CAM3_CLK_P / CAM3_CLK_N` |
| Camera-3 MIPI D0 pair | **Y15 / Y16** | `CAM3_0_P / CAM3_0_N` |
| Camera-3 MIPI D1 pair | **U15 / V16** | `CAM3_1_P / CAM3_1_N` |
| CSI-2 TX clock pair (Jetson) | **T6 / U6** | `CSI_TX_CLK_P / CSI_TX_CLK_N` |
| CSI-2 TX D0 pair | **R7 / T7** | `CSI_TX_D0_P / CSI_TX_D0_N` |
| CSI-2 TX D1 pair | **V6 / U7** | `CSI_TX_D1_P / CSI_TX_D1_N` |
| FSIN out (single, feeds U5+U9) | **R9** | `FSIN_FPGA` |
| CAM_RESET (shared all 4 cams) | **W18** | `CAM_RESET` |
| Cam-0 I²C (SDA / SCL) | **N4 / N5** | `CAM0_SDA / CAM0_SCL` |
| Cam-1 I²C (SDA / SCL) | **P5 / P6** | `CAM1_SDA / CAM1_SCL` |
| Cam-2 I²C (SDA / SCL) | **P1 / P2** | `CAM2_SDA / CAM2_SCL` |
| Cam-3 I²C (SDA / SCL) | **N6 / N7** | `CAM3_SDA / CAM3_SCL` |
| Host config I²C (Jetson side) | **G14 / G15** | `I2C_CON_SDA / I2C_CON_SCL` |
| IMU SPI CS / SCLK | **R8 / P8** | `IMU_CS / IMU_SCLK` |
| IMU SPI MISO / MOSI | **T8 / P7** | `IMU_MISO / IMU_MOSI` |
| IMU INT1 (DRDY) | **Y7** | `IMU_INT1` |
| LED0..LED3 | **E17 / F13 / G13 / F14** | `LED0..LED3` |
| Debug UART RX / TX | **F18 / F16** | `RXD_UART / TXD_UART` |

Camera connector pinout (per AXT524124, identical on J3–J6):

```
1  SDA (per-cam I2C)
2  SCL (per-cam I2C)
3  VCCIO5  (cam I/O power, supplies the 27 MHz xtal too)
4  CAM_CLK_P    \  MIPI clock pair
5  CAM_CLK_N    /
6,9,12,17,24..28  GND
7  CAM_D0_P     \  MIPI data lane 0
8  CAM_D0_N     /
10 CAM_D1_P     \  MIPI data lane 1
11 CAM_D1_N     /
15 1V8 (via ferrite FBx, separate per camera)
16 FSIN (from external buffer U5)
18 V1P2  (camera DVDD)
19 MCLK  (27 MHz, dedicated on-board crystal)
21 CAM_RESET (shared with all 4 cameras)
22 V2P8  (camera AVDD)
```

---

## 2. Findings & fixes

### 🔴 Critical mismatches (firmware would not have worked as-is)

| # | What was wrong | Where | Why it matters | Fix |
|---|---|---|---|---|
| 1 | FPGA pinout constraints used placeholder sites (`A2/A3`, `D2/D3`, … `M3`, `L5`, …) | `fpga/constraints/chronos_pinout.pdc` | Bitstream would have shorted MIPI nets, lost the JP2 12 MHz route, missed the GSR pin, and used non-DPHY-capable sites. | Rewrote PDC with the actual `LIFCL-40-BG400` sites taken from the netlist (see Section 1). |
| 2 | IMU driver was for **Bosch BMI088** but the board has **TDK ICM-42688-P** (different die, different SPI command set, different number of CS lines) | `drivers/imu/bmi088_chronos.c` | Driver would have failed WHO_AM_I, attempted writes to the wrong addresses, and never produced samples. | Replaced with `drivers/imu/icm42688_chronos.c` — full IIO driver with FSYNC tagging, INT1 IRQ, proper scales, and a sync-count sysfs surface for the user-space lib. |
| 3 | Device tree referenced `bosch,bmi088`, declared a single `ov9281@60`, and described a synthetic 24 MHz `chronos_clk` driving every camera's `xvclk` | `drivers/chronos-orin-nx.dts` | Wrong compatible string → no driver bound. Single sensor node → other three cameras invisible. The cameras get MCLK from their own 27 MHz crystals; the kernel does not drive the clock at all. | Rewrote DTS: four `ov9281` nodes under an FPGA `i2c-mux`, ICM-42688-P SPI node with INT1 mapped to GPIO-H-6, virtual 27 MHz fixed-clock provider, proper `tegra-camera-platform` modules list. |
| 4 | `chronos_top.sv` exposed **four** separate `cam_trigger[3:0]` pins and a fifth `imu_trigger` | `fpga/rtl/chronos_top.sv` | The board only routes **one** FSIN pin from the FPGA (`R9`) which is fanned out by external buffers `U5`/`U9`. Five outputs cannot be assigned because the sites don't exist on the board. | Single `fsin_fpga` port; instances of `cam_trigger`/`imu_trigger` removed and the on-board buffer chain documented. |
| 5 | `config_regs` had ports for `pulse_width`, `trigger_delay [4]`, `soft_reset`, `cam_enable`, `output_data_type`, `cam_sync_status`, `frame_count [4]` that were left **unconnected** in `chronos_top.sv` | `chronos_top.sv` ↔ `config_regs.sv` | Elaboration would have failed (or required black-box overrides); the host-visible register bank would have been functionally read-only. | All ports wired in `chronos_top.sv`. Frame counters now increment on each `frame_start` per camera; `cam_sync_status[i]` is asserted when there is no current RX/overflow error. |
| 6 | `trigger_generator` declared `trigger_delay[4]` while the generate loop iterated `i = 0 .. NUM_OUTPUTS-1` with `NUM_OUTPUTS = 5` | `fpga/rtl/trigger_generator.sv` | Out-of-bounds array access, undefined synthesis result. | Array sized by the `NUM_OUTPUTS` parameter; loop and connections rewritten. |
| 7 | `trigger_generator`'s "delayed trigger" logic deasserted the output mid-pulse if `trigger_delay > 0` | same | Cameras would have received pulses shorter than the configured width, breaking the OV9281's minimum-FSIN-width spec. | Replaced delay-counter scheme with a `DELAY_SR_DEPTH`-deep shift register fed by the master pulse — pulse width preserved by construction. |
| 8 | `chronos_pll.sv` synthesised to `clk_200m = clk_ref` and `clk_byte = clk_ref` (placeholder) | `fpga/rtl/chronos_pll.sv` | All "200 MHz" logic, CSI byte clock, and frame-period math would have run at the 25 MHz (assumed) input rate; in reality the reference is **12 MHz**. | Instantiated `PLL_CORE` with `CLKI_DIV=1`, `CLKFB_DIV=50`, `CLKOP_DIV=3`, `CLKOS_DIV=6` → 200 MHz / 100 MHz. Simulation path generates the same rates behaviourally. |

### 🟠 High-impact bugs

| # | What was wrong | Where | Fix |
|---|---|---|---|
| 9 | `chronos_configure()` was a `TODO` — nothing was ever pushed to the FPGA | `app/src/chronos_capture.c` | Implemented `push_config_to_fpga()` that talks to `/dev/i2c-1` (configurable) and writes `FRAME_RATE`, `PULSE_WIDTH_{L,H}`, `CAM_ENABLE`, `DATA_TYPE`, and per-camera `TRIG_DELAY_x`. Also pokes per-sensor `V4L2_CID_EXPOSURE`. |
| 10 | Capture thread's `select()` referenced `cam->fd` even for cameras that failed to open (fd = 0) and re-queued the wrong buffer indices | `chronos_capture.c` (`capture_thread_func`) | Active-camera count tracked at thread start; re-queue uses the exact indices that were dequeued this iteration. |
| 11 | IMU sysfs reader used `atoi()` (cannot parse negative numbers) and hard-coded scales | `chronos_capture.c` (`read_imu_data`) | Reworked to `strtol` + read `in_*_scale` from sysfs; also pulls `sync_count` / `last_sync_time` attrs from the new ICM-42688 driver. |
| 12 | Kernel driver disabled the FPGA trigger when VC0 stopped streaming, regardless of whether VC1..VC3 were still active (and only enabled it when VC0 started) | `drivers/chronos_csi/chronos_csi.c` | `chronos_count_streaming()` helper; trigger gated on "any channel streaming". Also down-graded the `i2c_client == NULL` path to an info-level message because the user-space lib already programs the FPGA. |

### 🟡 Smaller cleanups

* All `Makefile` references to BMI088 retargeted to the ICM-42688 module (`CONFIG_IIO_ICM42688_CHRONOS`).
* Removed the dangling `gpio-hog` entry that pinned `H,3` low forever (the cameras' shared reset is owned by the FPGA, not by a Jetson GPIO).
* `chronos_top.sv` now also exposes the debug UART, host I²C, per-camera I²C, IMU SPI, and `cam_resetn` ports so the constraint file's nets actually resolve.

### 🟢 Things that were already fine

* OV9281 V4L2 subdev driver (`drivers/ov9281/ov9281.c`) — register map, mode table, controls, and PM ops are correct for the OV9281 datasheet. The only behavioural concern is the lack of `reset-gpios` (now intentional — there's no per-camera reset to bind to), which is handled by the FPGA.
* CSI-2 RX/TX RTL is a behavioural skeleton, but is consistent across `csi2_rx.sv`, `csi2_tx.sv`, `frame_buffer.sv`, `tx_arbiter.sv`, and `i2c_slave.sv` — they all share a common 32-bit packet interface. The hardened D-PHY hard-IP still has to be instantiated through Lattice Radiant's IP catalog; the wrappers (`dphy_rx_wrapper`, `dphy_tx_wrapper`) leave the right hooks.

---

## 3. Bring-up checklist (post-fix)

1. **JP2 jumper installed** so 12 MHz from FT_OSCO reaches FPGA pin L13.
2. **FPGA bit-stream** built with the new `chronos_pinout.pdc`; PLL primitive
   wired exactly as in `chronos_pll.sv`.
3. **JTAG via FT2232HL** — flash `Chronos.bit` into `U6` (MX25L12833F);
   first power-cycle should leave `LED0` (PLL locked) lit within ~1 ms.
4. **Jetson Orin NX** boots JetPack 5.x. Build & load drivers:
   ```bash
   sudo make -C drivers/ov9281
   sudo make -C drivers/chronos_csi
   sudo make -C drivers/imu
   sudo cp drivers/chronos-orin-nx.dtbo /boot/dtb/overlays/
   sudo modprobe ov9281 chronos-csi icm42688-chronos
   ```
5. **Sanity over I²C-2** (default bus on the Jetson devkit):
   ```bash
   i2cdump -y 1 0x3C b
   ```
   Should expose firmware version `0x10`, device ID `0xC405` at `0xFE/0xFF`.
6. **Sensors over the FPGA I²C-mux** — each `/dev/i2c-N` child should show
   `ov9281` at address `0x60`. WHO_AM_I returns `0x9281`.
7. **IMU** — `cat /sys/bus/iio/devices/iio:device0/name` →
   `icm42688-chronos`. `dmesg` should show `ICM-42688-P detected
   (WHO_AM_I=0x47)`.
8. **Streaming**:
   ```bash
   chronos_demo --rate 60 --exposure 5000
   sync_test  -n 1000 -r 120
   ```
   `sync_test` exercises the `<100 ns` skew claim end-to-end.

---

## 4. Remaining work (not part of this fix-up pass)

The items below are needed for *full* functional bring-up but are outside
the scope of a hardware/firmware reconciliation; they are flagged here so
they don't get lost.

1. **CSI-2 D-PHY hard IP** — instantiate `DPHY` / `DPHY_RX` / `DPHY_TX`
   atoms from Radiant's IP catalog inside `dphy_rx_wrapper` and
   `dphy_tx_wrapper`. The current files are deliberate behavioural stubs.
2. **Per-camera FPGA I²C master cores** — the design currently tri-states
   the four camera I²C buses; the Jetson speaks to the cameras over those
   buses through the FPGA. A small I²C master (4 instances) selected by
   the host-side register bank is required.
3. **NVCSI/VI integration** — `chronos_csi.c` currently exposes four
   `videoX` nodes but has no DMA path to NVCSI. The right path on Orin is
   either tegracam plumbing (V4L2 sub-device with NVCSI underneath) or a
   custom kernel module that programs `vi5` channels directly with the
   four virtual channels demultiplexed.
4. **Bitstream-time `output_data_type` register** is currently exposed in
   the register bank but unused; pipe it into `csi2_tx.sv`'s packet
   builder so the host can switch between RAW8/RAW10/RAW12.
5. **CAM_RESET timing** — `chronos_top.sv` toggles the shared reset on
   `soft_reset`, but the OV9281 datasheet wants `t_reset >= 1 ms`. The
   200 MHz timer in this file is way too short (256 cycles = 1.28 µs).
   Either widen the counter or pull the reset signal high through a
   millisecond-scale debouncer before deassertion.

---

*This audit was produced on 2026-05-16 using only the files in this
repository — no live measurements taken.  Anything that does not match a
populated board should be verified against the schematic PDF
(`hardware/sch.pdf` / `Schematic/HW_CHRONOS_R1.PDF`).*
