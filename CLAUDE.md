# Chronos - Project Memory (CLAUDE.md)

Multi-camera MIPI synchronization system. Aggregates four MIPI CSI-2 cameras into a
single synchronized CSI-2 stream for an NVIDIA Jetson Orin NX, using a Lattice
CrossLink-NX FPGA for MIPI aggregation, frame-sync triggering, and host control.

This file is durable project memory. Update it when hard-won facts change.

---

## 1. Hardware (board HW_CHRONOS_R1)

- FPGA: Lattice CrossLink-NX, U3, package CABGA400 (BG400).
  - Chip marking: `LIFCL-40-8BG400C`.
  - CRITICAL: the silicon is an Engineering Sample (ES1). JTAG/SSPI IDCODE = `0x010F1043`
    (production parts read `0x110F1043`). The production-looking marking is misleading;
    trust the IDCODE. This single fact drove weeks of programming failures - see section 3.
- Config/boot flash: U6 = Macronix MX25L12833F, 128 Mbit (16 MB) SPI NOR, SO8.
  - Wired to the FPGA MSPI (master config SPI) pins: DQ0->U3-D13, DQ1->U3-D15,
    DQ2->U3-D14, DQ3->U3-D16, CS->U6-1, CLK(SPI_MCLK)->U6-6. Header J20 breaks out the
    flash quad-SPI signals.
- USB/JTAG/UART bridge: FT2232HL. Wired to the FPGA Slave SPI (SSPI) pins, NOT JTAG.
  Programming must use SSPI. Channel A = SSPI/config, the other channel = UART.
- Buttons: SW4 = GSRN (reset, active-low, pull-up R69). SW5 = PROGRAMN (config reset).
- Jumpers for programming: JP1 open; the config-enable jumper must be fitted (lights D17
  red). JP2 closed feeds the 12 MHz reference clock to the FPGA.
- Cameras: 4x SincereFirst SF-AOV9281 (sensor OV9281 CSP) on connectors J3..J6
  (AXT524124 socket, module tail = AXT624124 — direct mate, no adapter).
  Earlier candidates (Leopard LI-OV9281-MIPI, Waveshare OV9281-120) were dropped.
  - 1280x800 active, 10-bit RAW mono, 2 data lanes + clock. Rails from board:
    pin3=V1P8 (DOVDD), pin18=V1P2 (DVDD), pin22=V2P8 (AVDD) — netlist-verified.
  - XCLK: the module has NO oscillator. 27 MHz osc X2 -> FPGA L5 (clk_27m); FPGA
    forwards it to cam_mclk[3:0] = M3/M4/M5/M6 -> J3..J6 pin 19. No MCLK = no SCCB.
  - SID strap (pin 17) is mixed: J3/J5 grounded -> SCCB 7-bit 0x60; J4/J6 pulled to
    VCCIO6 -> 0x10. Each camera has a dedicated I2C bus; chronos_top sets DEV_ADDR
    per instance.
  - Module pins 13/14/20/23 (LPWM/GPIO2/STROBE/ULPM) are not connected on the board.
  - FSIN: single FPGA pin R9 -> U5/U9 buffers (V1P8-powered) -> J3..J6 pin 16.
    RESET: shared CAM_RESET, FPGA W18 -> all J*-21, active-low.
- IMU: ICM-42688-P (IC1), SPI.
- Reference clock: no crystal on the FPGA; 12 MHz from FT2232HL reaches FPGA pin L13 via JP2.
  Separate 27 MHz oscillator X2 feeds FPGA pin L5 (camera XCLK source).
- User LEDs LED0..3 (E17/F13/G13/F14) are ACTIVE-LOW: D3..D6 anodes hang from VCCIO1
  via R174..R177, FPGA sinks the cathode. (Q1..Q6 NPNs drive power-rail/DONE LEDs, not
  these.) Same topology as D17/INITN (red = config active).
- Bank 1 (VCCIO1) is 3.3 V: it hosts JTAG pull-ups, UART to FT2232 (3.3 V I/O), LEDs,
  GSRN and host I2C. PDC uses LVCMOS33 for all of them. Verify once at TP_VCCIO1.

---

## 2. Toolchain

- Radiant 2.0 SP1 (version 2.0.1) at `C:\lscc\radiant\2.0` - USE THIS for the ES silicon.
  - ES1 silicon (IDCODE 0x010F1043) is only supported by Radiant 2.0 SP1. Newer Radiant
    (2025.2.1 at `C:\lscc\radiant\2025.2.1`) dropped ES1 support and emits bitstreams with
    the production IDCODE 0x110F1043 -> "Mismatch to device ID code" on this board.
  - LSE synthesis. Radiant 2.0 SP1 does NOT fully support SystemVerilog (native SV arrived
    in Radiant 2.1). For 2.0 SP1, use plain Verilog-2001 (see the blinky note below).
- Radiant Programmer: use the 2025.2 Programmer for actual programming - its FTDI cable
  driver works with this board and it talks SSPI fine (reads 0x010F1043). The old 2.0
  programmer's cable driver fails to scan on this board (reads 0x7FFFFFFF).
  - So: build bitstream in Radiant 2.0 SP1, program with the 2025.2 Programmer.
- Windows gotcha: Smart App Control (SAC) blocks Radiant 2.0's unsigned DLLs
  ("Application Control policy has blocked this file"). Running Radiant as Administrator
  bypassed it; otherwise SAC must be turned off (permanent until Windows reinstall).
- OSC_CORE primitive interface is identical between 2.0 and 2025 (verified).

---

## 3. Programming the FPGA (proven, working recipe)

Build bitstream in Radiant 2.0 SP1, then program with the 2025.2 Radiant Programmer.

SRAM (volatile, for quick test):
- Device Family LIFCL_ENG, Device LIFCL-40-ES, Target Memory SRAM, Port Interface Slave SPI,
  Access Mode Direct Programming, Operation Fast Configuration, file = the .bit.

SPI flash (permanent, auto-boots on power-up):
- Target Memory External SPI Flash Memory, Port Interface SSPI2SPI (the FPGA bridges SSPI
  to the flash), Operation Erase,Program,Verify.
- SPI Flash: Family SPI Serial Flash, Vendor Macronix, Device MX25L12833F, Package 8-pin SOP,
  Start 0x00000000, file = the .bit.
- After programming, power-cycle (or press SW5/PROGRAMN); FPGA boots from flash, D17 red
  goes off when config completes.

Hardware prep: JP1 open, config jumper fitted (D17 red), TCK divider conservative (>=2;
30 worked). JTAG_EN low for SSPI.

Bitstream IDCODE verification: the embedded IDCODE byte sequence `01 0F 10 43` should appear
(around offset ~0x196). A 2025-built bitstream will instead contain `11 0F 10 43` and will be
rejected. Do NOT hand-patch the IDCODE - it invalidates the bitstream CRC ("Invalid Bitstream").

### Blinky bring-up (already validated end-to-end)
- RTL `fpga/blinky/chronos_blinky.v` (Verilog-2001; the `.sv` is for newer Radiant only).
- Constraints `fpga/blinky/chronos_blinky.pdc` (4 LEDs + reset).
- Project `fpga/blinky/es20/chronos_blinky_es` (Radiant 2.0 SP1, device LIFCL-40-8BG400C).
- Result: LEDs blink from flash after power-cycle. Toolchain + board + flash path proven.
- LEDs (ACTIVE-LOW, FPGA sinks the cathode - see section 1): LED0=E17, LED1=F13,
  LED2=G13, LED3=F14. (Blinky just looked inverted; harmless there.)

---

## 4. Firmware: main design (fpga/rtl/) - current state

Top: `fpga/rtl/chronos_top.sv`. Submodules: `csi2_rx.sv`, `csi2_tx.sv`, `frame_buffer.sv`,
`tx_arbiter.sv`, `trigger_generator.sv`, `chronos_pll.sv`, `i2c_slave.sv`, `config_regs.sv`,
`i2c_master.sv` + `ov9281_init.sv` (camera SCCB), `csi2_pkg.sv` (shared ECC/CRC), `cdc.sv`,
`sync_fifo.sv`, wrappers `rtl/mipi/mipi_dphy_rx.v`/`mipi_dphy_tx.v` (Lattice IP).
Pinout: `fpga/constraints/chronos_pinout.pdc`. Testbench: `fpga/sim/tb_chronos_csi2.sv`.

STATUS after the MIPI rebuild (all old CRITICAL/HIGH issues fixed): real soft-RX/hard-TX
D-PHY IP wrappers, per-camera recovered byte clocks, correct header/ECC/CRC via csi2_pkg,
FWFT async frame_buffer with pkt_avail gating, token-based tx_arbiter, FPGA-driven OV9281
SCCB init, per-port DEV_ADDR (SID), 27 MHz XCLK forwarding, active-low LED handling.

### Full-review fixes (2026-07, second from-scratch pass)
- clk_sys is 192 MHz, NOT 200: 200 MHz is unreachable from the 12 MHz ref with integer
  PLL dividers (old params implied VCO=1800 MHz, out of range -> never locks). New:
  CLKFB_DIV=16/CLKOP_DIV=6/CLKOS_DIV=12, PDC generated clock x16. All CLK_HZ constants
  (trigger_generator, ov9281_init, i2c_master, csi2_tx REF_CLOCK_FREQ) follow.
- i2c_slave fully REWRITTEN: old FSM drove ACK during bit-8's high phase (SDA falling
  while SCL high = spurious START) and released before clock 9 -> host always saw NACK;
  the Jetson could never talk to the FPGA. New FSM: sample on rise, drive only while SCL
  low, ACK spans the whole 9th clock, read bytes prefetched (also fixes stale first
  read byte + auto-increment).
- trigger_generator: runtime division (CLK_FREQ_HZ/fps combinational divider) replaced
  with a phase accumulator (exact average rate, <=1 clk jitter, trivially meets timing).
- csi2_rx: aligner false-lock recovery watchdog (3 consecutive header-ECC errors or
  ~170 ms locked-but-idle -> re-hunt); parser/aligner also held in reset while camera
  disabled (enable synced into byte domain).
- Truncated-packet hang escapes: tx_arbiter aborts ST_PAY on unexpected SOP head or
  ~0.3 ms stall; csi2_tx terminates/drops a packet whose payload FIFO stays empty ~40 us.
  Frame buffers get per-camera DUAL-side resets (wr AND rd gated by cam_enable[i]), so
  disable/re-enable flushes pointers coherently.
- i2c_master holds SCL low between bytes of an open transaction (bus_open) instead of
  floating both lines; config_regs CTRL bit1 (soft reset) now truly self-clearing;
  LED[1]/LED[2] pulses stretched ~44 ms (pulse_stretch in cdc.sv); tb decl-order +
  meaningful RX-error check.

### Remaining known items
- ov9281_init INIT_ROM is a structural placeholder: paste the production OV9281 register
  table (1280x800 RAW10, 2-lane, PLL for 27 MHz XCLK, external-trigger/FSIN mode, VTS for
  30 fps) between the marked rows.
- mipi_dphy_tx CM/CN/CO divider parameters are placeholders; take them from an IP Catalog
  run for clk_ref = 192 MHz and the chosen line rate before synthesis. chronos_pll should
  also be regenerated via IP Catalog for the tool-blessed settings of the x16 ratio.
### Simulation status (2026-07: SIMULATED AND PASSING)
Icarus Verilog (OSS CAD Suite, in %LOCALAPPDATA%\oss-cad-suite\bin) now runs locally:
- tb_chronos_csi2 PASS (3 packets decoded, 0 errors). Sim caught a real bug: the RX word
  aligner locked one cycle late, losing the first packet after every (re)lock -> csi2_rx now
  hunts combinationally and uses the hunt offset in the lock cycle.
- tb_i2c_slave PASS (new BFM tb): addressing ACK, write/readback, auto-inc multi-read
  (ID_L/ID_H) and multi-write (PULSE_WIDTH), self-clearing soft reset, foreign-address NACK,
  STATUS pack. The rewritten i2c_slave is validated against config_regs.
- tb_trigger_generator PASS (new tb, scaled clock): exactly N pulses/sec incl. non-divisor
  rates (7 fps), pulse width, >MAX clamp to 30, disable.
- Icarus portability: no 'ref' task args, no ternaries on enums/state types, found-guard
  instead of 'break' in always_comb for-loops (tx_arbiter/csi2_rx/csi2_tx/tb adjusted).

### Bandwidth ceiling (hard constraint)
4 cams x 2 lanes x 800 Mbps = 6.4 Gbps input. TX to Jetson is 2 lanes; hard D-PHY ~2.5 Gbps/lane
=> ~5 Gbps ceiling. So 4 cams @ 120fps simultaneously do NOT fit a 2-lane TX. Realistic: 4 cams
@ ~30-60 fps. Higher needs lower res/fewer concurrent cams or a hardware change (more TX lanes).

### MIPI D-PHY IP availability (Radiant 2.0)
`C:\lscc\radiant\2.0\ip\lifcl\mipi_dphy` provides D-PHY physical layer only (PPI: byte clock +
per-lane bytes), HARD_IP and soft (LATTICE/GDDR) modes, RX/TX, GEAR 8 (soft) / 8|16 (hard),
1/2/4 lanes, up to 2500 Mbps. CSI-2 packet layer (header/ECC/CRC) stays in fabric.
LIFCL-40 has only 2 hard D-PHY blocks (need 4 RX + 1 TX) -> use soft RX for cameras, hard for TX.

---

## 5. Rebuild plan

The agreed path: rebuild the MIPI subsystem on Lattice D-PHY IP + a corrected CSI-2 packet
layer, add the camera SCCB config path, fix datapath/clocking/constraints, and bring up in
phases (sim first, then hardware: 1 cam -> loopback -> 4 cam + VC -> FSIN sync).
Detailed plan: `.cursor/plans/chronos_mipi_rebuild_*.plan.md`.

---

## 5b. Jetson side (app/ + drivers/) - REWORKED (2026-07, after review)

Both trees were rebuilt to match the real architecture (review findings all addressed):
- drivers/: NEW chronos_fpga/ I2C driver ("chronos,fpga-ctrl" @0x3C, regmap_i2c, sysfs
  trigger_enable/frame_rate/pulse_width/cam_enable/status/error/frame_count/version/id,
  exported API incl. chronos_fpga_get_client with EPROBE_DEFER). chronos_csi.c REWRITTEN
  as an async V4L2 subdev bridge ("chronos,csi-bridge", one source pad, Y10_1X10
  1280x800, s_stream toggles FSIN via the FPGA driver) - all fake /dev/video + vb2 code
  removed (Tegra VI owns video nodes). chronos-orin-nx.dts REWRITTEN: FPGA @3c + one
  2-lane bridge->NVCSI->VI graph with 4 VI channels (vc-id 0..3); fictional i2c-mux,
  4x ov9281 nodes, Jetson spi0 IMU, reserved-memory all removed. ov9281/ and imu/ are
  QUARANTINED (do-not-load warning headers; excluded from top-level drivers/Makefile).
  Nothing compiled yet (no L4T tree here) - first build must check the 5.10/5.15
  v4l2_subdev state-arg version switch in chronos_csi.c.
- app/: chronos_capture.c reworked around V4L2 MMAP capture (NvBuffer/EGL path DELETED;
  CUDA is now an optional pitch-aware cudaMemcpy2D upload, CHRONOS_WITH_CUDA in CMake).
  Consumer-owned buffer lifecycle: capture thread never requeues; release_frame_set does
  VIDIOC_QBUF via an in-flight table. CTRL(0x00) written on start/stop per
  external_trigger; VERSION/ID verified at init (warn-only). get_frame_set loops on the
  cond var, -1 = infinite. Stats populated (fps, skew, seq-gap drops, fpga_status/error
  via persistent I2C fd). sync_valid = timestamp spread + STATUS bits. New config field
  fsin_pulse_width_cycles (0 = 2000-cycle default), pulse width decoupled from exposure.
  chronos_frame_t: dmabuf_fd/nvbuf_fd fields REMOVED (data = mmap pointer). sync_test
  defaults to 30 fps. Nothing compiled yet (needs Jetson/CUDA toolchain).

---

## 6. Conventions / gotchas

- Build with Radiant 2.0 SP1 (LSE, Verilog-2001 for the ES flow); program with 2025.2 Programmer.
- Keep imports/instantiations clean; no inline imports (workspace rule).
- Netlist source of truth: `hardware/Project Outputs for Chronos/ProtelNetlist/Chronos.NET`
  (Altium/Protel export). Verify pin sites there before changing the PDC - wrong BGA sites
  can short power rails or break D-PHY constraints. All PDC camera/TX/I2C/LED/FSIN sites
  were re-verified against it (2026-07); 27 MHz L5 + MCLK M3..M6 + SID straps came from it.
- Do not hand-edit bitstream IDCODE/CRC; generate native ES bitstreams from Radiant 2.0 SP1.
