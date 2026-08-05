# CHRONOS — Project Brain (full handoff document)

> Self-contained knowledge base for continuing this project in any IDE / with any
> AI assistant. Read this first. Companion file: `CLAUDE.md` (older memory doc,
> mostly overlapping). Last updated: 2026-08-05, git HEAD `afc7d91`.

---

## 0. What this project is

**Chronos** = custom board (HW_CHRONOS_R1) that aggregates **4x MIPI CSI-2
cameras into one synchronized 2-lane CSI-2 stream** for an NVIDIA **Jetson Orin
NX**, using a **Lattice CrossLink-NX (LIFCL-40) FPGA**. The FPGA:

- receives 4x 2-lane CSI-2 (soft D-PHY RX, one per camera),
- buffers packets per camera and multiplexes them onto one 2-lane CSI-2 TX
  (hard D-PHY) using **virtual channels 0..3**,
- generates a common **FSIN trigger** so all 4 sensors expose simultaneously,
- configures the sensors itself over SCCB/I2C at boot,
- is controlled by the Jetson over I2C (slave addr **0x3C**).

Target: 4 cameras @ **30 fps** aggregate (bandwidth ceiling of the 2-lane TX
makes 4x120fps impossible — see §5.6).

The Jetson sits on an **Aerium "Helios" carrier board**; Chronos connects to the
Helios CAM1 port through a **custom adapter PCB** (see §7 — pinouts are
incompatible, a direct cable would short 3.3V to GND).

### Current overall status (2026-08)

| Area | Status |
|---|---|
| Board bring-up / programming path | PROVEN (blinky runs from SPI flash) |
| FPGA RTL (fpga/rtl/) | Complete after 2 deep reviews; 3 testbenches PASS in Icarus; **not yet synthesized for hardware** (2 placeholders remain, §5.4) |
| Jetson kernel drivers (drivers/) | Rewritten to real architecture; **never compiled** (needs L4T tree) |
| Jetson userspace (app/) | Rewritten (V4L2 MMAP); **never compiled** (needs Jetson) |
| Adapter PCB (hardware/adapter_chronos_helios/) | v3 design DONE, DRC clean, JLC-ready, **not yet ordered** |
| Cameras | 1x SF-AOV9281 in hand, plugged into J3 (CAM0) |

---

## 1. Hardware facts (netlist-verified, do not re-derive)

Netlist source of truth: `hardware/Project Outputs for Chronos/ProtelNetlist/Chronos.NET`.
Always check pin sites there before touching the PDC.

- **FPGA**: U3, marking `LIFCL-40-8BG400C`, CABGA400. **The silicon is ES1**:
  JTAG/SSPI IDCODE = `0x010F1043` (production = `0x110F1043`). The marking lies;
  trust the IDCODE. This cost weeks — see §3.
- **Config flash**: U6 = Macronix **MX25L12833F** 128 Mbit SPI NOR, on FPGA MSPI
  pins (DQ0→D13, DQ1→D15, DQ2→D14, DQ3→D16). J20 breaks out quad-SPI.
- **USB bridge**: FT2232HL wired to FPGA **Slave SPI (SSPI), not JTAG**.
  Channel A = SSPI/config, channel B = UART.
- **Programming jumpers**: JP1 **open**; config-enable jumper fitted → D17 red.
  JP2 closed feeds the 12 MHz reference (FT2232) to FPGA pin **L13**.
- **Buttons**: SW4 = GSRN (active-low reset), SW5 = PROGRAMN.
- **Cameras**: 4x SincereFirst **SF-AOV9281** (OV9281, 1280x800 RAW10 mono,
  2-lane) on J3..J6 (AXT524124 socket — module plugs directly, no adapter).
  - **No oscillator on module.** 27 MHz osc X2 → FPGA **L5** (`clk_27m`); FPGA
    forwards to `cam_mclk[3:0]` = **M3/M4/M5/M6** → J3..J6 pin 19.
    No MCLK = no SCCB = dead camera.
  - **SID strap differs per port**: J3/J5 → SCCB addr **0x60**; J4/J6 → **0x10**.
    Each camera has a dedicated I2C bus; `chronos_top` sets DEV_ADDR per instance.
  - Rails: pin3 = V1P8 (DOVDD), pin18 = V1P2 (DVDD), pin22 = V2P8 (AVDD).
  - FSIN: FPGA **R9** → U5/U9 buffers → J3..J6 pin 16. RESET: FPGA **W18** →
    all J*-21, shared, active-low.
- **IMU**: ICM-42688-P (IC1) on FPGA SPI (NOT wired to the Jetson).
- **User LEDs** LED0..3 = E17/F13/G13/F14, **ACTIVE-LOW** (FPGA sinks cathode).
- **Bank 1 (VCCIO1) is 3.3 V** → PDC uses LVCMOS33 for UART/host-I2C/LEDs/GSRN.
- **Host I2C** (Jetson→FPGA): FPGA slave addr **0x3C**.
- **J7** = 22-pin 0.5 mm FFC (FH12-style, bottom-contact) = the aggregated
  CSI-2 TX output toward the Jetson. **Custom pinout** (see §7).

---

## 2. Toolchain

- **Radiant 2.0 SP1** (`C:\lscc\radiant\2.0`) — ONLY version supporting ES1
  silicon. Newer Radiant (2025.2.1) emits production-IDCODE bitstreams which the
  board rejects ("Mismatch to device ID code").
  - LSE synthesis. 2.0 SP1 has **no full SystemVerilog** support (native SV came
    in 2.1) → the main design in `fpga/rtl/*.sv` must be converted to
    Verilog-2001 OR synthesized with Synplify before it can build here. This is
    an open task (§5.4).
- **Radiant Programmer 2025.2** — use THIS for programming (its FTDI driver
  works; the 2.0 programmer reads 0x7FFFFFFF). Build in 2.0, program with 2025.2.
- **Windows gotcha**: Smart App Control blocks Radiant 2.0's unsigned DLLs →
  run Radiant as Administrator.
- **Simulation**: Icarus Verilog from OSS CAD Suite, portable install at
  `%LOCALAPPDATA%\oss-cad-suite\bin`. Run all sims:
  `powershell fpga/sim/run_sims.ps1` (3 testbenches, all must print PASS).
- **KiCad 10** (`C:\Program Files\KiCad\10.0`) for the adapter PCB.
  `kicad-cli` used for DRC/render/gerbers; zone fill needs KiCad's own python
  (`bin\python.exe _fill_zones.py`).

---

## 3. Programming the FPGA (proven recipe)

Build bitstream in **Radiant 2.0 SP1**, program with the **2025.2 Programmer**.

- **SRAM (volatile test)**: Device family `LIFCL_ENG`, device `LIFCL-40-ES`,
  Target Memory SRAM, Port = Slave SPI, Access = Direct Programming,
  Operation = Fast Configuration.
- **SPI flash (permanent)**: Target = External SPI Flash Memory, Port =
  **SSPI2SPI**, Operation = Erase,Program,Verify. Flash: Macronix MX25L12833F,
  8-pin SOP, start 0x0. Power-cycle after → boots from flash, D17 red goes off.
- Hardware prep: JP1 open, config jumper fitted (D17 red), TCK divider ≥2.
- Bitstream sanity: bytes `01 0F 10 43` near offset ~0x196. If you see
  `11 0F 10 43` it was built by new Radiant and will be rejected. **Never
  hand-patch the IDCODE** (breaks bitstream CRC).
- Blinky already validated end-to-end: `fpga/blinky/chronos_blinky.v` (+ .pdc,
  project in `fpga/blinky/es20/`). LEDs blink from flash after power-cycle.

---

## 4. FPGA design (fpga/rtl/) — architecture

Top `chronos_top.sv`. Data path per camera:

```
cam D-PHY pads → mipi_dphy_rx.v (Lattice soft D-PHY IP wrapper, per-cam byte clk)
  → csi2_rx.sv (word align + packet parse, ECC/CRC via csi2_pkg.sv)
  → frame_buffer.sv (async FIFO, Gray pointers, FWFT, token stream, pkt_avail)
  → tx_arbiter.sv (round-robin 4:1 packet mux, VC = camera index)
  → csi2_tx.sv (packet build + payload prefetch FIFO)
  → mipi_dphy_tx.v (hard D-PHY IP wrapper, 2 lanes)
```

Control plane: `i2c_slave.sv` (Jetson @0x3C) → `config_regs.sv`;
`trigger_generator.sv` (phase-accumulator FSIN rate engine);
`i2c_master.sv` + `ov9281_init.sv` per camera (SCCB init, DEV_ADDR per SID);
`chronos_pll.sv` (12 MHz → **192 MHz clk_sys** + 96 MHz clk_byte);
`cdc.sv` (2FF sync, pulse sync, pulse_stretch ~44 ms for LEDs).
Pinout: `fpga/constraints/chronos_pinout.pdc` (netlist-verified 2026-07).

Key register map (I2C @0x3C, 8-bit regs, pointer auto-increments):
CTRL(0x00: bit0=trigger_enable, bit1=soft-reset self-clearing),
FRAME_RATE, PULSE_WIDTH_L/H, CAM_ENABLE, STATUS, ERROR, FRAME_COUNT*,
VERSION, ID_L/ID_H. (Exact layout in `config_regs.sv` — source of truth.)

### 4.1 Non-obvious design decisions (don't undo these)

- **clk_sys = 192 MHz, not 200**: 200 is unreachable from 12 MHz with integer
  PLL dividers in the legal VCO range (old params → VCO 1800 MHz → never locks).
  CLKFB_DIV=16 / CLKOP_DIV=6 / CLKOS_DIV=12, VCO 1152 MHz. All CLK_HZ params
  in trigger_generator / ov9281_init / i2c_master / csi2_tx follow 192 MHz.
- **i2c_slave was fully rewritten** (old FSM created spurious START by moving
  SDA while SCL high and never held ACK through clock 9 → Jetson could never
  talk to it). Rules baked in: sample on SCL rise, drive only while SCL low,
  ACK spans whole 9th clock, read data prefetched one byte ahead.
- **csi2_rx word aligner hunts combinationally** — with a registered-only lock
  the first packet after every (re)lock was lost (found in simulation).
  A watchdog re-hunts after 3 consecutive header-ECC errors or ~170 ms
  locked-but-idle (false-lock recovery).
- **Deadlock escapes everywhere**: tx_arbiter aborts a packet on unexpected-SOP
  head or ~0.3 ms stall; csi2_tx force-terminates when its payload FIFO starves
  ~40 µs; frame buffers get dual-side (wr+rd) resets gated by cam_enable[i].
- **trigger_generator uses a phase accumulator** (exact average rate for
  non-divisor fps, ≤1 clk jitter, no giant combinational divider).
- **LEDs are inverted in RTL** (active-low hardware).
- **Icarus portability rules** used across the RTL: no `ref` task args, no
  ternaries assigning enum/state types, found-guard instead of `break` in
  `always_comb` for-loops.

### 4.2 Simulation (all passing)

`fpga/sim/run_sims.ps1` runs, via Icarus (`-g2012`):
- `tb_chronos_csi2` — full RX→buffer→arbiter→TX loopback, 3 packets, 0 errors.
- `tb_i2c_slave` — BFM master: addressing, write/readback, auto-inc multi-read/
  write, self-clearing soft reset, foreign-addr NACK.
- `tb_trigger_generator` — scaled clock, exact pulses/sec incl. 7 fps,
  pulse width, >MAX clamp, disable.

---

## 5. FPGA — what is left to do (in order)

1. **Fill `ov9281_init.sv` INIT_ROM** with the production OV9281 register table:
   1280x800 RAW10, 2-lane, PLL configured for **27 MHz XCLK**, external
   trigger/FSIN slave mode, VTS for 30 fps. Marked rows in the file show where.
   (Take values from the SincereFirst spec PDF in repo root + OV9281 datasheet.)
2. **Generate real IP cores in Radiant IP Catalog** (2.0 SP1):
   - `mipi_dphy_tx` CM/CN/CO dividers for clk_ref=192 MHz and the chosen line
     rate (wrapper params are placeholders);
   - regenerate `chronos_pll` for the x16 ratio with tool-blessed settings;
   - soft D-PHY RX instances.
3. **SystemVerilog → LSE problem**: Radiant 2.0 SP1 LSE cannot digest the
   `.sv` files. Either convert RTL to Verilog-2001 or use Synplify Pro inside
   Radiant. Budget real time for this; the RTL uses SV structs/enums/interfaces
   lightly but pervasively.
4. **Synthesize, meet timing** (192 MHz clk_sys; PDC has the clock groups),
   program SRAM first, then flash.
5. **Hardware bring-up phases**: 1 camera (J3, addr 0x60) → verify SCCB init +
   RX lock via STATUS regs/LEDs → TX loopback to Jetson → 4 cameras + VC →
   FSIN sync validation (LED1/LED2 pulse-stretched indicators help).
6. **Bandwidth ceiling** (hard constraint, don't promise more): 4 cams x 2 lanes
   x 800 Mbps = 6.4 Gbps in; TX = 2 lanes ≤ ~2.5 Gbps/lane ≈ 5 Gbps ceiling.
   4x30fps ≈ 1.6 Gbps — comfortable. 4x120fps does NOT fit.

---

## 6. Jetson side (app/ + drivers/)

Both trees rewritten 2026-07 to match the real architecture. **Neither has ever
been compiled** — first task on the Jetson is a build + fix pass.

### drivers/ (Linux kernel, L4T)
- `chronos_fpga/` — I2C driver, compatible `"chronos,fpga-ctrl"`, addr 0x3C,
  regmap_i2c; sysfs: trigger_enable, frame_rate, pulse_width, cam_enable,
  status, error, frame_count, version, id. Exports in-kernel API
  (`chronos_fpga_get_client`, EPROBE_DEFER pattern).
- `chronos_csi/` — V4L2 **async subdev bridge** (`"chronos,csi-bridge"`), one
  source pad, Y10_1X10 1280x800, `s_stream` toggles FSIN via chronos_fpga.
  `MODULE_SOFTDEP("pre: chronos_fpga")`. No /dev/video code — **Tegra VI owns
  the video nodes**.
- `chronos-orin-nx.dts` — overlay: FPGA @0x3C + bridge → NVCSI → VI graph,
  4 VI channels, `vc-id` 0..3, strict 1:1 endpoint pairing (Tegra requires it;
  modeled after L4T GMSL examples). tegra-camera-platform points at the bridge.
- `ov9281/` and `imu/` are **QUARANTINED** (do-not-load headers, excluded from
  the top Makefile): FPGA owns sensor config; IMU is FPGA-wired, not Jetson.
- First-build watchpoint: the 5.10/5.15 `v4l2_subdev` state-arg API switch in
  chronos_csi.c.

### app/ (userspace)
- `chronos_capture.c/.h` — library around **V4L2 MMAP** capture from
  /dev/video0..3 (NvBuffer/EGL deleted; optional CUDA upload via
  `CHRONOS_WITH_CUDA` CMake flag → pitch-aware cudaMemcpy2D).
  Consumer-owned buffers: capture thread never requeues;
  `chronos_release_frame_set()` re-queues via an in-flight table.
  FPGA CTRL written on start/stop; VERSION/ID verified at init (warn-only).
  `get_frame_set` timeout: loops on cond var, -1 = infinite. Stats: fps, skew,
  seq-gap drops, fpga_status/error over a persistent I2C fd.
  sync_valid = timestamp spread + STATUS bits. `fsin_pulse_width_cycles`
  config (0 → 2000-cycle default). Frame data = mmap pointer (no dmabuf fds).
- `demo_app.cpp` (optional OpenCV display), `sync_test.cpp` (defaults 30 fps).

### Jetson bring-up order
1. Build both modules against the L4T kernel tree; fix API drift.
2. Apply/compile the DTS overlay; boot; check `dmesg` for chronos_fpga probe
   (I2C 0x3C ACK proves the rewritten i2c_slave works on real wires).
3. `v4l2-ctl --list-devices` → 4 video nodes from Tegra VI.
4. Stream 1 channel raw (`v4l2-ctl --stream-mmap`), then `sync_test`.

---

## 7. Chronos ↔ Helios physical link (adapter PCB v3)

**Never connect J7 to Helios CAM1 with a plain cable**: J7 is custom
(SDA/SCL swapped vs RPi, 3.3V on pins 19/20, GND on 15) — a direct hookup
shorts Helios 3.3V to GND.

`hardware/adapter_chronos_helios/` contains the finished **v3** adapter
(28x28 mm, 4-layer JLC04161H-7628, DRC clean, JLC-ready):

- J1 = 22p FH12-type (XUNPU FPC-05F-22PH20, LCSC **C2856804**) → Chronos J7,
  **straight 22p 0.5 mm Type-A FFC**.
- J2 = 15p FH12-type (XFCN F0502-B-15-15T-R, LCSC **C2844857**) → Helios CAM1,
  **straight 15p 0.5 mm Type-A FFC**.
- **Pin-1 model (user-verified on the physical boards!)**: boards face to face,
  both pin 1s on the SAME absolute side → Chronos link mirrors through the
  cable (J7.i ↔ J1.23−i), Helios link maps **straight** (CAM.j ↔ J2.j).
  The Helios CAM connector is opposite-handed vs the standard FH12 footprint.
  v2 (one commit earlier) had this wrong; v3 is the corrected board.
- All 6 CSI conductors: identical top-layer diagonals, zero vias, zero skew.
  Only SDA crosses SCL once on In2.Cu. `_check_map.py` verifies the whole
  J7→CAM chain end-to-end (prints PASS).
- **Safety net**: SJ1 solder jumper OPEN = 3.3V rails isolated (each board
  powers itself). J1.19 intentionally NC. Test pads: TP1=Chronos 3V3,
  TP2=Helios 3V3 (V33H), TP3=SENSE (sees CAM GND when correct), TP4=GND.
  **Before full hookup: TP-V33H must read 3.3 V and TP-SENS must beep to GND.**
  Wrong cable type (A vs B) shows up there harmlessly; keep cables FLAT
  (no folds/twists — a fold flips the mapping).
- Ordering: JLC 4-layer 1.6 mm, **Impedance control: JLC04161H-7628**;
  upload `fab/chronos_helios_adapter_gerbers.zip`, BOM/CPL in `fab/`.
  Verify connector rotation in the JLC assembly preview (openings face the
  board edges).
- Regenerating the board: `python _gen_v3.py` → KiCad-python `_fill_zones.py`
  → `kicad-cli pcb drc` → `python _check_map.py`. The .kicad_pcb is generated;
  edit the script, not the board, for systematic changes.

**Helios CAM1 15p pinout (RPi std)**: 1 GND, 2/3 D0∓, 4 GND, 5/6 D1∓, 7 GND,
8/9 CLK∓, 10 GND, 11/12 unused, 13 SCL, 14 SDA, 15 3V3.
**Chronos J7 22p**: 1 GND, 2/3 D0∓, 4 GND, 5/6 D1∓, 7 GND, 8/9 CLK∓, 10 GND,
12 GND, 13 SDA, 14 SCL, 15 GND, 18 GND, 19/20 3V3, 21/22 GND (11/16/17 NC).

---

## 8. Repo map

```
fpga/rtl/            main design (SystemVerilog) — see §4
fpga/constraints/    chronos_pinout.pdc (netlist-verified)
fpga/sim/            3 testbenches + run_sims.ps1 + filelist_sim.f
fpga/blinky/         proven bring-up design (Verilog-2001 + Radiant 2.0 project)
fpga/README.md       build/sim instructions; DESIGN_DECISIONS.md = rationale
drivers/             chronos_fpga/, chronos_csi/, chronos-orin-nx.dts,
                     quarantined ov9281/ + imu/, top Makefile, README
app/                 chronos_capture lib + demo_app + sync_test + CMake
hardware/            Altium project, BOM, Chronos.NET netlist (source of truth),
                     adapter_chronos_helios/ (KiCad v3 + fab/ + scripts)
docs/                audit reports
CLAUDE.md            older memory document (superseded by this file)
SF-AOV9281 ... .pdf  camera module spec (repo root)
```

---

## 9. Gotchas checklist (things that already burned time once)

1. ES1 silicon: build with Radiant 2.0 SP1 only; program with 2025.2 Programmer.
2. Never hand-patch bitstream IDCODE (CRC breaks).
3. SSPI programming, not JTAG. JP1 open + config jumper (D17 red) + JP2 closed.
4. Smart App Control blocks Radiant 2.0 → run as Administrator.
5. Cameras are dead without FPGA-forwarded 27 MHz MCLK.
6. Camera SCCB addr differs per port (0x60 J3/J5, 0x10 J4/J6).
7. LEDs active-low; Bank 1 is 3.3 V (LVCMOS33).
8. clk_sys is 192 MHz — don't "fix" it back to 200.
9. Never cable J7 straight to Helios CAM1; use the adapter, TP-check first,
   SJ1 stays open, FFCs must lie flat.
10. Helios CAM connector numbering is opposite-handed vs standard FH12 — the
    v3 adapter accounts for this (verified with the boards, don't re-mirror).
11. Verify pin sites in Chronos.NET before any PDC change.
12. Icarus quirks: no `ref` ports, no enum ternaries, no `break` in
    always_comb loops.
13. drivers/ + app/ compile status: NEVER built; expect an API-fix pass.
14. `.gitignore` already handles KiCad `.history/`, sim artifacts, Altium noise.
