# Chronos ↔ SincereFirst SF-AOV9281 (direct connect)

The **SincereFirst SF-AOV9281** (OV9281 CSP, mono RAW, 2-lane MIPI) terminates in
a Panasonic **`AXT624124`** header on a flex tail — the exact mating part for the
Chronos `AXT524124` socket (J3=CAM0 … J6=CAM3). It plugs **directly** into the
board. **No adapter PCB, no 3.3 V boost, no I²C level shifter, no FSIN flying
wire.** (Spec: `QR-RD-F002 Rev-A`, V1.0, 2018-02-22.)

This replaces the Waveshare OV9281-120 plan (`waveshare_ov9281_adapter.md`,
obsolete) and **restores hardware FSIN sync**.

## Why it just works
- **Connector**: `AXT624124` header ↔ board `AXT524124` socket (0.4 mm, 24-pin).
- **FSIN on pin 16** → global-shutter hardware sync over the existing FPGA FSIN net.
- **DOVDD/IO = 1.8 V** → matches the board's 1.8 V SCCB and FSIN bank; no shifter.
- **XCLK on pin 19** → externally clocked. The 27 MHz oscillator X2 feeds the
  FPGA (pin L5); the FPGA forwards it to each camera's XCLK pin
  (M3/M4/M5/M6 → J3..J6 pin 19, 33 Ω series). The FPGA MUST drive MCLK or the
  sensor will not even respond on SCCB (no on-module oscillator).
- **RESET on pin 21** → matches the board `CAM_RESET`.
- The board's connector + FSIN@16 + XCLK@19 + RESET@21 + SDA@1/SCL@2 + 2 MIPI
  lanes match the standard OV9281 CSP module pinout — the board was designed for
  exactly this class of module.

## Pin map: SF-AOV9281 vs board J3 (CAM0)

| Pin | SF-AOV9281 | Board J3 (AXT524124) | Note |
|----|------------|----------------------|------|
| 1  | SDA            | CAM0_SDA (1.8 V)   | I²C data |
| 2  | SCL            | CAM0_SCL (1.8 V)   | I²C clock |
| 3  | DOVDD 1.8 V    | 1.8 V rail         | **confirmed** |
| 4  | MCP (CLK+)     | CAM0_CLK_P         | MIPI clock+ |
| 5  | MCN (CLK−)     | CAM0_CLK_N         | MIPI clock− |
| 6  | DGND           | GND                | |
| 7  | MDP0           | CAM0_D0_P          | MIPI lane0+ |
| 8  | MDN0           | CAM0_D0_N          | MIPI lane0− |
| 9  | DGND           | GND                | |
| 10 | MDP1           | CAM0_D1_P          | MIPI lane1+ |
| 11 | MDN1           | CAM0_D1_N          | MIPI lane1− |
| 12 | DGND           | GND                | |
| 13 | LPWM           | not connected      | floats (netlist: no net on J*-13) |
| 14 | GPIO2          | not connected      | floats |
| 15 | AGND           | AGND (via ferrite) | FB2/FB3/FB9/FB10 to GND |
| 16 | **FSIN**       | **FSIN0 (1.8 V)**  | hardware frame sync (U5 buffer, 33 Ω) |
| 17 | SID            | see note below     | sets the SCCB address per port |
| 18 | DVDD 1.2 V     | 1.2 V rail (V1P2)  | **confirmed (netlist)** |
| 19 | XCLK           | CAM0_MCLK (FPGA M3)| FPGA forwards 27 MHz from X2 |
| 20 | STROBE         | not connected      | floats |
| 21 | RESET          | CAM_RESET (W18)    | active-low, shared by all 4 |
| 22 | AVDD 2.8 V     | 2.8 V rail (V2P8)  | **confirmed (netlist)** |
| 23 | ULPM           | not connected      | floats |
| 24 | DGND           | GND                | |

Power rails (pins 3/18/22) verified against the netlist: **V1P8 / V1P2 / V2P8**,
matching the module.

### SID strapping → per-port SCCB address (netlist fact)
| Port | Pin 17 (SID) | 7-bit SCCB address |
|------|--------------|--------------------|
| J3 (CAM0) | GND | **0x60** |
| J4 (CAM1) | 4.7 k pull-up to VCCIO6 (R36) | **0x10** |
| J5 (CAM2) | GND | **0x60** |
| J6 (CAM3) | 4.7 k pull-up to VCCIO6 (R51) | **0x10** |

Each camera has a dedicated I²C bus, so the mixed addressing is harmless — but
the firmware must use the right address per port (`chronos_top` passes
`DEV_ADDR` accordingly).

## Firmware implications
- **FPGA must generate XCLK**: 27 MHz enters the FPGA at L5 (from X2) and is
  forwarded to `cam_mclk[3:0]` (M3..M6). Implemented in `chronos_top` +
  `chronos_pinout.pdc`. Keep MCLK running whenever cameras may be accessed —
  without it the sensor's SCCB is dead.
- **XCLK = 27 MHz**, not 24 MHz: the OV9281 init table's PLL registers must
  target a 27 MHz input clock.
- **SCCB addresses are mixed**: CAM0/CAM2 = 0x60, CAM1/CAM3 = 0x10 (SID
  strapping above). `chronos_top` parameterizes `ov9281_init` per camera.
- **External-trigger / FSIN snapshot mode** — unchanged requirement: the init ROM
  must put the sensor in slave-strobe mode so each FPGA FSIN pulse exposes a frame.
  The synchronized 30 fps comes from the FPGA trigger period, not free-running.
- FSIN buffer U5 (SN74LVC125A) is powered from V1P8 (netlist) → FSIN swings
  1.8 V at the camera, matching DOVDD. Confirmed, no action needed.

## Mechanical
- The module is a flex-tail camera with the `AXT624124` header at the tail end;
  it seats directly onto the board socket (1.0 mm mated height). Check FFC contact
  side/orientation and that the tail length reaches each of the 4 sockets.
- Optical: 2.3 mm f/2.2 lens, ~85° HFOV, focus 50 cm–∞.
