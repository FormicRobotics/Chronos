# Chronos ↔ Waveshare OV9281-120 Camera Adapter

> **OBSOLETE / SUPERSEDED (2026-06-01).** The selected camera is now the
> **SincereFirst SF-AOV9281**, which terminates in an **`AXT624124`** header — the
> direct mate for the board's `AXT524124` socket — and exposes **FSIN on pin 16**.
> No adapter, boost converter, I²C level shifter, or FSIN flying wire is needed.
> See **`docs/sf_aov9281_direct_connect.md`**. This file is kept only as a record
> of the Waveshare (RPi 15-pin FFC) adapter analysis.

Active mezzanine adapter to connect a **Waveshare OV9281-120** (Raspberry Pi
15-pin FFC camera) to a **Chronos HW_CHRONOS_R1** camera port (`AXT524124`).
One adapter is required **per camera** (×4).

> Design point: 4 cameras @ 30 fps, **hardware FSIN sync required**, camera 3.3 V
> generated on the adapter by boosting the 2.8 V camera rail.

## Why an active adapter (not a cable)

| Topic | HW_CHRONOS_R1 side (AXT524124) | Waveshare side (15-pin RPi) | Adapter action |
|------|-------------------------------|-----------------------------|----------------|
| Power | discrete 2.8 / 1.8 / 1.2 V rails, no 3.3 V | single **3.3 V** in, on-board LDOs | **boost 2.8 V → 3.3 V** |
| SCCB | 1.8 V, pulled up to 1.8 V | SCL/SDA pulled up to **3.3 V** | **1.8↔3.3 V I²C translator** |
| Clock | FPGA drives XCLK (MCLK) | self-clocked (on-board 24 MHz) | MCLK **unused** |
| Enable/Reset | active-low `CAM_RESET` (1.8 V) | `POWER-EN` (3.3 V), no reset pin | pull POWER-EN high (opt. sequencing) |
| FSIN | FPGA FSIN on pin 16 (1.8 V) | **not on the connector** | **flying wire to OV9281 FSIN pad** |
| MIPI | 4-lane-capable, lanes 2/3 unused | 2-lane | route 2 lanes + clock, 1:1 |

MIPI is electrically compatible (D-PHY swing is independent of the supply
question); only 2 data lanes + clock are used.

## Connector reference

### Board socket — `AXT524124` (J3=CAM0, J4=CAM1, J5=CAM2, J6=CAM3)
Panasonic F4S, 0.4 mm pitch, 24-pin, 2-row, 1.0 mm mated height.
**Adapter mates it with header `AXT624124`.**

CAM0 (J3) net map as routed on HW_CHRONOS_R1:

| Pin | Net | Pin | Net |
|----|-----|----|-----|
| 1 | CAM0_SDA (1.8 V) | 13 | *not connected* |
| 2 | CAM0_SCL (1.8 V) | 14 | *not connected* |
| 3 | **V1P8 (DOVDD)** | 15 | AGND (via ferrite) |
| 4 | CAM0_CLK_P | 16 | **FSIN0 (1.8 V)** |
| 5 | CAM0_CLK_N | 17 | SID strap (J3: GND) |
| 6 | GND | 18 | **V1P2 (DVDD)** |
| 7 | CAM0_D0_P | 19 | CAM0_MCLK (FPGA M3) |
| 8 | CAM0_D0_N | 20 | *not connected* |
| 9 | GND | 21 | CAM_RESET (shared) |
| 10 | CAM0_D1_P | 22 | **V2P8 (AVDD)** |
| 11 | CAM0_D1_N | 23 | *not connected* |
| 12 | GND | 24 | GND (25–28 GND/MNT) |

> CORRECTED against the netlist (`Chronos.NET`, 2026-07): pins 3/18/22 are
> V1P8/V1P2/V2P8 (this table originally guessed a different order), pin 17 is
> the SID strap (not plain GND on all ports), pin 19 MCLK is FPGA-driven, and
> pins 13/14/20/23 are unconnected (there are no D2/D3 lanes on this board).

### Camera — Raspberry Pi 15-pin FFC, 1.0 mm (PH1.0-15PIN)

| Pin | Name | Pin | Name |
|----|------|----|------|
| 1 | GND | 9 | MCP (CLK+) |
| 2 | MDN0 (D0−) | 10 | GND |
| 3 | MDP0 (D0+) | 11 | POWER-EN |
| 4 | GND | 12 | LED-EN |
| 5 | MDN1 (D1−) | 13 | SCL |
| 6 | MDP1 (D1+) | 14 | SDA |
| 7 | GND | 15 | VCC 3.3 V |
| 8 | MCN (CLK−) | | |

## Adapter schematic (per camera)

```
 AXT624124 header (JA, to board)                 15-pin FFC (JB, to camera)
 ----------------------------------              --------------------------
 JA.18 (2.8V AVDD) --+--> [U1 buck-boost] --3V3--+--> JB.15 VCC
                     |     L=1µH, Cin/Cout=10µF   +--> [10k] --> JB.11 POWER-EN
                     |                            +--> [PCA9306 VREF2 via 200k]
 JA.22 (1.8V) -------|--> PCA9306 VREF1 (200k)
 JA.6/9/12/17/24 GND-+--> GND plane --------------+--> JB.1/4/7/10 GND

 SCCB through U2 = PCA9306 (1.8↔3.3):
   JA.2 SCL(1.8) <-> U2 -> JB.13 SCL(3.3)
   JA.1 SDA(1.8) <-> U2 -> JB.14 SDA(3.3)
   (pull-ups already exist: 1.8V side on board, 3.3V side on camera)

 MIPI (1:1, 100Ω diff, length-matched):
   JA.4/5  CLK_P/N -> JB.9/8  MCP/MCN
   JA.7/8  D0_P/N  -> JB.3/2  MDP0/MDN0
   JA.10/11 D1_P/N -> JB.6/5  MDP1/MDN1

 FSIN (hardware sync) — NOT on the FFC:
   JA.16 FSIN0(1.8) -- [33Ω] --> test pad "FSIN" + GND pad
        -> flying wire to the OV9281 FSIN pad on the Waveshare PCB

 Unused: JA.19 (MCLK), JA.13/14/20/23 (lanes 2/3), JA.21 (CAM_RESET, see note),
         JB.12 (LED-EN) = NC
```

### Power: 2.8 V → 3.3 V boost (U1)
- Use a buck-boost (input 2.8 V is close to 3.3 V out): **TI TPS63802** (adj),
  set `Vout = 0.5·(1+Rt/Rb)` → Rt = 560 k, Rb = 100 k for 3.3 V; L = 1 µH,
  Cin = Cout = 10 µF. Budget ~150–200 mA out (≈250 mA from the 2.8 V rail; the
  board rail/ferrite is rated 500 mA — OK).
- A fixed-3.3 V boost (e.g. TPS61222) also works but margins are tighter when
  Vin≈Vout; the buck-boost is the robust choice.

### SCCB level translation (U2 = PCA9306)
- VREF1 → 1.8 V (JA.22), VREF2 → 3.3 V (U1 out), each via 200 k to its rail.
- Do **not** add extra pull-ups: the board already pulls the 1.8 V side
  (R25/R26 = 4.7 k) and the Waveshare board pulls the 3.3 V side.

### POWER-EN / reset
- Default: `POWER-EN` (JB.11) pulled to 3.3 V via 10 k → camera always enabled
  when the adapter is powered.
- Optional sequencing: gate POWER-EN from `CAM_RESET` (JA.21) via a small load
  switch / level shift so the FPGA reset sequence still powers the module.

### FSIN flying wire (mandatory for hardware sync)
1. The Waveshare 15-pin connector has **no FSIN**. Locate the **OV9281 FSIN pad**
   on the Waveshare module (inspect the PCB / request the pad from Waveshare).
2. Solder a thin wire from the adapter's **FSIN pad** (JA.16 via 33 Ω, 1.8 V CMOS)
   to that pad. The sensor's FSIN input is DOVDD(1.8 V)-referenced, so **no level
   shift is needed** — but confirm the board FSIN buffer rail (P1) is 1.8 V.
3. Keep this wire short and grounded-referenced (use the adjacent GND pad).

## BOM (per adapter, ×4 total)

| Ref | Part | Notes |
|----|------|------|
| JA | Panasonic AXT624124 | AXT6 header, mates board AXT524124 |
| JB | 1.0 mm 15-pos FFC/FPC conn | match Waveshare cable contact side |
| U1 | TI TPS63802 (buck-boost) | 2.8 V→3.3 V; Rt 560k / Rb 100k |
| U2 | NXP/TI PCA9306 | I²C 1.8↔3.3 V translator |
| L1 | 1 µH power inductor | for U1 |
| C | 2× 10 µF, several 0.1 µF | U1 in/out + decoupling |
| R | 10 k (POWER-EN), 33 Ω (FSIN), 2× 200 k (VREF) | |

## Firmware implications (already aligned for 30 fps)
- **XCLK unused**: the FPGA `CAM_MCLK` outputs can be left idle; Waveshare
  self-clocks at 24 MHz.
- **SCCB**: unchanged logic (open-drain). OV9281 default 7-bit address 0x60
  matches `ov9281_init` `DEV_ADDR`.
- **External-trigger mode**: the `ov9281_init` ROM must enable FSIN snapshot mode
  (per OV9281: reg 0x3030 bit2 + 0x3006 + frame-count 0x303F) so a rising FSIN
  edge captures a frame.
- **30 fps is the safe target**: OV9281 FSIN snapshot wake-up costs ≈61 396 XCLK
  (~2.55 ms at 24 MHz), which caps trigger-mode rate near ~90 fps. 30 fps
  (33.3 ms period) has wide margin — another reason the 30 fps design point holds.

## Mechanical notes
- Adapter = small rigid PCB: `AXT624124` on the bottom (mates the board socket,
  1.0 mm stack), FFC connector on top, then FFC cable to the camera.
- 0.4 mm header placement is accuracy-sensitive; follow Panasonic's recommended
  land pattern and solder-paste reduction.
- Route MIPI pairs 100 Ω differential, intra-pair ≤ 0.1 mm, inter-pair ≤ 1.0 mm
  (same rules as the main board camera nets).
