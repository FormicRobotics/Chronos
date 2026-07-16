# Chronos J7 <-> Aerium Helios CAM1 adapter PCB

Small (28 x 25 mm, 4-layer) adapter that safely connects the Chronos board's
custom 22-pin CSI output (J7) to the Aerium Helios carrier's standard
Raspberry-Pi-style 15-pin CAM1 port. A direct cable between the two is **not**
possible: J7 uses a custom pinout (SDA/SCL order reversed, 3.3V on pins 19/20,
GND on pin 15) that would short Helios 3.3V to GND and put 3.3V on I2C lines.

Open `chronos_helios_adapter.kicad_pcb` in KiCad 10 (board file is the source
of truth; there is no schematic - the netlist lives in the board).
DRC passes with 0 violations / 0 unconnected items (`drc_report.txt`).

## Connections

```
CHRONOS J7                    ADAPTER                     HELIOS CAM1
(22p custom)   straight 22p   J1 <-routing-> J2   Aerium 15p-22p     (15p RPi std)
     [J7] ==== Type-A FFC ==== [J1]          [J2] ==== FORWARD cable ==== [CAM1]
```

- **J1** (FH12-22S-0.5SH, bottom contact): to Chronos J7 with a **straight
  22-pin 0.5 mm Type-A (same-side contacts) FFC**. Because the two receptacles
  face each other, the mapping through the cable is mirrored (J7 pin i lands on
  J1 pad 23-i); the board's netlist already accounts for this.
- **J2** (FH12-22S-0.5SH, bottom contact): wired to the standard RPi 22-pin
  camera pinout, so the **Aerium "FPC Cable 15P to 22P forward direction"**
  you already have plugs its 22-pin end here and its 15-pin end into Helios
  CAM1.

### Signal map

| Function | Chronos J7 | Adapter J1 pad | Adapter J2 pad | Helios CAM1 (15p) |
|---|---|---|---|---|
| CSI D0- / D0+ | 2 / 3 | 21 / 20 | 2 / 3 | 2 / 3 |
| CSI D1- / D1+ | 5 / 6 | 18 / 17 | 5 / 6 | 5 / 6 |
| CSI CLK- / CLK+ | 8 / 9 | 15 / 14 | 8 / 9 | 8 / 9 |
| I2C SDA | 13 | 10 | 21 | 14 |
| I2C SCL | 14 | 9 | 20 | 13 |
| Chronos 3.3V | 19, 20 | 3, 4 | - (TP1 / SJ1 only) | - |
| Helios 3.3V | - | - | 22 (TP2 / SJ1 only) | 15 |
| GND | 1,4,7,10,12,15,18,21,22 | 1,2,5,8,11,13,16,22 | 4,7,10 (+13,16,19) | 1,4,7,10 |

The I2C swap (Chronos has SDA=13/SCL=14, RPi standard has SCL=13/SDA=14) is
done on the adapter. The two 3.3V rails are **never** connected to each other:
SJ1 solder jumper stays **OPEN** (each board powers itself). J1 pad 19 is left
unconnected on purpose (safety: it would receive Chronos 3.3V if the wrong
cable type is used).

## Bring-up / polarity verification (do this BEFORE full hookup)

FFC orientation mistakes are the only real risk; the board is designed so a
wrong cable type causes no damage and is detectable:

1. **Chronos side**: power Chronos (USB), connect only J7 -> J1 with the 22p
   cable. Measure **TP1 (3V3_CHRONOS)**: must read **3.3 V** (and TP4 = GND
   beeps to Chronos ground). If TP1 does not read 3.3 V, the cable is the
   wrong contact type - swap the 22p cable between Type-A (same-side) and
   Type-B (reverse/opposite-side). Keep this wrong-state power-on brief.
2. **Helios side**: with Chronos disconnected, power Helios, connect CAM1 ->
   J2 with the Aerium forward cable. Measure **TP2 (3V3_HELIOS)**: must read
   **3.3 V**, and **TP3 (SENSE_H)** must beep to ground. If instead TP3 shows
   3.3 V, the cable orientation is mirrored - use the Aerium **reverse**
   variant cable instead.
3. Only after both checks pass, connect everything together (SJ1 open).

## Differential pair design (100 ohm, MIPI D-PHY)

Stackup: **4-layer 1.6 mm, JLCPCB JLC04161H-7628** (or equivalent):
L1 signal / L2 GND / L3 GND / L4 GND pour. Outer copper 35 um over
0.21 mm 7628 prepreg, Er ~ 4.4.

Coupled-microstrip check (IPC-2141 approximations, h = 0.21 mm, t = 35 um):

- w = 0.20 mm -> Z0(single) = 87/sqrt(Er+1.41) * ln(5.98h/(0.8w+t)) ~ 67 ohm
- s = 0.174 mm -> Zdiff = 2*Z0*(1 - 0.48*exp(-0.96*s/h)) ~ 105 ohm

=> **w/s = 0.20/0.174 mm** gives ~100-105 ohm differential (JLC's own
calculator gives 100.5 ohm for this geometry), well inside the D-PHY
80-125 ohm window. That geometry is what is routed: the pairs neck from the
0.5 mm pad pitch to 0.374 mm center spacing for the parallel run. Pairs are
straight, mirror-symmetric, so P/N skew is zero by construction; total run
is ~5.6 mm (~35 ps), negligible at 800 Mbps (UI = 1.25 ns).

A 2-layer build was rejected on purpose: with h = 1.5 mm, 100 ohm diff
would need ~0.96 mm wide traces (impossible to fan out of a 0.5 mm-pitch
connector cleanly), and thin traces would sit at ~145 ohm.

**When ordering choose "Impedance control: JLC04161H-7628"** and keep L2 as
the solid GND reference (it is a full GND zone in the design).

## Mechanical

- 4x M2 mounting holes (H1-H4, 2.2 mm NPTH) in the corners, centers 2.3 mm
  from each edge - fits M2 screws + standoffs.
- Both FH12 connectors have 3D models assigned (KiCad standard lib), so the
  3D viewer (Alt+3) shows the assembled board.

## Fabrication at JLCPCB (verified 2026-07)

Ready-to-upload outputs are in `fab/`:

- `chronos_helios_adapter_gerbers.zip` - Gerbers + Excellon drills (upload as-is)
- `jlc_bom.csv` / `jlc_cpl.csv` - BOM and CPL in JLC assembly format

Order options:

- **Layers 4, thickness 1.6 mm**, and set **Impedance Control: yes ->
  JLC04161H-7628**. Layer order in the zip: `.gtl` = L1, `.g1` = L2 (GND),
  `.g2` = L3 (GND), `.gbl` = L4.
- Everything is inside standard capabilities (checked against JLC 4-layer
  rules): min track/gap used 0.20/0.174 mm (cap. 0.09/0.09), vias 0.6/0.3 mm
  (cap. 0.45/0.2), NPTH 2.2 mm, outer copper-to-edge 0.3 mm, inner
  copper-to-edge 0.5 mm, silkscreen line width >= 0.15 mm.
- LeadFree/HASL either is fine; ENIG is nicer for the 0.5 mm pads but not
  required.

### Connector (BOM check)

The genuine Hirose FH12-22S-0.5SH(55) is **out of stock at LCSC (C597981)**,
so for JLC assembly the board uses the pin-compatible clone:

- **XUNPU FPC-05F-22PH20, LCSC C2856804** - 22P 0.5 mm bottom-contact,
  hinged-lid (flip-lock) ZIF, 0.3 mm FFC, in stock (~$0.10/pc, MOQ 5,
  JLC "Extended" part). Same 10.5 mm contact span, 0.5 mm pitch, 0.3 x 1.5 mm
  recommended pads (ours are 0.3 x 1.3, within both patterns).
- The board's mounting-tab pads were **enlarged to 2.1 x 2.9 mm** to cover
  both the Hirose tab position (+-7.15, offset 1.4) and the XUNPU tab
  position (+-6.94, offset 0.825) - so either brand solders onto this board.
- If you prefer genuine Hirose, buy FH12-22S-0.5SH(55) from
  Mouser/DigiKey/Ariat and hand-solder or consign; nothing changes on the PCB.
- In the JLC assembly preview, verify the connector **opening faces the board
  edge** (cable exits outward) on both J1 and J2; fix rotation in their tool
  if their part library orientation differs.

TPs, SJ1 and H1-H4 are bare copper/holes - no other parts to source.

### Cables

- 1x 22-pin 0.5 mm same-side (Type-A) FFC, ~50-100 mm, plus one Type-B
  (reverse) as insurance - they cost cents (also available via LCSC custom
  cables, MOQ 1).
- The Aerium 15p-22p forward cable you already have for the Helios side.

## Files

- `chronos_helios_adapter.kicad_pcb` - the board (open in KiCad 10)
- `chronos_helios_adapter.kicad_pro` - project file (incl. CSI_100R netclass,
  diff pair w/gap 0.20/0.174 mm)
- `drc_report.txt` - last DRC run (clean)
- `render_top.png` / `render_3d.png` - previews
- `fab/` - JLCPCB-ready Gerber zip, drills, BOM (`jlc_bom.csv`),
  CPL (`jlc_cpl.csv`)
