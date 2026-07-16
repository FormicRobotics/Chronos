# Chronos J7 <-> Aerium Helios CAM1 adapter PCB (v3)

Small (28 x 28 mm, 4-layer) adapter that safely connects the Chronos board's
custom 22-pin CSI output (J7) to the Aerium Helios carrier's standard
Raspberry-Pi-style 15-pin CAM1 port. A direct cable between the two is **not**
possible: J7 uses a custom pinout (SDA/SCL order reversed, 3.3V on pins 19/20,
GND on pin 15) that would short Helios 3.3V to GND and put 3.3V on I2C lines.

**v3 change (pin-1 orientation verified on hardware):** with the two boards
face to face and flat cables, **both pin 1s sit on the same absolute side**
(user-verified on the physical boards, 2026-07). The Chronos link mirrors
through its cable as normal facing FH12s do (J7 pin i lands on J1 pad 23-i),
but the Helios CAM1 connector is opposite-handed, so its link maps **straight**
(CAM1 pin j lands on J2 pad j). The two effects nearly cancel: v3 routes all
six CSI conductors as identical straight diagonals on the top layer with
**zero vias**, and only the I2C SDA/SCL swap needs one inner-layer crossing.

Open `chronos_helios_adapter.kicad_pcb` in KiCad 10 (board file is the source
of truth; there is no schematic - the netlist lives in the board).
DRC: 0 errors / 0 unconnected items; only informational "library footprint
mismatch" warnings remain (universal mounting-tab pads, see `drc_report.txt`).

## Connections

```
CHRONOS J7                      ADAPTER                       HELIOS CAM1
(22p custom)  straight 22p   J1 <- remap -> J2     straight 15p   (15p RPi std)
    [J7] ==== Type-A FFC ==== [J1]          [J2] ==== Type-A FFC ==== [CAM1]
              (mirrors i->23-i)              (straight j->j)
```

- **J1** (22P 0.5 mm bottom contact, FH12-22S / XUNPU FPC-05F-22PH20): to
  Chronos J7 with a **straight 22-pin 0.5 mm Type-A (same-side contacts) FFC**.
  Facing receptacles mirror the mapping through the cable (J7 pin i lands on
  J1 pad 23-i); the netlist accounts for this.
- **J2** (15P 0.5 mm bottom contact, FH12-15S / XFCN F0502-B-15-15T-R): to
  Helios CAM1 with a **straight 15-pin 0.5 mm Type-A FFC**. The Helios CAM
  connector is opposite-handed (pin 1 on the same absolute side as J7's, as
  verified on the hardware), so this link maps **straight**: CAM1 pin j lands
  on J2 pad j, baked into the netlist.
- Net result on the board: every CSI net leaves J1 and reaches J2 shifted a
  constant 1.75 mm sideways - all six conductors are equal-length top-layer
  diagonals (no vias, no skew). Only SDA crosses SCL once, on inner layer
  In2.Cu over the solid B.Cu ground plane.

### Signal map

| Function | Chronos J7 | Adapter J1 pad | Adapter J2 pad | Helios CAM1 (15p) |
|---|---|---|---|---|
| CSI D0- / D0+ | 2 / 3 | 21 / 20 | 2 / 3 | 2 / 3 |
| CSI D1- / D1+ | 5 / 6 | 18 / 17 | 5 / 6 | 5 / 6 |
| CSI CLK- / CLK+ | 8 / 9 | 15 / 14 | 8 / 9 | 8 / 9 |
| I2C SDA | 13 | 10 | 14 | 14 |
| I2C SCL | 14 | 9 | 13 | 13 |
| Chronos 3.3V | 19, 20 | 3, 4 | - (TP1 / SJ1 only) | - |
| Helios 3.3V | - | - | 15 (TP2 / SJ1 only) | 15 |
| SENSE (orientation check) | - | - | 1 (TP3 only) | 1 (GND) |
| GND | 1,4,7,10,12,15,18,21,22 | 1,2,5,8,11,13,16,22 | 4,7,10 | 4,7,10 |

The I2C swap (Chronos has SDA=13/SCL=14, RPi standard has SCL=13/SDA=14) is
done on the adapter. The two 3.3V rails are **never** connected to each other:
SJ1 solder jumper stays **OPEN** (each board powers itself). J1 pad 19 is left
unconnected on purpose (safety if a wrong cable type is used), and J2 pad 1
is the SENSE_H net (TP3 only) - in the correct orientation it sees CAM1 GND;
with a wrong/mirror cable it would see CAM1 3.3V harmlessly.

## Bring-up / polarity verification (do this BEFORE full hookup)

FFC orientation mistakes are the only real risk; the board is designed so a
wrong cable type causes no damage and is detectable:

1. **Chronos side**: power Chronos (USB), connect only J7 -> J1 with the 22p
   cable. Measure **TP1 (3V3C)**: must read **3.3 V** (and TP4 = GND beeps to
   Chronos ground). If TP1 does not read 3.3 V, the cable is the wrong
   contact type - swap the 22p cable between Type-A (same-side) and Type-B
   (reverse/opposite-side). Keep this wrong-state power-on brief.
2. **Helios side**: with Chronos disconnected, power Helios, connect CAM1 ->
   J2 with the 15p cable. Measure **TP2 (V33H)**: must read **3.3 V**, and
   **TP3 (SENS)** must beep to ground. If instead TP3 shows 3.3 V and TP2
   reads 0, the cable is the wrong contact type - swap the 15p cable between
   Type-A and Type-B.
3. Only after both checks pass, connect everything together (SJ1 open).

Cables must lie **flat** between the boards (no folds or twists) - a folded
FFC flips the mapping and will show up in the TP checks above.

## Differential pair design (100 ohm, MIPI D-PHY)

Stackup: **4-layer 1.6 mm, JLCPCB JLC04161H-7628** (or equivalent):
L1 signal / L2 GND plane / L3 (SDA crossing only) / L4 GND plane.
Outer prepreg 7628 = 0.21 mm on both sides, Er ~ 4.4/4.6.

- All six CSI conductors run on **L1 over the L2 plane** for their entire
  length: w = 0.20 mm, pair pitch 0.5 mm at the pads (Zdiff ~ 105-117 ohm,
  within the 80-125 ohm D-PHY tolerance), total length ~9.6 mm (~65 ps),
  **zero vias**, and all six diagonals are geometrically identical so
  intra-pair and inter-pair skew are both zero by construction.
- I2C: SCL runs on L1; SDA dives to **In2.Cu (L3)** through two 0.5/0.3 mm
  vias to cross SCL (no impedance requirement at 400 kHz).
- In2.Cu carries nothing else and has no copper pour; In1.Cu and B.Cu are
  solid GND zones stitched with 0.6/0.3 mm vias.

**When ordering choose "Impedance control: JLC04161H-7628"** and keep L2/L4
as the solid GND planes.

## Mechanical

- 4x M2 mounting holes (H1-H4, 2.2 mm NPTH) in the corners, centers 2.3 mm
  from each edge - fits M2 screws + standoffs.
- Both connectors have 3D models assigned (KiCad standard lib FH12 models),
  so the 3D viewer (Alt+3) shows the assembled board.
- Cable exits: J2 opening faces the top edge (toward Helios), J1 opening
  faces the bottom edge (toward Chronos).

## Fabrication at JLCPCB (verified 2026-07)

Ready-to-upload outputs are in `fab/`:

- `chronos_helios_adapter_gerbers.zip` - Gerbers + Excellon drills (upload as-is)
- `jlc_bom.csv` / `jlc_cpl.csv` - BOM and CPL in JLC assembly format

Order options:

- **Layers 4, thickness 1.6 mm**, and set **Impedance Control: yes ->
  JLC04161H-7628**. Layer order in the zip: `.gtl` = L1, `.g1` = L2 (GND),
  `.g2` = L3 (SDA crossing), `.gbl` = L4 (GND).
- Everything is inside standard capabilities (checked against JLC 4-layer
  rules): min track/gap used 0.20/0.15 mm (cap. 0.09/0.09), signal vias
  0.5/0.3 mm and GND vias 0.6/0.3 mm (cap. 0.45/0.2), NPTH 2.2 mm,
  copper-to-edge >= 0.3 mm outer / 0.4 mm inner, silkscreen >= 0.8 mm text.
- LeadFree/HASL either is fine; ENIG is nicer for the 0.5 mm pads but not
  required.

### Connectors (BOM check)

Genuine Hirose FH12 parts are out of stock at LCSC, so the BOM uses
pin-compatible clones (both in stock, JLC "Extended" parts):

- **J1: XUNPU FPC-05F-22PH20, LCSC C2856804** - 22P 0.5 mm bottom-contact,
  hinged-lid ZIF for 0.3 mm FFC (~$0.10/pc, MOQ 5).
- **J2: XFCN F0502-B-15-15T-R, LCSC C2844857** - 15P 0.5 mm bottom-contact,
  hinged-lid ZIF for 0.3 mm FFC (in stock ~2.4k pcs).
- Mounting-tab pads on both footprints are **enlarged to 2.1 x 2.9 mm** to
  cover the tab positions of both the Hirose originals and the clones - so
  either brand solders onto this board.
- If you prefer genuine Hirose (FH12-22S-0.5SH(55) / FH12-15S-0.5SH(55)),
  buy from Mouser/DigiKey and hand-solder; nothing changes on the PCB.
- In the JLC assembly preview, verify the connector **opening faces the board
  edge** (cable exits outward) on both J1 and J2; fix rotation in their tool
  if their part library orientation differs.

TPs, SJ1 and H1-H4 are bare copper/holes - no other parts to source.

### Cables (both plain, no special Aerium cable)

- 1x **22-pin 0.5 mm Type-A** (same-side contacts) FFC, ~50-100 mm.
- 1x **15-pin 0.5 mm Type-A** (same-side contacts) FFC, ~50-100 mm.
- Order one Type-B (reverse) of each as insurance - they cost cents (LCSC
  custom cables, MOQ 1). The TP checks above tell you which type is right
  without risking damage.

## Files

- `chronos_helios_adapter.kicad_pcb` - the board (open in KiCad 10)
- `chronos_helios_adapter.kicad_pro` - project file (incl. CSI_100R netclass)
- `drc_report.txt` - last DRC run (0 errors, footprint-lib warnings only)
- `render_top.png` / `render_3d.png` - previews
- `fab/` - JLCPCB-ready Gerber zip, drills, BOM (`jlc_bom.csv`),
  CPL (`jlc_cpl.csv`)
- `_gen_v3.py` / `_fill_zones.py` - generator scripts used to build the board
- `_check_map.py` - end-to-end J7 -> CAM1 signal-map verification (PASS)
