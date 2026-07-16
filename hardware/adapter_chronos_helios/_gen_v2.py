# Generates chronos_helios_adapter.kicad_pcb v2:
#   J1 = FH12-22S (to Chronos J7, straight 22p Type-A FFC)
#   J2 = FH12-15S (to Helios CAM1, straight 15p Type-A FFC)
#   Full pin-order crossover routed on the board (F.Cu stubs + In2.Cu lanes),
#   In1.Cu and B.Cu are solid GND planes (In2.Cu references B.Cu, 0.21mm prepreg).
# Reuses footprint blocks from the committed v1 file (git show) as templates.
import re
import subprocess
import uuid as uuidlib

REPO = r"C:\Projects\Github\Chronos"
OUT = REPO + r"\hardware\adapter_chronos_helios\chronos_helios_adapter.kicad_pcb"

v1 = subprocess.run(
    ["git", "-C", REPO, "show",
     "39097af:hardware/adapter_chronos_helios/chronos_helios_adapter.kicad_pcb"],
    capture_output=True, text=True, encoding="utf-8", check=True).stdout


def U():
    return str(uuidlib.uuid4())


def balanced(text, start):
    depth = 0
    i = start
    while True:
        c = text[i]
        if c == '"':
            i += 1
            while text[i] != '"':
                i += 1
        elif c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return i + 1
        i += 1


fps = {}
for m in re.finditer(r'\(footprint ', v1):
    blk = v1[m.start():balanced(v1, m.start())]
    ref = re.search(r'\(property "Reference" "([^"]+)"', blk)
    fps[ref.group(1) if ref else '?'] = blk


def move(blk, newat, refat=None):
    blk = re.sub(r'\(at [-\d\. ]+\)', '(at %s)' % newat, blk, count=1)
    if refat:
        # override the Reference property position (first (at ...) after it)
        def sub_ref(mm):
            return mm.group(1) + '(at %s)' % refat
        blk = re.sub(r'(\(property "Reference" "[^"]+"\s*\n\s*)\(at [-\d\. ]+\)',
                     sub_ref, blk, count=1)
    return blk


# ---------------------------------------------------------------- header ----
header = v1[:v1.find('(footprint ')]
header = header.replace(
    '(comment 1 "J1 = to Chronos J7 via straight 22p 0.5mm Type-A (same-side) FFC")',
    '(comment 1 "J1 -> Chronos J7: straight 22p 0.5mm Type-A (same-side) FFC")')
header = re.sub(r'\(comment 2 "[^"]*"\)',
                '(comment 2 "J2 -> Helios CAM1: straight 15p 0.5mm Type-A (same-side) FFC")',
                header)
header = re.sub(r'\(comment 3 "[^"]*"\)',
                '(comment 3 "Full pin-order crossover on this board; SJ1 open = 3V3 rails isolated")',
                header)
header = re.sub(r'\(rev "1"\)', '(rev "2")', header)

# ------------------------------------------------------------ footprints ----
parts = []
j1blk = move(fps['J1'], '112 119')
# shorten J1 silk verticals so they stop clear of the enlarged universal MP pads
j1blk = j1blk.replace('(end -7.15 0.04)', '(end -7.15 -0.55)')
j1blk = j1blk.replace('(end 7.15 0.04)', '(end 7.15 -0.55)')
parts.append(j1blk)
parts.append(move(fps['TP1'], '104.8 116'))                          # V33_CHRONOS
parts.append(move(fps['TP2'], '117.6 107.85', refat='1.9 0.9 0'))    # V33_HELIOS
parts.append(move(fps['TP3'], '106.2 107.85', refat='1.2 1.5 0'))    # SENSE_H
parts.append(move(fps['TP4'], '105.3 110.1', refat='1.9 0.9 0'))     # GND
parts.append(move(fps['SJ1'], '103.5 110.1'))
parts.append(move(fps['H1'], '100.3 100.3'))
parts.append(move(fps['H2'], '123.7 100.3'))
parts.append(move(fps['H3'], '100.3 123.7'))
parts.append(move(fps['H4'], '123.7 123.7'))

# J2: 15-pin FH12-15S / XFCN F0502-B-15-15T-R at (112,106) rot 180.
# pad q abs x = 115.5 - 0.5(q-1) (pin 1 east), pad row abs y = 107.85.
J2_NETS = {
    1: "V33_HELIOS", 2: "I2C_SDA", 3: "I2C_SCL", 4: None, 5: None,
    6: "GND", 7: "CSI_CLK_P", 8: "CSI_CLK_N", 9: "GND", 10: "CSI_D1_P",
    11: "CSI_D1_N", 12: "GND", 13: "CSI_D0_P", 14: "CSI_D0_N", 15: "SENSE_H",
}

pads = []
for q in range(1, 16):
    relx = -3.5 + 0.5 * (q - 1)
    netline = '\n\t\t(net "%s")' % J2_NETS[q] if J2_NETS[q] else ''
    pads.append('''\t(pad "%d" smd rect
\t\t(at %.6g -1.85 180)
\t\t(size 0.3 1.3)
\t\t(layers "F.Cu" "F.Paste" "F.Mask")%s
\t\t(uuid "%s")
\t)''' % (q, relx, netline, U()))
for sx in (-5.3, 5.3):  # universal MP tabs (Hirose FH12-15S + XFCN F0502 clone)
    pads.append('''\t(pad "MP" smd rect
\t\t(at %.6g 1.05 180)
\t\t(size 2.1 2.9)
\t\t(layers "F.Cu" "F.Paste" "F.Mask")
\t\t(net "GND")
\t\t(uuid "%s")
\t)''' % (sx, U()))

j2 = '''(footprint "Connector_FFC-FPC:Hirose_FH12-15S-0.5SH_1x15-1MP_P0.50mm_Horizontal"
\t(layer "F.Cu")
\t(uuid "%s")
\t(at 112 106 180)
\t(descr "15 pos 0.5mm bottom-contact FFC (Hirose FH12-15S / XFCN F0502-B-15-15T-R), cable exits -y (toward Helios)")
\t(tags "connector FH12 FFC")
\t(property "Reference" "J2"
\t\t(at -7.9 0.5 180)
\t\t(layer "F.SilkS")
\t\t(uuid "%s")
\t\t(effects
\t\t\t(font
\t\t\t\t(size 1 1)
\t\t\t\t(thickness 0.15)
\t\t\t)
\t\t)
\t)
\t(property "Value" "F0502-B-15-15T-R"
\t\t(at 0 5.6 180)
\t\t(layer "F.Fab")
\t\t(uuid "%s")
\t\t(effects
\t\t\t(font
\t\t\t\t(size 1 1)
\t\t\t\t(thickness 0.15)
\t\t\t)
\t\t)
\t)
\t(property "Datasheet" ""
\t\t(at 0 0 180)
\t\t(layer "F.Fab")
\t\t(hide yes)
\t\t(uuid "%s")
\t\t(effects
\t\t\t(font
\t\t\t\t(size 1.27 1.27)
\t\t\t)
\t\t)
\t)
\t(property "Description" ""
\t\t(at 0 0 180)
\t\t(layer "F.Fab")
\t\t(hide yes)
\t\t(uuid "%s")
\t\t(effects
\t\t\t(font
\t\t\t\t(size 1.27 1.27)
\t\t\t)
\t\t)
\t)
\t(property "LCSC" "C2844857"
\t\t(at 0 0 180)
\t\t(layer "F.Fab")
\t\t(hide yes)
\t\t(uuid "%s")
\t\t(effects
\t\t\t(font
\t\t\t\t(size 1 1)
\t\t\t\t(thickness 0.15)
\t\t\t)
\t\t)
\t)
\t(attr smd)
\t(duplicate_pad_numbers_are_jumpers no)
\t(fp_line
\t\t(start -5.4 2.8)
\t\t(end -5.4 5)
\t\t(stroke
\t\t\t(width 0.15)
\t\t\t(type solid)
\t\t)
\t\t(layer "F.SilkS")
\t\t(uuid "%s")
\t)
\t(fp_line
\t\t(start -5.4 5)
\t\t(end 5.4 5)
\t\t(stroke
\t\t\t(width 0.15)
\t\t\t(type solid)
\t\t)
\t\t(layer "F.SilkS")
\t\t(uuid "%s")
\t)
\t(fp_line
\t\t(start 5.4 2.8)
\t\t(end 5.4 5)
\t\t(stroke
\t\t\t(width 0.15)
\t\t\t(type solid)
\t\t)
\t\t(layer "F.SilkS")
\t\t(uuid "%s")
\t)
\t(fp_circle
\t\t(center -4.1 -2.9)
\t\t(end -3.95 -2.9)
\t\t(stroke
\t\t\t(width 0.3)
\t\t\t(type solid)
\t\t)
\t\t(fill no)
\t\t(layer "F.SilkS")
\t\t(uuid "%s")
\t)
\t(fp_rect
\t\t(start -6.85 -3)
\t\t(end 6.85 5.4)
\t\t(stroke
\t\t\t(width 0.05)
\t\t\t(type solid)
\t\t)
\t\t(fill no)
\t\t(layer "F.CrtYd")
\t\t(uuid "%s")
\t)
\t(fp_rect
\t\t(start -5.3 -1.3)
\t\t(end 5.3 4.9)
\t\t(stroke
\t\t\t(width 0.1)
\t\t\t(type solid)
\t\t)
\t\t(fill no)
\t\t(layer "F.Fab")
\t\t(uuid "%s")
\t)
%s
\t(embedded_fonts no)
\t(model "${KICAD10_3DMODEL_DIR}/Connector_FFC-FPC.3dshapes/Hirose_FH12-15S-0.5SH_1x15-1MP_P0.50mm_Horizontal.step"
\t\t(offset
\t\t\t(xyz 0 0 0)
\t\t)
\t\t(scale
\t\t\t(xyz 1 1 1)
\t\t)
\t\t(rotate
\t\t\t(xyz 0 0 0)
\t\t)
\t)
)''' % tuple([U() for _ in range(12)] + ['\n'.join(pads)])
parts.append(j2)

# ------------------------------------------------------------- graphics -----
gfx = []
gfx.append('''(gr_rect
\t(start 98 98)
\t(end 126 126)
\t(stroke
\t\t(width 0.1)
\t\t(type solid)
\t)
\t(fill no)
\t(layer "Edge.Cuts")
\t(uuid "%s")
)''' % U())


def gr_text(txt, x, y, layer, size, mirror=False):
    just = '\n\t\t(justify mirror)' if mirror else ''
    return '''(gr_text "%s"
\t(at %.6g %.6g 0)
\t(layer "%s")
\t(uuid "%s")
\t(effects
\t\t(font
\t\t\t(size %.6g %.6g)
\t\t\t(thickness %.6g)
\t\t)%s
\t)
)''' % (txt, x, y, layer, U(), size, size, max(0.1, round(size * 0.16, 2)), just)


gfx.append(gr_text("CHRONOS-HELIOS ADAPTER v2", 112, 99.1, "F.SilkS", 0.8))
gfx.append(gr_text("^ HELIOS CAM1: 15P TYPE-A", 112, 100.15, "F.SilkS", 0.8))
gfx.append(gr_text("v CHRONOS J7: 22P TYPE-A", 112, 125.0, "F.SilkS", 0.8))
gfx.append(gr_text("3V3C", 104.8, 114.6, "F.SilkS", 0.8))
gfx.append(gr_text("SENS", 103.5, 107.85, "F.SilkS", 0.8))
gfx.append(gr_text("V33H", 120.0, 107.6, "F.SilkS", 0.8))
gfx.append(gr_text("CROSSOVER ON BOARD", 112, 110.2, "B.SilkS", 0.8, True))
gfx.append(gr_text("BOTH CABLES: STRAIGHT TYPE-A FFC", 112, 111.6, "B.SilkS", 0.8, True))
gfx.append(gr_text("SJ1 OPEN = 3V3 RAILS ISOLATED", 112, 113.0, "B.SilkS", 0.8, True))
gfx.append(gr_text("BEFORE POWER:", 112, 114.4, "B.SilkS", 0.8, True))
gfx.append(gr_text("TP-V33H=3.3V, TP-SENS=GND", 112, 115.8, "B.SilkS", 0.8, True))

# --------------------------------------------------------------- tracks -----
segs = []   # (net, x1, y1, x2, y2, width, layer)
J1Y = 117.15
J2Y = 107.85

# Diff pair stubs on F.Cu. Vias staggered 0.3mm outward wherever a straight
# drop would leave exactly 0.15mm to the partner net's stub; a short elbow at
# lane depth reaches the shifted via.
# (net, pad_x, lane_y, via_x)
j1_stub = [("CSI_CLK_P", 113.25, 110.6, 113.25), ("CSI_CLK_N", 113.75, 111.2, 114.05),
           ("CSI_D1_P", 114.75, 112.0, 114.75), ("CSI_D1_N", 115.25, 112.6, 115.55),
           ("CSI_D0_P", 116.25, 113.4, 116.25), ("CSI_D0_N", 116.75, 114.0, 117.05)]
j2_stub = [("CSI_CLK_P", 112.5, 110.6, 112.5),
           ("CSI_D1_P", 111.0, 112.0, 111.3), ("CSI_D1_N", 110.5, 112.6, 110.5),
           ("CSI_D0_P", 109.5, 113.4, 109.8), ("CSI_D0_N", 109.0, 114.0, 109.0)]
for net, x, y, vx in j1_stub:
    segs.append((net, x, J1Y, x, y, 0.2, "F.Cu"))
    if vx != x:
        segs.append((net, x, y, vx, y, 0.2, "F.Cu"))
for net, x, y, vx in j2_stub:
    segs.append((net, x, J2Y, x, y, 0.2, "F.Cu"))
    if vx != x:
        segs.append((net, x, y, vx, y, 0.2, "F.Cu"))
# CLK_N J2 needs a west dogleg (pair window too narrow for the standard stagger)
segs.append(("CSI_CLK_N", 112.0, J2Y, 112.0, 110.4, 0.2, "F.Cu"))
segs.append(("CSI_CLK_N", 112.0, 110.4, 111.7, 110.4, 0.2, "F.Cu"))
segs.append(("CSI_CLK_N", 111.7, 110.4, 111.7, 111.2, 0.2, "F.Cu"))
j2_stub.append(("CSI_CLK_N", 112.0, 111.2, 111.7))
# I2C F.Cu stubs (with elbows to staggered vias)
segs.append(("I2C_SCL", 110.75, J1Y, 110.75, 116.0, 0.2, "F.Cu"))
segs.append(("I2C_SCL", 110.75, 116.0, 110.45, 116.0, 0.2, "F.Cu"))
segs.append(("I2C_SDA", 111.25, J1Y, 111.25, 115.75, 0.2, "F.Cu"))
segs.append(("I2C_SCL", 114.5, J2Y, 114.5, 109.05, 0.2, "F.Cu"))
segs.append(("I2C_SCL", 114.5, 109.05, 114.2, 109.05, 0.2, "F.Cu"))
segs.append(("I2C_SDA", 115.0, J2Y, 115.0, 109.35, 0.2, "F.Cu"))

# In2.Cu crossover lanes (between the staggered via x positions)
lanes = [("CSI_CLK_P", 112.5, 113.25, 110.6), ("CSI_CLK_N", 111.7, 114.05, 111.2),
         ("CSI_D1_P", 111.3, 114.75, 112.0), ("CSI_D1_N", 110.5, 115.55, 112.6),
         ("CSI_D0_P", 109.8, 116.25, 113.4), ("CSI_D0_N", 109.0, 117.05, 114.0)]
for net, xa, xb, y in lanes:
    segs.append((net, xa, y, xb, y, 0.25, "In2.Cu"))
# I2C C-routes on In2.Cu around the east side (I2C swaps sides entirely)
segs.append(("I2C_SCL", 114.2, 109.05, 114.2, 108.75, 0.25, "In2.Cu"))
segs.append(("I2C_SCL", 114.2, 108.75, 119.0, 108.75, 0.25, "In2.Cu"))
segs.append(("I2C_SCL", 119.0, 108.75, 119.0, 116.3, 0.25, "In2.Cu"))
segs.append(("I2C_SCL", 119.0, 116.3, 110.45, 116.3, 0.25, "In2.Cu"))
segs.append(("I2C_SCL", 110.45, 116.3, 110.45, 116.0, 0.25, "In2.Cu"))
segs.append(("I2C_SDA", 115.0, 109.35, 118.4, 109.35, 0.25, "In2.Cu"))
segs.append(("I2C_SDA", 118.4, 109.35, 118.4, 115.75, 0.25, "In2.Cu"))
segs.append(("I2C_SDA", 118.4, 115.75, 111.25, 115.75, 0.25, "In2.Cu"))

# V33_CHRONOS (F.Cu only, west)
segs.append(("V33_CHRONOS", 107.75, J1Y, 108.25, J1Y, 0.3, "F.Cu"))
segs.append(("V33_CHRONOS", 107.75, J1Y, 107.75, 116.0, 0.3, "F.Cu"))
segs.append(("V33_CHRONOS", 107.75, 116.0, 103.5, 116.0, 0.3, "F.Cu"))
segs.append(("V33_CHRONOS", 103.5, 116.0, 103.5, 110.75, 0.3, "F.Cu"))
# V33_HELIOS (F.Cu east; In2.Cu west run to SJ1)
segs.append(("V33_HELIOS", 115.5, J2Y, 118.3, J2Y, 0.3, "F.Cu"))
segs.append(("V33_HELIOS", 118.3, J2Y, 118.3, 108.0, 0.3, "F.Cu"))
segs.append(("V33_HELIOS", 118.3, 108.0, 103.5, 108.0, 0.3, "In2.Cu"))
segs.append(("V33_HELIOS", 103.5, 108.0, 103.5, 109.45, 0.3, "F.Cu"))
# SENSE_H to TP3
segs.append(("SENSE_H", 108.5, J2Y, 106.2, J2Y, 0.25, "F.Cu"))
# TP4 GND stub to its via
segs.append(("GND", 105.3, 110.1, 105.3, 110.9, 0.25, "F.Cu"))

j1_gnd = [(106.75, 118.5), (107.25, 119.2), (108.75, 118.5), (110.25, 119.2),
          (111.75, 118.5), (112.75, 119.2), (114.25, 118.5), (117.25, 119.2)]
for x, y in j1_gnd:
    segs.append(("GND", x, J1Y, x, y, 0.25, "F.Cu"))
j2_gnd = [(113.0, 106.9), (111.5, 106.3), (110.0, 106.9)]
for x, y in j2_gnd:
    segs.append(("GND", x, J2Y, x, y, 0.25, "F.Cu"))

# ----------------------------------------------------------------- vias -----
vias = []
sig_vias = [(net, vx, y) for net, _x, y, vx in j1_stub] + \
           [(net, vx, y) for net, _x, y, vx in j2_stub] + \
           [("I2C_SCL", 110.45, 116.0), ("I2C_SCL", 114.2, 109.05),
            ("I2C_SDA", 111.25, 115.75), ("I2C_SDA", 115.0, 109.35)]
for net, x, y in sig_vias:
    vias.append((net, x, y, 0.5, 0.3))
vias.append(("V33_HELIOS", 118.3, 108.0, 0.6, 0.3))
vias.append(("V33_HELIOS", 103.5, 108.0, 0.6, 0.3))
gnd_vias = j1_gnd + [(105.0, 120.05), (119.0, 120.05)] + j2_gnd + \
    [(106.7, 104.95), (117.3, 104.95), (102.2, 110.1), (120.5, 110.1),
     (105.3, 110.9), (101.8, 101.8), (122.2, 101.8), (101.8, 122.2), (122.2, 122.2)]
for x, y in gnd_vias:
    vias.append(("GND", x, y, 0.6, 0.3))

# ---------------------------------------------------------------- zones -----
def zone(layer, x1, y1, x2, y2):
    return '''(zone
\t(net "GND")
\t(layer "%s")
\t(uuid "%s")
\t(hatch edge 0.508)
\t(connect_pads yes
\t\t(clearance 0.25)
\t)
\t(min_thickness 0.25)
\t(fill yes
\t\t(thermal_gap 0.3)
\t\t(thermal_bridge_width 0.4)
\t\t(island_removal_mode 0)
\t)
\t(polygon
\t\t(pts
\t\t\t(xy %.6g %.6g) (xy %.6g %.6g) (xy %.6g %.6g) (xy %.6g %.6g)
\t\t)
\t)
)''' % (layer, U(), x1, y1, x2, y1, x2, y2, x1, y2)


zones = [zone("F.Cu", 98.3, 98.3, 125.7, 125.7),
         zone("In1.Cu", 98.4, 98.4, 125.6, 125.6),
         zone("B.Cu", 98.3, 98.3, 125.7, 125.7)]
# In2.Cu deliberately has NO zone: it is the crossover signal layer.

# ----------------------------------------------------------------- emit -----
def indent(block):
    return '\t' + block.replace('\n', '\n\t')


out = [header.rstrip('\n')]
for p in parts:
    out.append(indent(p))
for g in gfx:
    out.append(indent(g))
for net, x1, y1, x2, y2, w, layer in segs:
    out.append('''\t(segment
\t\t(start %.6g %.6g)
\t\t(end %.6g %.6g)
\t\t(width %.6g)
\t\t(layer "%s")
\t\t(net "%s")
\t\t(uuid "%s")
\t)''' % (x1, y1, x2, y2, w, layer, net, U()))
for net, x, y, size, drill in vias:
    out.append('''\t(via
\t\t(at %.6g %.6g)
\t\t(size %.6g)
\t\t(drill %.6g)
\t\t(layers "F.Cu" "B.Cu")
\t\t(net "%s")
\t\t(uuid "%s")
\t)''' % (x, y, size, drill, net, U()))
for z in zones:
    out.append(indent(z))
out.append('\t(embedded_fonts no)')
out.append(')')

with open(OUT, 'w', encoding='utf-8', newline='\n') as f:
    f.write('\n'.join(out) + '\n')
print("wrote", OUT)
