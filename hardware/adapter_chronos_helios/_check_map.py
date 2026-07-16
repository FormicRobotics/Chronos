# End-to-end mapping sanity check for adapter v3.
# Chain: J7 pin i --(mirror cable)--> J1 pad 23-i --(board net)--> J2 pad q
#        --(straight cable)--> CAM pin q. Verify signal-correct.
import re
import sys

sys.stdout.reconfigure(encoding='utf-8')

src = open(r'C:\Projects\Github\Chronos\hardware\adapter_chronos_helios\chronos_helios_adapter.kicad_pcb',
           encoding='utf-8').read()


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


def pads_of(ref):
    i = src.find('"%s"' % ref)
    start = src.rfind('(footprint', 0, i)
    blk = src[start:balanced(src, start)]
    out = {}
    for m in re.finditer(r'\(pad "(\d+)"', blk):
        pblk = blk[m.start():balanced(blk, m.start())]
        n = re.search(r'\(net \d+ "([^"]+)"\)|\(net "([^"]+)"\)', pblk)
        out[int(m.group(1))] = (n.group(1) or n.group(2)) if n else None
    return out


j1 = pads_of('J1')
j2 = pads_of('J2')

# Chronos J7 pinout (netlist-verified) and Helios CAM1 pinout (RPi std)
J7 = {1: 'GND', 2: 'D0_N', 3: 'D0_P', 4: 'GND', 5: 'D1_N', 6: 'D1_P',
      7: 'GND', 8: 'CLK_N', 9: 'CLK_P', 10: 'GND', 11: None, 12: 'GND',
      13: 'SDA', 14: 'SCL', 15: 'GND', 16: None, 17: None, 18: 'GND',
      19: '3V3', 20: '3V3', 21: 'GND', 22: 'GND'}
CAM = {1: 'GND', 2: 'D0_N', 3: 'D0_P', 4: 'GND', 5: 'D1_N', 6: 'D1_P',
       7: 'GND', 8: 'CLK_N', 9: 'CLK_P', 10: 'GND', 11: None, 12: None,
       13: 'SCL', 14: 'SDA', 15: '3V3'}

print('J7 pin -> J1 pad (23-i) -> net:')
for i in sorted(J7):
    p = 23 - i
    print('  J7.%-2d %-5s -> J1.%-2d net=%s' % (i, J7[i], p, j1.get(p)))

print()
print('CAM pin -> J2 pad (straight) -> net:')
for q in sorted(CAM):
    print('  CAM.%-2d %-5s -> J2.%-2d net=%s' % (q, CAM[q], q, j2.get(q)))

# automated cross-check of the signal chain
sig_of_net = {}
errs = 0
for i, sig in J7.items():
    if sig in (None, 'GND', '3V3'):
        continue
    net = j1.get(23 - i)
    sig_of_net[net] = sig
for q, sig in CAM.items():
    if sig in (None, 'GND', '3V3'):
        continue
    net = j2.get(q)
    got = sig_of_net.get(net)
    ok = got == sig
    if not ok:
        errs += 1
    print('%s CAM.%d %s via net %s carries J7 signal %s' %
          ('OK ' if ok else 'ERR', q, sig, net, got))

# power / gnd checks
for i in (19, 20):
    net = j1.get(23 - i)
    print(('OK ' if net == 'V33_CHRONOS' else 'ERR'),
          'J7.%d 3V3 -> J1.%d net=%s (expect V33_CHRONOS)' % (i, 23 - i, net))
    errs += net != 'V33_CHRONOS'
net = j2.get(15)
print(('OK ' if net == 'V33_HELIOS' else 'ERR'),
      'CAM.15 3V3 -> J2.15 net=%s (expect V33_HELIOS)' % net)
errs += net != 'V33_HELIOS'
net = j2.get(1)
print(('OK ' if net == 'SENSE_H' else 'ERR'),
      'CAM.1 GND -> J2.1 net=%s (expect SENSE_H sense pad)' % net)
errs += net != 'SENSE_H'
for i, sig in J7.items():
    if sig != 'GND':
        continue
    net = j1.get(23 - i)
    if 23 - i == 19:
        # J1.19 is intentionally NC: with a wrong-type (mirrored) 22p cable it
        # would receive J7.19 = 3.3V, so leaving it open prevents a 3V3-GND
        # short. 8 other J7 grounds remain connected.
        print(('OK ' if net is None else 'ERR'),
              'J7.%d GND -> J1.19 intentionally NC (wrong-cable safety)' % i)
        errs += net is not None
        continue
    if net != 'GND':
        print('ERR J7.%d GND -> J1.%d net=%s' % (i, 23 - i, net))
        errs += 1
for q, sig in CAM.items():
    if sig != 'GND' or q == 1:
        continue
    net = j2.get(q)
    if net != 'GND':
        print('ERR CAM.%d GND -> J2.%d net=%s' % (q, q, net))
        errs += 1

print()
print('RESULT:', 'PASS' if errs == 0 else 'FAIL (%d errors)' % errs)
