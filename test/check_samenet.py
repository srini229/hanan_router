#!/usr/bin/env python3
"""Same-net DRC on a routed DEF, counting the placed cells' pin shapes as the net's metal.

usage: check_samenet.py DEF LEF LAYERS
  cut     two cuts of a net on one via layer closer than its cut spacing (touching or overlapping; the same
          cut drawn twice is fine), or a cut not of the layer's cut size (overlapping cuts merge into one)
  gap     two shapes of a net on one metal facing each other across less than min spacing, not bridged
  neck    a net's merged metal on one layer narrower than the layer width somewhere between two wider parts
          (it falls apart when eroded by just under half the width)
Prints one line per violation; exit 1 if any.
"""
import json, re, sys
from shapely.geometry import box
from shapely.ops import unary_union


def layers(path):
    rules = {}
    for a in json.load(open(path))['Abstraction']:
        if 'Stack' in a:
            rules[a['Layer']] = ('cut', a.get('SpaceX', 0), a.get('SpaceY', 0), a.get('WidthX', 0), a.get('WidthY', 0))
        elif 'Pitch' in a and 'Width' in a:
            sp = max(a['Pitch'] - a['Width'], a.get('MinSpacing', 0))
            rules[a['Layer']] = ('metal', sp, a['Width'], 0, 0)
    return rules


def lef_pins(path):
    pins, pin, layer = {}, None, None
    for line in open(path):
        w = line.split()
        if not w:
            continue
        if w[0] == 'PIN':
            pin = w[1]
        elif w[0] == 'END' and len(w) > 1 and w[1] == pin:
            pin = None
        elif w[0] == 'OBS':
            pin = None
        elif w[0] == 'LAYER':
            layer = w[1]
        elif w[0] == 'RECT' and pin:
            pins.setdefault(pin, []).append((layer, tuple(int(round(float(v))) for v in w[1:5])))
    return pins


def def_nets(path, pins):
    nets, net = {}, None
    for line in open(path):
        m = re.match(r'\s*- (\S+)\s*$', line)
        if m:
            net = m.group(1)
            nets[net] = []
            continue
        if net is None:
            continue
        for comp, pin in re.findall(r'\( (\S+) (\S+) \)', line):
            if comp != 'PIN' and not re.match(r'-?\d', comp):
                nets[net] += pins.get(pin, [])
        m = re.search(r'RECT (\w+) \( (-?\d+) (-?\d+) \) \( (-?\d+) (-?\d+) \)', line)
        if m:
            nets[net].append((m.group(1), tuple(int(v) for v in m.groups()[1:])))
        if line.strip() == ';':
            net = None
    return nets


def covered(region, others):
    return others is not None and region.difference(others).area < 1


def check(nets, rules):
    bad = []
    for net, shapes in nets.items():
        bylayer = {}
        for l, r in shapes:
            if l in rules and r[2] > r[0] and r[3] > r[1]:
                bylayer.setdefault(l, set()).add(r)
        for l, rs in bylayer.items():
            kind, sa, sb, wx, wy = rules[l]
            rs = sorted(rs)
            if kind == 'cut' and wx and wy:
                bad += [f'cut {net} {l} {r} size' for r in rs if sorted((r[2] - r[0], r[3] - r[1])) != sorted((wx, wy))]
            union = unary_union([box(*r) for r in rs])
            for i, a in enumerate(rs):
                for b in rs[i + 1:]:
                    gx = max(a[0], b[0]) - min(a[2], b[2])
                    gy = max(a[1], b[1]) - min(a[3], b[3])
                    if kind == 'cut':
                        if (max(gx, 0) < sa and max(gy, 0) < sb) and not (a[0] >= b[0] and a[1] >= b[1] and a[2] <= b[2] and a[3] <= b[3]) \
                                and not (b[0] >= a[0] and b[1] >= a[1] and b[2] <= a[2] and b[3] <= a[3]):
                            bad.append(f'cut {net} {l} {a} {b}')
                        continue
                    if gx > 0 and gy < 0 and gx < sa:
                        g = box(min(a[2], b[2]), max(a[1], b[1]), max(a[0], b[0]), min(a[3], b[3]))
                    elif gy > 0 and gx < 0 and gy < sa:
                        g = box(max(a[0], b[0]), min(a[3], b[3]), min(a[2], b[2]), max(a[1], b[1]))
                    else:
                        g = None
                    if g is not None and not covered(g, union):
                        bad.append(f'gap {net} {l} {a} {b}')
            if kind == 'metal':
                # eroded by just under half the width, a shape with a neck narrower than the width falls apart
                for part in getattr(union, 'geoms', [union]):
                    core = part.buffer(-(sb / 2 - 1))
                    if len(getattr(core, 'geoms', [core])) > 1:
                        bad.append(f'neck {net} {l} {tuple(int(v) for v in part.bounds)}')
    return bad


if __name__ == '__main__':
    d, lef, lay = sys.argv[1:4]
    bad = check(def_nets(d, lef_pins(lef)), layers(lay))
    for b in bad:
        print(b)
    print(f'{len(bad)} same-net violation(s)')
    sys.exit(1 if bad else 0)
