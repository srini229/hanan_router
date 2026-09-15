#!/usr/bin/env python3
"""Generate a power grid from a layers.json abstraction.

The grid layers, their direction, pitch and width all come out of layers.json,
the same information ALIGN builds its grid from. Straps alternate between the
power nets and span the block.

The grid is emitted as a LEF macro whose pins are the straps, plus a placement
file that instantiates it over the block. That is all hanan_router needs to wire
each cell's power pins up to the grid: the straps are simply another instance's
pins, so the existing router connects them with no changes at all.

    gen_pwr_grid.py -l layers.json -p PLACEMENT.json \\
        --lef pgrid.lef --placement-out pgrid_placement.json

The original GDS path is kept: with -g/-o the straps are drawn into a copy of
the layout instead.
"""

import argparse
import json
import os
import sys


def layer_table(js):
    """Layer name -> {direction, pitch, width, offset, gds layer/datatype}."""
    out = {}
    for e in js.get('Abstraction', []):
        name = e.get('Layer')
        if not name:
            continue
        out[name] = {
            'dir': e.get('Direction'),
            'pitch': e.get('Pitch'),
            'width': e.get('Width'),
            'offset': e.get('Offset') or 0,
            'gds': e.get('GdsLayerNo'),
            'dt': (e.get('GdsDatatype') or {}).get('Draw', 0),
        }
    return out


def grid_layers(js, layers, bottom=None, top=None):
    """The metal layers to put a grid on, bottom-up, from design_info."""
    di = js.get('design_info', {})
    bottom = bottom or di.get('bottom_power_grid_layer')
    top = top or di.get('top_power_grid_layer')
    order = [e['Layer'] for e in js.get('Abstraction', [])
             if e.get('Layer') in layers and layers[e['Layer']]['dir'] in ('H', 'V')]
    if bottom not in order or top not in order:
        sys.exit(f'grid layers {bottom}..{top} are not both routable metals')
    lo, hi = order.index(bottom), order.index(top)
    return order[min(lo, hi):max(lo, hi) + 1]


def straps(bbox, info, nets, stride, widen):
    """Lay straps across bbox on one layer -> [(net, rect), ...].

    Straps sit on the layer's own pitch (times `stride`) and carry the layer's
    own width (times `widen`), so the grid is legal by construction on any PDK
    the abstraction describes.
    """
    x0, y0, x1, y1 = bbox
    pitch = info['pitch'] * max(1, stride)
    width = int(round(info['width'] * widen))
    if not pitch or not width:
        return []
    vertical = info['dir'] == 'V'
    lo, hi = (x0, x1) if vertical else (y0, y1)
    out = []
    # first strap centre on or after the low edge, keeping the whole strap inside
    first = lo + width // 2 + info['offset'] % pitch
    n = 0
    c = first
    while c + width - width // 2 <= hi:
        net = nets[n % len(nets)]
        a, b = c - width // 2, c + width - width // 2
        out.append((net, (a, y0, b, y1) if vertical else (x0, a, x1, b)))
        n += 1
        c += pitch
    return out


def lef_macros(path):
    """Minimal LEF read: macro -> [(layer, rect), ...] over pins and blockages."""
    macros, cur, layer = {}, None, None
    for line in open(path, errors='ignore'):
        t = line.split()
        if not t:
            continue
        if t[0] == 'MACRO':
            cur, layer = t[1], None
            macros[cur] = []
        elif t[0] == 'END' and cur and len(t) > 1 and t[1] == cur:
            cur = None
        elif t[0] == 'LAYER' and cur:
            layer = t[1].rstrip(';')
        elif t[0] == 'RECT' and cur and layer:
            v = [int(float(x)) for x in t[1:5]]
            macros[cur].append((layer, tuple(v)))
    return macros


def placed_shapes(placement, macros, wanted):
    """Existing geometry on `wanted` layers, in block coordinates."""
    d = json.load(open(placement))
    out = []
    for inst in d['modules'][0].get('instances', []):
        name = inst.get('concrete_template_name')
        tr = inst.get('transformation', {})
        ox, oy = tr.get('oX', 0), tr.get('oY', 0)
        sx, sy = tr.get('sX', 1), tr.get('sY', 1)
        for layer, r in macros.get(name, []):
            if layer not in wanted:
                continue
            xs = sorted((ox + sx * r[0], ox + sx * r[2]))
            ys = sorted((oy + sy * r[1], oy + sy * r[3]))
            out.append((layer, (xs[0], ys[0], xs[1], ys[1])))
    return out


def carve(pins, blockers, layers, keep_min):
    """Split straps around metal that is already there.

    A strap laid straight over an existing shape on its own layer is a short
    nothing downstream can fix, so each strap is cut back to the pieces that
    clear every blocker by the layer's spacing. Pieces shorter than keep_min are
    dropped rather than left as stubs.
    """
    dropped = trimmed = 0
    for net, shapes in pins.items():
        out = []
        for layer, r in shapes:
            info = layers.get(layer, {})
            if info.get('dir') not in ('H', 'V'):
                out.append((layer, r))
                continue
            space = max(0, (info['pitch'] or 0) - (info['width'] or 0))
            vertical = info['dir'] == 'V'
            lo, hi = (r[1], r[3]) if vertical else (r[0], r[2])
            cuts = []
            for bl, b in blockers:
                if bl != layer:
                    continue
                if vertical:
                    if b[2] + space <= r[0] or b[0] - space >= r[2]:
                        continue
                    cuts.append((b[1] - space, b[3] + space))
                else:
                    if b[3] + space <= r[1] or b[1] - space >= r[3]:
                        continue
                    cuts.append((b[0] - space, b[2] + space))
            if not cuts:
                out.append((layer, r))
                continue
            pieces, start = [], lo
            for a, b in sorted(cuts):
                if a > start:
                    pieces.append((start, min(a, hi)))
                start = max(start, b)
            if start < hi:
                pieces.append((start, hi))
            kept = 0
            for a, b in pieces:
                if b - a < keep_min:
                    continue
                out.append((layer, (r[0], a, r[2], b) if vertical else (a, r[1], b, r[3])))
                kept += 1
            trimmed += 1
            if not kept:
                dropped += 1
        pins[net] = out
    return trimmed, dropped


def via_layers(js, layers):
    """Cut layer name -> {lower, upper, width, space, enclosures}."""
    out = {}
    for e in js.get('Abstraction', []):
        st = e.get('Stack')
        if not st or len(st) != 2 or not (st[0] and st[1]):
            continue
        out[e['Layer']] = {
            'lo': st[0], 'hi': st[1],
            'wx': e.get('WidthX'), 'wy': e.get('WidthY'),
            'sx': e.get('SpaceX'), 'sy': e.get('SpaceY'),
            'enca_l': e.get('VencA_L') or 0, 'encp_l': e.get('VencP_L') or 0,
            'enca_h': e.get('VencA_H') or 0, 'encp_h': e.get('VencP_H') or 0,
        }
    return out


def stitch(pins, vias, layers):
    """Tie crossing straps of one net together with cut arrays.

    A grid is only a grid if every crossing is connected. At a crossing the cut
    array has to stay inside both straps, but only across each strap: along a
    strap the enclosure is met by the strap running the width of the block. So
    the x margin comes from the vertical layer's VencA and the y margin from the
    horizontal one's, and the cut pitch is the abstraction's own width + space.
    """
    added = {}
    for net, shapes in pins.items():
        cuts = []
        for cl, v in vias.items():
            los = [r for l, r in shapes if l == v['lo']]
            his = [r for l, r in shapes if l == v['hi']]
            if not (los and his and v['wx'] and v['wy']):
                continue
            encx = enc0 = 0
            for metal, enc in ((v['lo'], v['enca_l']), (v['hi'], v['enca_h'])):
                if layers.get(metal, {}).get('dir') == 'V':
                    encx = max(encx, enc)
                else:
                    enc0 = max(enc0, enc)
            ency = enc0
            for a in los:
                for b in his:
                    x0, y0 = max(a[0], b[0]) + encx, max(a[1], b[1]) + ency
                    x1, y1 = min(a[2], b[2]) - encx, min(a[3], b[3]) - ency
                    if x1 - x0 < v['wx'] or y1 - y0 < v['wy']:
                        continue
                    nx = max(1, (x1 - x0 + v['sx']) // (v['wx'] + v['sx']))
                    ny = max(1, (y1 - y0 + v['sy']) // (v['wy'] + v['sy']))
                    spanx = nx * v['wx'] + (nx - 1) * v['sx']
                    spany = ny * v['wy'] + (ny - 1) * v['sy']
                    ox = x0 + (x1 - x0 - spanx) // 2
                    oy = y0 + (y1 - y0 - spany) // 2
                    for i in range(nx):
                        for j in range(ny):
                            cx = ox + i * (v['wx'] + v['sx'])
                            cy = oy + j * (v['wy'] + v['sy'])
                            cuts.append((cl, (cx, cy, cx + v['wx'], cy + v['wy'])))
        if cuts:
            shapes.extend(cuts)
            added[net] = len(cuts)
    return added


def write_ndr(path, module, nets, driver_inst, widths):
    """Route power as a star out of the grid, not a chain through the cells."""
    entry = {'module': module, 'clock_nets': [
        {'name': n, 'driver': f'{driver_inst}/{n}'} for n in nets]}
    if widths:
        entry['nets'] = [{'name': n, 'widths': dict(widths), 'spaces': dict(widths)}
                         for n in nets]
    with open(path, 'w') as f:
        json.dump([entry], f, indent=1)


def write_lef(path, name, bbox, pins, units=1000):
    """pins: {net: [(layer, rect), ...]}"""
    x0, y0, x1, y1 = bbox
    with open(path, 'w') as f:
        f.write(f'# power grid generated from the layer abstraction\n')
        f.write(f'MACRO {name}\n')
        f.write('  UNITS\n')
        f.write(f'    DATABASE MICRONS UNITS {units};\n')
        f.write('  END UNITS\n')
        f.write('  ORIGIN 0 0 ;\n')
        f.write(f'  FOREIGN {name} 0 0 ;\n')
        f.write(f'  SIZE {x1 - x0} BY {y1 - y0} ;\n')
        for net, shapes in pins.items():
            f.write(f'  PIN {net}\n')
            f.write('    DIRECTION INOUT ;\n')
            f.write('    USE POWER ;\n')
            f.write('    PORT\n')
            for layer, r in shapes:
                f.write(f'      LAYER {layer} ;\n')
                f.write(f'        RECT {r[0]} {r[1]} {r[2]} {r[3]} ;\n')
            f.write('    END\n')
            f.write(f'  END {net}\n')
        f.write(f'END {name}\n')


def write_placement(path, src, name, bbox, pins):
    """Copy the placement and instantiate the grid over the top block."""
    d = json.load(open(src))
    terminals = [{'name': net, 'rect': list(shapes[0][1])}
                 for net, shapes in pins.items() if shapes]
    d.setdefault('leaves', []).append({
        'abstract_name': name, 'concrete_name': name,
        'bbox': list(bbox), 'terminals': terminals,
    })
    top = d['modules'][0]
    top.setdefault('instances', []).append({
        'abstract_template_name': name, 'concrete_template_name': name,
        'instance_name': 'X_PGRID',
        'fa_map': [{'actual': net, 'formal': net} for net in pins],
        'transformation': {'oX': bbox[0], 'oY': bbox[1], 'sX': 1, 'sY': 1},
    })
    with open(path, 'w') as f:
        json.dump(d, f, indent=1)


def draw_gds(ingds, outgds, pins, layers):
    import gdspy
    lib = gdspy.GdsLibrary()
    lib.read_gds(ingds)
    tops = lib.top_level()
    if not tops:
        sys.exit(f'{ingds} has no top-level cell')
    cell = tops[0]
    for net, shapes in pins.items():
        for layer, r in shapes:
            info = layers[layer]
            cell.add(gdspy.Rectangle((r[0], r[1]), (r[2], r[3]),
                                     layer=info['gds'], datatype=info['dt']))
    lib.write_gds(outgds)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-l', '--layers', required=True, help='layers.json')
    ap.add_argument('-p', '--placement', default='',
                    help='placement json; its top block gives the grid extent')
    ap.add_argument('--bbox', default='', help='x0,y0,x1,y1 instead of -p')
    ap.add_argument('--nets', default='',
                    help='power nets, in strap order; default: the placement\'s '
                         'own global_power signals, else VDD,VSS')
    ap.add_argument('--bottom', default='', help='lowest grid layer (default from layers.json)')
    ap.add_argument('--top', default='', help='highest grid layer (default from layers.json)')
    ap.add_argument('--stride', type=int, default=1,
                    help='strap every Nth track of the layer pitch (default 1)')
    ap.add_argument('--widen', type=float, default=1.0,
                    help='strap width as a multiple of the layer width (default 1)')
    ap.add_argument('--name', default='PGRID', help='macro name (default PGRID)')
    ap.add_argument('--lef', default='', help='write the grid as a LEF macro')
    ap.add_argument('--placement-out', default='',
                    help='write a placement that instantiates the grid')
    ap.add_argument('--avoid', default='',
                    help='LEF of the placed cells; straps are cut back around '
                         'metal already on the grid layers')
    ap.add_argument('--no-stitch', action='store_true',
                    help='leave strap crossings untied')
    ap.add_argument('--ndr-out', default='',
                    help='write an NDR that taps every power pin to the grid')
    ap.add_argument('--ndr-width', type=int, default=0,
                    help='with --ndr-out, route the power nets at this width')
    ap.add_argument('-g', '--gds', default='', help='draw the grid into this layout')
    ap.add_argument('-o', '--out', default='', help='output GDS for -g')
    a = ap.parse_args()

    js = json.load(open(a.layers))
    layers = layer_table(js)
    nets = [n.strip() for n in a.nets.split(',') if n.strip()]
    if not nets and a.placement:
        # the design names its own supplies; a grid on nets it does not have is
        # just metal in the way, and shorts against everything it crosses
        pl = json.load(open(a.placement))
        nets = [g['actual'] for g in pl.get('global_signals', [])
                if 'power' in g.get('prefix', '') and g.get('actual')]
    if not nets:
        nets = ['VDD', 'VSS']

    if a.bbox:
        bbox = tuple(int(v) for v in a.bbox.split(','))
    elif a.placement:
        bbox = tuple(json.load(open(a.placement))['modules'][0]['bbox'])
    else:
        sys.exit('need -p or --bbox for the grid extent')

    gl = grid_layers(js, layers, a.bottom or None, a.top or None)
    pins = {n: [] for n in nets}
    for name in gl:
        for net, r in straps(bbox, layers[name], nets, a.stride, a.widen):
            pins[net].append((name, r))
    if a.avoid:
        blockers = placed_shapes(a.placement, lef_macros(a.avoid), set(gl))
        keep = min(layers[l]['pitch'] or 0 for l in gl) * 2
        trimmed, dropped = carve(pins, blockers, layers, keep)
        print(f'avoiding    : {len(blockers)} existing shape(s) on {", ".join(gl)}'
              f' -> {trimmed} strap(s) cut, {dropped} fully removed')
    stitched = {} if a.no_stitch else stitch(pins, via_layers(js, layers), layers)

    total = sum(len(v) for v in pins.values())
    print(f'grid layers : {", ".join(gl)}')
    print(f'extent      : {bbox}')
    for net in nets:
        per = {}
        for layer, _ in pins[net]:
            per[layer] = per.get(layer, 0) + 1
        print(f'  {net:6} {len(pins[net]):4} shape(s)  ' +
              ' '.join(f'{k}:{v}' for k, v in per.items()) +
              (f'  ({stitched[net]} stitch cut(s))' if net in stitched else ''))
    if not total:
        sys.exit('no straps fit in the given extent')

    if a.lef:
        write_lef(a.lef, a.name, bbox, pins)
        print(f'wrote {a.lef}')
    if a.placement_out:
        if not a.placement:
            sys.exit('--placement-out needs -p')
        write_placement(a.placement_out, a.placement, a.name, bbox, pins)
        print(f'wrote {a.placement_out}')
    if a.ndr_out:
        if not a.placement:
            sys.exit('--ndr-out needs -p')
        top = json.load(open(a.placement))['modules'][0]['concrete_name']
        w = {l: a.ndr_width for l in gl} if a.ndr_width else {}
        write_ndr(a.ndr_out, top, nets, 'X_' + a.name, w)
        print(f'wrote {a.ndr_out}')
    if a.gds:
        if not os.path.isfile(a.gds):
            sys.exit(f'{a.gds} not found')
        draw_gds(a.gds, a.out or 'pgrid.gds', pins, layers)
        print(f'wrote {a.out or "pgrid.gds"}')


if __name__ == '__main__':
    main()
