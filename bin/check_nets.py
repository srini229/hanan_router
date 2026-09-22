#!/usr/bin/env python3
"""Trace a routed GDS and check it holds exactly the nets the netlist declares.

The router's own open count is reported against its internal solution. This
checks the artefact instead: flatten the GDS, join metal that touches and metal
a via cut bridges, and count the conductors that come out. A design the router
called fully routed must produce one conductor per top-level net -- any net in
more than one piece is an open the DEF or the GDS build introduced after the
router was done with it, which is how a two-pass power flow overwriting its own
DEF stayed invisible for as long as it did.

Two details decide whether the trace means anything:

  * one GDS layer can carry two abstractions -- sky130 draws the M4-M5 via and
    the MIM capacitor contact both on 70/44 -- so a via wins over a non-via and
    a requested layer over an unrequested one, otherwise the cuts silently
    vanish and every strap reads as floating.

  * a conductor is named by the DEF wires that land on it *on the same layer*.
    Matching geometry alone tags a conductor with every net that merely crosses
    over it.
"""
import argparse
import collections
import json
import re
import sys

import gdstk
from shapely.geometry import box
from shapely.strtree import STRtree


def layer_map(path, want=None):
    """{(gds layer, datatype): name}, and the via stack, from layers.json."""
    draw, via = {}, {}
    for e in json.load(open(path)).get('Abstraction', []):
        g, d = e.get('GdsLayerNo'), (e.get('GdsDatatype') or {}).get('Draw')
        if g is None or d is None:
            continue
        isvia = bool(e.get('Stack')) and len(e['Stack']) == 2 and all(e['Stack'])
        key, name = (g, d), e['Layer']
        if key in draw and not ((want is not None and name in want
                                 and draw[key] not in want)
                                or (isvia and key not in via)):
            continue
        draw[key] = name
        if isvia:
            via[key] = (e['Stack'][0], e['Stack'][1])
        elif key in via:
            del via[key]
    return draw, via


def def_nets(path, dbu=1000.0):
    """(net names, [(net, layer, box)]) out of a DEF's NETS section, in microns."""
    names, rects, innets, net = [], [], False, None
    for line in open(path, errors='ignore'):
        if line.startswith('NETS'):
            innets = True
            continue
        if line.startswith('END NETS'):
            break
        if not innets:
            continue
        m = re.match(r'\s*- (\S+)', line)
        if m:
            net = m.group(1)
            names.append(net)
            continue
        m = re.match(r'\s*\+ RECT (\w+) \( (-?\d+) (-?\d+) \) \( (-?\d+) (-?\d+) \)', line)
        if m:
            v = [int(x) / dbu for x in m.groups()[1:]]
            rects.append((net, m.group(1), box(v[0], v[1], v[2], v[3])))
    return names, rects


def trace(gds, top, layers, stack='', layer_source=None):
    """(shapes, DEF-net tags, union-find root) for the flattened cell's conductors."""
    want = {x.strip() for x in stack.split(',') if x.strip()} if stack else None
    draw, via = layer_source if layer_source is not None else layer_map(layers, want)
    if want:
        draw = {k: v for k, v in draw.items() if v in want}
        via = {k: v for k, v in via.items()
               if k in draw and v[0] in want and v[1] in want}

    lib = gdstk.read_gds(gds)
    cells = [c for c in lib.cells if c.name == top]
    if not cells:
        sys.exit(f'{top} not in {gds}')
    cell = cells[0].copy('flat')
    cell.flatten()
    k = lib.unit / 1e-6

    shapes, tags = [], []
    for p in cell.polygons:
        name = draw.get((p.layer, p.datatype))
        if not name:
            continue
        (x0, y0), (x1, y1) = p.bounding_box()
        g = box(x0 * k, y0 * k, x1 * k, y1 * k)
        if g.is_empty or g.area == 0:
            continue          # some writers emit degenerate polygons at the origin
        shapes.append(g)
        tags.append(name)

    parent = list(range(len(shapes)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(i, j):
        a, b = find(i), find(j)
        if a != b:
            parent[b] = a

    by = collections.defaultdict(list)
    for i, t in enumerate(tags):
        by[t].append(i)
    for idx in by.values():                      # same layer: touching is connected
        tree = STRtree([shapes[i] for i in idx])
        for i in idx:
            for m in tree.query(shapes[i]):
                if idx[m] != i and shapes[i].intersects(shapes[idx[m]]):
                    union(i, idx[m])
    for cut, (lo, hi) in ((draw[k2], v) for k2, v in via.items()):
        cuts = by.get(cut, [])
        for lname in (lo, hi):                   # a cut joins the metals it sits between
            idx = by.get(lname, [])
            if not idx or not cuts:
                continue
            tree = STRtree([shapes[i] for i in idx])
            for ci in cuts:
                for m in tree.query(shapes[ci]):
                    if shapes[ci].intersects(shapes[idx[m]]):
                        union(ci, idx[m])
    return shapes, tags, find


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('gds')
    ap.add_argument('--top', required=True)
    ap.add_argument('--layers', required=True, help='layers.json')
    ap.add_argument('--def', dest='deffile', required=True,
                    help='routed DEF naming the top-level nets')
    ap.add_argument('--stack', default='',
                    help='comma list of layers to trace; default is every drawn layer')
    ap.add_argument('--dbu', type=float, default=1000.0)
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()

    shapes, tags, find = trace(a.gds, a.top, a.layers, a.stack)
    names, rects = def_nets(a.deffile, a.dbu)

    tree = STRtree(shapes)
    owner = collections.defaultdict(collections.Counter)
    for net, layer, g in rects:
        for m in tree.query(g):
            if tags[m] == layer and g.intersects(shapes[m]):
                owner[find(m)][net] += 1

    comps = collections.defaultdict(list)
    for i in range(len(shapes)):
        comps[find(i)].append(i)
    pieces, rows, loose, shorts = collections.Counter(), [], 0, []
    for root, members in sorted(comps.items(), key=lambda kv: -len(kv[1])):
        got = owner.get(root) or collections.Counter()
        # every net whose wires land here, not just the commonest: two names on
        # one conductor is a short, and picking a winner would hide it
        for net in got:
            pieces[net] += 1
        if len(got) > 1:
            shorts.append((len(members), sorted(got)))
        elif not got:
            loose += 1
        b = [shapes[i].bounds for i in members]
        rows.append('  %5d shape(s)  %-14s %-44s bbox (%.2f %.2f) (%.2f %.2f)'
                    % (len(members), '+'.join(sorted(got)) or '(no DEF wire)',
                       dict(collections.Counter(tags[i] for i in members)),
                       min(x[0] for x in b), min(x[1] for x in b),
                       max(x[2] for x in b), max(x[3] for x in b)))

    split = {n: c for n, c in pieces.items() if c > 1}
    missing = [n for n in names if n not in pieces]
    if not a.quiet:
        print(f'{a.gds}: {len(shapes)} shape(s) -> {len(comps)} conductor(s), '
              f'{len(names)} net(s) in {a.deffile}')
        print('\n'.join(rows))
    for n, c in sorted(split.items()):
        print(f'OPEN  {n} is in {c} pieces')
    for n, nets in sorted(shorts):
        print(f'SHORT {" + ".join(nets)} share one conductor of {n} shape(s)')
    for n in missing:
        print(f'ABSENT {n} has no metal in the GDS')
    if loose:
        print(f'note  {loose} conductor(s) carry no DEF wire '
              f'(device-internal nodes do this legitimately)')
    if split or missing or shorts:
        print(f'FAIL {a.top}: {len(comps)} conductor(s) for {len(names)} net(s)')
        return 1
    print(f'OK {a.top}: {len(names)} net(s), one conductor each'
          + (f' (+{loose} device-internal)' if loose else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
