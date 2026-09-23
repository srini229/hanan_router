#!/usr/bin/env python3
"""Route signals, then build the power grid around them, then tap the supplies.

The order matters and is not the obvious one. Building the grid first looks
right, but signal routing itself creates metal on the grid layers: a net routed
up to M4 becomes a block pin there, and a strap laid earlier is then shorted by
geometry that did not exist when the strap was placed. ALIGN's own flow routes,
then creates the grid, then routes power, and the same order is used here.

    pass 1  supplies excluded            -> signal routing
    grid    carved around that routing   -> straps + stitch cuts
    pass 2  signals excluded, pass-1 metal as obstacles -> supply taps

Each pass is an ordinary hanan_router invocation; the only new inputs are the
generated NDR files that say which nets to skip and where the committed metal is.
"""
import argparse
import json
import os
import shutil
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GEN = os.path.join(HERE, 'gen_pwr_grid.py')
SIG_LAYERS = re.compile(r'^M\d+$')


def supplies(placement):
    d = json.load(open(placement))
    return [g['actual'] for g in d.get('global_signals', [])
            if 'power' in g.get('prefix', '') and g.get('actual')]


def top_module(placement):
    return json.load(open(placement))['modules'][0]['concrete_name']


def module_nets(placement):
    """{module: [nets]} for every module, not just the top: a sub-block left
    routable in pass 2 is re-routed from scratch and undoes pass 1."""
    d = json.load(open(placement))
    glob = {g['actual'] for g in d.get('global_signals', []) if g.get('actual')}
    nets, kids = {}, {}
    for m in d['modules']:
        nets[m['concrete_name']] = {fa['actual'] for i in m.get('instances', [])
                                    for fa in i.get('fa_map', []) if fa.get('actual')}
        kids[m['concrete_name']] = {i['concrete_template_name'] for i in m.get('instances', [])}
    # a supply a sub-module uses reaches its parent too, though no fa_map says so
    grew = True
    while grew:
        grew = False
        for m, ks in kids.items():
            add = {g for k in ks if k in nets for g in nets[k] & glob} - nets[m]
            if add:
                nets[m] |= add
                grew = True
    return {m: sorted(n) for m, n in nets.items()}


def def_shapes(path, skip=()):
    """Routed rectangles per layer out of a DEF's NETS section, in microns.

    The router takes NDR obstacle coordinates in microns and scales them itself;
    a DEF states them in database units. Handing the DEF's own numbers over puts
    every obstacle a factor of DBU outside the block, where it blocks nothing and
    the pass reports no short because as far as it knows there is none.
    """
    out = {}
    innets, net = False, None
    dbu = 1000.0
    for line in open(path, errors='ignore'):
        if innets:
            m = re.match(r'\s*- (\S+)', line)
            if m:
                net = m.group(1)
        if not innets:
            m = re.match(r'\s*UNITS DISTANCE MICRONS (\d+)', line)
            if m:
                dbu = float(m.group(1))
        if line.startswith('NETS'):
            innets = True
            continue
        if line.startswith('END NETS'):
            break
        if not innets:
            continue
        m = re.match(r'\s*\+ RECT (\w+) \( (-?\d+) (-?\d+) \) \( (-?\d+) (-?\d+) \)', line)
        if m and SIG_LAYERS.match(m.group(1)) and net not in skip:
            out.setdefault(m.group(1), []).append([int(x) / dbu for x in m.groups()[1:]])
    return out


def def_net_rects(path):
    """{net: ['  + RECT ...']} -- the wire lines each net carries in a DEF."""
    out, innets, net = {}, False, None
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
        elif net and re.match(r'\s*\+ RECT ', line):
            out.setdefault(net, []).append(line)
    return out


def lef_pin_shapes(path):
    """Pin rectangles per layer out of a LEF, in microns."""
    out, layer, units = {}, None, 1000.0
    for line in open(path, errors='ignore'):
        t = line.split()
        if len(t) >= 4 and t[:3] == ['DATABASE', 'MICRONS', 'UNITS']:
            units = float(t[3].rstrip(';'))
        elif t[:1] == ['LAYER']:
            layer = t[1]
        elif t[:1] == ['RECT'] and layer and SIG_LAYERS.match(layer):
            out.setdefault(layer, []).append([float(v) / units for v in t[1:5]])
    return out


def module_frames(placement):
    """{module: [(transform, bbox)]}: every placed copy of each module, in top coordinates, microns."""
    d = json.load(open(placement))
    mods = {m['concrete_name']: m for m in d['modules']}
    out = {}

    def walk(name, t):
        for i in mods[name].get('instances', []):
            c = i['concrete_template_name']
            if c not in mods:
                continue
            s = i['transformation']
            tt = (t[0] + t[2] * s['oX'] / 1000.0, t[1] + t[3] * s['oY'] / 1000.0,
                  t[2] * s['sX'], t[3] * s['sY'])
            out.setdefault(c, []).append((tt, [v / 1000.0 for v in mods[c]['bbox']]))
            walk(c, tt)

    walk(d['modules'][0]['concrete_name'], (0.0, 0.0, 1, 1))
    return out


def into_frame(shapes, frames):
    """Top-level shapes mapped into a module's own frame, kept where they overlap it."""
    out = {}
    for (ox, oy, sx, sy), (bx0, by0, bx1, by1) in frames:
        for layer, rects in shapes.items():
            for x0, y0, x1, y1 in rects:
                a, b = sorted((sx * (x0 - ox), sx * (x1 - ox)))
                c, e = sorted((sy * (y0 - oy), sy * (y1 - oy)))
                if a < bx1 and b > bx0 and c < by1 and e > by0:
                    out.setdefault(layer, []).append([a, c, b, e])
    return out


def out_of_frame(shapes, frames):
    """A module's own shapes placed into top coordinates, once per copy."""
    out = {}
    for (ox, oy, sx, sy), _ in frames:
        for layer, rects in shapes.items():
            for x0, y0, x1, y1 in rects:
                a, b = sorted((ox + sx * x0, ox + sx * x1))
                c, e = sorted((oy + sy * y0, oy + sy * y1))
                out.setdefault(layer, []).append([a, c, b, e])
    return out


def merge_shapes(*shape_sets):
    out = {}
    for s in shape_sets:
        for layer, rects in (s or {}).items():
            out.setdefault(layer, []).extend(rects)
    return out


def write_ndr(path, entries):
    """entries: [(module, skip_nets, obstacles, drivers)]"""
    out = []
    for module, skip, obstacles, drivers in entries:
        e = {'module': module}
        if skip:
            # module-level list; the per-net "do_not_route": 1 form the README
            # shows is not implemented and is silently ignored
            e['do_not_route'] = list(skip)
        if obstacles:
            e['obstacles'] = [{'shapes': obstacles}]
        if drivers:
            e['clock_nets'] = [{'name': n, 'driver': d} for n, d in drivers.items()]
        out.append(e)
    with open(path, 'w') as f:
        json.dump(out, f, indent=1)


def merge_def(dfl, sigdef, sup, cuts):
    """Rewrite dfl as pass 1's DEF (sigdef) with each supply net's wires
    replaced by what pass 2 routed into dfl, plus any stitch cuts."""
    pwr = def_net_rects(dfl)
    lines = open(sigdef).read().splitlines(True)
    out, net, n, sw = [], None, 0, 0

    def flush():
        # inside the net block, ahead of its ';': a DEF reader that stops at the
        # terminator drops anything written past it
        nonlocal n, sw
        for line in pwr.get(net, []) if net in sup else []:
            out.append(line)
            sw += 1
        for c in cuts.get(net, []):
            out.append(f'  + RECT {c[0]} ( {c[1]} {c[2]} ) ( {c[3]} {c[4]} )\n')
            n += 1

    for line in lines:
        m = re.match(r'\s*- (\S+)', line)
        if m:
            net = m.group(1)
        elif net and line.strip() == ';':
            flush()
            net = None
        elif line.startswith('END NETS'):
            flush()
            net = None
        elif net in sup and re.match(r'\s*\+ RECT ', line):
            continue          # pass 1 only stubbed the supplies out
        out.append(line)
    open(dfl, 'w').writelines(out)
    return sw, n


def run(cmd, cwd, log):
    with open(os.path.join(cwd, log), 'w') as f:
        return subprocess.call(cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT)


def summarise(logpath, label):
    nets = opens = wl = shorts = 0
    drc = None
    for line in open(logpath, errors='ignore'):
        m = re.search(r'ROUTE_SUMMARY module=\S+ nets=(\d+) unrouted=(\d+)', line)
        if m:
            nets += int(m.group(1))
            opens += int(m.group(2))
        m = re.match(r'WIRELENGTH TOTAL \S+ : (\d+)', line)
        if m:
            wl += int(m.group(1))
        m = re.search(r'router-caused spacing violations = (\d+)', line)
        if m:
            drc = int(m.group(1))
        if re.search(r'SHORT.*between', line):
            shorts += 1
    print(f'  {label:16} nets={nets:<4} opens={opens:<3} wl={wl:<10} '
          f'shorts={shorts:<3} drc={drc}', flush=True)
    return dict(nets=nets, opens=opens, wl=wl, shorts=shorts, drc=drc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-d', '--layers', required=True)
    ap.add_argument('-p', '--placement', required=True)
    ap.add_argument('-l', '--lef', required=True, help='cell LEF')
    ap.add_argument('-o', '--outdir', default='.')
    ap.add_argument('--router', default=os.path.join(os.path.dirname(HERE), 'hanan_router'))
    ap.add_argument('--bottom', default='', help='lowest grid layer')
    ap.add_argument('--top', default='', help='highest grid layer')
    ap.add_argument('--stride', default='1',
                    help='strap every Nth track; a comma list gives one value per '
                         'grid layer, bottom-up (7,8 matches ALIGN on sky130)')
    ap.add_argument('--args', default='-uu 1000 -escapepitch 8 -threads 1',
                    help='switches passed to every router pass')
    a = ap.parse_args()

    d = os.path.abspath(a.outdir)
    os.makedirs(d, exist_ok=True)
    layers = os.path.abspath(a.layers)
    place = os.path.abspath(a.placement)
    lef = os.path.abspath(a.lef)
    extra = a.args.split()
    top = top_module(place)
    sup = supplies(place)
    if not sup:
        sys.exit('placement declares no global power signals')
    mods = module_nets(place)

    # --- pass 1: signals only, supplies held out as obstacles
    write_ndr(os.path.join(d, 'ndr_sig.json'),
              [(m, [n for n in nets if n in sup], None, None)
               for m, nets in mods.items()])
    rc = run([a.router, '-d', layers, '-p', place, '-l', lef,
              '-ndr', 'ndr_sig.json', '-o', './', '-log', 'sig.log'] + extra,
             d, 'sig.out')
    print(f'pass 1 (signals) rc={rc}', flush=True)
    r1 = summarise(os.path.join(d, 'sig.log'), 'signals')
    # keep each module's pass-1 DEF: pass 2 rewrites them all
    shutil.copy(os.path.join(d, f'{top}.def'), os.path.join(d, 'sig.def'))
    for m in mods:
        if m != top and os.path.exists(os.path.join(d, f'{m}.def')):
            shutil.copy(os.path.join(d, f'{m}.def'), os.path.join(d, f'sig_{m}.def'))

    # --- grid, carved around what pass 1 committed at every level
    routed = def_shapes(os.path.join(d, f'{top}.def'))
    frames = module_frames(place)
    committed = merge_shapes(routed, *(
        out_of_frame(def_shapes(os.path.join(d, f'sig_{m}.def')), frames.get(m, []))
        for m in mods if m != top and os.path.exists(os.path.join(d, f'sig_{m}.def'))))
    with open(os.path.join(d, 'committed.def'), 'w') as f:
        f.write('NETS 1 ;\n- committed\n')
        for layer, rects in committed.items():
            for r in rects:
                v = [round(x * 1000) for x in r]
                f.write(f'  + RECT {layer} ( {v[0]} {v[1]} ) ( {v[2]} {v[3]} )\n')
        f.write(';\nEND NETS\n')
    gen = [sys.executable, GEN, '-l', layers, '-p', place,
           '--avoid', lef, '--avoid-def', os.path.join(d, 'committed.def'),
           '--stride', str(a.stride), '--cuts-out', 'pgrid_cuts.json',
           '--lef', 'pgrid.lef', '--placement-out', 'pg_place.json']
    if a.bottom:
        gen += ['--bottom', a.bottom]
    if a.top:
        gen += ['--top', a.top]
    rc = run(gen, d, 'gen.log')
    print(f'grid rc={rc}: ' + open(os.path.join(d, 'gen.log')).read().strip().replace('\n', ' | '),
          flush=True)
    if rc != 0:
        sys.exit('grid generation failed')

    # --- pass 2: supplies only, pass-1 metal as blockage, grid as the driver
    with open(os.path.join(d, 'all.lef'), 'w') as f:
        f.write(open(lef).read())
        f.write(open(os.path.join(d, 'pgrid.lef')).read())
    # pass 2 re-routes each module alone: give it all pass-1 metal over it, and the straps
    straps = lef_pin_shapes(os.path.join(d, 'pgrid.lef'))
    # pass 1 leaves stubs on the supply pins: those are what pass 2 must reach, not obstacles
    committed = merge_shapes(def_shapes(os.path.join(d, f'{top}.def'), sup), *(
        out_of_frame(def_shapes(os.path.join(d, f'sig_{m}.def'), sup), frames.get(m, []))
        for m in mods if m != top and os.path.exists(os.path.join(d, f'sig_{m}.def'))))
    obs = {m: committed if m == top else into_frame(merge_shapes(committed, straps), frames.get(m, []))
           for m in mods}
    write_ndr(os.path.join(d, 'ndr_pwr.json'),
              [(m, [n for n in nets if n not in sup],
                obs[m] or None,
                {n: f'X_PGRID/{n}' for n in nets if n in sup} if m == top else None)
               for m, nets in mods.items()])
    rc = run([a.router, '-d', layers, '-p', 'pg_place.json', '-l', 'all.lef',
              '-ndr', 'ndr_pwr.json', '-o', './', '-log', 'pwr.log'] + extra,
             d, 'pwr.out')
    print(f'pass 2 (supplies) rc={rc}', flush=True)
    r2 = summarise(os.path.join(d, 'pwr.log'), 'supplies')

    # --- rebuild the routed DEF: signals as pass 1 left them, supplies as
    #     pass 2 routed them, and the grid's own stitch cuts merged in
    cj = os.path.join(d, 'pgrid_cuts.json')
    cuts = json.load(open(cj)) if os.path.exists(cj) else {}
    for m in mods:
        sig = os.path.join(d, 'sig.def' if m == top else f'sig_{m}.def')
        if not os.path.exists(sig):
            continue
        # the grid and its stitch cuts exist only at the top
        sw, n = merge_def(os.path.join(d, f'{m}.def'), sig, sup,
                          cuts if m == top else {})
        print(f'  {"def merged":16} {sw} supply wire(s), {n} stitch cut(s) '
              f'onto pass 1 -> {m}.def', flush=True)

    print(f'  {"TOTAL":16} opens={r1["opens"] + r2["opens"]:<3} '
          f'wl={r1["wl"] + r2["wl"]:<10} shorts={r1["shorts"] + r2["shorts"]}', flush=True)


if __name__ == '__main__':
    main()
