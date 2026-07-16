#!/usr/bin/env python3

import re
import json
import argparse
import os
from collections import defaultdict

import gdstk

ap = argparse.ArgumentParser()
ap.add_argument("-g", "--gds",     required=True, help="Input GDS file")
ap.add_argument("-l", "--layers",  required=True, help="layers.json")
ap.add_argument("-n", "--netlist", default="",    help="Optional SCS netlist")
ap.add_argument("-o", "--out",     default="out.placement_verilog.json",
                help="Output JSON file")
args = ap.parse_args()

rev_map      = {}
boundary_key = (101, 0)

with open(args.layers) as fp:
    ldata = json.load(fp)

for entry in ldata.get("Abstraction", []):
    name  = entry.get("Layer")
    gno   = entry.get("GdsLayerNo")
    dtmap = entry.get("GdsDatatype", {})
    if name is None or gno is None:
        continue
    for purpose, dt in dtmap.items():
        rev_map[(gno, dt)] = (name, purpose.lower())
    if name == "Boundary":
        boundary_key = (gno, dtmap.get("Draw", 0))

_CONC_RE = re.compile(r'_CONC_\d+$')

def abstract_name(concrete):
    return _CONC_RE.sub('', concrete)

def _n(v):
    return int(v) if v == int(v) else v

def parse_scs(path):
    subckts    = {}
    cur_name   = None
    cur_sub    = None

    with open(path) as fp:
        for raw in fp:
            line = raw.strip()
            if not line or line.startswith('//') or line.startswith('*'):
                continue
            tokens = line.split()
            if not tokens:
                continue

            if tokens[0] == 'subckt' and len(tokens) >= 2:
                cur_name = tokens[1]
                ports = [t.strip('()') for t in tokens[2:]
                         if '=' not in t and t.strip('()')]
                cur_sub = {"ports": ports, "instances": []}
                subckts[cur_name] = cur_sub

            elif tokens[0] == 'ends':
                cur_name = None
                cur_sub  = None

            elif cur_sub is not None:
                inst_name = tokens[0]
                m = re.match(r'\(([^)]*)\)\s+(\S+)',
                             line[len(inst_name):].strip())
                if m:
                    actuals   = m.group(1).split()
                    cell_type = m.group(2)
                    cur_sub["instances"].append((inst_name, cell_type, actuals))

    return subckts


subckts = {}
if args.netlist:
    subckts = parse_scs(args.netlist)
    print(f"loaded netlist: {len(subckts)} subckts ({', '.join(sorted(subckts))})")

lib          = gdstk.read_gds(args.gds)
cell_by_name = {c.name: c for c in lib.cells}
leaf_names   = {c.name for c in lib.cells if not c.references}
module_names = {c.name for c in lib.cells if c.references}

_bbox_cache = {}

def flat_bbox(cell):
    if cell.name in _bbox_cache:
        return _bbox_cache[cell.name]

    pts = []
    for poly in cell.polygons:
        bb = poly.bounding_box()
        pts.extend([bb[0], bb[1]])
    for ref in cell.references:
        sub = flat_bbox(ref.cell)
        if sub is not None:
            ox, oy = ref.origin
            pts.append((sub[0] + ox, sub[1] + oy))
            pts.append((sub[2] + ox, sub[3] + oy))

    result = None
    if pts:
        result = (min(x for x, _ in pts), min(y for _, y in pts),
                  max(x for x, _ in pts), max(y for _, y in pts))
    _bbox_cache[cell.name] = result
    return result


def cell_bbox(cell):
    """Boundary polygon first; fall back to flat recursive bbox."""
    for poly in cell.polygons:
        if (poly.layer, poly.datatype) == boundary_key:
            bb = poly.bounding_box()
            x0, y0 = bb[0]; x1, y1 = bb[1]
            return [_n(x0), _n(y0), _n(x1), _n(y1)]
    bb = flat_bbox(cell)
    if bb:
        return [_n(bb[0]), _n(bb[1]), _n(bb[2]), _n(bb[3])]
    return [0, 0, 0, 0]

def leaf_terminals(cell):
    pin_rects  = defaultdict(list)
    draw_rects = defaultdict(list)
    labels     = []

    for poly in cell.polygons:
        key = (poly.layer, poly.datatype)
        if key == boundary_key or key not in rev_map:
            continue
        lname, purpose = rev_map[key]
        bb   = poly.bounding_box()
        rect = (bb[0][0], bb[0][1], bb[1][0], bb[1][1])
        if purpose == "pin":
            pin_rects[lname].append(rect)
        elif purpose == "draw":
            draw_rects[lname].append(rect)

    for lbl in cell.labels:
        key = (lbl.layer, lbl.texttype)
        if key in rev_map:
            lname, purpose = rev_map[key]
            if purpose == "label":
                lx, ly = lbl.origin
                labels.append((lbl.text, lx, ly, lname))

    pin_shapes = defaultdict(lambda: defaultdict(list))
    assigned   = set()

    for (text, lx, ly, llayer) in labels:
        candidates = pin_rects.get(llayer) or draw_rects.get(llayer, [])
        for rect in candidates:
            x0, y0, x1, y1 = rect
            if x0 <= lx <= x1 and y0 <= ly <= y1:
                k = (llayer, rect)
                if k not in assigned:
                    pin_shapes[text][llayer].append(rect)
                    assigned.add(k)

    terminals = []
    for pin_name in sorted(pin_shapes):
        rects = [r for rs in pin_shapes[pin_name].values() for r in rs]
        if rects:
            x0, y0, x1, y1 = rects[0]
            terminals.append({"name": pin_name,
                               "rect": [_n(x0), _n(y0), _n(x1), _n(y1)]})
    return terminals

leaves_out = []
for name in sorted(leaf_names):
    c = cell_by_name[name]
    leaves_out.append({
        "abstract_name": abstract_name(name),
        "bbox":          cell_bbox(c),
        "concrete_name": name,
        "terminals":     leaf_terminals(c),
    })

modules_out = []
for name in sorted(module_names):
    cell    = cell_by_name[name]
    ab_name = abstract_name(name)
    scs_sub = subckts.get(ab_name)

    scs_insts = scs_sub["instances"] if scs_sub else []

    instances = []
    for idx, ref in enumerate(cell.references):
        sub_name = ref.cell.name
        ox, oy   = ref.origin

        if idx < len(scs_insts):
            inst_name, cell_type_abs, actuals = scs_insts[idx]
            ref_scs  = subckts.get(abstract_name(sub_name)) or subckts.get(cell_type_abs)
            formals  = ref_scs["ports"] if ref_scs else []
            fa_map   = [{"actual": a, "formal": f}
                        for f, a in zip(formals, actuals)]
        else:
            inst_name = f"I_{idx}"
            fa_map    = []

        instances.append({
            "abstract_template_name": abstract_name(sub_name),
            "concrete_template_name": sub_name,
            "fa_map":                 fa_map,
            "instance_name":          inst_name,
            "transformation":         {"oX": _n(ox), "oY": _n(oy),
                                       "sX": 1, "sY": 1},
        })

    parameters = scs_sub["ports"] if scs_sub else []

    modules_out.append({
        "abstract_name":  ab_name,
        "bbox":           cell_bbox(cell),
        "concrete_name":  name,
        "instances":      instances,
        "parameters":     parameters,
    })

result = {
    "global_signals": [],
    "leaves":         leaves_out,
    "modules":        modules_out,
}

with open(args.out, "w") as fp:
    json.dump(result, fp, indent=2)
    fp.write("\n")

print(f"wrote {args.out}")
print(f"  leaves:  {len(leaves_out)}")
print(f"  modules: {len(modules_out)}")
for m in modules_out:
    print(f"    {m['concrete_name']}: bbox={m['bbox']}  "
          f"{len(m['instances'])} instances  params={m['parameters']}")
