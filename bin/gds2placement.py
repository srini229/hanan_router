#!/usr/bin/env python3

import re
import sys
import json
import argparse
import os
from collections import defaultdict

import gdstk

ap = argparse.ArgumentParser()
ap.add_argument("-g", "--gds",     required=True, help="Routed _out.gds file")
ap.add_argument("-l", "--layers",  required=True, help="layers.json")
ap.add_argument("-o", "--out",     default="out.placement_verilog.json",
                help="Output JSON file")
args = ap.parse_args()

rev_map = {}
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
    """Strip _CONC_N suffix to get the abstract (template) name."""
    return _CONC_RE.sub('', concrete)

def leaf_terminals(cell):
    """Return list of {"name": str, "rect": [x0,y0,x1,y1]} from a GDS leaf cell."""
    pin_rects   = defaultdict(list)
    draw_rects  = defaultdict(list)
    labels      = []

    for poly in cell.polygons:
        key = (poly.layer, poly.datatype)
        if key == boundary_key or key not in rev_map:
            continue
        lname, purpose = rev_map[key]
        bb = poly.bounding_box()
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

    pin_shapes  = defaultdict(lambda: defaultdict(list))
    assigned    = set()

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
        all_rects = [r for rects in pin_shapes[pin_name].values() for r in rects]
        if all_rects:
            x0, y0, x1, y1 = all_rects[0]
            terminals.append({"name": pin_name,
                               "rect": [_n(x0), _n(y0), _n(x1), _n(y1)]})
    return terminals

def _n(v):
    """Return int if lossless, else float."""
    return int(v) if v == int(v) else v

def parse_def(path):
    """
    Returns:
        diearea   : [x0, y0, x1, y1]
        comps     : {inst_name: {"cell": cell_name, "x": x, "y": y}}
        nets      : {net_name:  [(inst_name, port_name), ...]}
    """
    diearea = None
    comps   = {}
    nets    = {}

    state   = None
    cur_net = None

    with open(path) as fp:
        for raw in fp:
            line   = raw.strip()
            tokens = line.split()
            if not tokens:
                continue

            if tokens[0] == "DIEAREA":
                nums = [float(t.rstrip(';')) for t in tokens
                        if t not in ("DIEAREA", "(", ")", ";") and t.rstrip(';')]
                if len(nums) >= 4:
                    diearea = [_n(v) for v in nums[:4]]

            elif tokens[0] == "COMPONENTS":
                state = "comps"
            elif tokens[0] == "END" and len(tokens) > 1 and tokens[1] == "COMPONENTS":
                state = None

            elif state == "comps" and tokens[0] == "-":
                inst_name = tokens[1]
                cell_name = tokens[2]
                x = y = 0
                for i, t in enumerate(tokens):
                    if t == "PLACED" and i + 3 < len(tokens):
                        try:
                            x = int(tokens[i + 2])
                            y = int(tokens[i + 3])
                        except ValueError:
                            pass
                        break
                comps[inst_name] = {"cell": cell_name, "x": x, "y": y}

            elif tokens[0] == "NETS":
                state = "nets"
                cur_net = None
            elif tokens[0] == "END" and len(tokens) > 1 and tokens[1] == "NETS":
                state = None
                cur_net = None

            elif state == "nets":
                if tokens[0] == "-" and len(tokens) >= 2 and tokens[1] != ";":
                    cur_net = tokens[1].rstrip(";")
                    nets.setdefault(cur_net, [])
                elif tokens[0] == ";":
                    cur_net = None
                elif cur_net and tokens[0] not in ("+", "RECT", "VIA", "PROPERTY"):
                    pairs = re.findall(r'\(\s*(\S+)\s+(\S+)\s*\)', line)
                    for inst, port in pairs:
                        nets[cur_net].append((inst, port))

    return diearea or [0, 0, 0, 0], comps, nets

lib = gdstk.read_gds(args.gds)

cell_by_name = {c.name: c for c in lib.cells}
leaf_names   = {c.name for c in lib.cells if not c.references}
module_names = {c.name for c in lib.cells if c.references}

def cell_bbox_from_gds(cell):
    for poly in cell.polygons:
        if (poly.layer, poly.datatype) == boundary_key:
            bb = poly.bounding_box()
            x0, y0 = bb[0]; x1, y1 = bb[1]
            return [_n(x0), _n(y0), _n(x1), _n(y1)]
    pts = [pt for poly in cell.polygons for pt in poly.bounding_box()]
    if not pts:
        return [0, 0, 0, 0]
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    return [_n(min(xs)), _n(min(ys)), _n(max(xs)), _n(max(ys))]

leaves_out = []
for name in sorted(leaf_names):
    cell = cell_by_name[name]
    leaves_out.append({
        "abstract_name":  abstract_name(name),
        "bbox":           cell_bbox_from_gds(cell),
        "concrete_name":  name,
        "terminals":      leaf_terminals(cell),
    })

modules_out = []

for name in sorted(module_names):
    cell = cell_by_name[name]

    bbox  = cell_bbox_from_gds(cell)
    instances  = []
    parameters = []

    for idx, ref in enumerate(cell.references):
        sub_name = ref.cell.name
        ox, oy   = ref.origin
        instances.append({
            "abstract_template_name":  abstract_name(sub_name),
            "concrete_template_name":  sub_name,
            "fa_map":                  [],
            "instance_name":           f"I_{idx}",
            "transformation":          {"oX": _n(ox), "oY": _n(oy),
                                        "sX": 1, "sY": 1},
        })

    modules_out.append({
        "abstract_name":  abstract_name(name),
        "bbox":           bbox,
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
    print(f"    {m['concrete_name']}: {len(m['instances'])} instances, "
          f"{len(m['parameters'])} params")
