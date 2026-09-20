#!/usr/bin/env python3

import math
import re
import json
import argparse
import os
import sys
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

def ref_orient(ref):
    """(sX, sY) matching the router's Geom::Transform convention -- a pure
    axis-flip model, no arbitrary rotation -- from a gdstk Reference's
    rotation/x_reflection. Only 0/180 degree rotation is representable;
    anything else is warned about and treated as 0."""
    rot = (ref.rotation or 0.0) % (2 * math.pi)
    is180 = abs(rot - math.pi) < 1e-6
    if not is180 and abs(rot) > 1e-6:
        print(f"warning: reference to {ref.cell.name} has rotation "
              f"{math.degrees(ref.rotation):.3f} deg (only 0/180 are "
              f"representable); treating as 0", file=sys.stderr)
    mag = ref.magnification or 1.0
    if abs(mag - 1.0) > 1e-9:
        print(f"warning: reference to {ref.cell.name} has magnification "
              f"{mag}, ignored", file=sys.stderr)
    refl = bool(ref.x_reflection)
    sX = -1 if is180 else 1
    sY = 1 if (refl == is180) else -1
    return sX, sY

def transform_rect(rect, oX, oY, sX, sY):
    """A local (x0,y0,x1,y1) rect through Geom::Transform's own formula:
    world = (o + s*local). Sign flips can swap which corner is min/max."""
    x0, y0, x1, y1 = rect
    wx0, wx1 = sorted((oX + sX * x0, oX + sX * x1))
    wy0, wy1 = sorted((oY + sY * y0, oY + sY * y1))
    return (wx0, wy0, wx1, wy1)

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

# Every direct (parent, ref) occurrence of each cell, anywhere in the
# library -- used to find a leaf's pin names from a label that lives one
# level up (see leaf_terminals, below).
parent_occurrences = defaultdict(list)
for _pcell in lib.cells:
    for _ref in _pcell.references:
        parent_occurrences[_ref.cell.name].append((_pcell, _ref))

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

def cell_labels(cell):
    """[(text, x, y, layer_name)] for this cell's own "Label"-purpose labels."""
    out = []
    for lbl in cell.labels:
        key = (lbl.layer, lbl.texttype)
        if key in rev_map:
            lname, purpose = rev_map[key]
            if purpose == "label":
                lx, ly = lbl.origin
                out.append((lbl.text, lx, ly, lname))
    return out

def leaf_geometry(cell):
    """(pin_rects, draw_rects): {layer_name: [(x0,y0,x1,y1), ...]} for this
    cell's own "Pin"/"Draw"-purpose polygons, in the cell's local frame."""
    pin_rects  = defaultdict(list)
    draw_rects = defaultdict(list)
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
    return pin_rects, draw_rects

def _ancestor_labels_in_local_frame(cell):
    """[(text, local_x, local_y, layer, priority)] from every direct parent
    occurrence of `cell`, transformed into `cell`'s own local frame.

    Every hand-drawn device leaf in the bandgap/OTA hierarchy carries no
    label of its own and no dedicated "Pin"-datatype geometry either --
    only plain "Draw" metal -- the design's own tooling labels a net only
    where it surfaces in the *parent* that instantiates the device.
    Occurrences are visited in a stable order so that when the same leaf is
    reused with genuinely different meaning in different places (as opposed
    to sharing nets, the common case for multi-finger devices), which one
    "wins" a contested rect is at least deterministic -- see the conflict
    warning below.
    """
    hits = []
    occurrences = sorted(parent_occurrences.get(cell.name, []),
                          key=lambda po: (po[0].name, po[1].origin))
    for priority, (parent_cell, ref) in enumerate(occurrences, start=1):
        oX, oY = ref.origin
        sX, sY = ref_orient(ref)
        for (text, lx, ly, llayer) in cell_labels(parent_cell):
            # Inverse of Geom::Transform: world = o + s*local, and s is
            # self-inverse (+-1), so local = s*(world - o).
            local_x = sX * (lx - oX)
            local_y = sY * (ly - oY)
            hits.append((text, local_x, local_y, llayer, priority))
    return hits

def leaf_terminals(cell):
    pin_rects, draw_rects = leaf_geometry(cell)
    # Local labels (priority 0, the existing ALIGN/gds2lef.py convention)
    # take precedence over anything found one level up.
    labels = [(text, lx, ly, llayer, 0) for (text, lx, ly, llayer) in cell_labels(cell)]
    labels += _ancestor_labels_in_local_frame(cell)

    pin_shapes = defaultdict(lambda: defaultdict(list))
    assigned   = {}   # (layer, rect) -> (text, priority)

    for (text, lx, ly, llayer, priority) in sorted(labels, key=lambda h: h[4]):
        candidates = pin_rects.get(llayer) or draw_rects.get(llayer, [])
        for rect in candidates:
            x0, y0, x1, y1 = rect
            if not (x0 <= lx <= x1 and y0 <= ly <= y1):
                continue
            k = (llayer, rect)
            if k not in assigned:
                pin_shapes[text][llayer].append(rect)
                assigned[k] = (text, priority)
            elif assigned[k][0] != text:
                print(f"warning: {cell.name} pin at {rect} on {llayer} named "
                      f"both {assigned[k][0]!r} and {text!r} by different "
                      f"placements; keeping {assigned[k][0]!r}", file=sys.stderr)

    # A device with a drawn "Pin"-purpose shape but no label anywhere (local
    # or ancestor) would otherwise vanish from the placement with zero
    # terminals. Give it a generic, position-stable name instead. Only
    # "Pin"-purpose geometry gets this treatment -- falling back to "Draw"
    # too would turn every via cut's landing-pad polygon into a bogus
    # synthetic pin, since vias have Draw metal but no Pin datatype either.
    anon = 0
    for llayer in sorted(pin_rects):
        for rect in sorted(pin_rects[llayer]):
            if (llayer, rect) in assigned:
                continue
            anon += 1
            pin_shapes[f"PIN{anon}"][llayer].append(rect)
            assigned[(llayer, rect)] = (f"PIN{anon}", None)

    terminals = []
    for pin_name in sorted(pin_shapes):
        rects = [r for rs in pin_shapes[pin_name].values() for r in rs]
        if rects:
            x0, y0, x1, y1 = rects[0]
            terminals.append({"name": pin_name,
                               "rect": [_n(x0), _n(y0), _n(x1), _n(y1)]})
    return terminals

leaves_out       = []
leaf_terminal_by = {}   # concrete_name -> {terminal_name: local_rect}
for name in sorted(leaf_names):
    c = cell_by_name[name]
    terminals = leaf_terminals(c)
    leaf_terminal_by[name] = {t["name"]: tuple(t["rect"]) for t in terminals}
    leaves_out.append({
        "abstract_name": abstract_name(name),
        "bbox":          cell_bbox(c),
        "concrete_name": name,
        "terminals":     terminals,
    })

def labels_derived_fa_map(parent_cell, ref):
    """fa_map for one leaf reference, from labels in its *parent* cell.

    Every hand-drawn device leaf in the bandgap/OTA hierarchy carries no
    label of its own -- the design's own tooling labels a net only where it
    surfaces in the parent that instantiates the device. For each of the
    leaf's own terminals (named by
    leaf_terminals(), above -- a real label if the leaf has one, else a
    synthetic PIN<n>), transform its local rect into the parent's frame and
    check whether one of the parent's own labels lands inside it; if so,
    that label names the *actual* net this specific placement connects to.

    A terminal with no ancestor label landing on it is simply left out of
    the returned fa_map -- either it is genuinely internal at this level, or
    its net surfaces even further up the hierarchy, out of scope here.
    """
    terms = leaf_terminal_by.get(ref.cell.name)
    if not terms:
        return []
    oX, oY   = ref.origin
    sX, sY   = ref_orient(ref)
    labels   = cell_labels(parent_cell)
    fa_map   = []
    for formal, rect in terms.items():
        wx0, wy0, wx1, wy1 = transform_rect(rect, oX, oY, sX, sY)
        for (text, lx, ly, _llayer) in labels:
            if wx0 <= lx <= wx1 and wy0 <= ly <= wy1:
                fa_map.append({"actual": text, "formal": formal})
                break
    return fa_map

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
        sX, sY   = ref_orient(ref)

        if idx < len(scs_insts):
            inst_name, cell_type_abs, actuals = scs_insts[idx]
            ref_scs  = subckts.get(abstract_name(sub_name)) or subckts.get(cell_type_abs)
            formals  = ref_scs["ports"] if ref_scs else []
            fa_map   = [{"actual": a, "formal": f}
                        for f, a in zip(formals, actuals)]
        else:
            inst_name = f"I_{idx}"
            fa_map    = []

        # No netlist covered this instance -- fall back to whatever
        # connectivity a label in this module's own frame can tell us about
        # the leaf it's placing directly (see labels_derived_fa_map). Only
        # applies to leaves: a module-type reference's ports aren't simple
        # local rects, so it keeps relying on the netlist path.
        if not fa_map and sub_name in leaf_names:
            fa_map = labels_derived_fa_map(cell, ref)

        instances.append({
            "abstract_template_name": abstract_name(sub_name),
            "concrete_template_name": sub_name,
            "fa_map":                 fa_map,
            "instance_name":          inst_name,
            "transformation":         {"oX": _n(ox), "oY": _n(oy),
                                       "sX": sX, "sY": sY},
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
