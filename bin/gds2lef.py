#!/usr/bin/env python3

import gdstk
import json
import argparse
from collections import defaultdict

ap = argparse.ArgumentParser(description="Convert GDS cells to LEF macros")
ap.add_argument("-g", "--gds",    required=True, help="Input GDS file")
ap.add_argument("-l", "--layers", required=True, help="layers.json with GDS layer mapping")
ap.add_argument("-o", "--out",    default="out.lef", help="Output LEF file")
ap.add_argument("-s", "--scale",  type=float, default=0,
                help="LEF database units per micron (default 1.0)")
args = ap.parse_args()

def fmt(v):
    return f"{v:.6g}"

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

lib = gdstk.read_gds(args.gds)
scale = args.scale if args.scale != 0 else (lib.unit/lib.precision)

FRAC_PREC = min(1e-7, lib.precision / lib.unit) if lib.unit else 1e-7

def s(v):
    return round(v * scale)

def poly_to_rects(poly):
    pts = poly.points
    if len(pts) == 4:
        xs = sorted({round(float(p[0]), 9) for p in pts})
        ys = sorted({round(float(p[1]), 9) for p in pts})
        if len(xs) == 2 and len(ys) == 2:
            return [(xs[0], ys[0], xs[1], ys[1])]

    out = []
    for piece in poly.fracture(max_points=5, precision=FRAC_PREC):
        bb = piece.bounding_box()
        if bb is None:
            continue
        (x0, y0), (x1, y1) = bb
        if x1 - x0 <= 0 or y1 - y0 <= 0:
            continue
        out.append((x0, y0, x1, y1))
    return out

def iter_shapes(cell):
    for poly in cell.polygons:
        for rect in poly_to_rects(poly):
            yield poly.layer, poly.datatype, rect
    for path in cell.paths:
        for poly in path.to_polygons():
            for rect in poly_to_rects(poly):
                yield poly.layer, poly.datatype, rect

lef_macros = []

top_cells = lib.top_level()
for t in top_cells:
    t.flatten(apply_repetitions=True)

for cell in top_cells:
    boundary_rects = []
    draw_rects     = defaultdict(list)
    pin_rects      = defaultdict(list)
    blockage_rects = defaultdict(list)
    labels = []
    oX = cell.bounding_box()[0][0]
    oY = cell.bounding_box()[0][1]
    for layer, datatype, rect in iter_shapes(cell):
        key = (layer, datatype)
        rect = (rect[0] - oX, rect[1] - oY, rect[2] - oX, rect[3] - oY)

        if key == boundary_key:
            boundary_rects.append(rect)
            continue

        if key not in rev_map:
            continue
        lname, purpose = rev_map[key]

        if purpose == "draw":
            draw_rects[lname].append(rect)
        elif purpose == "pin":
            pin_rects[lname].append(rect)
        elif purpose in ("blockage", "obs"):
            blockage_rects[lname].append(rect)

    for bucket in (draw_rects, pin_rects, blockage_rects):
        for k in list(bucket):
            bucket[k] = list(dict.fromkeys(bucket[k]))

    for lbl in cell.labels:
        key = (lbl.layer, lbl.texttype)
        if key in rev_map:
            lname, _purpose = rev_map[key]
            lx, ly = lbl.origin
            lx -= oX
            ly -= oY
            labels.append((lbl.text, lx, ly, lname))

    pin_shapes  = defaultdict(lambda: defaultdict(list))
    assigned    = set()

    for (text, lx, ly, llayer) in labels:
        candidates = pin_rects.get(llayer) or draw_rects.get(llayer, [])
        for rect in candidates:
            x0, y0, x1, y1 = rect
            if x0 <= lx <= x1 and y0 <= ly <= y1:
                key = (llayer, rect)
                if key not in assigned:
                    pin_shapes[text][llayer].append(rect)
                    assigned.add(key)

    anon = 0
    for lname, rects in pin_rects.items():
        for rect in rects:
            if (lname, rect) not in assigned:
                anon += 1
                pin_shapes[f"PIN{anon}"][lname].append(rect)
                assigned.add((lname, rect))

    for lname, rects in draw_rects.items():
        for rect in rects:
            if (lname, rect) not in assigned:
                blockage_rects[lname].append(rect)
                assigned.add((lname, rect))
    for k in list(blockage_rects):
        blockage_rects[k] = list(dict.fromkeys(blockage_rects[k]))

    if boundary_rects:
        bx0, by0, bx1, by1 = boundary_rects[0]
        width  = s(bx1 - bx0)
        height = s(by1 - by0)
    else:
        max_x = cell.bounding_box()[1][0]
        max_y = cell.bounding_box()[1][1]
        width  = s(max_x - oX)
        height = s(max_y - oY)
        if oX < 0 or oY < 0:
            print(f"  warning: {cell.name} geometry starts at "
                  f"({fmt(oX)}, {fmt(oY)}) but ORIGIN is 0 0")

    lef_macros.append({
        "name":     cell.name,
        "width":    width,
        "height":   height,
        "pins":     pin_shapes,
        "blockage": blockage_rects,
    })

out = []

for m in lef_macros:
    name = m["name"]
    out.append(f"MACRO {name}")
    out.append(f"  CLASS BLOCK ;")
    out.append(f"  ORIGIN 0 0 ;")
    out.append(f"  FOREIGN {name} 0 0 ;")
    out.append(f"  SIZE {fmt(m['width'])} BY {fmt(m['height'])} ;")
    out.append(f"  SYMMETRY X Y ;")

    for pin_name in sorted(m["pins"]):
        layers_dict = m["pins"][pin_name]
        out.append(f"  PIN {pin_name}")
        out.append(f"    DIRECTION INOUT ;")
        out.append(f"    USE SIGNAL ;")
        out.append(f"    PORT")
        for lname in sorted(layers_dict):
            out.append(f"      LAYER {lname} ;")
            for (x0, y0, x1, y1) in layers_dict[lname]:
                out.append(f"        RECT {fmt(s(x0))} {fmt(s(y0))} {fmt(s(x1))} {fmt(s(y1))} ;")
        out.append(f"    END")
        out.append(f"  END {pin_name}")

    if m["blockage"]:
        out.append(f"  OBS")
        for lname in sorted(m["blockage"]):
            out.append(f"    LAYER {lname} ;")
            for (x0, y0, x1, y1) in m["blockage"][lname]:
                out.append(f"      RECT {fmt(s(x0))} {fmt(s(y0))} {fmt(s(x1))} {fmt(s(y1))} ;")
        out.append(f"  END")

    out.append(f"END {name}")
    out.append("")

with open(args.out, "w") as fp:
    fp.write("\n".join(out) + "\n")

print(f"wrote {args.out}  ({len(lef_macros)} macros)")
for m in lef_macros:
    print(f"  {m['name']}: {m['width']} x {m['height']}, "
          f"{len(m['pins'])} pins, "
          f"{sum(len(v) for v in m['blockage'].values())} obs shapes")
