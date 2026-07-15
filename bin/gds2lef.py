#!/usr/bin/env python3

import gdstk
import json
import argparse
from collections import defaultdict

ap = argparse.ArgumentParser(description="Convert GDS cells to LEF macros")
ap.add_argument("-g", "--gds",    required=True, help="Input GDS file")
ap.add_argument("-l", "--layers", required=True, help="layers.json with GDS layer mapping")
ap.add_argument("-o", "--out",    default="out.lef", help="Output LEF file")
ap.add_argument("-s", "--scale",  type=float, default=1.0,
                help="LEF database units per micron (default 1.0)")
args = ap.parse_args()

def fmt(v):
    """Format a coordinate: up to 6 significant figures, strip trailing zeros."""
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
scale = args.scale

def s(v):
    return v * scale

lef_macros = []

for cell in lib.cells:
    boundary_rects = []
    draw_rects     = defaultdict(list)
    pin_rects      = defaultdict(list)
    blockage_rects = defaultdict(list)
    labels = []

    for poly in cell.polygons:
        key = (poly.layer, poly.datatype)
        bb  = poly.bounding_box()
        x0, y0 = bb[0]
        x1, y1 = bb[1]

        if key == boundary_key:
            boundary_rects.append((x0, y0, x1, y1))
            continue

        if key not in rev_map:
            continue
        lname, purpose = rev_map[key]

        if purpose == "draw":
            draw_rects[lname].append((x0, y0, x1, y1))
        elif purpose == "pin":
            pin_rects[lname].append((x0, y0, x1, y1))
        elif purpose in ("blockage", "obs"):
            blockage_rects[lname].append((x0, y0, x1, y1))

    for lbl in cell.labels:
        key = (lbl.layer, lbl.texttype)
        if key in rev_map:
            lname, purpose = rev_map[key]
            if purpose == "label":
                lx, ly = lbl.origin
                labels.append((lbl.text, lx, ly, lname))

    if boundary_rects:
        bx0, by0, bx1, by1 = boundary_rects[0]
        width  = s(bx1 - bx0)
        height = s(by1 - by0)
    else:
        all_v = [(x, y)
                 for rects in list(pin_rects.values()) + list(draw_rects.values())
                 for (x0, y0, x1, y1) in rects
                 for x, y in ((x0, y0), (x1, y1))]
        width  = s(max((x for x, _ in all_v), default=0))
        height = s(max((y for _, y in all_v), default=0))

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

    lef_macros.append({
        "name":     cell.name,
        "width":    width,
        "height":   height,
        "pins":     pin_shapes,
        "blockage": blockage_rects,
    })

out = []
out.append("VERSION 5.8 ;")
out.append("")
out.append("UNITS")
out.append(f"    DATABASE MICRONS {int(scale)} ;")
out.append("END UNITS")
out.append("")

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

out.append("END LIBRARY")

with open(args.out, "w") as fp:
    fp.write("\n".join(out) + "\n")

print(f"wrote {args.out}  ({len(lef_macros)} macros)")
for m in lef_macros:
    print(f"  {m['name']}: {m['width']} x {m['height']}, "
          f"{len(m['pins'])} pins, "
          f"{sum(len(v) for v in m['blockage'].values())} obs shapes")
