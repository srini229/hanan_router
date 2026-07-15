#!/usr/bin/env python3

import gdstk
import json
import argparse

ap = argparse.ArgumentParser(description="Convert LEF macros to GDS using gdstk")
ap.add_argument("-f", "--lef",    required=True, help="Input LEF file")
ap.add_argument("-l", "--layers", required=True, help="layers.json with GDS layer mapping")
ap.add_argument("-o", "--out",    default="out.gds", help="Output GDS file")
ap.add_argument("-u", "--unit",   type=float, default=1e-6,
                help="GDS unit in metres (default 1e-6 = 1 µm)")
ap.add_argument("-p", "--precision", type=float, default=1e-9,
                help="GDS precision in metres (default 1e-9 = 1 nm)")
args = ap.parse_args()

layer_map = {}
bbox_gds   = (100, 5)
boundary_gds = (101, 0)

with open(args.layers) as fp:
    ldata = json.load(fp)

for entry in ldata.get("Abstraction", []):
    name  = entry.get("Layer")
    gno   = entry.get("GdsLayerNo")
    dtmap = entry.get("GdsDatatype", {})
    if name is None or gno is None:
        continue
    layer_map[name] = {k.lower(): (gno, v) for k, v in dtmap.items()}
    if name == "Bbox":
        bbox_gds = (gno, dtmap.get("Draw", 5))
    if name == "Boundary":
        boundary_gds = (gno, dtmap.get("Draw", 0))

def gds_layer(lname, dtype="draw"):
    info = layer_map.get(lname, {})
    return info.get(dtype, info.get("draw", (0, 0)))

class LefMacro:
    def __init__(self, name):
        self.name   = name
        self.width  = 0.0
        self.height = 0.0
        self.pins   = {}
        self.obs    = []

macros = []
scale  = 1.0

with open(args.lef) as fp:
    lines = fp.readlines()

i = 0
cur_macro  = None
cur_pin    = None
cur_layer  = None
in_port    = False
in_obs     = False

while i < len(lines):
    tokens = lines[i].split()
    if not tokens:
        i += 1
        continue

    kw = tokens[0].upper()

    if kw == "DATABASE" and len(tokens) >= 2 and tokens[1].upper() == "MICRONS":
        for t in tokens[2:]:
            try:
                scale = float(t.rstrip(";"))
                break
            except ValueError:
                pass

    elif kw == "MACRO":
        cur_macro = LefMacro(tokens[1])
        macros.append(cur_macro)

    elif kw == "SIZE" and cur_macro:
        cur_macro.width  = float(tokens[1]) / scale
        cur_macro.height = float(tokens[3]) / scale

    elif kw == "PIN" and cur_macro:
        cur_pin   = tokens[1]
        cur_layer = None
        in_port   = False
        cur_macro.pins.setdefault(cur_pin, [])

    elif kw == "PORT":
        in_port = True

    elif kw == "END" and len(tokens) > 1:
        end_target = tokens[1].upper()
        if end_target == "PORT":
            in_port = False
        elif cur_pin and tokens[1] == cur_pin:
            cur_pin = None
        elif cur_macro and tokens[1] == cur_macro.name:
            cur_macro = None
        elif end_target == "OBS":
            in_obs = False

    elif kw == "OBS" and cur_macro:
        in_obs = True
        cur_layer = None

    elif kw == "LAYER" and cur_macro:
        cur_layer = tokens[1]

    elif kw == "RECT" and cur_macro and cur_layer:
        # RECT <x0> <y0> <x1> <y1> ;
        x0 = float(tokens[1]) / scale
        y0 = float(tokens[2]) / scale
        x1 = float(tokens[3]) / scale
        y1 = float(tokens[4].rstrip(";")) / scale
        coords = (x0, y0, x1, y1)
        if in_obs:
            cur_macro.obs.append((cur_layer, coords))
        elif cur_pin is not None:
            cur_macro.pins[cur_pin].append((cur_layer, coords))

    i += 1

lib = gdstk.Library(unit=args.unit, precision=args.precision)

for macro in macros:
    cell = lib.new_cell(macro.name)

    if macro.width > 0 and macro.height > 0:
        bl, bd = boundary_gds
        cell.add(gdstk.rectangle(
            (0, 0), (macro.width, macro.height),
            layer=bl, datatype=bd))

    for pin_name, shapes in macro.pins.items():
        for (lname, (x0, y0, x1, y1)) in shapes:
            l, d = gds_layer(lname, "draw")
            cell.add(gdstk.rectangle((x0, y0), (x1, y1), layer=l, datatype=d))
            pin_gds = gds_layer(lname, "pin")
            if pin_gds != gds_layer(lname, "draw"):
                lp, dp = pin_gds
                cell.add(gdstk.rectangle((x0, y0), (x1, y1), layer=lp, datatype=dp))
            label_gds = gds_layer(lname, "label")
            if label_gds and label_gds != gds_layer(lname, "draw"):
                ll, dl = label_gds
                cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                cell.add(gdstk.Label(pin_name, (cx, cy), layer=ll, texttype=dl))

    for (lname, (x0, y0, x1, y1)) in macro.obs:
        lb, db = gds_layer(lname, "blockage")
        cell.add(gdstk.rectangle((x0, y0), (x1, y1), layer=lb, datatype=db))

    print(f"  macro {macro.name}: {macro.width} x {macro.height}, "
          f"{len(macro.pins)} pins, {len(macro.obs)} obs shapes")

lib.write_gds(args.out)
print(f"wrote {args.out}  ({len(macros)} macros)")
