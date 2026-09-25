#!/usr/bin/env python3
"""Complete broken nets in a KLayout layout with hanan_router (headless klayout.db, or pya inside KLayout)."""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

import klayout.db as db

EPS = 1  # dbu; a zero-size probe box never counts as interacting in klayout


def load_layers(path):
    """(metals, labels, vias, draw_by_name, widths) from a layers.json."""
    metals, labels, vias, draw_by_name, widths = {}, {}, [], {}, {}
    with open(path) as f:
        data = json.load(f)
    for e in data.get("Abstraction", []):
        name = e.get("Layer")
        gno = e.get("GdsLayerNo")
        dt = e.get("GdsDatatype", {})
        if name is None or gno is None or "Draw" not in dt:
            continue
        draw_by_name[name] = (gno, dt["Draw"])
        stack = e.get("Stack")
        if stack and len(stack) == 2 and all(stack):
            vias.append((name, gno, dt["Draw"], stack[0], stack[1]))
        else:
            metals[name] = (gno, dt["Draw"])
            if "Label" in dt:
                labels[name] = (gno, dt["Label"])
            if "Width" in e:
                widths[name] = e["Width"]
    return metals, labels, vias, draw_by_name, widths


def metal_region(layout, cell, layer_spec):
    """Flattened (whole hierarchy, any depth) Region for one (gds, dt) layer."""
    idx = layout.find_layer(*layer_spec)
    if idx is None:
        return db.Region()
    return db.Region(db.RecursiveShapeIterator(layout, cell, idx))


def all_label_hits(layout, cell, layer_spec):
    """[(text, x, y)] for every label on the given (gds, texttype) layer,
    anywhere in the cell's hierarchy, world coordinates."""
    idx = layout.find_layer(*layer_spec)
    if idx is None:
        return []
    hits = []
    it = db.RecursiveShapeIterator(layout, cell, idx)
    while not it.at_end():
        s = it.shape()
        if s.is_text():
            t = s.text
            p = it.trans() * db.Point(t.x, t.y)
            hits.append((s.text_string, p.x, p.y))
        it.next()
    return hits


def label_hits(layout, cell, layer_spec, text):
    """[(x, y)] world positions of every label reading exactly `text`."""
    return [(x, y) for (txt, x, y) in all_label_hits(layout, cell, layer_spec)
            if txt == text]


def which_polygon(polygons, x, y):
    """Index of the polygon in `polygons` containing (x, y), or None."""
    probe = db.Region(db.Box(x - EPS, y - EPS, x + EPS, y + EPS))
    for i, poly in enumerate(polygons):
        if not db.Region(poly).interacting(probe).is_empty():
            return i
    return None


def _merged_islands(layout, cell, metals, vias):
    """(merged conductor polygons, {layer: layer index}) for the whole hierarchy."""
    all_by_layer = {name: metal_region(layout, cell, spec)
                     for name, spec in metals.items()}
    combined = db.Region()
    for r in all_by_layer.values():
        combined += r
    for _name, gno, dt, _a, _b in vias:
        combined += metal_region(layout, cell, (gno, dt))
    combined.merge()
    return list(combined.each()), all_by_layer


def _island_per_layer(islands_all, all_by_layer, idx):
    """{layer_name: db.Region} -- one merged island's own contribution on
    each metal layer, e.g. for building virtual_pins/obstacles from it."""
    poly_region = db.Region(islands_all[idx])
    per_layer = {}
    for lname, lregion in all_by_layer.items():
        part = lregion & poly_region
        if not part.is_empty():
            per_layer[lname] = part
    return per_layer


def net_islands(layout, cell, metals, labels, vias, net_name):
    """Conductor islands carrying a label equal to `net`."""
    islands_all, all_by_layer = _merged_islands(layout, cell, metals, vias)

    hit_islands = set()
    for lname, lspec in labels.items():
        for (x, y) in label_hits(layout, cell, lspec, net_name):
            idx = which_polygon(islands_all, x, y)
            if idx is not None:
                hit_islands.add(idx)

    result = [_island_per_layer(islands_all, all_by_layer, idx)
              for idx in sorted(hit_islands)]
    return result, all_by_layer


def net_name_for_point(layout, cell, metals, labels, vias, x, y):
    """Net label reachable from the island containing (x, y), or None."""
    islands_all, _all_by_layer = _merged_islands(layout, cell, metals, vias)
    idx = which_polygon(islands_all, x, y)
    if idx is None:
        return None
    island_region = db.Region(islands_all[idx])
    for lname, lspec in labels.items():
        for (text, lx, ly) in all_label_hits(layout, cell, lspec):
            probe = db.Region(db.Box(lx - EPS, ly - EPS, lx + EPS, ly + EPS))
            if not (island_region & probe).is_empty():
                return text
    return None


def region_to_um_rects(region, dbu):
    """Region as [[x0,y0,x1,y1], ...] in microns; holes are split into rectangles."""
    return [[b.left * dbu, b.bottom * dbu, b.right * dbu, b.top * dbu]
            for p in region.decompose_trapezoids_to_region().each() for b in [p.bbox()]]


def island_anchor(per_layer, widths):
    """(layer, box): a wire-width rectangle inside an island, on its largest layer."""
    lname, region = max(per_layer.items(), key=lambda kv: kv[1].area())
    box = None
    for p in region.each_merged():
        box = p.bbox() if box is None else box + p.bbox()
    w = widths.get(lname, box.width())
    size = max(1, min(w, box.width(), box.height()))
    cx, cy = box.center().x, box.center().y
    anchor = db.Box(cx - size // 2, cy - size // 2,
                     cx - size // 2 + size, cy - size // 2 + size)
    return lname, anchor


def build_router_inputs(work_dir, net_name, islands, obstacles_by_layer,
                         layout_dbu, uu, bbox_um, widths, virtual_pins_um=None,
                         corridor_topology_um=None, corridor_pitch=None,
                         corridor_guide_weight=None):
    """Write placement.json, LEF and NDR that make one net's islands pins of one net."""
    to_router_units = lambda v: round(v * layout_dbu * uu)
    # widths are in layers.json units (microns * uu); convert to layout dbu
    widths_layout_units = {name: w / uu / layout_dbu for name, w in widths.items()}

    leaves, instances, lef_macros = [], [], []
    for i, per_layer in enumerate(islands):
        lname, box = island_anchor(per_layer, widths_layout_units)
        macro = f"FRAG{i}"
        w = to_router_units(box.width())
        h = to_router_units(box.height())
        # pad the macro so the router has room for an escape point off the pin
        margin = max(w, h, 1)
        mw, mh = w + 2 * margin, h + 2 * margin
        lef_macros.append(
            f"MACRO {macro}\n  CLASS BLOCK ;\n  ORIGIN 0 0 ;\n"
            f"  UNITS\n    DATABASE MICRONS UNITS {uu} ;\n  END UNITS\n"
            f"  SIZE {mw} BY {mh} ;\n"
            f"  PIN P\n    DIRECTION INOUT ;\n    PORT\n"
            f"      LAYER {lname} ;\n"
            f"        RECT {margin} {margin} {margin + w} {margin + h} ;\n"
            f"    END\n  END P\nEND {macro}\n"
        )
        leaves.append({
            "abstract_name": macro, "concrete_name": macro,
            "bbox": [0, 0, mw, mh],
            "terminals": [{"name": "P", "rect": [margin, margin,
                                                   margin + w, margin + h]}],
        })
        instances.append({
            "abstract_template_name": macro, "concrete_template_name": macro,
            "fa_map": [{"actual": net_name, "formal": "P"}],
            "instance_name": f"I_{i}",
            "transformation": {"oX": to_router_units(box.left) - margin,
                                "oY": to_router_units(box.bottom) - margin,
                                "sX": 1, "sY": 1},
        })

    placement = {
        "global_signals": [],
        "leaves": leaves,
        "modules": [{
            "abstract_name": "TOP", "concrete_name": "TOP_CONC_0",
            "bbox": [round(v * uu) for v in bbox_um],  # bbox_um is already microns
            "instances": instances, "parameters": [],
        }],
    }
    obstacle_shapes = {lname: region_to_um_rects(region, layout_dbu)
                        for lname, region in obstacles_by_layer.items()
                        if not region.is_empty()}
    ndr = [{
        "module": "TOP_CONC_0",
        "obstacles": [{"shapes": obstacle_shapes}] if obstacle_shapes else [],
    }]
    if virtual_pins_um or corridor_topology_um or corridor_pitch or corridor_guide_weight:
        net_entry = {"name": net_name}
        if virtual_pins_um:
            net_entry["virtual_pins"] = [{lname: [list(r) for r in rects]}
                                          for lname, rects in virtual_pins_um.items()]
        if corridor_topology_um:
            net_entry["corridor_topology"] = [list(p) for p in corridor_topology_um]
        if corridor_pitch:
            net_entry["corridor_pitch"] = corridor_pitch
        if corridor_guide_weight:
            net_entry["corridor_guide_weight"] = corridor_guide_weight
        ndr[0]["nets"] = [net_entry]

    placement_path = os.path.join(work_dir, "autocomplete.netlist.json")
    ndr_path = os.path.join(work_dir, "autocomplete_ndr.json")
    lef_path = os.path.join(work_dir, "autocomplete.lef")
    with open(placement_path, "w") as f:
        json.dump(placement, f, indent=2)
    with open(ndr_path, "w") as f:
        json.dump(ndr, f, indent=2)
    with open(lef_path, "w") as f:
        f.write("\n".join(lef_macros) + "\n")
    return placement_path, ndr_path, lef_path


def build_router_inputs_multi(work_dir, nets, obstacles_by_layer, layout_dbu,
                               uu, bbox_um, widths, corridor_pitch=None,
                               corridor_guide_weight=None):
    """build_router_inputs for several nets in one router pass."""
    to_router_units = lambda v: round(v * layout_dbu * uu)
    widths_layout_units = {name: w / uu / layout_dbu for name, w in widths.items()}

    leaves, instances, lef_macros, ndr_nets = [], [], [], []
    for ni, (net_name, islands, corridor_topology_um) in enumerate(nets):
        for i, per_layer in enumerate(islands):
            lname, box = island_anchor(per_layer, widths_layout_units)
            macro = f"FRAG{ni}_{i}"
            w = to_router_units(box.width())
            h = to_router_units(box.height())
            margin = max(w, h, 1)
            mw, mh = w + 2 * margin, h + 2 * margin
            lef_macros.append(
                f"MACRO {macro}\n  CLASS BLOCK ;\n  ORIGIN 0 0 ;\n"
                f"  UNITS\n    DATABASE MICRONS UNITS {uu} ;\n  END UNITS\n"
                f"  SIZE {mw} BY {mh} ;\n"
                f"  PIN P\n    DIRECTION INOUT ;\n    PORT\n"
                f"      LAYER {lname} ;\n"
                f"        RECT {margin} {margin} {margin + w} {margin + h} ;\n"
                f"    END\n  END P\nEND {macro}\n"
            )
            leaves.append({
                "abstract_name": macro, "concrete_name": macro,
                "bbox": [0, 0, mw, mh],
                "terminals": [{"name": "P", "rect": [margin, margin,
                                                       margin + w, margin + h]}],
            })
            instances.append({
                "abstract_template_name": macro, "concrete_template_name": macro,
                "fa_map": [{"actual": net_name, "formal": "P"}],
                "instance_name": f"I_{ni}_{i}",
                "transformation": {"oX": to_router_units(box.left) - margin,
                                    "oY": to_router_units(box.bottom) - margin,
                                    "sX": 1, "sY": 1},
            })
        if corridor_topology_um or corridor_pitch or corridor_guide_weight:
            net_entry = {"name": net_name}
            if corridor_topology_um:
                net_entry["corridor_topology"] = [list(p) for p in corridor_topology_um]
            if corridor_pitch:
                net_entry["corridor_pitch"] = corridor_pitch
            if corridor_guide_weight:
                net_entry["corridor_guide_weight"] = corridor_guide_weight
            ndr_nets.append(net_entry)

    placement = {
        "global_signals": [],
        "leaves": leaves,
        "modules": [{
            "abstract_name": "TOP", "concrete_name": "TOP_CONC_0",
            "bbox": [round(v * uu) for v in bbox_um],
            "instances": instances, "parameters": [],
        }],
    }
    obstacle_shapes = {lname: region_to_um_rects(region, layout_dbu)
                        for lname, region in obstacles_by_layer.items()
                        if not region.is_empty()}
    ndr = [{
        "module": "TOP_CONC_0",
        "obstacles": [{"shapes": obstacle_shapes}] if obstacle_shapes else [],
    }]
    if ndr_nets:
        ndr[0]["nets"] = ndr_nets

    placement_path = os.path.join(work_dir, "autocomplete.netlist.json")
    ndr_path = os.path.join(work_dir, "autocomplete_ndr.json")
    lef_path = os.path.join(work_dir, "autocomplete.lef")
    with open(placement_path, "w") as f:
        json.dump(placement, f, indent=2)
    with open(ndr_path, "w") as f:
        json.dump(ndr, f, indent=2)
    with open(lef_path, "w") as f:
        f.write("\n".join(lef_macros) + "\n")
    return placement_path, ndr_path, lef_path


def run_router(router_bin, layers_json, placement_path, lef_path, ndr_path,
               work_dir, uu, net_name, rsmt=True):
    """Run hanan_router on the inputs in `work`; return the routed DEF path."""
    out_dir = os.path.join(work_dir, "route_out")
    os.makedirs(out_dir, exist_ok=True)
    cmd = [router_bin, "-d", layers_json, "-p", placement_path,
           "-l", lef_path, "-uu", str(uu), "-ndr", ndr_path, "-o", out_dir]
    if rsmt:
        cmd.append("-rsmt")
    env = dict(os.environ, HANAN_DEBUG_NET=net_name)
    p = subprocess.run(cmd, capture_output=True, text=True, cwd=work_dir, env=env)
    routed_def = os.path.join(out_dir, "TOP_CONC_0.def")
    debug_lef = os.path.join(work_dir, f"net_TOP_CONC_0_{net_name}.lef")
    return p.returncode, p.stdout + p.stderr, routed_def, debug_lef


def def_net_rects(def_path, net_name):
    """[(layer, x0, y0, x1, y1)] (dbu) for one net's RECT entries in a
    routed DEF -- vias included, so a bridging cut gets painted too."""
    out, innets, net = [], False, None
    with open(def_path, errors="ignore") as f:
        for line in f:
            if line.startswith("NETS"):
                innets = True
                continue
            if line.startswith("END NETS"):
                break
            if not innets:
                continue
            m = re.match(r"\s*- (\S+)", line)
            if m:
                net = m.group(1)
                continue
            m = re.match(r"\s*\+ RECT (\w+) \( (-?\d+) (-?\d+) \) \( (-?\d+) (-?\d+) \)", line)
            if m and net == net_name:
                v = [int(x) for x in m.groups()[1:]]
                out.append((m.group(1), *v))
    return out


USER_CORRIDOR_LAYER = (997, 0)  # hand-drawn corridor; 998/999 are the router's own

WAYPOINT_LAYER = (996, 0)  # hand-drawn ordered path of waypoints


def read_all_waypoint_paths(layout, cell, layer_spec=WAYPOINT_LAYER):
    """Every Path on `layer_spec` in `cell`, as lists of world points in click order."""
    idx = layout.find_layer(*layer_spec)
    if idx is None:
        return []
    trans = db.Trans()
    out = []
    for s in cell.shapes(idx).each():
        if not s.is_path():
            continue
        out.append([((trans * p).x, (trans * p).y) for p in s.path.each_point()])
    return out


def nearest_waypoint_path(layout, cell, islands, layer_spec=WAYPOINT_LAYER):
    """The drawn Path nearest to `islands`, or [] if none."""
    paths = read_all_waypoint_paths(layout, cell, layer_spec)
    if not paths:
        return []
    centers = []
    for island in islands:
        for region in island.values():
            b = region.bbox()
            centers.append(((b.left + b.right) / 2, (b.bottom + b.top) / 2))
    if not centers:
        return paths[0]
    def dist(path):
        return min(abs(px - cx) + abs(py - cy) for (px, py) in path for (cx, cy) in centers)
    return min(paths, key=dist)


def read_waypoints(layout, cell, layer_spec=WAYPOINT_LAYER):
    """The first Path on `layer_spec` in `cell`, as world points in click order."""
    paths = read_all_waypoint_paths(layout, cell, layer_spec)
    return paths[0] if paths else []


def user_corridor_region(layout, cell, layer_spec=USER_CORRIDOR_LAYER):
    """Everything drawn on `layer_spec`, merged, as a Region."""
    idx = layout.find_layer(*layer_spec)
    if idx is None:
        return db.Region()
    r = db.Region(db.RecursiveShapeIterator(layout, cell, idx))
    r.merge()
    return r


CORRIDOR_LAYER = (999, 0)  # not used by any real sky130 (or other) layers.json
                            # layer number seen so far -- purely a debug
                            # visualization layer, never a routable one.
CORRIDOR_WALL_LAYER = (998, 0)  # -rsmt corridor wall keepouts


def lef_obs_layer_rects(lef_path, layer_name):
    """[(x0, y0, x1, y1)] in microns for one OBS layer of a router-written LEF."""
    if not os.path.exists(lef_path):
        return []
    with open(lef_path, errors="ignore") as f:
        text = f.read()
    m = re.search(r"\n    OBS\n(.*?)\n    END\n", text, re.S)
    if not m:
        return []
    out, layer = [], None
    for line in m.group(1).splitlines():
        lm = re.match(r"\s*LAYER (\S+) ;", line)
        if lm:
            layer = lm.group(1)
            continue
        rm = re.match(r"\s*RECT (\S+) (\S+) (\S+) (\S+) ;", line)
        if rm and layer == layer_name:
            out.append(tuple(float(v) for v in rm.groups()))
    return out


def paint_corridor_walls(layout, cell, hier_lef_path, net_name, dbu,
                          layer_spec=CORRIDOR_WALL_LAYER):
    """Paint the -rsmt corridor walls the router used onto a debug layer."""
    idx = layout.layer(*layer_spec)
    inserted = 0
    for x0, y0, x1, y1 in lef_obs_layer_rects(hier_lef_path, f"RSMT_{net_name}"):
        box = db.Box(round(x0 / dbu), round(y0 / dbu),
                      round(x1 / dbu), round(y1 / dbu))
        cell.shapes(idx).insert(box)
        inserted += 1
    return inserted


def paint_rsmt_corridor(layout, cell, def_path, net_name, dbu,
                         layer_spec=CORRIDOR_LAYER):
    """Paint the -rsmt corridor bands for `net_name` onto a debug layer."""
    idx = layout.layer(*layer_spec)
    scale = (1.0 / dbu) / layout.dbu
    inserted = 0
    for _layer_name, x0, y0, x1, y1 in def_net_rects(def_path, f"{net_name}_RSMT_CORRIDOR"):
        box = db.Box(round(x0 * scale), round(y0 * scale),
                      round(x1 * scale), round(y1 * scale))
        cell.shapes(idx).insert(box)
        inserted += 1
    return inserted


def paint_routed_def(layout, cell, def_path, net_name, draw_by_name, dbu):
    """Insert the routed geometry for `net_name` from a DEF into `cell`."""
    scale = (1.0 / dbu) / layout.dbu
    inserted = 0
    for layer_name, x0, y0, x1, y1 in def_net_rects(def_path, net_name):
        spec = draw_by_name.get(layer_name)
        if spec is None:
            continue
        idx = layout.layer(*spec)
        box = db.Box(round(x0 * scale), round(y0 * scale),
                      round(x1 * scale), round(y1 * scale))
        cell.shapes(idx).insert(box)
        inserted += 1
    return inserted


def net_debug_obstacles(lef_path):
    """{layer: [rects]} the router saw for a net, from its HANAN_DEBUG_NET dump."""
    if not os.path.exists(lef_path):
        return {}
    with open(lef_path, errors="ignore") as f:
        text = f.read()
    m = re.search(r"\n  OBS\n(.*?)\n  END\n", text, re.S)
    if not m:
        return {}
    out, layer = {}, None
    for line in m.group(1).splitlines():
        lm = re.match(r"\s*LAYER (\S+) ;", line)
        if lm:
            layer = lm.group(1)
            continue
        rm = re.match(r"\s*RECT (\S+) (\S+) (\S+) (\S+) ;", line)
        if rm and layer and layer != "BBOX":
            out.setdefault(layer, []).append(tuple(float(v) for v in rm.groups()))
    return out


OBSTACLE_DATATYPE = 98  # on each metal's own layer number


def paint_net_obstacles(layout, cell, lef_path, draw_by_name, dbu,
                         datatype=OBSTACLE_DATATYPE):
    """Paint net_debug_obstacles() onto debug layers."""
    inserted = 0
    for layer_name, rects in net_debug_obstacles(lef_path).items():
        spec = draw_by_name.get(layer_name)
        if spec is None:
            continue
        idx = layout.layer(spec[0], datatype)
        for x0, y0, x1, y1 in rects:
            box = db.Box(round(x0 / dbu), round(y0 / dbu),
                          round(x1 / dbu), round(y1 / dbu))
            cell.shapes(idx).insert(box)
            inserted += 1
    return inserted


def complete_net_on_layout(layout, cell, layers_json, net_name, router_bin,
                            work_dir, uu=1000, margin_um=5.0, rsmt=True,
                            show_corridor=True, show_obstacles=True,
                            corridor_layer_spec=None, corridor_margin_um=0.0,
                            waypoints_layer_spec=None, corridor_pitch=None,
                            corridor_guide_weight=None):
    """Route and paint the missing connections of one net; return a result dict."""
    metals, labels, vias, draw_by_name, widths = load_layers(layers_json)
    islands, all_by_layer = net_islands(layout, cell, metals, labels, vias, net_name)

    if not islands:
        return {"status": "not_found", "net": net_name}
    if len(islands) == 1:
        return {"status": "already_connected", "net": net_name}

    bbox = None
    for island in islands:
        for region in island.values():
            b = region.bbox()
            bbox = b if bbox is None else bbox + b
    corridor = (user_corridor_region(layout, cell, corridor_layer_spec)
                if corridor_layer_spec else db.Region())
    if not corridor.is_empty():
        if corridor_margin_um:
            # grow before it feeds the work-area bbox so the corridor stays inside
            corridor = corridor.sized(round(corridor_margin_um / layout.dbu))
        # the work area must cover the corridor, not just the islands
        bbox = bbox + corridor.bbox()

    waypoints = (nearest_waypoint_path(layout, cell, islands, waypoints_layer_spec)
                 if waypoints_layer_spec else [])
    if waypoints:
        # the work area must cover the clicked path, or detours get clipped
        wxs = [p[0] for p in waypoints]
        wys = [p[1] for p in waypoints]
        bbox = bbox + db.Box(min(wxs), min(wys), max(wxs), max(wys))

    margin = round(margin_um / layout.dbu)
    die_box = bbox.enlarged(margin, margin)
    work_area = db.Region(die_box)

    target_shapes = db.Region()
    for island in islands:
        for region in island.values():
            target_shapes += region
    obstacles_by_layer = {lname: (region & work_area) - target_shapes
                           for lname, region in all_by_layer.items()}

    if not corridor.is_empty():
        # pad each pin island so the escape-point search has a legal direction
        pin_clearance = max(round(1.0 / layout.dbu), 1)
        allowed = corridor + target_shapes.sized(pin_clearance)
        # solid confinement: everything outside the corridor is an obstacle
        outside = work_area - allowed
        for lname in metals:
            obstacles_by_layer[lname] = obstacles_by_layer.get(lname, db.Region()) + outside

    dbu_um = layout.dbu  # native layout units are already microns/dbu
    bbox_um = (die_box.left * dbu_um, die_box.bottom * dbu_um,
               die_box.right * dbu_um, die_box.top * dbu_um)

    virtual_pins_um = None
    if not corridor.is_empty():
        # virtual pins along the corridor give the Hanan grid coordinates inside it
        pin_layer = min(widths, key=widths.get, default=None) if widths else None
        pin_layer = pin_layer if pin_layer in metals else next(iter(metals), None)
        if pin_layer:
            rects_um = []
            pieces = corridor.decompose_trapezoids_to_region()
            for p in pieces.each():
                b = p.bbox()
                w, h = b.width(), b.height()
                if min(w, h) < pin_clearance:
                    continue  # sliver too small to safely hold a pin
                size = max(1, min(w, h, pin_clearance * 4))
                cx, cy = b.center().x, b.center().y
                vp = db.Box(cx - size // 2, cy - size // 2, cx - size // 2 + size, cy - size // 2 + size)
                rects_um.append((vp.left * dbu_um, vp.bottom * dbu_um, vp.right * dbu_um, vp.top * dbu_um))
            if rects_um:
                virtual_pins_um = {pin_layer: rects_um}

    corridor_topology_um = ([(x * dbu_um, y * dbu_um) for x, y in waypoints]
                             if waypoints else None)

    placement_path, ndr_path, lef_path = build_router_inputs(
        work_dir, net_name, islands, obstacles_by_layer, dbu_um, uu, bbox_um,
        widths, virtual_pins_um, corridor_topology_um, corridor_pitch,
        corridor_guide_weight)
    rc, log, routed_def, debug_lef = run_router(
        router_bin, layers_json, placement_path, lef_path, ndr_path,
        work_dir, uu, net_name, rsmt)
    if rc != 0 or not os.path.exists(routed_def):
        return {"status": "route_failed", "net": net_name, "log": log}

    inserted = paint_routed_def(layout, cell, routed_def, net_name, draw_by_name, uu)
    hier_lef = os.path.join(work_dir, "route_out", "TOP_CONC_0_interim_hier.lef")
    corridor_shapes = (paint_rsmt_corridor(layout, cell, routed_def, net_name, uu)
                        if rsmt and show_corridor else 0)
    corridor_wall_shapes = (paint_corridor_walls(layout, cell, hier_lef, net_name, dbu_um)
                             if rsmt and show_corridor else 0)
    obstacle_shapes = (paint_net_obstacles(layout, cell, debug_lef, draw_by_name, dbu_um)
                        if show_obstacles else 0)
    islands2, _ = net_islands(layout, cell, metals, labels, vias, net_name)
    return {"status": "completed" if len(islands2) == 1 else "still_broken",
            "net": net_name, "islands_before": len(islands),
            "islands_after": len(islands2), "shapes_inserted": inserted,
            "corridor_shapes": corridor_shapes,
            "corridor_wall_shapes": corridor_wall_shapes,
            "obstacle_shapes": obstacle_shapes,
            "layout": layout, "log": log}


def complete_net_from_points(layout, cell, layers_json, points, router_bin,
                              work_dir, uu=1000, margin_um=5.0, rsmt=True,
                              show_corridor=True, show_obstacles=True,
                              waypoints_layer_spec=WAYPOINT_LAYER,
                              corridor_pitch=None, corridor_guide_weight=None):
    """complete_net_on_layout for the net under the given world points."""
    metals, labels, vias, _draw, _widths = load_layers(layers_json)
    if not points:
        return {"status": "no_selection",
                "note": "select at least one shape on the broken net first"}

    names, unresolved = set(), 0
    for (x, y) in points:
        n = net_name_for_point(layout, cell, metals, labels, vias, x, y)
        if n is None:
            unresolved += 1
        else:
            names.add(n)

    if not names:
        return {"status": "no_net_found",
                "note": "none of the selected shapes reach a net-name label"}
    if len(names) > 1:
        return {"status": "ambiguous_selection", "nets": sorted(names),
                "note": "selection spans more than one net"}

    net_name = next(iter(names))
    result = complete_net_on_layout(layout, cell, layers_json, net_name,
                                     router_bin, work_dir, uu, margin_um, rsmt,
                                     show_corridor, show_obstacles,
                                     waypoints_layer_spec=waypoints_layer_spec,
                                     corridor_pitch=corridor_pitch,
                                     corridor_guide_weight=corridor_guide_weight)
    result["resolved_from_selection"] = True
    if unresolved:
        result["unresolved_points"] = unresolved
    return result


def all_net_names(layout, cell, labels):
    """Every net label text in the hierarchy."""
    names = set()
    for lname, lspec in labels.items():
        for (text, x, y) in all_label_hits(layout, cell, lspec):
            names.add(text)
    return sorted(names)


def complete_nets_on_layout(layout, cell, layers_json, net_names, router_bin,
                             work_dir, uu=1000, margin_um=5.0, rsmt=True,
                             show_corridor=True,
                             waypoints_layer_spec=WAYPOINT_LAYER,
                             corridor_pitch=None, corridor_guide_weight=None):
    """Route several nets in one router pass, each optionally along its own drawn corridor."""
    metals, labels, vias, draw_by_name, widths = load_layers(layers_json)

    results = {}
    per_net_islands = {}
    all_by_layer = None
    for net_name in net_names:
        islands, layer_metal = net_islands(layout, cell, metals, labels, vias, net_name)
        all_by_layer = all_by_layer or layer_metal
        if not islands:
            results[net_name] = {"status": "not_found", "net": net_name}
        elif len(islands) == 1:
            results[net_name] = {"status": "already_connected", "net": net_name}
        else:
            per_net_islands[net_name] = islands

    if not per_net_islands:
        return results

    bbox = None
    for islands in per_net_islands.values():
        for island in islands:
            for region in island.values():
                b = region.bbox()
                bbox = b if bbox is None else bbox + b

    def island_centers(islands):
        centers = []
        for island in islands:
            for region in island.values():
                b = region.bbox()
                centers.append(((b.left + b.right) / 2, (b.bottom + b.top) / 2))
        return centers

    def nearest_dist(centers, path):
        best = None
        for (px, py) in path:
            for (cx, cy) in centers:
                d = abs(px - cx) + abs(py - cy)
                if best is None or d < best:
                    best = d
        return best if best is not None else float("inf")

    dbu_um = layout.dbu
    net_corridor_topology_um = {}
    paths = read_all_waypoint_paths(layout, cell, waypoints_layer_spec) if waypoints_layer_spec else []
    if paths:
        # greedy nearest-first assignment, one path per net at most -- two
        # nets wanting the same drawn corridor need two copies of it.
        candidates = []
        for net_name, islands in per_net_islands.items():
            centers = island_centers(islands)
            for pi, path in enumerate(paths):
                candidates.append((nearest_dist(centers, path), net_name, pi))
        candidates.sort(key=lambda t: t[0])
        assigned_nets, used_paths = set(), set()
        for dist, net_name, pi in candidates:
            if net_name in assigned_nets or pi in used_paths:
                continue
            assigned_nets.add(net_name)
            used_paths.add(pi)
            net_corridor_topology_um[net_name] = [(x * dbu_um, y * dbu_um) for x, y in paths[pi]]
        for pi in used_paths:
            xs = [p[0] for p in paths[pi]]
            ys = [p[1] for p in paths[pi]]
            bbox = bbox + db.Box(min(xs), min(ys), max(xs), max(ys))

    margin = round(margin_um / layout.dbu)
    die_box = bbox.enlarged(margin, margin)
    work_area = db.Region(die_box)

    target_shapes = db.Region()
    for islands in per_net_islands.values():
        for island in islands:
            for region in island.values():
                target_shapes += region
    obstacles_by_layer = {lname: (region & work_area) - target_shapes
                           for lname, region in all_by_layer.items()}

    bbox_um = (die_box.left * dbu_um, die_box.bottom * dbu_um,
               die_box.right * dbu_um, die_box.top * dbu_um)
    nets_for_router = [(net_name, islands, net_corridor_topology_um.get(net_name))
                        for net_name, islands in per_net_islands.items()]
    placement_path, ndr_path, lef_path = build_router_inputs_multi(
        work_dir, nets_for_router, obstacles_by_layer, dbu_um, uu, bbox_um,
        widths, corridor_pitch, corridor_guide_weight)

    # HANAN_DEBUG_NET=1 dumps a debug LEF for every net
    rc, log, routed_def, _debug_lef = run_router(
        router_bin, layers_json, placement_path, lef_path, ndr_path,
        work_dir, uu, "1", rsmt)
    if rc != 0 or not os.path.exists(routed_def):
        for net_name in per_net_islands:
            results[net_name] = {"status": "route_failed", "net": net_name, "log": log}
        return results

    for net_name, islands in per_net_islands.items():
        inserted = paint_routed_def(layout, cell, routed_def, net_name, draw_by_name, uu)
        corridor_shapes = (paint_rsmt_corridor(layout, cell, routed_def, net_name, uu)
                            if rsmt and show_corridor else 0)
        islands2, _ = net_islands(layout, cell, metals, labels, vias, net_name)
        results[net_name] = {
            "status": "completed" if len(islands2) == 1 else "still_broken",
            "net": net_name, "islands_before": len(islands),
            "islands_after": len(islands2), "shapes_inserted": inserted,
            "corridor_shapes": corridor_shapes,
            "had_corridor": net_name in net_corridor_topology_um,
        }
    results["layout"] = layout
    results["log"] = log
    return results


def complete_net(gds_path, layers_json, net_name, router_bin, work_dir,
                  top_cell=None, uu=1000, margin_um=5.0, rsmt=True,
                  show_corridor=True, show_obstacles=True):
    """File-based wrapper around `complete_net_on_layout` for CLI/test use --
    reads `gds_path` fresh and returns the (modified, in-memory) layout for
    the caller to write out; never touches the file on disk itself."""
    layout = db.Layout()
    layout.read(gds_path)
    cell = layout.top_cell() if top_cell is None else layout.cell(top_cell)
    return complete_net_on_layout(layout, cell, layers_json, net_name,
                                   router_bin, work_dir, uu, margin_um, rsmt,
                                   show_corridor, show_obstacles)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-g", "--gds", required=True)
    ap.add_argument("-l", "--layers", required=True)
    ap.add_argument("-n", "--net", required=True)
    ap.add_argument("-o", "--out", required=True, help="GDS to write the result to")
    ap.add_argument("--router", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "hanan_router"))
    ap.add_argument("--top-cell", default=None)
    ap.add_argument("--uu", type=int, default=1000)
    ap.add_argument("--margin", type=float, default=5.0)
    ap.add_argument("--no-rsmt", action="store_true",
                     help="don't confine routing to an RSMT corridor (on by default)")
    ap.add_argument("--no-corridor", action="store_true",
                     help=f"don't paint the RSMT corridor onto layer {CORRIDOR_LAYER} (on by default)")
    ap.add_argument("--no-obstacles", action="store_true",
                     help="don't paint the obstacles the router actually saw for this net, onto "
                          f"each obstacle metal's own layer at datatype {OBSTACLE_DATATYPE} (on by default)")
    a = ap.parse_args()

    with tempfile.TemporaryDirectory() as work_dir:
        result = complete_net(a.gds, a.layers, a.net, a.router, work_dir,
                               top_cell=a.top_cell, uu=a.uu, margin_um=a.margin,
                               rsmt=not a.no_rsmt, show_corridor=not a.no_corridor,
                               show_obstacles=not a.no_obstacles)
    layout = result.pop("layout", None)
    print(json.dumps(result, indent=2, default=str))
    if layout is not None:
        layout.write(a.out)
        print(f"wrote {a.out}")
    sys.exit(0 if result["status"] == "completed" else 1)


if __name__ == "__main__":
    main()
