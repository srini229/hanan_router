#!/usr/bin/env python3
"""Autocomplete a broken net directly on a loaded KLayout design.

A net whose drawn metal (across the whole cell hierarchy, any depth) forms
more than one physically-connected island is "broken". This finds those
islands from the design's own net-name labels (no netlist needed), gives
each island its own trivial one-pin leaf device so the router has something
real to connect, declares every *other* net's metal in the working area an
obstacle, runs `hanan_router`, and paints the routed result back.

Written against `klayout.db` (the standalone, pip-installed binding) so it
is testable headlessly. `klayout.db` and `pya` (the binding available
*inside* the running KLayout application) expose the same classes with the
same API, so the GUI macro that drives this can just call straight into the
functions here -- the connectivity/routing logic itself never changes
between the two contexts.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

import klayout.db as db

EPS = 1  # dbu; a probe box this much larger than a point still reliably
         # registers as "interacting" with a zero-width touch (verified: a
         # truly zero-size probe box does not count as interacting with
         # anything in klayout, even a polygon it sits inside)


def load_layers(path):
    """From a sky130.layers.json-style file:
    metals: {name: (gds_layer, draw_datatype)} -- routable metal, virtual_pins
            and obstacles are always expressed on these
    labels: {name: (gds_layer, label_texttype)}  -- keyed by the *metal*
            layer name a label belongs to, same convention gds2placement.py
            uses (a layer's own "Label" datatype, not a separate layer)
    vias:   [(via_layer_name, gds_layer, via_datatype, layer_a_name, layer_b_name)]
    draw_by_name: {name: (gds_layer, datatype)} for *every* Draw-purpose
            layer, metal or via -- what a routed DEF's own RECT lines are
            named after, so painting the result back needs this, not just
            `metals`.
    widths: {name: nominal wire width, in the layers.json's own router-native
            units (matching its Pitch/Width fields directly, not microns)}
            for each metal layer -- a fragment's own pin needs to be sized to
            this, not to the fragment's full (possibly much larger) extent;
            see `island_anchor`.
    """
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
    """Index into `polygons` (a list of db.Polygon) containing (x, y), or
    None. A truly zero-size probe box does not register as "interacting"
    with anything in klayout, even a polygon it sits inside -- verified --
    so the probe is enlarged by EPS on each side."""
    probe = db.Region(db.Box(x - EPS, y - EPS, x + EPS, y + EPS))
    for i, poly in enumerate(polygons):
        if not db.Region(poly).interacting(probe).is_empty():
            return i
    return None


def _merged_islands(layout, cell, metals, vias):
    """(list of db.Polygon, {layer_name: db.Region}) -- every physically
    connected group of metal+via shapes anywhere in the hierarchy
    (independent of net identity), and each metal layer's own flattened
    geometry, reused by callers so they don't rescan the hierarchy."""
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
    """{layer_name: db.Region} per physically-disjoint island of `net_name`,
    found from every label reading `net_name` across the whole hierarchy.
    A net with exactly one island is complete; more than one is broken.
    Also returns `all_by_layer`, the full per-layer flattened metal (reused
    by the caller to build obstacles without re-scanning the hierarchy)."""
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
    """The net-name label reachable from the physically-connected island
    containing world point (x, y) -- the reverse of net_islands: "what net
    is this shape part of", for driving completion from a GUI selection
    instead of a typed name. None if the point isn't on any metal, or its
    island reaches no label at all (a genuinely bare, unlabeled fragment --
    which net it belongs to can't be recovered this way; the plan's own
    fallback, geometric union-find, still finds *that* it's broken, just
    not what to call it)."""
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
    """[[x0,y0,x1,y1], ...] um, one per RECT a LEF OBS block can hold --
    LEF has no polygon-with-hole primitive, so a merged polygon that has a
    hole (a region built as "everything except some enclosed area", e.g. a
    corridor's own U-shaped keep-in -- see `complete_net_on_layout`'s
    `corridor_layer_spec`) cannot go through `each_merged()` and `.bbox()`
    directly: a hole's bbox is the same as its surrounding polygon's, so
    that path silently reports the *whole* bbox as solid obstacle,
    including the hole -- verified: it turns a correctly-computed
    corridor-shaped keep-in region into one obstacle rectangle covering
    the corridor's own path along with everything outside it, walling the
    route in completely. `decompose_trapezoids_to_region()` splits any
    region, holes included, into plain non-overlapping rectangles/
    trapezoids first, so this always reports the true shape, not just an
    outer bbox -- verified against the same hole case."""
    return [[b.left * dbu, b.bottom * dbu, b.right * dbu, b.top * dbu]
            for p in region.decompose_trapezoids_to_region().each() for b in [p.bbox()]]


def island_anchor(per_layer, widths):
    """(layer_name, db.Box) -- one small, wire-scale representative rect for
    an island, on whichever layer carries the most area in it. A router pin
    only needs *somewhere* real to escape from that's part of the fragment;
    the island is already internally connected by definition (that's what
    makes it one island, not several), so unlike `virtual_pins` -- which the
    router requires an *already-existing* multi-pin net to attach to
    (`Netlist.cpp`, `Module::addVirtualPin` looks the net up by name and
    silently no-ops otherwise) -- a single real pin per island, wired
    through an ordinary one-pin leaf instance, is both
    sufficient and is what actually makes the net exist in the first place.

    The returned rect is deliberately *not* the fragment's own full extent:
    a pin exactly as large as a whole (possibly big, blob-shaped) fragment
    gives the router's escape-point search nowhere to stand -- verified
    empirically, "pruned N blocked escape point(s)" and a failed search
    every time, on a fragment as small as a single 1x1um box. A pin sized to
    the layer's own nominal wire width, centered on the fragment and clamped
    to fit inside it, behaves exactly like every other pin the router
    already routes to correctly."""
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
                         corridor_topology_um=None, corridor_pitch=None):
    """placement.json + LEF: one trivial one-pin leaf macro per island,
    instantiated exactly over that island's own anchor rect (see
    `island_anchor`) and fa_map'd to `net_name` -- this is what actually
    creates the net (`Netlist.cpp` only ever creates a net from an
    instance's `fa_map`, or from `global_signals`, which turned out to add
    an unwanted phantom module-boundary pin with no location and made the
    router treat the net as already trivially satisfied). NDR carries
    obstacles (everything else in the working area) and, when given,
    `virtual_pins_um` -- extra `{layer: [(x0,y0,x1,y1), ...]}` fragments in
    microns that `net_name` must also route through (`Netlist.cpp`'s
    `addVirtualPin`; requires the net already have a real pin, which the
    per-island leaves above always supply). A real per-island pin is
    normally all a caller needs; `complete_net_on_layout`'s
    `corridor_layer_spec` uses this for a second reason beyond "must touch
    this point": a solid corridor-confinement obstacle can leave the
    Hanan grid with no coordinate strictly inside a narrow corridor band
    (verified: every track candidate comes from an obstacle or pin edge,
    so a bare band between two obstacle walls with nothing else in it is
    literally unroutable, "no target is reachable from any source", even
    though the space is geometrically wide open) -- a virtual pin's own
    rectangle inside that band supplies the missing edge, the same way a
    device pin would.
    `bbox_um` is the module's die area in microns -- the router clips
    everything to it, so it must cover the working area, not be a
    placeholder. `layout_dbu` (microns/unit) and `uu` (the router's own
    units-per-micron, i.e. its `-uu`) may differ -- LEF/placement
    coordinates are in the router's own scaled units, converted here, not
    assumed equal to the layout's. Returns (placement_path, ndr_path, lef_path)."""
    to_router_units = lambda v: round(v * layout_dbu * uu)
    # `widths` is in the layers.json's own router-native units (matches its
    # Pitch/Width fields, i.e. microns * uu) -- convert to the layout's own
    # dbu so island_anchor can compare it directly against fragment extents.
    widths_layout_units = {name: w / uu / layout_dbu for name, w in widths.items()}

    leaves, instances, lef_macros = [], [], []
    for i, per_layer in enumerate(islands):
        lname, box = island_anchor(per_layer, widths_layout_units)
        macro = f"FRAG{i}"
        w = to_router_units(box.width())
        h = to_router_units(box.height())
        # A pin exactly filling its macro (no margin at all) leaves the
        # router no room to generate an escape point off its boundary --
        # verified: the router reports "pruned N blocked escape point(s)"
        # and fails to route at all when tried without this. Centering the
        # pin in a macro margin.max(w, h) larger on every side gives it the
        # same generous clearance the working sky130-scale fixtures used.
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
            "bbox": [round(v * uu) for v in bbox_um],  # bbox_um is already
                                                        # microns, not raw
                                                        # layout dbu -- do
                                                        # not also multiply
                                                        # by layout_dbu here
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
    if virtual_pins_um or corridor_topology_um or corridor_pitch:
        net_entry = {"name": net_name}
        if virtual_pins_um:
            net_entry["virtual_pins"] = [{lname: [list(r) for r in rects]}
                                          for lname, rects in virtual_pins_um.items()]
        if corridor_topology_um:
            net_entry["corridor_topology"] = [list(p) for p in corridor_topology_um]
        if corridor_pitch:
            net_entry["corridor_pitch"] = corridor_pitch
        ndr[0]["nets"] = [net_entry]

    placement_path = os.path.join(work_dir, "autocomplete.placement_verilog.json")
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
                               uu, bbox_um, widths, corridor_pitch=None):
    """Like `build_router_inputs`, but for several nets sharing one router
    pass instead of one net per call -- the actual entry point for "route
    these nets together, each optionally on its own drawn corridor".
    `nets` is `[(net_name, islands, corridor_topology_um_or_None), ...]`.
    Kept as a separate function rather than folding multi-net support into
    `build_router_inputs` itself: that function is exercised directly by
    the existing test suite and other callers, and a net-count-dependent
    branch inside it risks a regression there for no benefit -- the
    per-island leaf/instance/LEF-writing logic below is copied, not
    shared, on purpose. `corridor_pitch` is one value applied to every net
    in this batch that has a drawn corridor (the GUI dialog asks for one
    number per routing run, not one per net); the NDR schema itself
    supports a different pitch per net if a future caller ever needs
    that, this one just doesn't expose it.
    Returns (placement_path, ndr_path, lef_path)."""
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
        if corridor_topology_um or corridor_pitch:
            net_entry = {"name": net_name}
            if corridor_topology_um:
                net_entry["corridor_topology"] = [list(p) for p in corridor_topology_um]
            if corridor_pitch:
                net_entry["corridor_pitch"] = corridor_pitch
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

    placement_path = os.path.join(work_dir, "autocomplete.placement_verilog.json")
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
    """`rsmt` confines the router's search to a corridor around the true
    rectilinear Steiner minimal tree over the net's own pins
    (`hanan_router -rsmt`). For the handful of pins a GUI completion ever
    deals with, the raw wirelength difference this makes is small -- both
    modes land within about 1% of the true RSMT length on the fixtures
    this was checked against -- but *without* it the router's default
    net-ordering can still pick a technically-tied-cost path that climbs
    an extra metal layer and back down for no reason a straight run on the
    layer the pins are already on would have served just as well: a plain
    two-pin connection with nothing in the way came back as a single flat
    M1 wire with `-rsmt` and an M1->M2->M3->M2->M1 detour (4 unneeded vias)
    without it, on the exact same pins. On a whole-chip multi-net run
    `-rsmt` is a real search-space restriction with its own tradeoffs, but
    for one net's worth of pins at GUI-interactive scale it costs nothing
    and removes a real, observed defect, so it defaults on here.

    `HANAN_DEBUG_NET=net_name` makes the router dump `net_TOP_CONC_0_
    <net_name>.lef` -- a LEF holding exactly the pins and the obstacles it
    saw for this net *before* routing (see `Net::route`/`Placement.cpp`'s
    `applyDebug`) -- always on, since it's one small file and it is the
    only direct way to confirm the obstacles a caller declared actually
    reached the router unchanged (`net_debug_obstacles`/
    `paint_net_obstacles` below read it back). Runs with `cwd=work_dir` so
    that file (and the router's own `route.log`/`err.log`, which it always
    writes by bare relative name) land somewhere the caller controls
    instead of wherever the calling process's cwd happened to be."""
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


USER_CORRIDOR_LAYER = (997, 0)  # scratch layer a person draws a routing
                                 # corridor on, by hand, in the KLayout GUI --
                                 # distinct from CORRIDOR_LAYER (999) and
                                 # CORRIDOR_WALL_LAYER (998), which are the
                                 # router's *own* -rsmt corridor, painted back
                                 # for inspection, never drawn by a person.

WAYPOINT_LAYER = (996, 0)  # scratch layer a person draws an ordered path
                            # (KLayout's own multi-point Path shape) on, to
                            # mark a routing topology by clicking points --
                            # see `read_waypoints`. Distinct from
                            # USER_CORRIDOR_LAYER (997), which is a filled
                            # region, not an ordered point sequence.


def read_all_waypoint_paths(layout, cell, layer_spec=WAYPOINT_LAYER):
    """[[(x,y), ...], ...] world coordinates, one list per Path shape drawn
    on `layer_spec` in `cell`, each in click order (see `read_waypoints`
    for the single-path case this generalizes). Draw one path per net you
    want to guide when routing several nets together -- `complete_nets_on_
    layout` matches each net to whichever path is nearest it, so paths
    don't need to be labelled or otherwise tied to a specific net by
    anything other than proximity."""
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


def read_waypoints(layout, cell, layer_spec=WAYPOINT_LAYER):
    """[(x,y), ...] world coordinates, in the order a person clicked them,
    from the first Path shape found on `layer_spec` in `cell` -- KLayout's
    own multi-point path/ruler shape already carries point order, so no
    separate click-sequence capture is needed: draw a path with the Path
    tool, each vertex is a waypoint in order drawn. Empty, not an error,
    if nothing's drawn there. Only the first path shape found is used --
    draw exactly one path per corridor (`read_all_waypoint_paths` reads
    every path, for routing several nets at once)."""
    paths = read_all_waypoint_paths(layout, cell, layer_spec)
    return paths[0] if paths else []


def user_corridor_region(layout, cell, layer_spec=USER_CORRIDOR_LAYER):
    """Whatever a person drew on `layer_spec` (any depth -- normally just
    the top cell, but a corridor sketched inside a sub-cell still counts),
    merged into one Region. Empty, not an error, if nothing's drawn there;
    callers treat that as "no corridor given", not a failure."""
    idx = layout.find_layer(*layer_spec)
    if idx is None:
        return db.Region()
    r = db.Region(db.RecursiveShapeIterator(layout, cell, idx))
    r.merge()
    return r


CORRIDOR_LAYER = (999, 0)  # not used by any real sky130 (or other) layers.json
                            # layer number seen so far -- purely a debug
                            # visualization layer, never a routable one.
CORRIDOR_WALL_LAYER = (998, 0)  # the corridor's own boundary-wall obstacles
                                 # (what `-rsmt` actually adds to the router as
                                 # a keepout, not just the band area) -- a
                                 # route can stay inside CORRIDOR_LAYER's
                                 # bands and still be blocked by one of these
                                 # walls; distinct from CORRIDOR_LAYER so the
                                 # two can be toggled independently.


def lef_obs_layer_rects(lef_path, layer_name):
    """[(x0, y0, x1, y1) microns] for one named OBS layer in a router-written
    LEF (`Module::writeLEF`'s hierarchical interim LEF, `TOP_CONC_0_interim_
    hier.lef` -- distinct from `Net::writeLEF`'s per-net debug dump that
    `net_debug_obstacles` reads). Empty, not an error, if the file or that
    layer isn't present -- e.g. routed without `-rsmt`, or the router
    predates this OBS layer."""
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
    """Paint the `-rsmt` corridor's actual boundary-wall obstacles -- what
    `Net::route()` adds to the router as a keepout around `CORRIDOR_LAYER`'s
    bands -- onto their own scratch layer. These walls are what can block a
    port pair even while every routed shape still ends up inside the
    corridor's own area: the wall is a real obstacle in its own right, not
    just an outline of where the bands are. Reads `net_TOP_CONC_0_<net>`'s
    sibling `TOP_CONC_0_interim_hier.lef` (written unconditionally by
    `Module::writeLEF`, not gated behind `HANAN_DEBUG_NET` -- that env var
    only controls the *per-net pre-route snapshot* `net_debug_obstacles`
    reads, a different file entirely)."""
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
    """Paint the `-rsmt` search corridor the router actually confined
    `net_name` to (a `<net>_RSMT_CORRIDOR` pseudo-net the DEF carries
    whenever `-rsmt` was used) onto a scratch layer, so it's visible
    alongside the routed wires -- e.g. to see whether a suboptimal topology
    genuinely had no better path inside the corridor, or whether the
    corridor itself already covered a shorter one the port-pairing missed.
    A DEF without that pseudo-net (routed without `-rsmt`) paints nothing
    and returns 0, not an error."""
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
    """Insert the router's new geometry for `net_name` directly into `cell`
    (metal *and* via-cut layers -- `draw_by_name` covers both, keyed exactly
    as the DEF's own RECT lines name them, since a via's layers.json "Layer"
    entry is what the router echoes there). `dbu` is the router's own
    micron-per-unit (matches -uu); the layout's own dbu may differ, so
    coordinates are rescaled, not assumed equal."""
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
    """{layer_name: [(x0, y0, x1, y1) microns]} from a `net_TOP_CONC_0_
    <net>.lef` debug dump (see `run_router`) -- the OBS block only, "BBOX"
    (the net's own bounding box, not a real obstacle) excluded. Empty dict,
    not an error, if the file doesn't exist (`HANAN_DEBUG_NET` unsupported
    by an older router, or the OBS block came back empty)."""
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


OBSTACLE_DATATYPE = 98  # paired with each metal's own GDS layer number (not
                         # CORRIDOR_LAYER's dummy number) so toggling one
                         # metal's obstacle overlay in the Layers panel sits
                         # right next to that metal's own routed geometry.


def paint_net_obstacles(layout, cell, lef_path, draw_by_name, dbu,
                         datatype=OBSTACLE_DATATYPE):
    """Paint exactly the obstacles `run_router`'s `HANAN_DEBUG_NET` dump
    says the router saw for this net -- before any routing happened -- onto
    each obstacle metal's own GDS layer number at `datatype`, so "did my
    obstacle actually reach the router" is a direct visual diff against the
    real drawn shape on the same layer, not a guess. A layer name the debug
    LEF mentions that isn't in `draw_by_name` (shouldn't happen -- the
    router only ever echoes layer names from `layers.json`) is skipped
    rather than raising, consistent with `paint_routed_def`."""
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
                            waypoints_layer_spec=None, corridor_pitch=None):
    """The reusable core: find islands, route if broken, paint back into
    `cell` in place. Takes an already-open `db.Layout`/`pya.Layout` and
    `cell` directly -- this is what a GUI macro calls on the live, currently
    open layout, with no GDS export/reimport round trip; `complete_net`
    (below) is the thin file-based wrapper CLI/test use goes through.
    `rsmt` is passed straight to `run_router` -- see there for why it
    defaults on. `show_corridor` (only meaningful with `rsmt`) additionally
    paints the RSMT search corridor onto a scratch layer (`CORRIDOR_LAYER`)
    -- a debug aid, on by default while the router's own MST-vs-Steiner
    topology gap is still being characterized; a caller happy to trust the
    routed result without inspecting the corridor can pass `False`. Returns
    a status dict -- never raises for an ordinary "not broken" or "net not
    found" outcome, only for a real I/O or router failure.

    `corridor_layer_spec`, when given, confines the route to whatever a
    person drew on that layer (see `user_corridor_region`) instead of --
    or, if `rsmt` is also left on, on top of -- the router's own
    RSMT-derived corridor: read the drawn shape, widen every *other*
    metal layer's obstacle set to also block everything in the work area
    outside it, on every layer, not just where real metal already sits
    (a plain "avoid other nets' metal" obstacle never blocks empty space).
    An empty drawn layer (nothing sketched) is silently treated as no
    corridor at all, not an error -- draw nothing, get the router's
    default behavior. `corridor_margin_um` grows the drawn shape before
    using it, e.g. to give a von-Neumann pixel-perfect sketch some real
    clearance to route/via in.

    `waypoints_layer_spec`, when given, reads an ordered click-path (see
    `read_waypoints`) and routes it via the router's *native*
    `corridor_topology` NDR field instead: `Net::rsmtCorridor()` builds the
    keepout from just the clicked path -- consecutive waypoints joined by
    the same Manhattan-L bands and pitch margin the auto -rsmt tree uses --
    rather than a Python-side obstacle fill. Every pin (any count) gets its
    own bloated bubble regardless of topology source, so this works for
    N-fragment nets too: the path just needs to pass near enough each pin
    to reach it, not touch it exactly or act as an explicit endpoint. This
    is the preferred way to hand the router a corridor -- it reuses
    proven, already-working machinery (same retry-widening ladder, same
    wall construction) instead of reimplementing confinement here, and
    doesn't hit the coordinate-sparsity trap a solid "obstacle = everything
    outside" fill can on a sparse layout (`corridor_layer_spec` above is
    kept for a filled-region sketch, but prefer this for a clicked path).
    Fewer than 2 waypoints (nothing drawn, or a single click) is passed
    through unused, falling back to the net's normal routing.

    `corridor_pitch`, when given, is written as this net's own NDR
    "corridor_pitch" -- how many multiples of the routing layer's pitch
    the corridor (auto or drawn alike) bloats by, replacing the router's
    own default (`Net.cpp`'s `RSMT_CORRIDOR_PITCHES`, 4) for this net
    only. `None` leaves the router's default in place."""
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
            # grown *before* it feeds the work-area bbox below -- sizing
            # it afterwards let a grown corridor stick out past the very
            # work area meant to contain it.
            corridor = corridor.sized(round(corridor_margin_um / layout.dbu))
        # the work area has to cover wherever the corridor actually goes,
        # not just the islands -- a detour outside the islands' own
        # bounding box is exactly the point of drawing one.
        bbox = bbox + corridor.bbox()

    waypoints = read_waypoints(layout, cell, waypoints_layer_spec) if waypoints_layer_spec else []
    if waypoints:
        # same reasoning as the drawn-region corridor above: the work
        # area (and so the router's own die box, and so every obstacle
        # this function clips to it) has to cover wherever the clicked
        # path actually goes, or a detour past the islands' own bounding
        # box gets silently cut off before the router ever sees it --
        # verified: a path drawn well outside the default margin came
        # back with the corridor's own painted geometry capped at the
        # old work area's edge, nowhere near the actual clicked point.
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
        # a pin's own island, *padded* by a fixed escape clearance, is
        # always allowed -- not just the island's bare footprint. Without
        # this pad, "outside corridor" starts flush against the pin's own
        # edge on any side the corridor doesn't happen to extend past,
        # leaving the escape-point search no legal direction to stand in
        # at all (verified: an unpadded union here reproduces the
        # router's "no possible escape" failure on every pin, even for a
        # corridor that visibly contains both islands).
        pin_clearance = max(round(1.0 / layout.dbu), 1)
        allowed = corridor + target_shapes.sized(pin_clearance)
        # solid confinement: everything in the work area outside the
        # corridor (+ pin clearance) is a real obstacle, on every layer --
        # not just a thin wall at the boundary. A thin wall alone (matching
        # -rsmt's own outlineBoxes()) leaves the far side of it completely
        # open, so on a design where nothing else already blocks the
        # direct path, the router just goes straight through and ignores
        # the drawn detour entirely -- verified on a fixture with no other
        # obstacle in the way. A real design usually has other nets'
        # metal doing that blocking already; a hand-drawn corridor can't
        # assume that, so it has to supply its own.
        outside = work_area - allowed
        for lname in metals:
            obstacles_by_layer[lname] = obstacles_by_layer.get(lname, db.Region()) + outside

    dbu_um = layout.dbu  # native layout units are already microns/dbu
    bbox_um = (die_box.left * dbu_um, die_box.bottom * dbu_um,
               die_box.right * dbu_um, die_box.top * dbu_um)

    virtual_pins_um = None
    if not corridor.is_empty():
        # Solid confinement alone starves the Hanan grid: the router only
        # ever places a track on a coordinate some obstacle or pin edge
        # actually introduces, and a corridor band with nothing but its
        # own two walls contributes exactly those two coordinates -- a
        # geometrically wide-open detour still comes back "no target is
        # reachable from any source" (verified: a 6um-wide corridor with
        # no other geometry inside it). A device pin fixes this by
        # existing; a `virtual_pins` fragment (Netlist.cpp's
        # `addVirtualPin`, the same mechanism a hand-drawn "pretend a wire
        # is already here" fragment uses) does the same job by design --
        # small real rectangles the route must also touch, one per
        # sizable piece of the drawn corridor so grid density exists
        # along its whole length, not just at its ends.
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
        widths, virtual_pins_um, corridor_topology_um, corridor_pitch)
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
                              corridor_pitch=None):
    """Like `complete_net_on_layout`, but the net is derived from a set of
    world-space points -- typically one per shape the user selected in the
    GUI -- instead of being typed in: "select the pin shape and get the
    associated net" (see `net_name_for_point`). Distinguishes the ways this
    can fail to resolve to exactly one net from the underlying routing
    failure modes, since a caller (the GUI macro) needs to word them very
    differently -- "you selected nothing" is not "the net was already
    connected". `waypoints_layer_spec` defaults to `WAYPOINT_LAYER`, not
    `None`: this is the GUI entry point, so a path drawn there before
    "Complete Net" is used automatically, no separate opt-in -- nothing
    drawn is still the ordinary no-corridor route, exactly as
    `complete_net_on_layout` already treats an empty layer."""
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
                                     corridor_pitch=corridor_pitch)
    result["resolved_from_selection"] = True
    if unresolved:
        result["unresolved_points"] = unresolved
    return result


def all_net_names(layout, cell, labels):
    """Every distinct net-name label text found anywhere in the hierarchy,
    across every metal layer's own label datatype -- the full set of
    names `net_islands`/`complete_net_on_layout` could be asked about,
    for "route everything" without the caller enumerating names by hand.
    `labels` is `load_layers()`'s own return, not a layers.json path."""
    names = set()
    for lname, lspec in labels.items():
        for (text, x, y) in all_label_hits(layout, cell, lspec):
            names.add(text)
    return sorted(names)


def complete_nets_on_layout(layout, cell, layers_json, net_names, router_bin,
                             work_dir, uu=1000, margin_um=5.0, rsmt=True,
                             show_corridor=True,
                             waypoints_layer_spec=WAYPOINT_LAYER,
                             corridor_pitch=None):
    """Route several nets in one router pass, each optionally following
    its own drawn corridor -- draw one Path per net you want to guide, all
    on `waypoints_layer_spec`; each net is matched to whichever path sits
    closest to it (`read_all_waypoint_paths`), so paths don't need to be
    labelled or otherwise tied to a net beyond proximity. A net you don't
    draw a path for just routes normally, same as any other multi-net run.

    Nets already in one piece, or with no matching label at all, are
    reported but never sent to the router. Every net actually routed
    shares one obstacle field -- everything else's real metal in the
    combined work area, minus every routed net's own islands -- so they
    can't obstruct each other but still respect a third net's real metal,
    the same as routing them one at a time would.

    Deliberately simpler than `complete_net_on_layout`: no
    `corridor_layer_spec` (filled-region) option here, and no
    `show_obstacles` debug painting -- `corridor_topology` is the
    preferred corridor mechanism there too (see its own docstring for
    why), and per-net obstacle-dump painting would need one debug LEF per
    net tracked separately, not worth the complexity for what this is
    for. Returns `{net_name: status_dict, ...}` (status dicts shaped like
    `complete_net_on_layout`'s), plus `"layout"`/`"log"` keys once any net
    actually reached the router."""
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
        widths, corridor_pitch)

    # "1", not a single net's name -- HANAN_DEBUG_NET=1 dumps a debug LEF
    # for every net in this run, matching what routing them one at a time
    # would each have produced.
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
