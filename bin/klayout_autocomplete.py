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
    return [[b.left * dbu, b.bottom * dbu, b.right * dbu, b.top * dbu]
            for p in region.each_merged() for b in [p.bbox()]]


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
                         layout_dbu, uu, bbox_um, widths):
    """placement.json + LEF: one trivial one-pin leaf macro per island,
    instantiated exactly over that island's own anchor rect (see
    `island_anchor`) and fa_map'd to `net_name` -- this is what actually
    creates the net (`Netlist.cpp` only ever creates a net from an
    instance's `fa_map`, or from `global_signals`, which turned out to add
    an unwanted phantom module-boundary pin with no location and made the
    router treat the net as already trivially satisfied). NDR carries only
    obstacles (everything else in the working area) -- no `virtual_pins`,
    since a real per-island pin already gives the router what it needs.
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
               work_dir, uu, rsmt=True):
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
    and removes a real, observed defect, so it defaults on here."""
    out_dir = os.path.join(work_dir, "route_out")
    os.makedirs(out_dir, exist_ok=True)
    cmd = [router_bin, "-d", layers_json, "-p", placement_path,
           "-l", lef_path, "-uu", str(uu), "-ndr", ndr_path, "-o", out_dir]
    if rsmt:
        cmd.append("-rsmt")
    p = subprocess.run(cmd, capture_output=True, text=True)
    routed_def = os.path.join(out_dir, "TOP_CONC_0.def")
    return p.returncode, p.stdout + p.stderr, routed_def


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


def complete_net_on_layout(layout, cell, layers_json, net_name, router_bin,
                            work_dir, uu=1000, margin_um=5.0, rsmt=True):
    """The reusable core: find islands, route if broken, paint back into
    `cell` in place. Takes an already-open `db.Layout`/`pya.Layout` and
    `cell` directly -- this is what a GUI macro calls on the live, currently
    open layout, with no GDS export/reimport round trip; `complete_net`
    (below) is the thin file-based wrapper CLI/test use goes through.
    `rsmt` is passed straight to `run_router` -- see there for why it
    defaults on. Returns a status dict -- never raises for an ordinary "not
    broken" or "net not found" outcome, only for a real I/O or router
    failure."""
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
    margin = round(margin_um / layout.dbu)
    die_box = bbox.enlarged(margin, margin)
    work_area = db.Region(die_box)

    target_shapes = db.Region()
    for island in islands:
        for region in island.values():
            target_shapes += region
    obstacles_by_layer = {lname: (region & work_area) - target_shapes
                           for lname, region in all_by_layer.items()}

    dbu_um = layout.dbu  # native layout units are already microns/dbu
    bbox_um = (die_box.left * dbu_um, die_box.bottom * dbu_um,
               die_box.right * dbu_um, die_box.top * dbu_um)
    placement_path, ndr_path, lef_path = build_router_inputs(
        work_dir, net_name, islands, obstacles_by_layer, dbu_um, uu, bbox_um,
        widths)
    rc, log, routed_def = run_router(router_bin, layers_json, placement_path,
                                      lef_path, ndr_path, work_dir, uu, rsmt)
    if rc != 0 or not os.path.exists(routed_def):
        return {"status": "route_failed", "net": net_name, "log": log}

    inserted = paint_routed_def(layout, cell, routed_def, net_name, draw_by_name, uu)
    islands2, _ = net_islands(layout, cell, metals, labels, vias, net_name)
    return {"status": "completed" if len(islands2) == 1 else "still_broken",
            "net": net_name, "islands_before": len(islands),
            "islands_after": len(islands2), "shapes_inserted": inserted,
            "layout": layout, "log": log}


def complete_net_from_points(layout, cell, layers_json, points, router_bin,
                              work_dir, uu=1000, margin_um=5.0, rsmt=True):
    """Like `complete_net_on_layout`, but the net is derived from a set of
    world-space points -- typically one per shape the user selected in the
    GUI -- instead of being typed in: "select the pin shape and get the
    associated net" (see `net_name_for_point`). Distinguishes the ways this
    can fail to resolve to exactly one net from the underlying routing
    failure modes, since a caller (the GUI macro) needs to word them very
    differently -- "you selected nothing" is not "the net was already
    connected"."""
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
                                     router_bin, work_dir, uu, margin_um, rsmt)
    result["resolved_from_selection"] = True
    if unresolved:
        result["unresolved_points"] = unresolved
    return result


def complete_net(gds_path, layers_json, net_name, router_bin, work_dir,
                  top_cell=None, uu=1000, margin_um=5.0, rsmt=True):
    """File-based wrapper around `complete_net_on_layout` for CLI/test use --
    reads `gds_path` fresh and returns the (modified, in-memory) layout for
    the caller to write out; never touches the file on disk itself."""
    layout = db.Layout()
    layout.read(gds_path)
    cell = layout.top_cell() if top_cell is None else layout.cell(top_cell)
    return complete_net_on_layout(layout, cell, layers_json, net_name,
                                   router_bin, work_dir, uu, margin_um, rsmt)


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
    a = ap.parse_args()

    with tempfile.TemporaryDirectory() as work_dir:
        result = complete_net(a.gds, a.layers, a.net, a.router, work_dir,
                               top_cell=a.top_cell, uu=a.uu, margin_um=a.margin,
                               rsmt=not a.no_rsmt)
    layout = result.pop("layout", None)
    print(json.dumps(result, indent=2, default=str))
    if layout is not None:
        layout.write(a.out)
        print(f"wrote {a.out}")
    sys.exit(0 if result["status"] == "completed" else 1)


if __name__ == "__main__":
    main()
