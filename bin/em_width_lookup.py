#!/usr/bin/env python3
"""Current -> required wire width, per metal layer, for sky130.

SkyWater does not publish official electromigration rules for this PDK.
The limits below are the community-maintained *estimate* from the widely
used "sky130 Layer Resistances and Capacitances" reference sheet
(https://docs.google.com/spreadsheets/d/1N9To-xTiA7FLfQ1SNzWKe-wMckFEXVE9WPkPPjYkaxE,
tab "Layer resistances and capacitances", "Electromigration rules
(estimated)" section) -- explicitly labelled there as estimated, derived
from the metal's material and cross-section, not a foundry-qualified
number. Treat this as a reasonable planning figure, not a signoff rule:
replace EM_MA_PER_UM with real numbers from a qualified source before
using this for anything beyond a prototype/demo.

li1 (local interconnect) has no estimate in that source at all -- it is
intentionally left out of EM_MA_PER_UM, not defaulted to some other
layer's number, so a net that only touches li1 falls back to the PDK's
own minimum width (MIN_WIDTH_UM) rather than a fabricated EM figure.

sky130 metal (li1, M1-M5) has no LEF PREFERRED DIRECTION -- every layer
is bidirectional -- so, unlike PDKs where a layer's own preferred
direction would narrow which layers are even candidates for a given
net's path, no direction filtering happens here.
"""
import json

EM_MA_PER_UM = {
    # layer: estimated DC electromigration current-density limit, mA/um
    "M1": 0.65,
    "M2": 0.65,
    "M3": 1.50,
    "M4": 1.50,
    "M5": 2.30,
}

LAYERS_JSON = "/data1/scratch/sky130-benchmarks/bench/primitives/lib/sky130.layers.json"


def min_widths_um(layers_json=LAYERS_JSON):
    """{layer: PDK minimum routable width, um} for li1/M1-M5, from the
    router's own layers.json ("Width" field, router units -- UU=1000)."""
    data = json.load(open(layers_json))
    out = {}
    for e in data.get("Abstraction", []):
        name = e.get("Layer")
        w = e.get("Width")
        if name in ("li1", "M1", "M2", "M3", "M4", "M5") and w is not None:
            out[name] = w / 1000.0
    return out


def required_width_um(current_ma, layer, floors=None, derate=1.0):
    """Width (um) needed to carry `current_ma` on `layer` within the
    estimated EM limit, floored at the PDK's own minimum width for that
    layer. `derate` >1 adds margin (e.g. 1.3 for a 30% safety margin);
    <1 is refused -- derating a reliability limit downward defeats its
    purpose. Raises for a layer with no EM estimate (li1) unless the
    caller only wants the floor -- call min_widths_um()[layer] directly
    for that instead of pretending an EM number exists."""
    if derate < 1.0:
        raise ValueError(f"derate must be >= 1.0 (got {derate}); "
                          "a EM safety margin only ever widens the wire")
    if layer not in EM_MA_PER_UM:
        raise KeyError(f"no EM estimate for layer {layer!r} "
                        f"(have: {sorted(EM_MA_PER_UM)})")
    limit = EM_MA_PER_UM[layer]
    w = (current_ma * derate) / limit
    floor = (floors or min_widths_um()).get(layer, 0.0)
    return max(w, floor)


def widths_for_net(current_ma, layers=("M1", "M2", "M3", "M4", "M5"), derate=1.0):
    """{layer: required width um} for every layer in `layers` that has an
    EM estimate -- the direct input to an NDR file's per-net "widths"
    block. Layers without an estimate (li1) are silently skipped, not
    defaulted, so a caller doesn't route a real current over a width this
    table never actually justified."""
    floors = min_widths_um()
    out = {}
    for layer in layers:
        if layer in EM_MA_PER_UM:
            out[layer] = round(required_width_um(current_ma, layer, floors, derate), 4)
    return out


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("current_ma", type=float, help="DC current the net carries, mA")
    ap.add_argument("--derate", type=float, default=1.0, help="safety margin multiplier, >=1.0")
    a = ap.parse_args()
    for layer, w in widths_for_net(a.current_ma, derate=a.derate).items():
        print(f"{layer}: {w:.4f} um  (limit {EM_MA_PER_UM[layer]} mA/um, estimated)")
