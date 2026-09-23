#!/usr/bin/env python3
"""Derive the same (draw, via) mapping check_nets.py's layer_map() reads out
of a hand-maintained sky130.layers.json, straight from the PDK's own KLayout
technology file (<PDK_ROOT>/libs.tech/klayout/tech/sky130A.lyt) instead.

Cross-checked against sky130.layers.json for every M1-M5/li1/V0-V4 entry:
GDS layer number, draw datatype, and via stack (which two metals a cut
connects) all match exactly -- the .lyt's <connectivity> block is a complete,
authoritative substitute for that part of layers.json, not an approximation.
layers.json still carries data a .lyt doesn't (routing width/pitch/
resistance for the router's own cost model), so this replaces only the
layer-identity/via-stack lookup check_nets.trace() and this geometric
extractor actually use -- not the whole file.
"""
import re
import xml.etree.ElementTree as ET


# .lyt names -> sky130.layers.json's own names, for the layers the router
# actually routes on (poly/licon/capm/cap2m have no routing-layer entry in
# layers.json at all -- device/cap-level only, out of scope here).
LYT_TO_LAYERS_JSON_NAME = {
    "li": "li1", "mcon": "V0",
    "met1": "M1", "via1": "V1",
    "met2": "M2", "via2": "V2",
    "met3": "M3", "via3": "V3",
    "met4": "M4", "via4": "V4",
    "met5": "M5",
}


def lyt_layer_map(lyt_path, rename=True):
    """(draw, via) in exactly check_nets.layer_map()'s own shape:
    draw: {(gds layer, datatype): name}
    via:  {(gds layer, datatype): (lower_metal_name, upper_metal_name)}
    """
    root = ET.parse(lyt_path).getroot()
    conn = root.find("connectivity")
    if conn is None:
        raise ValueError(f"no <connectivity> block in {lyt_path}")

    # <symbols>name='68/20+68/5-68/14-68/15'</symbols> -- the layer's
    # *primary* draw term is consistently the first (layer/datatype) pair
    # before any +/- operator, matching layers.json's own "Draw" datatype
    # for every layer checked above.
    symbol_key = {}   # name -> (gds layer, datatype)
    for el in conn.findall("symbols"):
        text = el.text or ""
        m = re.match(r"\s*(\w+)\s*=\s*'?(\d+)/(\d+)", text)
        if not m:
            continue
        name, layer, dt = m.group(1), int(m.group(2)), int(m.group(3))
        symbol_key[name] = (layer, dt)

    draw = {key: (LYT_TO_LAYERS_JSON_NAME.get(name, name) if rename else name)
            for name, key in symbol_key.items()}

    # <connection>met1,via1,met2</connection> -- via1 is the cut between
    # met1 and met2 (lower,upper by declaration order in the chain).
    via = {}
    for el in conn.findall("connection"):
        parts = (el.text or "").split(",")
        if len(parts) != 3:
            continue
        lo, cut, hi = (p.strip() for p in parts)
        key = symbol_key.get(cut)
        if key and lo in symbol_key and hi in symbol_key:
            if rename:
                lo = LYT_TO_LAYERS_JSON_NAME.get(lo, lo)
                hi = LYT_TO_LAYERS_JSON_NAME.get(hi, hi)
            via[key] = (lo, hi)

    return draw, via


if __name__ == "__main__":
    import sys
    draw, via = lyt_layer_map(sys.argv[1] if len(sys.argv) > 1
                               else "/cad/share/pdk/sky130A/libs.tech/klayout/tech/sky130A.lyt")
    print(f"{len(draw)} drawn layer(s), {len(via)} via stack(s)")
    for key, name in sorted(draw.items()):
        tag = f"  (via: {via[key][0]} <-> {via[key][1]})" if key in via else ""
        print(f"  {key} -> {name}{tag}")
