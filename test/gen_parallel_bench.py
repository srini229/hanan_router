#!/usr/bin/env python3
"""Generate a heavy-but-disjoint routing benchmark for the parallel-routing test.

Each net lives in its own tall vertical band, well separated from the others, so
the nets are mutually non-overlapping and the router can batch them for parallel
routing. Each band is filled with a dense staggered field of obstacles, which
makes the per-net Hanan grid large and the A* search genuinely expensive -- so
the total routing work is big enough that parallelising it gives a clear,
measurable wall-clock speedup.

Usage: gen_parallel_bench.py <num_nets> <placement_out.json> <ndr_out.json>
"""
import json
import sys

N        = int(sys.argv[1]) if len(sys.argv) > 1 else 32
PL_OUT   = sys.argv[2] if len(sys.argv) > 2 else "parallel_bench.placement_verilog.json"
NDR_OUT  = sys.argv[3] if len(sys.argv) > 3 else "parallel_bench_ndr.json"

W      = 4000        # band width (route span)
BANDH  = 900         # band height (vertical room to weave)
PITCH  = 2200        # vertical pitch between bands (keeps bands disjoint)
PIN    = 32
NCOL   = 26          # obstacle columns per band
NROW   = 14          # obstacle rows per band
LAYERS = ["M1", "M2", "M3", "M4", "M5"]

insts = []
obs = {l: [] for l in LAYERS}
modH = N * PITCH + BANDH

for i in range(N):
    yb = i * PITCH + 100        # band bottom
    yt = yb + BANDH             # band top
    # pins at opposite corners so the pin bbox spans the whole band
    insts.append({"abstract_template_name": "PINCELL", "concrete_template_name": "PINCELL",
                  "fa_map": [{"actual": f"N{i}", "formal": "P"}], "instance_name": f"I_L{i}",
                  "transformation": {"oX": 0, "oY": yb, "sX": 1, "sY": 1}})
    insts.append({"abstract_template_name": "PINCELL", "concrete_template_name": "PINCELL",
                  "fa_map": [{"actual": f"N{i}", "formal": "P"}], "instance_name": f"I_R{i}",
                  "transformation": {"oX": W - PIN, "oY": yt - PIN, "sX": 1, "sY": 1}})
    # dense staggered brick field with a diagonal weaving channel left open
    bw = (W - 400) // NCOL
    bh = (BANDH - 120) // NROW
    for c in range(NCOL):
        for r in range(NROW):
            if (r + c) % NROW == c % NROW:   # leave the weave channel open
                continue
            x0 = 200 + c * bw + (15 if r % 2 else 0)
            y0 = yb + 60 + r * bh + (11 if c % 2 else 0)
            rect = [x0, y0, x0 + bw - 30, y0 + bh - 24]
            for l in LAYERS:
                obs[l].append(rect)

placement = {
    "global_signals": [],
    "leaves": [{"abstract_name": "PINCELL", "bbox": [0, 0, 32, 32], "concrete_name": "PINCELL",
                "terminals": [{"name": "P", "rect": [0, 0, 32, 32]}]}],
    "modules": [{"abstract_name": "PARBENCH", "bbox": [0, 0, W, modH],
                 "concrete_name": "PARBENCH_CONC_0", "instances": insts}],
}
ndr = [{"module": "PARBENCH_CONC_0", "obstacles": [{"shapes": obs}]}]

json.dump(placement, open(PL_OUT, "w"))
json.dump(ndr, open(NDR_OUT, "w"))
print(f"generated {N} disjoint heavy nets ({W}x{modH})")
