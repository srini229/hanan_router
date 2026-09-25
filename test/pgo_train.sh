#!/bin/bash
# training runs for the profile-guided build (make pgo): a few fixtures that exercise pattern routing, A*,
# reordering, exact symmetry and the symmetric fallback
# usage: pgo_train.sh ROUTER
R=$(realpath "$1"); IN=$(cd "$(dirname "$0")" && pwd); W=$(mktemp -d)
run() { local d=$W/$1; shift; mkdir -p "$d"; (cd "$d" && "$R" "$@" -o ./ > /dev/null 2>&1); }
run ota2 -d $IN/magical/layers.json -p $IN/magical/ota2/placement.json -l $IN/magical/ota2/cell.lef \
  -ndr $IN/magical/ota2_ndr.json -uu 1000 -reorder 30
run ota3 -d $IN/magical/layers.json -p $IN/magical/ota3/placement.json -l $IN/magical/ota3/cell.lef \
  -ndr $IN/magical/ota3/ndr.json -uu 1000 -reorder 30
run hoilee -d $IN/magical/layers.json -p $IN/magical/hoilee_affc/placement.json -l $IN/magical/hoilee_affc/cell.lef \
  -ndr $IN/magical/hoilee_affc/ndr.json -uu 1000 -reorder 30
run latch -d $IN/layers_sky130_bench.json -p $IN/sym_latch/placement.json -l $IN/sym_latch/combined.lef \
  -ndr $IN/sym_latch/route_ndr.json -uu 1000 -reorder 30
run halo -d $IN/layers_sky130_bench.json -p $IN/halofallback.netlist.json -l $IN/halofallback.lef \
  -ndr $IN/halofallback_ndr.json -uu 1000 -reorder 30
run maze -d $IN/layers.json -p $IN/maze30.netlist.json -l $IN/m1adj_escape.lef -ndr $IN/maze30_ndr.json
run net30 -d $IN/layers.json -p $IN/net30.netlist.json -l $IN/m1adj_escape.lef
run reorder -d $IN/layers.json -p $IN/reorder.netlist.json -l $IN/m1adj_escape.lef -ndr $IN/reorder_ndr.json
run syms -d $IN/layers.json -p $IN/symmetric_s.netlist.json -l $IN/symmetric.lef -ndr $IN/symmetric_s_ndr.json
rm -rf "$W"
