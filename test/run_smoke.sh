#!/usr/bin/env bash
# Smoke tests for hanan_router using the example inputs in this directory.
#
# Usage: ./run_smoke.sh [path-to-hanan_router]
#
# Each case runs the router in its own directory under smoke_out/ and checks:
#   - the router exits 0
#   - every source/target pair found a solution (no "sol not found")
#   - the expected DEF files were written and contain routed shapes
#   - the number of reported SHORTs matches the recorded baseline
# plus case-specific log checks (excluded nets, virtual pins, obstacles, ...).

set -u
cd "$(dirname "$0")"

ROUTER=${1:-../hanan_router}
if [ ! -x "$ROUTER" ]; then
  echo "router binary not found: $ROUTER (run make first)" >&2
  exit 2
fi
ROUTER=$(cd "$(dirname "$ROUTER")" && pwd)/$(basename "$ROUTER")

OUTROOT=smoke_out
rm -rf "$OUTROOT"
mkdir -p "$OUTROOT"

PASS=0
FAIL=0
ERRS=""

# run_case <name> <expected-defs (comma separated, may be empty)> [router args...]
run_case() {
  local name=$1 expdefs=$2
  shift 2
  local dir="$OUTROOT/$name"
  local errs=""
  mkdir -p "$dir"
  ( cd "$dir" && "$ROUTER" "$@" -o ./ >/dev/null 2>stderr.log )
  local rc=$?
  [ $rc -eq 0 ] || errs="$errs exit=$rc;"
  local log="$dir/route.log"
  if [ ! -f "$log" ]; then
    errs="$errs no-route.log;"
  else
    # a pair is only unrouted if both the forward and the reversed attempt fail.
    # pin width success ("sol found with pin width escape for <name>") cancels the
    # two prior "sol not found for <name>" entries for that pair.
    local unrouted
    unrouted=$(awk '
      /sol not found for/{f[$5]++}
      /sol found with pin width for/{f[$7]--}
      END{c=0; for(k in f) if(f[k]>=2) c++; print c+0}
    ' "$log")
    if [ "$unrouted" != "0" ]; then
      errs="$errs unrouted-pairs=$unrouted;"
    fi
    if grep -q "unable to open\|missing " "$log" "$dir/err.log" 2>/dev/null; then
      errs="$errs input-error;"
    fi
    # "Checking SHORTS" headers always appear; actual violations must not
    local shorts
    shorts=$(grep -c "SHORT.*between" "$log")
    if [ "$shorts" != "0" ]; then
      errs="$errs short-violations=$shorts;"
    fi
  fi
  if [ -n "$expdefs" ]; then
    local def
    for def in ${expdefs//,/ }; do
      if [ ! -s "$dir/$def" ]; then
        errs="$errs missing-def:$def;"
      elif ! grep -q "+ RECT M" "$dir/$def"; then
        errs="$errs no-routes-in:$def;"
      fi
    done
  fi
  # extra per-case log checks: LOGMUST is a newline-free '|'-separated list
  if [ -n "${LOGMUST:-}" ] && [ -f "$log" ]; then
    local pat
    IFS='|' read -ra pats <<< "$LOGMUST"
    for pat in "${pats[@]}"; do
      grep -q "$pat" "$log" || errs="$errs log-missing:'$pat';"
    done
  fi
  if [ -z "$errs" ]; then
    echo "PASS $name"
    PASS=$((PASS+1))
  else
    echo "FAIL $name :$errs"
    FAIL=$((FAIL+1))
    ERRS="$ERRS$name:$errs\n"
  fi
  LOGMUST=""
}

IN=../..   # inputs relative to each case directory

# 1. base testcase from the README
run_case basic "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef

# 2. the simple single-module test
run_case simple "BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test1.placement_verilog.json -l $IN/test.lef

# 3. NDR: per-module widths/spaces, preferred layers, virtual pin, clock driver
LOGMUST="added virtual pin|clock net : D with driver : J_1/Y"
run_case ndr_full "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -ndr $IN/ndr.json

# 4. NDR: do_not_route + per-net NDR + net-scoped obstacles
LOGMUST="excluding net : Y|Adding obstacle to net : D"
run_case ndr_donotroute "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -ndr $IN/smoke_ndr1.json

# 5. NDR: module-level obstacles applied to all nets
LOGMUST="Adding obstacle to module TEST_CONC_0"
run_case ndr_obstacles "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -ndr $IN/smoke_ndr2.json

# 6. NDR: module-wide preferred layers + custom via array
run_case ndr_vias "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -ndr $IN/smoke_ndr3.json

# 7. coordinate precision rounding
run_case precision "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -r 4

# 8. debug plot outputs (HANAN_DEBUG_WIRE exercises the per-wire dump routines,
#    HANAN_DEBUG_NET the per-net debug LEF dump)
export HANAN_DEBUG_WIRE=1 HANAN_DEBUG_NET=X,Y
LOGMUST="writing sto to|sol("
run_case debug_plot "BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test1.placement_verilog.json -l $IN/test.lef
unset HANAN_DEBUG_WIRE HANAN_DEBUG_NET

# 9. NDR: per-layer directions, large_detour, routing_order, use_pin_width
LOGMUST="use pin width : 1"
run_case ndr_extras "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -ndr $IN/smoke_ndr4.json

# 10. layers.json with the optional MinL/MaxL/EndToEnd/Offset keys
run_case layers_ext "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/smoke_layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef

# 11. leaf LEF with an OBS section (macro obstacles transformed into instances)
run_case lef_obs "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/smoke_obs.lef

# 12. mirrored instance placement (sX/sY = -1, orientation S in the DEF)
run_case flipped "BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/smoke_flip.placement_verilog.json -l $IN/test.lef

# 13. many obstacles on one layer (forces R-tree node splits)
LOGMUST="Adding obstacle to module TEST_CONC_0"
run_case many_obstacles "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -ndr $IN/smoke_ndr5.json

# 14. hierarchical reuse: route once, then reload the interim LEFs (-uil)
run_case uil_stage "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef
LOGMUST="loading macro BLOCK_B_CONC_0"
run_case uil_reuse "" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -uil $IN/smoke_out/uil_stage

# 15. ViaArrayGenerators testcase from the README
run_case ViaArrayGenerators "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers_viagen.json -p $IN/test.placement_verilog.json -l $IN/test.lef

# 16. use_pin_width_escape: pins narrower than the layer width block standard routing
#     (OBS column at x=36..80 bloats to cover pin centre x=4 with standard widthy=32,
#     but not with narrow widthy=8 derived from the pin x-span)
#LOGMUST="retrying.*with pin width escape|sol found with narrow escape for"
run_case pin_width_escape "NARROW_M_CONC_0.def" \
  -d $IN/layers_M1_O.json -p $IN/narrow_escape.placement_verilog.json \
  -l $IN/narrow_escape.lef -ndr $IN/narrow_escape_ndr.json

echo
echo "smoke tests : $PASS passed, $FAIL failed"
if [ $FAIL -ne 0 ]; then
  printf "$ERRS"
  exit 1
fi
exit 0
