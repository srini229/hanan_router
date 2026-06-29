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
    # Total unrouted nets, summed over modules. The router emits one authoritative
    # "ROUTE_SUMMARY module=<m> nets=<n> unrouted=<u>" line per module reflecting
    # its FINAL state (after the adjacent-obstacle retry and the -reorder search),
    # so we read that directly rather than scraping per-attempt "sol not found"
    # lines, which over-count across sub-passes/reorder passes.
    local unrouted
    unrouted=$(awk '
      /ROUTE_SUMMARY/ { for (i = 1; i <= NF; i++)
                          if ($i ~ /^unrouted=/) { split($i, a, "="); total += a[2] } }
      END { print total + 0 }
    ' "$log")
    # ALLOW_UNROUTED marks a diagnostic case that is expected to leave a net
    # open (e.g. proving the SAT escape check flags an unroutable boxed pin).
    if [ "$unrouted" != "0" ] && [ -z "${ALLOW_UNROUTED:-}" ]; then
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
  # NETROUTED is a '|'-separated list of net names whose DEF block in the first
  # expected DEF must contain actual routing (a via). A net that only fell back
  # to its pin shapes has no "+ RECT V*", so this catches a specific net that
  # was left unrouted even when the rest of the design routed.
  if [ -n "${NETROUTED:-}" ] && [ -n "$expdefs" ]; then
    local ndef=${expdefs%%,*} net
    IFS='|' read -ra nets <<< "$NETROUTED"
    for net in "${nets[@]}"; do
      if [ -f "$dir/$ndef" ]; then
        awk -v n="$net" '
          $0 ~ "^ *- "n"$"{p=1; next}
          p && /^ *- /{p=0}
          p && /\+ RECT V/{f=1}
          END{exit !f}' "$dir/$ndef" || errs="$errs net-unrouted:$net;"
      fi
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
  NETROUTED=""
  ALLOW_UNROUTED=""
}

# cli_check <name> <pattern> [router args...]
# For argument-handling / error paths that do not produce a normal route: run the
# router with the given args verbatim (no implicit -o) and assert <pattern> shows
# up on stderr (pre-setup messages like the usage text) or in err.log (where the
# router redirects std::cerr once running).
cli_check() {
  local name=$1 pat=$2
  shift 2
  local dir="$OUTROOT/$name"
  rm -rf "$dir"; mkdir -p "$dir"
  ( cd "$dir" && "$ROUTER" "$@" >/dev/null 2>stderr.log )
  if grep -qE "$pat" "$dir/stderr.log" "$dir/err.log" 2>/dev/null; then
    echo "PASS $name"
    PASS=$((PASS+1))
  else
    echo "FAIL $name :no-match:'$pat';"
    FAIL=$((FAIL+1))
    ERRS="$ERRS$name:no-match:'$pat';\n"
  fi
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
#LOGMUST="writing sto to|sol("
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

# 17. m1_pin_adj_obstacle: net A (routed first, smaller HPWL) would naturally run
#     a straight M2 wire directly over net B's wide M1 pin. That M2 covers B's
#     whole pin footprint, so B (routed second) cannot drop a via up off M1 and
#     is left unrouted. The fix projects every other unrouted net's M1 pins onto
#     the adjacent metal (M2) as obstacles for the current net, so A is forced to
#     detour around B's pin, leaving the footprint free for B to escape upward.
#     NETROUTED=B asserts B actually routed (has a via) -- it would be only pin
#     shapes, i.e. unrouted, without the fix.
NETROUTED="B"
run_case m1_pin_adj_obstacle "M1ADJ_CONC_0.def" \
  -d $IN/layers.json -p $IN/m1adj_escape.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/m1adj_escape_ndr.json

# 18. unconnected_pin: instance I_U has a pin (M1) that is not wired to any net,
#     sitting directly in net A's straight M1 column. Such pins are otherwise
#     invisible to the router; they must be added as obstacles so no route lands
#     on them. With the protection, A detours (off M1 over the pin) yet still
#     connects; LOGMUST checks the pin was recognised, NETROUTED that A routed.
LOGMUST="protecting unconnected pin I_U/P"
NETROUTED="A"
run_case unconnected_pin "UNCONN_CONC_0.def" \
  -d $IN/layers.json -p $IN/unconnected_pin.placement_verilog.json -l $IN/m1adj_escape.lef

# 19. sat_pin_escape: pin I_A0 is fully boxed (M1 obstacles on all four sides +
#     an M2 obstacle over it, and M1 is the bottom layer), so it has neither a
#     via nor a same-layer escape. The pre-routing SAT feasibility check must
#     prove this and report the stranded pin before any net is routed.
LOGMUST="pin escape SAT : BOXEDPIN_CONC_0 is infeasible|no escape for pin : BOXEDPIN_CONC_0/I_A0/P"
ALLOW_UNROUTED=1
run_case sat_pin_escape "" \
  -d $IN/layers.json -p $IN/boxedpin.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/boxedpin_ndr.json
# 20. reorder: 5 nets criss-cross through one capacity-limited gap in a wall (left
#     pins top->down, right pins bottom->up). The default HPWL net order strands
#     one net; the reorder search promotes blocked nets up the routing order
#     (priority grows each pass a net stays open) and routes all five. Built by
#     gen_reorder.py (N=5 GAP=280,540). LOGMUST proves the default order failed
#     (that message only prints when the first attempt leaves nets open); the
#     ROUTE_SUMMARY unrouted count (0) and NETROUTED (every net has a via) both
#     confirm the reorder routed the whole module.
LOGMUST="promoting blocked nets up the routing order"
NETROUTED="N0|N1|N2|N3|N4"
run_case reorder "REORDER_CONC_0.def" \
  -d $IN/layers.json -p $IN/reorder.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/reorder_ndr.json

# 21. reorder_disabled: the same case with -reorder 0 turns the net-ordering search
#     off, so the one net the default HPWL order strands stays open. Exercises the
#     -reorder argument (Router::setReorderPasses) and confirms the search is what
#     routes it: LOGMUST asserts the ROUTE_SUMMARY still reports one unrouted net.
LOGMUST="ROUTE_SUMMARY module=REORDER_CONC_0 nets=5 unrouted=1"
ALLOW_UNROUTED=1
run_case reorder_disabled "" \
  -d $IN/layers.json -p $IN/reorder.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/reorder_ndr.json -reorder 0

# 22-25. argument-handling / error paths (cover main.cpp CLI parsing and the
#     std::cerr diagnostics). Each asserts the expected message on stderr/err.log.
cli_check usage_no_args   "usage :"                                       # argc<=1 -> usage text
cli_check missing_layers  "missing or unable to read layers" \
  -d $IN/does_not_exist.json -p $IN/reorder.placement_verilog.json -l $IN/m1adj_escape.lef
cli_check bad_precision   "invalid -r precision" \
  -d $IN/layers.json -p $IN/reorder.placement_verilog.json -l $IN/m1adj_escape.lef -r notanint
cli_check bad_reorder_arg "invalid -reorder value" \
  -d $IN/layers.json -p $IN/reorder.placement_verilog.json -l $IN/m1adj_escape.lef \
  -ndr $IN/reorder_ndr.json -reorder xyz
# Netlist input-error paths (cover Netlist.cpp open/parse failure handling).
cli_check no_placement_file "unable to open placement file" \
  -d $IN/layers.json -p $IN/does_not_exist.json -l $IN/m1adj_escape.lef
cli_check bad_placement_json "parse error" \
  -d $IN/layers.json -p $IN/bad_placement.json -l $IN/m1adj_escape.lef
cli_check no_lef_file "unable to open leffile" \
  -d $IN/layers.json -p $IN/reorder.placement_verilog.json -l $IN/does_not_exist.lef

# 26. reorder_reroute: a harder 6-net criss-cross (gap fits fewer than 6) that the
#     reorder search improves but cannot fully solve, so it exhausts its passes
#     with the best order found mid-search rather than last -- exercising the
#     "re-route with the best ordering" replay path. LOGMUST asserts that replay
#     ran; ALLOW_UNROUTED because the case is intentionally over-subscribed.
#     Fixture: 6 nets, wall gap [300,440] (criss-cross, capacity < demand).
LOGMUST="re-routing with best ordering|promoting blocked nets up the routing order"
ALLOW_UNROUTED=1
run_case reorder_reroute "REORDER_CONC_0.def" \
  -d $IN/layers.json -p $IN/reorder_reroute.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/reorder_reroute_ndr.json

# 27. m2_pin_escape: a 2-pin net whose pins sit on M2 (not the bottom layer M1).
#     The pin-escape SAT then builds a via-DOWN escape candidate (M2->M1), which
#     M1-only pins never trigger. LOGMUST checks the SAT ran for the M2 module.
LOGMUST="pin escape SAT : all 2 pins in M2T_CONC_0"
run_case m2_pin_escape "M2T_CONC_0.def" \
  -d $IN/layers.json -p $IN/m2pin.placement_verilog.json -l $IN/m2pin.lef

# 28. ndr_via_detour: NDR forces net A (M1 pins) onto preferred_layers M3/M4, so
#     it must via UP off M1 and via DOWN back onto M1 -- exercising multi-layer
#     via routing. A wall obstacle spans all four layers across x[300,360] for
#     y<360, so the only crossing is over the top (y>360), far outside the pins'
#     bbox; large_detour="allowed" lets the router expand the search to find it.
#     The case is unroutable without both features, so NETROUTED=A (the net routed
#     with vias) confirms the via-down + large-detour path worked.
NETROUTED="A"
run_case ndr_via_detour "VIADET_CONC_0.def" \
  -d $IN/layers.json -p $IN/ndr_viadetour.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/ndr_viadetour.json

# 29. global_net: a non-empty "global_signals" list (VDD) adds the global net as a
#     pin on every leaf and module -- a path all other inputs (empty list) skip.
#     LOGMUST confirms the global net was created.
LOGMUST="net : VDD num pins"
run_case global_net "GLOB_CONC_0.def" \
  -d $IN/layers.json -p $IN/global_net.placement_verilog.json -l $IN/m1adj_escape.lef

# 30. coincident_pin: nets A and B have a pin at the IDENTICAL location (700,300)
#     -- physically one point. Routing them separately must short; instead the
#     router warns and merges them into one connected net (dropping the redundant
#     coincident pin). LOGMUST checks the warning; the suite's short check (must be
#     0) and ROUTE_SUMMARY unrouted=0 confirm the merged net routes cleanly.
LOGMUST="has pin(s) coincident with net|merging them into one connected net"
NETROUTED="A"
run_case coincident_pin "COIN_CONC_0.def" \
  -d $IN/layers.json -p $IN/coincident_pin.placement_verilog.json -l $IN/m1adj_escape.lef

# 31. net30: a 30-net module (30 parallel 2-pin nets on a 120-unit pitch) -- a
#     larger throughput check. All thirty route; the harness flags any unrouted
#     net via ROUTE_SUMMARY. LOGMUST asserts the full 30-net, 0-unrouted summary.
LOGMUST="ROUTE_SUMMARY module=NET30_CONC_0 nets=30 unrouted=0"
run_case net30 "NET30_CONC_0.def" \
  -d $IN/layers.json -p $IN/net30.placement_verilog.json -l $IN/m1adj_escape.lef

# 31b. net30 routed in parallel (-threads 4): the 30 disjoint nets get grouped
#      into non-overlapping batches and routed concurrently. Must still route all
#      30 with no shorts, AND produce a DEF byte-identical to the sequential
#      net30 case above -- parallelism must not change the result (the DEF is
#      deterministic: pins are emitted name-sorted, not in pointer order).
LOGMUST="ROUTE_SUMMARY module=NET30_CONC_0 nets=30 unrouted=0"
run_case net30_threads "NET30_CONC_0.def" \
  -d $IN/layers.json -p $IN/net30.placement_verilog.json -l $IN/m1adj_escape.lef \
  -threads 4
if diff -q "$OUTROOT/net30/NET30_CONC_0.def" \
           "$OUTROOT/net30_threads/NET30_CONC_0.def" >/dev/null 2>&1; then
  echo "PASS net30_threads_parity"
  PASS=$((PASS+1))
else
  echo "FAIL net30_threads_parity :def-differs-from-sequential;"
  FAIL=$((FAIL+1))
  ERRS="${ERRS}net30_threads_parity:def-differs-from-sequential;\n"
fi

# 31. maze30 (opt-in stress, ~25s): the same 30 nets crossing a staggered-gap
#     wall, every net flagged large_detour so it can weave around it. Deliberately
#     hard -- the router leaves ~1/3 of the nets unrouted. Off by default because
#     of its runtime; run with:  MAZE_STRESS=1 ./run_smoke.sh
#     Solution quality: python3 maze_quality.py <def> maze30.placement_verilog.json
if [ -n "${MAZE_STRESS:-}" ]; then
  ALLOW_UNROUTED=1
  run_case maze30 "MAZE_CONC_0.def" \
    -d $IN/layers.json -p $IN/maze30.placement_verilog.json \
    -l $IN/m1adj_escape.lef -ndr $IN/maze30_ndr.json
fi

echo
echo "smoke tests : $PASS passed, $FAIL failed"
if [ $FAIL -ne 0 ]; then
  printf "$ERRS"
  exit 1
fi
exit 0
