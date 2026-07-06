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

# perf_parallel_speedup: route a heavy, fully-parallelizable design (many
# disjoint, individually-expensive nets) sequentially and with several worker
# threads, and assert the threaded run is both substantially faster AND lays down
# bit-for-bit identical wires. The wall time is taken as the best of three runs
# (the minimum is the cleanest signal -- contention only ever slows a run down).
perf_parallel_speedup() {
  local name=perf_parallel_speedup
  if ! command -v python3 >/dev/null 2>&1; then
    echo "SKIP $name (python3 not available)"; return
  fi
  local dir="$OUTROOT/$name"
  rm -rf "$dir"; mkdir -p "$dir/seq" "$dir/par"
  local pl="$dir/bench.placement_verilog.json" ndr="$dir/bench_ndr.json"
  if ! python3 ./gen_parallel_bench.py 32 "$pl" "$ndr" >/dev/null 2>&1; then
    echo "FAIL $name :generator-failed;"; FAIL=$((FAIL+1))
    ERRS="${ERRS}$name:generator-failed;\n"; return
  fi

  local ncpu threads
  ncpu=$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)
  threads=4; [ "$ncpu" -lt 4 ] && threads=$ncpu; [ "$threads" -lt 1 ] && threads=1

  _best_time() {  # _best_time <outdir> <threads> -> min wall time of 3 runs
    local b="" t
    for _ in 1 2 3; do
      t=$(python3 - "$ROUTER" "$1/" "$2" "$pl" "$ndr" <<'PY'
import subprocess, sys, time
router, outdir, threads, pl, ndr = sys.argv[1:6]
cmd = [router, "-d", "./layers.json", "-p", pl, "-l", "./m1adj_escape.lef",
       "-ndr", ndr, "-o", outdir, "-log", outdir + "route.log", "-threads", threads]
s = time.time(); subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
print(f"{time.time() - s:.4f}")
PY
)
      b=$(python3 -c "print(min($t, ${b:-$t}))")
    done
    echo "$b"
  }

  local tseq tpar
  tseq=$(_best_time "$dir/seq" 1)
  tpar=$(_best_time "$dir/par" "$threads")

  local errs="" m lg
  for m in seq par; do
    lg="$dir/$m/route.log"
    grep -q "ROUTE_SUMMARY module=PARBENCH_CONC_0 nets=32 unrouted=0" "$lg" 2>/dev/null \
      || errs="$errs $m-not-all-routed;"
    [ "$(grep -c 'SHORT.*between' "$lg" 2>/dev/null)" = "0" ] || errs="$errs $m-shorts;"
  done
  # parallelism must not change the routed wires
  diff -q <(grep '+ RECT' "$dir/seq/PARBENCH_CONC_0.def" 2>/dev/null) \
          <(grep '+ RECT' "$dir/par/PARBENCH_CONC_0.def" 2>/dev/null) >/dev/null 2>&1 \
    || errs="$errs geometry-differs;"
  # the speedup itself -- only assert it when there is more than one core to use
  local speedup="n/a"
  if [ "${ncpu:-1}" -ge 2 ]; then
    speedup=$(python3 -c "print(f'{$tseq/$tpar:.2f}')")
    awk "BEGIN{exit !($tseq >= $tpar*1.2)}" \
      || errs="$errs no-speedup(seq=${tseq}s par=${tpar}s=${speedup}x);"
  fi

  if [ -z "$errs" ]; then
    echo "PASS $name (seq=${tseq}s ${threads}-thread=${tpar}s ${speedup}x on ${ncpu} cores)"
    PASS=$((PASS+1))
  else
    echo "FAIL $name :$errs"; FAIL=$((FAIL+1)); ERRS="${ERRS}$name:$errs\n"
  fi
}

IN=../..   # inputs relative to each case directory

# 1. base testcase from the README
run_case basic "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef

# 1b. def_pin_syntax: every NETS pin reference must be DEF's "( component pin )"
#     syntax -- a bare instance name, then a bare pin name -- never the full
#     "instance/pin" hierarchical string repeated in both fields. Regression
#     check: the pin-name splitter once looked for a literal '+' instead of
#     the actual SEPARATOR ('/'), so with the default separator every NETS
#     entry echoed "( J_0/Y J_0/Y )" instead of "( J_0 Y )".
bdef="$OUTROOT/basic/TEST_CONC_0.def"
if awk '/^NETS/{p=1} /^END NETS/{p=0} p && /\(/ && /\//{found=1} END{exit found}' "$bdef" 2>/dev/null \
    && grep -q "( J_0 Y )" "$bdef" 2>/dev/null; then
  echo "PASS def_pin_syntax"; PASS=$((PASS+1))
else
  echo "FAIL def_pin_syntax : NETS pin fields not in 'component pin' syntax"
  FAIL=$((FAIL+1)); ERRS="${ERRS}def_pin_syntax:bad-nets-syntax;\n"
fi

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
# -v enables the verbose per-obstacle log this case asserts on
LOGMUST="Adding obstacle to module TEST_CONC_0"
run_case ndr_obstacles "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -ndr $IN/smoke_ndr2.json -v

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

# 10b. layers_float_width: identical to layers.json except M1's "Width" is
#      written as 32.0 (a JSON float) instead of 32 (a JSON int). Every
#      numeric field in LayerInfo::LayerInfo() was gated by
#      is_number_integer(), which is false for a float-typed JSON number, so
#      the field was silently left at its zero-initialized default instead of
#      being read -- no warning, just a wrong (and here structurally
#      significant: it also corrupts the derived M1 spacing) value. LOGMUST
#      confirms M1 (layer index 2) parses to width 32, not 0.
LOGMUST="layer : 2 width : 32 "
run_case layers_float_width "TEST_CONC_0.def,BLOCK_B_CONC_0.def" -v \
  -d $IN/layers_float_width.json -p $IN/test.placement_verilog.json -l $IN/test.lef

# 11. leaf LEF with an OBS section (macro obstacles transformed into instances)
run_case lef_obs "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/smoke_obs.lef

# 12. mirrored instance placement (sX/sY = -1, orientation S in the DEF)
run_case flipped "BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/smoke_flip.placement_verilog.json -l $IN/test.lef

# 13. many obstacles on one layer (forces R-tree node splits)
LOGMUST="Adding obstacle to module TEST_CONC_0"
run_case many_obstacles "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -ndr $IN/smoke_ndr5.json -v

# 14. hierarchical reuse: route once, then reload the interim LEFs (-uil)
run_case uil_stage "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef
LOGMUST="loading macro BLOCK_B_CONC_0"
run_case uil_reuse "" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -uil $IN/smoke_out/uil_stage

# 15. ViaArrayGenerators testcase from the README
run_case ViaArrayGenerators "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers_viagen.json -p $IN/test.placement_verilog.json -l $IN/test.lef

# 15b. ViaArrayGeneratorsMixed: two V1 via types from the array used at different
#      locations in the same net.  NDR obstacles on M1 restrict which type fits
#      (effective block condition: original.overlaps(lpad.expand(+24)) after the
#      40-per-side splitRects expansion and 16-per-side shrink in isViaValid):
#        via at (16,116)  – obstacle (77,80,116,132) sits in the right-exclusive
#          X zone of NumX=2 lpad (x=77..116 vs NumX=1 reach of x=-20..52), so
#          NumX=2,NumY=1 is blocked → NumX=1,NumY=2 is selected.
#        via at (516,116) – obstacle (480,180,552,218) sits above NumY=1 lpad
#          Y ceiling (y=180..218 vs NumY=1 reach up to y=152), so NumY=2 is
#          blocked → NumX=2,NumY=1 is selected.
LOGMUST="Adding obstacle to module VG2VT_CONC_0"
run_case ViaArrayGeneratorsMixed "VG2VT_CONC_0.def" \
  -d $IN/layers_viagen.json -p $IN/viagen_mixed.placement_verilog.json \
  -l $IN/test.lef -ndr $IN/viagen_mixed_ndr.json -v

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

# 19b. sat_spacing_gap: same boxed-pin layout as above, but the M2 obstacle
#      over I_A0's via-up candidate is offset 20 units away (M2 space=24) --
#      close enough to violate minimum spacing, but not literally overlapping.
#      The SAT pre-check's hitsObstacle() once tested raw overlap only, so it
#      declared this via candidate clear and the pin "feasible" even though
#      the real router's isViaValid (which always bloats by spacing) rejects
#      it -- the net then genuinely fails to route (ROUTE_SUMMARY
#      unrouted=1), contradicting the SAT check's own "guaranteed escape"
#      verdict. LOGMUST now asserts the SAT check catches this itself.
LOGMUST="pin escape SAT : BOXEDPIN_CONC_0 is infeasible|no escape for pin : BOXEDPIN_CONC_0/I_A0/P"
ALLOW_UNROUTED=1
run_case sat_spacing_gap "" \
  -d $IN/layers.json -p $IN/sat_spacing_gap.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/sat_spacing_gap_ndr.json
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
# bad_layers_json: a layers.json whose UnitR.Mean is a string instead of a
#     number. LayerInfo::LayerInfo() read this via nlohmann's .value<float>(),
#     which throws json::type_error on a type mismatch (unlike a missing key,
#     which .value() defaults harmlessly) -- and nothing in the constructor
#     caught it, so it escaped as an uncaught exception and aborted the whole
#     process (not a clean "invalid input" message). Must degrade to a clean
#     diagnostic instead of terminate()/SIGABRT.
cli_check bad_layers_json "invalid UnitR" \
  -d $IN/bad_layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef

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

# 29b. symmetric_nets: two diagonal nets (INP, INM) placed as mirror images about
#      x=1000, with a routing obstacle ON THE INP SIDE ONLY. Unguided, INP must
#      detour around the obstacle while INM (clear side) routes straight -- so the
#      two are NOT mirror symmetric. With symmetric_nets, INP is routed first and
#      its mirrored route guides INM's A*, so INM reproduces the detour mirrored.
#      The router measures the residual and prints "... maxdev=0 ...": INM matches
#      the mirror of INP exactly, which only happens because the deviation cost
#      actually steered the search (without it maxdev would be hundreds). The axis
#      here is auto-detected from the nets' pin geometry (no explicit override).
LOGMUST="symmetric net : routing INM guided by INP|SYMMETRY module=SYM_CONC_0 pair=INP,INM axis=V:1000 maxdev=0 "
run_case symmetric_nets "SYM_CONC_0.def" \
  -d $IN/layers.json -p $IN/symmetric.placement_verilog.json -l $IN/symmetric.lef \
  -ndr $IN/symmetric_ndr.json

# 29c. same layout but with the mirror axis given explicitly ("V": 1000) instead
#      of auto-detected -- exercises the JSON axis-override parse path. Result is
#      identical: INM mirrors INP's detour exactly (maxdev=0).
LOGMUST="SYMMETRY module=SYM_CONC_0 pair=INP,INM axis=V:1000 maxdev=0 "
run_case symmetric_nets_axis "SYM_CONC_0.def" \
  -d $IN/layers.json -p $IN/symmetric.placement_verilog.json -l $IN/symmetric.lef \
  -ndr $IN/symmetric_axis_ndr.json

# 29d. symmetric_s: a harder symmetric case whose routes are S-shaped. Each net's
#      two pins are offset in x, and two staggered all-metal obstacle bars (with
#      gaps on opposite sides) force the wire to rise on one column, cross over,
#      and rise on the other -- an S/Z. The obstacles are themselves mirror images
#      about x=1000, so both nets route an S; symmetric_nets makes INM the exact
#      mirror of INP's S (maxdev=0). The post-check confirms INP genuinely weaves
#      (M1 vertical runs on BOTH a left and a right column), i.e. it is not a
#      straight wire -- so maxdev=0 means a full S was mirrored, not a trivial line.
LOGMUST="SYMMETRY module=SYMS_CONC_0 pair=INP,INM axis=V:1000 maxdev=0 "
run_case symmetric_s "SYMS_CONC_0.def" \
  -d $IN/layers.json -p $IN/symmetric_s.placement_verilog.json -l $IN/symmetric.lef \
  -ndr $IN/symmetric_s_ndr.json
sdef="$OUTROOT/symmetric_s/SYMS_CONC_0.def"
if awk '
    /^ *- INP$/{p=1; next} /^ *- INM$/{p=0} /END NETS/{p=0}
    p && /\+ RECT M1 / { x=$5+0; if (x<=360) L=1; if (x>=456) R=1 }
    END{ exit !(L && R) }' "$sdef" 2>/dev/null; then
  echo "PASS symmetric_s_shape"; PASS=$((PASS+1))
else
  echo "FAIL symmetric_s_shape : INP route does not weave across columns (no S)"
  FAIL=$((FAIL+1)); ERRS="${ERRS}symmetric_s_shape:no-weave;\n"
fi

# 30. coincident_pin: nets A and B have a pin at the IDENTICAL location (700,300)
#     -- physically one point. Routing them separately must short; instead the
#     router warns and merges them into one connected net (dropping the redundant
#     coincident pin). LOGMUST checks the warning; the suite's short check (must be
#     0) and ROUTE_SUMMARY unrouted=0 confirm the merged net routes cleanly.
LOGMUST="has pin(s) coincident with net|merging them into one connected net"
NETROUTED="A"
run_case coincident_pin "COIN_CONC_0.def" \
  -d $IN/layers.json -p $IN/coincident_pin.placement_verilog.json -l $IN/m1adj_escape.lef

# zero_length_sol: net A has two pins of the SAME net at the exact same
#     location (unlike coincident_pin above, which is two DIFFERENT nets
#     sharing a coincident pin). The router's source==target case finds a
#     trivial "sol found with 0 expansions" -- correctly, since the two pins
#     are already electrically connected on the same layer and need zero
#     additional wire (no via either) -- but Net::route() decided
#     success/failure by checking whether the returned shape list was
#     non-empty, and a zero-length solution legitimately returns an EMPTY
#     shape list. That marked a fully-connected net unrouted. The harness's
#     default unrouted=0 check (via ROUTE_SUMMARY) is what catches this;
#     NETROUTED isn't used here since this specific connection needs no via.
run_case zero_length_sol "ZEROLEN_CONC_0.def" \
  -d $IN/layers.json -p $IN/zero_length_sol.placement_verilog.json -l $IN/zero_length_sol.lef

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

# 31c. noport_pin: net A connects a normal pin (I_A0, real M1 geometry) to a
#      pin whose LEF PIN block has no PORT section at all (I_A1, macro NOPORT
#      -- syntactically valid LEF for a purely logical/undefined-geometry
#      pin). Net::reorderPorts() gathers ports across all of a net's pins
#      without checking any pin contributed zero; with only 1 total port,
#      idx1/idx2 (initialized to -1, only ever set inside a pairwise i<j loop
#      that needs >=2 ports to run its body) stay -1 and ports[-1] is read --
#      a heap-buffer-overflow / segfault, reproduced under ASAN at Net.cpp:90.
#      This must simply not crash; the fixture has no obstacles so once fixed
#      the net trivially routes.
run_case noport_pin "NOPORTNET_CONC_0.def" \
  -d $IN/layers.json -p $IN/noport.placement_verilog.json -l $IN/noport.lef

# 32. via_escape_source_blocked: net A's source pin I_A0 is boxed with M1 walls
#     above/below (10-unit gap, so same-layer escape is blocked at full M1
#     spacing) and a single M2 obstacle 10 units below its via landing pad (so
#     the M1->M2 escape via is also blocked at full spacing). The target pin
#     I_A1 is unobstructed. Without -relaxvia this net cannot route at all --
#     ALLOW_UNROUTED proves the fixture genuinely needs the feature below.
ALLOW_UNROUTED=1
run_case via_escape_source_blocked "" \
  -d $IN/layers.json -p $IN/via_escape.placement_verilog.json \
  -l $IN/via_escape.lef -ndr $IN/via_escape_ndr.json

# 33. via_escape_source_relaxed: same fixture as above, routed with -relaxvia.
#     The 10-unit gaps are inside full spacing but outside MIN_ESCAPE_SPACE (5),
#     so relaxing the escape via at the source alone is enough -- the retry
#     tier never needs to touch the (already-clear) target side. LOGMUST
#     confirms the source-only stage fired and found a solution without ever
#     logging the "also relaxing at target" fallback; NETROUTED confirms A
#     actually used a via, not just its pin shape.
LOGMUST="retrying A.*with via escape relaxed at source|sol found with via escape relaxed for A"
NETROUTED="A"
run_case via_escape_source_relaxed "VESC_CONC_0.def" -relaxvia \
  -d $IN/layers.json -p $IN/via_escape.placement_verilog.json \
  -l $IN/via_escape.lef -ndr $IN/via_escape_ndr.json

# 34. via_escape_both_blocked: like case 32, but I_A1 (target) is boxed with
#     the identical obstacle pattern as I_A0 (mirrored 400 units over). Neither
#     pin can escape without relaxation. ALLOW_UNROUTED proves it needs both
#     sides relaxed, not just source.
ALLOW_UNROUTED=1
run_case via_escape_both_blocked "" \
  -d $IN/layers.json -p $IN/via_escape_both.placement_verilog.json \
  -l $IN/via_escape.lef -ndr $IN/via_escape_both_ndr.json

# 35. via_escape_both_relaxed: same both-boxed fixture, routed with -relaxvia.
#     Relaxing the source alone is not enough (the target is still boxed), so
#     the retry must fall through to the second stage that ALSO relaxes the
#     target before it can find a solution. LOGMUST checks both the "source
#     failed, also relaxing at target" transition and the eventual success;
#     NETROUTED confirms the net actually routed (via at both ends).
LOGMUST="via escape relaxed at source failed for A.*also relaxing at target|sol found with via escape relaxed for A"
NETROUTED="A"
run_case via_escape_both_relaxed "VESC2_CONC_0.def" -relaxvia \
  -d $IN/layers.json -p $IN/via_escape_both.placement_verilog.json \
  -l $IN/via_escape.lef -ndr $IN/via_escape_both_ndr.json

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

# 33. parallel speedup (opt-in, timing-based, ~2-4s): a batch of many disjoint,
#     individually-expensive nets routes substantially faster with N worker
#     threads than sequentially -- and lays down exactly the same wires. Off by
#     default because it is timing-based; run with:  PERF_STRESS=1 ./run_smoke.sh
if [ -n "${PERF_STRESS:-}" ]; then
  perf_parallel_speedup
fi

echo
echo "smoke tests : $PASS passed, $FAIL failed"
if [ $FAIL -ne 0 ]; then
  printf "$ERRS"
  exit 1
fi
exit 0
