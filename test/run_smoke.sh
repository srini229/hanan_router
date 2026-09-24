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
  [ $rc -eq "${EXPECT_EXIT:-0}" ] || errs="$errs exit=$rc;"   # EXPECT_EXIT: a replay left open exits 1
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
  # LOGNOT is the mirror: patterns that must NOT appear. Used to prove a switch
  # actually switched something off, rather than only that it was accepted.
  if [ -n "${LOGNOT:-}" ] && [ -f "$log" ]; then
    local npat
    IFS='|' read -ra npats <<< "$LOGNOT"
    for npat in "${npats[@]}"; do
      grep -q "$npat" "$log" && errs="$errs log-present:'$npat';"
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
  LOGNOT=""
  NETROUTED=""
  ALLOW_UNROUTED=""
  EXPECT_EXIT=""
}

# same_defs <name> <case-a> <case-b> <defs (comma separated)>
# Asserts two already-run cases produced byte-identical DEFs. This is how a
# performance change is tested: it has to leave the routing exactly as it was.
same_defs() {
  local name=$1 a=$2 b=$3 defs=$4 errs="" def
  for def in ${defs//,/ }; do
    if [ ! -s "$OUTROOT/$a/$def" ] || [ ! -s "$OUTROOT/$b/$def" ]; then
      errs="$errs missing-def:$def;"
    elif ! cmp -s "$OUTROOT/$a/$def" "$OUTROOT/$b/$def"; then
      errs="$errs def-differs:$def;"
    fi
  done
  if [ -z "$errs" ]; then
    echo "PASS $name"; PASS=$((PASS+1))
  else
    echo "FAIL $name :$errs"; FAIL=$((FAIL+1)); ERRS="$ERRS$name:$errs\n"
  fi
}

# log_count_lt <name> <case-fewer> <case-more> <pattern> <field>
# Asserts a numeric field summed over matching log lines is strictly smaller in
# the first case than in the second -- e.g. fewer escape points seeded.
log_count_lt() {
  local name=$1 a=$2 b=$3 pat=$4 fld=$5
  local va vb
  va=$(awk -v p="$pat" -v f="$fld" '$0 ~ p { t += $f } END { print t + 0 }' "$OUTROOT/$a/route.log" 2>/dev/null)
  vb=$(awk -v p="$pat" -v f="$fld" '$0 ~ p { t += $f } END { print t + 0 }' "$OUTROOT/$b/route.log" 2>/dev/null)
  if [ "${va:-0}" -gt 0 ] && [ "${vb:-0}" -gt 0 ] && [ "$va" -lt "$vb" ]; then
    echo "PASS $name"; PASS=$((PASS+1))
  else
    echo "FAIL $name :expected $a($va) < $b($vb) for '$pat';"
    FAIL=$((FAIL+1)); ERRS="$ERRS$name:count $va vs $vb;\n"
  fi
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

# 7b. outdir_no_slash: -o given without a trailing '/' (main.cpp appends one
#     if missing). Every other case relies on run_case's own appended
#     "-o ./", which already ends in '/' and never exercises that append --
#     passing "-o ." here as part of the case's own args takes priority
#     (parseArgs returns the first -o it finds) and still resolves to the
#     current directory, so run_case's own file-existence checks apply as-is.
run_case outdir_no_slash "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -o .

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

# 15c. via_array_venc: V1's ViaCut is a ViaArrayGenerators array of two entries
#      with IDENTICAL cut geometry (WidthX/Y=32, NumX=NumY=1) so the only
#      possible source of any pad-size difference is the enclosure fields
#      themselves -- entry 1 has no VencX_L/Y_L/X_H/Y_H of its own and falls
#      back to the layer-level VencA/P_L/H=20 (giving a uniform 72x72 lower
#      AND upper pad: cut 32 + 2*20), while entry 2 overrides
#      VencX_L=VencY_L=10 and VencX_H=VencY_H=30 (giving a smaller 52x52
#      lower pad [cut 32 + 2*10] and a larger 92x92 upper pad [cut 32 + 2*30]).
#      LOGMUST checks both exact via lines to confirm the per-array-entry
#      override actually reaches the constructed Via (previously silently
#      lost: addViaArray() returns a reference into the layer's stored
#      ViaArray, but the caller assigned it to a by-value `auto`, so the
#      VencX_L/etc. writes landed on a throwaway copy and every via's real
#      pad stayed at whatever uninitialized memory the struct happened to
#      hold).
LOGMUST="via : l: 2 u: 3 c: 8 center: (0,0) lb: \[(-36,-36),(36,36)\] ub: \[(-36,-36),(36,36)\]|via : l: 2 u: 3 c: 8 center: (0,0) lb: \[(-26,-26),(26,26)\] ub: \[(-46,-46),(46,46)\]"
#      (-v 1: the via table is per-net detail, not a result, so it only prints
#      above the default verbosity)
run_case via_array_venc "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers_via_venc.json -p $IN/test.placement_verilog.json -l $IN/test.lef -v 1

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
LOGMUST="pin escape SAT (pre-route) : BOXEDPIN_CONC_0 is infeasible|no escape for pin : BOXEDPIN_CONC_0/I_A0/P"
ALLOW_UNROUTED=1
run_case sat_pin_escape "" \
  -d $IN/layers.json -p $IN/boxedpin.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/boxedpin_ndr.json

# 19a. sat_pin_escape_point: the same boxed pin under -satpoint; a pin with no escape
#      point at all is flagged by the point model too.
LOGMUST="model=point|no escape for pin : BOXEDPIN_CONC_0/I_A0/P net "
ALLOW_UNROUTED=1
run_case sat_pin_escape_point "" \
  -d $IN/layers.json -p $IN/boxedpin.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/boxedpin_ndr.json -satpoint

# 19b. sat_spacing_gap: same boxed-pin layout as above, but the M2 obstacle
#      over I_A0's via-up candidate is offset 20 units away (M2 space=24) --
#      close enough to violate minimum spacing, but not literally overlapping.
#      The SAT pre-check's hitsObstacle() once tested raw overlap only, so it
#      declared this via candidate clear and the pin "feasible" even though
#      the real router's isViaValid (which always bloats by spacing) rejects
#      it -- the net then genuinely fails to route (ROUTE_SUMMARY
#      unrouted=1), contradicting the SAT check's own "guaranteed escape"
#      verdict. LOGMUST now asserts the SAT check catches this itself.
LOGMUST="pin escape SAT (pre-route) : BOXEDPIN_CONC_0 is infeasible|no escape for pin : BOXEDPIN_CONC_0/I_A0/P"
ALLOW_UNROUTED=1
run_case sat_spacing_gap "" \
  -d $IN/layers.json -p $IN/sat_spacing_gap.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/sat_spacing_gap_ndr.json

# 19c. sat_clash: two DIFFERENT nets (A, B) each have one pin (I_A0, I_B0)
#      boxed on three sides so their only escape is a via straight up to M2;
#      the two pins sit immediately adjacent on M1 (I_A0 at x=[100,132],
#      I_B0 touching at x=[132,164]) so their M2 via footprints are within
#      the M2 spacing of each other. Escape::feasible's own per-pin
#      candidates are each individually fine, but the cross-net clash clause
#      (Escape.cpp's "escapes of different nets that... would clash") makes
#      choosing both simultaneously UNSAT -- this is the only existing
#      fixture that ever exercises that clause, and the SAT solver's own
#      conflict/undo() path in Sat.h (previously 0% covered; verified via
#      `make coverage`). LOGMUST checks the specific "mutually conflict
#      (unsat)" reason string, distinct from sat_pin_escape's/
#      sat_spacing_gap's "no possible escape" (a single hard-blocked pin,
#      resolved before the SAT solver even runs).
LOGMUST="pin escape SAT (pre-route) : SATCLASH_CONC_0 is infeasible (escapes mutually conflict (unsat))"
ALLOW_UNROUTED=1
run_case sat_clash "" \
  -d $IN/layers.json -p $IN/sat_clash.placement_verilog.json \
  -l $IN/sat_clash.lef -ndr $IN/sat_clash_ndr.json

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
cli_check bad_threads_arg "invalid -threads value" \
  -d $IN/layers.json -p $IN/reorder.placement_verilog.json -l $IN/m1adj_escape.lef \
  -ndr $IN/reorder_ndr.json -threads xyz
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

# bad_ndr_via: an NDR "vias" WidthX given as a JSON string instead of a
#     number. readNDR() read every via field (WidthX/Y, SpaceX/Y, NumX/Y) via
#     a bare `*itvia` conversion with no type check -- unlike layers.json's
#     fields (see bad_layers_json above, and the is_number() widening this
#     suite already covers via layers_float_width), so a type mismatch threw
#     an uncaught json::type_error and aborted the whole process. Must just
#     silently default that one field to 0 and route normally, matching how
#     every other malformed-but-present NDR field in this codebase behaves.
run_case bad_ndr_via "TEST_CONC_0.def,BLOCK_B_CONC_0.def" \
  -d $IN/layers.json -p $IN/test.placement_verilog.json -l $IN/test.lef -ndr $IN/bad_ndr_via.json

# 26. reorder_reroute: a harder 6-net criss-cross (gap fits fewer than 6) that the
#     reorder search improves on (promotes blocked nets) but cannot fully solve;
#     ALLOW_UNROUTED because the case is intentionally over-subscribed. The
#     "re-route with the best ordering" replay path (taken when the final pass's
#     ordering isn't the best one found mid-search) isn't reliably exercised by
#     this fixture any more -- the current candidate-point geometry converges to
#     the same ordering by the last pass -- so it's no longer asserted here.
#     Fixture: 6 nets, wall gap [300,440] (criss-cross, capacity < demand).
LOGMUST="promoting blocked nets up the routing order"
ALLOW_UNROUTED=1
run_case reorder_reroute "REORDER_CONC_0.def" \
  -d $IN/layers.json -p $IN/reorder_reroute.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/reorder_reroute_ndr.json

# 27. m2_pin_escape: a 2-pin net whose pins sit on M2 (not the bottom layer M1).
#     The pin-escape SAT then builds a via-DOWN escape candidate (M2->M1), which
#     M1-only pins never trigger. LOGMUST checks the SAT ran for the M2 module.
LOGMUST="pin escape SAT (pre-route) : all 2 pins in M2T_CONC_0"
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

# 29a. global_hier: a sub-module uses global VDD but its instance's fa_map omits
#      it, as ALIGN writes it; the top must still route its VDD to the sub-module.
LOGMUST="routing : VDD__X_SUB/VDD_port_0__I_T/P_port_0"
run_case global_hier "GH_CONC_0.def" \
  -d $IN/layers.json -p $IN/global_hier.placement_verilog.json -l $IN/m1adj_escape.lef

# 29a2. -nopattern: every wire goes to A*, none is accepted as an L/Z pattern,
#       and the pre-route escape check reports its instance size and outcome.
LOGMUST="pin escape SAT stats (pre-route) : module=GH_CONC_0 model=whole pins="
LOGNOT="sol found with pattern"
run_case nopattern "GH_CONC_0.def" \
  -d $IN/layers.json -p $IN/global_hier.placement_verilog.json -l $IN/m1adj_escape.lef -nopattern

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

# 36. MinSpacing: a metal layer whose "MinSpacing" is tighter than its pitch can
#     deliver (M1 pitch 56 - width 32 = 24, MinSpacing 40). The router keeps
#     routing at the pitch spacing but must warn that the final DRC check uses
#     the larger of the two, naming both numbers and the pitch that would be
#     needed. Covers DRC::MetalLayer::setMinSpace, which no other fixture sets.
run_case minspacing "BLOCK_B_CONC_0.def" \
  -d $IN/layers_minspace.json -p $IN/test1.placement_verilog.json -l $IN/test.lef
MSLOG="$OUTROOT/minspacing/err.log"
if grep -q "layer M1 pitch gives spacing 24 but MinSpacing is 40" "$MSLOG" 2>/dev/null; then
  echo "PASS minspacing_warning"
  PASS=$((PASS+1))
else
  echo "FAIL minspacing_warning :no-minspacing-warning;"
  FAIL=$((FAIL+1))
  ERRS="${ERRS}minspacing_warning:no-minspacing-warning;\n"
fi

# 37. deviation_cost: the symmetric-pair guide weight, read from the NDR module
#     entry alongside symmetric_nets. Pins the JSON key -> Module::_devweight
#     wiring (DRC/Placement::Module::setDeviationCost); this fixture's mirrored
#     route is already the natural optimum, so the value itself does not change
#     the result -- 0, 4 and 200 all give maxdev=0 -- and the assertion is that
#     a non-default weight parses and still produces the exact mirror.
LOGMUST="symmetric net : routing INM guided by INP|SYMMETRY module=SYM_CONC_0 pair=INP,INM axis=V:1000 maxdev=0 "
run_case deviation_cost "SYM_CONC_0.def" \
  -d $IN/layers.json -p $IN/symmetric.placement_verilog.json -l $IN/symmetric.lef \
  -ndr $IN/symmetric_devcost_ndr.json

# 38. blocked-via diagnostics: the via_escape fixture (net A cannot route without
#     -relaxvia) with per-wire debug dumping on. Every via escape is blocked, so
#     writeLEF emits a BLOCKED_VIA_* pin per attempt, and for each one the
#     obstacles that actually block it as a DRC_BLOCKED_VIA_* pin -- the latter
#     only appears when Router::intersectPObstacles returns shapes, which no
#     other case reaches.
export HANAN_DEBUG_WIRE=1
ALLOW_UNROUTED=1
run_case via_escape_blocked_diag "" \
  -d $IN/layers.json -p $IN/via_escape.placement_verilog.json \
  -l $IN/via_escape.lef -ndr $IN/via_escape_ndr.json
unset HANAN_DEBUG_WIRE
if grep -lq "PIN DRC_BLOCKED_VIA" "$OUTROOT/via_escape_blocked_diag"/ATTEMPT_*.lef 2>/dev/null; then
  echo "PASS via_escape_blocking_obstacles"
  PASS=$((PASS+1))
else
  echo "FAIL via_escape_blocking_obstacles :no-drc-blocked-via-pin;"
  FAIL=$((FAIL+1))
  ERRS="${ERRS}via_escape_blocking_obstacles:no-drc-blocked-via-pin;\n"
fi

# 39. replay: -replay re-routes a single wire straight out of a HANAN_DEBUG_WIRE
#     dump, with no placement or LEF, so a failing net can be debugged on its
#     own in the context it failed in. Reuses the ATTEMPT_*.lef that case 38
#     just wrote for the via_escape net, which cannot route without -relaxvia,
#     and checks the replay reaches the same verdict from the file alone.
RPDUMP=$(ls "$OUTROOT/via_escape_blocked_diag"/ATTEMPT_*_0_*.lef 2>/dev/null | head -1)
if [ -n "$RPDUMP" ]; then
  RPDIR="$OUTROOT/replay"; rm -rf "$RPDIR"; mkdir -p "$RPDIR"
  ( cd "$RPDIR" && "$ROUTER" -d "$IN/layers.json" -replay "../../$RPDUMP" \
      -ndr "$IN/via_escape_ndr.json" -o ./ >/dev/null 2>stderr.log )
  rperrs=""
  grep -q "REPLAY RESULT open" "$RPDIR/route.log" 2>/dev/null || rperrs="$rperrs no-open-verdict;"
  grep -q "obstacles=[1-9]" "$RPDIR/route.log" 2>/dev/null || rperrs="$rperrs no-obstacles-loaded;"
  grep -q "num src : [1-9]" "$RPDIR/route.log" 2>/dev/null || rperrs="$rperrs no-source-nodes;"
  grep -q "ndr=" "$RPDIR/route.log" 2>/dev/null || rperrs="$rperrs ndr-not-applied;"
  if [ -z "$rperrs" ]; then
    echo "PASS replay"; PASS=$((PASS+1))
  else
    echo "FAIL replay :$rperrs"; FAIL=$((FAIL+1)); ERRS="${ERRS}replay:$rperrs\n"
  fi
else
  echo "FAIL replay :no-attempt-dump-to-replay;"; FAIL=$((FAIL+1))
  ERRS="${ERRS}replay:no-attempt-dump-to-replay;\n"
fi

# ---------------------------------------------------------------------------
# Runtime work: escape thinning, blocked-escape pruning, the seed-polygon limit,
# the expansion budget, the reorder budget and its convergence stop, failure
# memoisation, the reachability pre-check, and the log verbosity levels.
#
# Two kinds of assertion. A feature that changes what the router does is checked
# by the log marker it emits, and by LOGNOT on the flag that turns it off -- so
# the case proves the switch switched something, not merely that it parsed. A
# change meant to leave routing alone is checked with same_defs: byte identity
# against the run it has to match.
# ---------------------------------------------------------------------------

# 34. maxexp: the A* node budget per search stage. A budget of 1 is a legal but
#     hopeless setting, so the pair proves the cap binds rather than being
#     ignored -- same fixture, routed at the default budget and open at 1.
cli_check bad_maxexp_arg "invalid -maxexp value" \
  -maxexp zzz -d $IN/layers.json -p $IN/net30.placement_verilog.json -l $IN/m1adj_escape.lef
LOGMUST=" -maxexp 1 "
ALLOW_UNROUTED=1
run_case maxexp_budget "" -maxexp 1 \
  -d $IN/layers.json -p $IN/net30.placement_verilog.json -l $IN/m1adj_escape.lef

# 35. escapepitch: escape points closer together than a wire pitch cannot serve
#     as distinct tracks, so only the one nearest each pin rectangle's centre is
#     seeded. -escapepitch 0 turns that off; the thinned run must seed strictly
#     fewer entry points, and both must still route all 30 nets.
run_case escapepitch_on "ESCB_CONC_0.def" \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json \
  -l $IN/escblock.lef -ndr $IN/escblock_ndr.json
run_case escapepitch_off "ESCB_CONC_0.def" -escapepitch 0 \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json \
  -l $IN/escblock.lef -ndr $IN/escblock_ndr.json
log_count_lt escapepitch_thins escapepitch_on escapepitch_off "^num src :" 4

# 36. blocked_escape_prune: escblock is a wide M1 pin with an obstacle whose
#     spacing bloat covers one end of it. Those escape points can neither be left
#     nor entered, so they are dropped before the search -- and the net still
#     routes through the clear end. -keepblockedescapes restores the old
#     behaviour and the "pruned" line must then be absent.
LOGMUST="pruned 1 blocked escape point"
run_case blocked_escape_prune "ESCB_CONC_0.def" \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json \
  -l $IN/escblock.lef -ndr $IN/escblock_ndr.json
LOGNOT="blocked escape point"
run_case blocked_escape_keep "ESCB_CONC_0.def" -keepblockedescapes \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json \
  -l $IN/escblock.lef -ndr $IN/escblock_ndr.json

# 37. seedpolys: splitpin's pin is four disjoint M1 stripes, so each end of the
#     net presents four polygons. -seedpolys 2 keeps only the two nearest the
#     other end and the net still routes; the default (0) seeds all of them, so
#     the marker must be absent.
LOGMUST="seeding 2 of 4 source pin polygon|seeding 2 of 4 target pin polygon"
run_case seedpolys_limit "SPLIT_CONC_0.def" -seedpolys 2 -seedpolysalways \
  -d $IN/layers.json -p $IN/splitpin.placement_verilog.json -l $IN/splitpin.lef
LOGNOT="pin polygon(s) nearest the other end"
run_case seedpolys_default "SPLIT_CONC_0.def" \
  -d $IN/layers.json -p $IN/splitpin.placement_verilog.json -l $IN/splitpin.lef

# 38. reorder budget: a block whose base route is already expensive cannot afford
#     ten reorder passes, so the count is scaled by measured search work. Needs a
#     block that actually reorders, which the over-subscribed reorder_reroute
#     fixture provides. -reorderbudget 0 restores the uncapped behaviour.
LOGMUST="capping reorder at"
ALLOW_UNROUTED=1
run_case reorder_budget "REORDER_CONC_0.def" -reorderbudget 1 \
  -d $IN/layers.json -p $IN/reorder_reroute.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/reorder_reroute_ndr.json
LOGNOT="capping reorder at"
ALLOW_UNROUTED=1
run_case reorder_uncapped "REORDER_CONC_0.def" -reorderbudget 0 \
  -d $IN/layers.json -p $IN/reorder_reroute.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/reorder_reroute_ndr.json

# 39. reorder_converged: passes that stop improving on the best end the loop
#     early rather than running out the allowance.
LOGMUST="no improvement in"
ALLOW_UNROUTED=1
run_case reorder_converged "REORDER_CONC_0.def" \
  -d $IN/layers.json -p $IN/reorder_reroute.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/reorder_reroute_ndr.json

# 40. search_skips: a wire that keeps failing is not searched again from
#     scratch. Both shortcuts fire on this fixture -- the memo when the whole
#     problem repeats unchanged, the reachability sweep when no target is
#     reachable at all -- and neither may invent a route (no shorts, and the
#     open count still matches the baseline the fixture is expected to leave).
LOGMUST="identical problem already failed|no target is reachable from any source"
ALLOW_UNROUTED=1
run_case search_skips "REORDER_CONC_0.def" \
  -d $IN/layers.json -p $IN/reorder_reroute.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/reorder_reroute_ndr.json

# 41. reachability_proof: a source pin boxed in on every side has no reachable
#     target, which the segment sweep proves without letting A* exhaust the grid.
LOGMUST="no target is reachable from any source"
ALLOW_UNROUTED=1
run_case reachability_proof "" \
  -d $IN/layers.json -p $IN/boxedpin.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/boxedpin_ndr.json

# 42. verbosity levels: -v is a level, not a switch. escblock emits exactly one
#     marker per level -- the via table at 1, the escape-point dump at 2, the
#     per-layer expansion breakdown at 3 -- so each case asserts its own level
#     arrived and that nothing above it leaked down.
LOGNOT="via : l:| points : |expanded :"
run_case verbose_default "ESCB_CONC_0.def" \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json \
  -l $IN/escblock.lef -ndr $IN/escblock_ndr.json
LOGMUST=" -v 1|via : l:"
LOGNOT=" points : |expanded :"
run_case verbose_net "ESCB_CONC_0.def" -v \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json \
  -l $IN/escblock.lef -ndr $IN/escblock_ndr.json
LOGMUST=" -v 2| points : |via : l:"
LOGNOT="expanded :"
run_case verbose_element "ESCB_CONC_0.def" -v 2 \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json \
  -l $IN/escblock.lef -ndr $IN/escblock_ndr.json
LOGMUST=" -v 3|expanded :| points : |via : l:"
run_case verbose_trace "ESCB_CONC_0.def" -v 3 \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json \
  -l $IN/escblock.lef -ndr $IN/escblock_ndr.json
same_defs verbose_defs_stable verbose_default verbose_trace "ESCB_CONC_0.def"

# 43. determinism: the node map is hashed rather than ordered now, and the
#     priority queue is an indexed heap rather than a tree, so neither sorted
#     iteration nor tree order is available to lean on. Two identical runs must
#     still produce identical DEFs.
run_case determinism_a "NET30_CONC_0.def" \
  -d $IN/layers.json -p $IN/net30.placement_verilog.json -l $IN/m1adj_escape.lef
run_case determinism_b "NET30_CONC_0.def" \
  -d $IN/layers.json -p $IN/net30.placement_verilog.json -l $IN/m1adj_escape.lef
same_defs determinism_stable determinism_a determinism_b "NET30_CONC_0.def"

# 50. satfirst: the pre-route escape check names the pins with no guaranteed
#     escape; -satfirst routes those nets before the rest. boxedpin's pin is one.
LOGMUST="1 net(s) with a pin the escape check could not clear, routing them first"
ALLOW_UNROUTED=1
run_case satfirst_order "" -satfirst \
  -d $IN/layers.json -p $IN/boxedpin.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/boxedpin_ndr.json

# 51. hopeless_retire: a net the escape check flagged that then routes nothing
#     for N whole attempts is retired from the reorder loop. -hopeless 1 makes
#     boxedpin's net retire after its first barren attempt; -hopeless 0 never
#     retires anything, so the marker must be absent.
LOGMUST="routed nothing in 1 attempt(s); not retrying it"
ALLOW_UNROUTED=1
run_case hopeless_retire "" -hopeless 1 \
  -d $IN/layers.json -p $IN/boxedpin.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/boxedpin_ndr.json
LOGNOT="not retrying it"
ALLOW_UNROUTED=1
run_case hopeless_never "" -hopeless 0 \
  -d $IN/layers.json -p $IN/boxedpin.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/boxedpin_ndr.json

# 52. hopeless_needs_sat: retirement requires the escape check's flag as well as
#     the barren attempts. reorder_reroute's nets fail for capacity, not for
#     escape, so none is flagged and none may be retired however low N is.
LOGNOT="not retrying it"
ALLOW_UNROUTED=1
run_case hopeless_needs_sat "REORDER_CONC_0.def" -hopeless 1 \
  -d $IN/layers.json -p $IN/reorder_reroute.placement_verilog.json \
  -l $IN/m1adj_escape.lef -ndr $IN/reorder_reroute_ndr.json

# 53. abutescape: a shape already touching a pin has no spacing to that pin left
#     to protect, so with -abutescape it does not block the pin's escape via --
#     but a pad that would actually run into it still does. escblock's obstacle
#     bloats over the pin without touching it, so the rule must NOT fire there:
#     this is the guard against it firing on merely-near shapes.
LOGNOT="ABUT "
run_case abut_not_near "ESCB_CONC_0.def" -abutescape \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json \
  -l $IN/escblock.lef -ndr $IN/escblock_ndr.json
# and with the flag off nothing changes for anyone
run_case abut_off "ESCB_CONC_0.def" \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json \
  -l $IN/escblock.lef -ndr $IN/escblock_ndr.json
same_defs abut_no_effect_when_clear abut_off abut_not_near "ESCB_CONC_0.def"

# 54. pindup_obs: an OBS rect covered by one of the macro's own pin polygons is
#     the pin redrawn as blockage, not a real blockage -- whether it matches the
#     pin rect exactly or merely sits inside it. It is dropped unconditionally,
#     and the result must match the same design without that copy. A clean
#     design must not report a drop at all.
LOGNOT="covered by a pin shape"
run_case pindup_clean "ESCB_CONC_0.def" \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json -l $IN/escblock.lef
LOGMUST="dropped 2 obstacle(s) covered by a pin shape"
run_case pindup_exact "ESCB_CONC_0.def" \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json -l $IN/pindupobs.lef
LOGMUST="dropped 2 obstacle(s) covered by a pin shape"
run_case pindup_inside "ESCB_CONC_0.def" \
  -d $IN/layers.json -p $IN/escblock.placement_verilog.json -l $IN/pininobs.lef
same_defs pindup_exact_as_clean pindup_clean pindup_exact "ESCB_CONC_0.def"
same_defs pindup_inside_as_clean pindup_clean pindup_inside "ESCB_CONC_0.def"

# 55. pattern_cycle: createNode returns the node already at a coordinate, so a
#     pattern path that comes back to a waypoint it has used would make that node
#     its own ancestor -- and buildSol walks parents, so it allocated via shapes
#     until the machine died (126GB on the design this wire came from). The
#     waypoint is a Z whose runs collapse: up a via and straight back down at the
#     same point. patterncycle.lef is that one wire, dumped with
#     HANAN_DEBUG_WIRE and replayed on its own: the pattern must be rejected in
#     favour of A*, the wire must still route, and no cycle may reach buildSol.
LOGMUST="pattern path revisits a node|REPLAY RESULT routed"
LOGNOT="cycle in the solution path"
run_case pattern_cycle "" -replay $IN/patterncycle.lef \
  -d $IN/layers_sky130.json -uu 1000 -v 2

# 55b. via_rotate: a supply tap on a 280-wide vertical M3 pin whose neighbours
#      sit at minimum spacing; the M3/M4 via's 370-wide pad overhangs it, so the
#      tap routes only with the via turned 90 degrees (the wire is replayed).
LOGMUST="REPLAY RESULT routed"
run_case via_rotate "" -replay $IN/viarotate_tap.lef -d $IN/layers_align_sky130.json -uu 1000 -v 2
LOGMUST="REPLAY RESULT open"
ALLOW_UNROUTED=1
EXPECT_EXIT=1
run_case via_rotate_off "" -replay $IN/viarotate_tap.lef -d $IN/layers_align_sky130.json -uu 1000 -v 2 -noviarotate

# 55c. halo_fallback: a strong-arm latch whose OUTP/TAIL connections need a grid line where a via pad clears
#      an obstacle; open after the corner-escape pass, routed by the -padhalo re-route of that hierarchy.
LOGMUST="re-routing with via-pad halo grid lines"
run_case halo_fallback "STRONG_ARM_LATCH_0.def" -d $IN/layers_sky130_bench.json -p $IN/halofallback.placement_verilog.json \
  -l $IN/halofallback.lef -ndr $IN/halofallback_ndr.json -uu 1000 -reorder 30
LOGMUST="unrouted=2"
ALLOW_UNROUTED=1
run_case halo_fallback_off "STRONG_ARM_LATCH_0.def" -d $IN/layers_sky130_bench.json -p $IN/halofallback.placement_verilog.json \
  -l $IN/halofallback.lef -ndr $IN/halofallback_ndr.json -uu 1000 -reorder 30 -nohalofallback

# 56. pwr_grid: bin/gen_pwr_grid.py builds the power grid out of the layer
#     abstraction and hanan_router makes the connections to it. The unit tests
#     cover the generator's geometry on synthetic abstractions; this case is the
#     flow itself -- generate a grid over the test design, hand the straps to the
#     router as another instance's pins, and require it to route everything with
#     no short against the new metal.
if python3 ./test_gen_pwr_grid.py >"$OUTROOT/pwr_grid_unit.log" 2>&1; then
  echo "PASS pwr_grid_unit ($(sed -n 's/^Ran \([0-9]*\) tests.*/\1/p' "$OUTROOT/pwr_grid_unit.log") checks)"
  PASS=$((PASS+1))
else
  echo "FAIL pwr_grid_unit : see $OUTROOT/pwr_grid_unit.log"
  FAIL=$((FAIL+1)); ERRS="${ERRS}pwr_grid_unit:generator-tests-failed;\n"
fi

# 57. check_nets: the router's open count is its own bookkeeping. bin/check_nets.py
#     checks the artefact -- flatten the routed GDS, join metal that touches and
#     metal a cut bridges, and require one conductor per top-level net. It caught
#     a two-pass power flow whose second pass overwrote the first pass's DEF, and
#     a tracer blind to a via sharing its GDS layer with a second abstraction
#     (sky130 draws V4 and CapMIMContact both on 70/44). These are the generator's
#     own tests on synthetic GDS; the flow cases above cover the router.
if python3 ./test_check_nets.py >"$OUTROOT/check_nets_unit.log" 2>&1; then
  echo "PASS check_nets_unit ($(sed -n 's/^Ran \([0-9]*\) tests.*/\1/p' "$OUTROOT/check_nets_unit.log") checks)"
  PASS=$((PASS+1))
else
  echo "FAIL check_nets_unit : see $OUTROOT/check_nets_unit.log"
  FAIL=$((FAIL+1)); ERRS="${ERRS}check_nets_unit:tracer-tests-failed;\n"
fi

pgdir="$OUTROOT/pwr_grid_route"
mkdir -p "$pgdir"
if python3 ../bin/gen_pwr_grid.py -l ./layers.json \
     -p ./pwrcell.placement_verilog.json --bottom M3 --top M4 --stride 2 \
     --avoid ./pwrcell.lef \
     --lef "$pgdir/pgrid.lef" --placement-out "$pgdir/pg_place.json" \
     >"$pgdir/gen.log" 2>&1; then
  cat ./pwrcell.lef "$pgdir/pgrid.lef" > "$pgdir/all.lef"
  # the grid must declare the cells' own database units, or the router scales it
  # differently and the straps collapse to nothing
  grep -q "DATABASE MICRONS UNITS 1;" "$pgdir/pgrid.lef" \
    || { echo "FAIL pwr_grid_units : grid LEF does not match the cell LEF units"
         FAIL=$((FAIL+1)); ERRS="${ERRS}pwr_grid_units:unit-mismatch;\n"; }
  # every supply pin taps the grid: the DEF must show the supplies climbing off
  # M1 onto a grid layer. M3 is the lower grid layer and the one the taps land
  # on; this PDK's V3 enclosure is wider than its M3/M4 straps, so the M3-to-M4
  # stitch has nowhere to go here and the unit tests cover stitching instead.
  NETROUTED="VDD|VSS"
  LOGNOT="coincident with net"
  run_case pwr_grid_route "PWRB_CONC_0.def" \
    -d $IN/layers.json -p pg_place.json -l all.lef
  pgdef="$pgdir/PWRB_CONC_0.def"
  if [ -s "$pgdef" ] && grep -q "+ RECT M3" "$pgdef" && grep -q "+ RECT V2" "$pgdef"; then
    echo "PASS pwr_grid_taps"; PASS=$((PASS+1))
  else
    echo "FAIL pwr_grid_taps : no M1-to-grid connection for the supplies"
    FAIL=$((FAIL+1)); ERRS="${ERRS}pwr_grid_taps:no-tap;\n"
  fi
else
  echo "FAIL pwr_grid_route : grid generation failed"
  FAIL=$((FAIL+1)); ERRS="${ERRS}pwr_grid_route:gen-failed;\n"
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
