# Hanan Grid Router

[![CI](../../actions/workflows/ci.yml/badge.svg)](../../actions/workflows/ci.yml)

This is a rectilinear detail router useful for routing and visualizing signal nets on a few blocks (both digital and analog).
It uses a modified version of the [A\* search algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) at its core, with the routes laid on the [Hanan grid](https://doi.org/10.1137/0114025).

The following back-end-of-line design rules are honored:
* Manhattan spacing between metal and via shapes on each layer.
* Metal widths per layer.
* Single / multi-cut vias.
* User-specified non-default rules (NDR) at a block or net level. A user can override:
  - metal spacing and widths on any layer,
  - the set of routing layers allowed for a net,
  - per-layer routing direction,
  - net/block specific routing obstacles.

Beyond the core router it also provides:
* a **pre-routing pin-escape feasibility check** that proves (via a small SAT solver) that every pin can leave its cell before any net is routed;
* an **automatic net-ordering search** that, when nets are left open, promotes the blocked nets up the routing order and retries (reorder), bounded by a search-work budget and stopped once passes stop improving;
* an **adjacent-obstacle retry** that frees a blocked pin's escape layer;
* optional **multi-threaded routing** that routes non-overlapping nets in parallel (`-threads N`);


# Building the router

Requirements:
  * A C++ compiler with support for C++14 or higher (GCC or Clang).
  * `make`.
  * Boost development headers (used by `boost.polygon`); e.g. `libboost-dev` on Debian/Ubuntu.

The router has **no other external dependencies** — the JSON parser, LEF parser and R-Tree are vendored in the repository.

Steps to clone and build:
  ```
  git clone https://github.com/srini229/hanan_router.git
  cd hanan_router
  make -j4
  ```
The build has been verified on Ubuntu (GCC), Fedora (GCC), Debian on WSL (GCC) and macOS (Clang).

`make` produces a `hanan_router` binary in the repository root.
The repository bundles a frozen copy of the [`nlohmann/JSON`](https://github.com/nlohmann/json) parser (netlist, layers and constraints) and a copy of the [`RTree`](https://superliminal.com/sources/RTreeTemplate.zip) template used for fast geometric queries. It also has a small LEF parser that understands basic LEF syntax.

The JSON file formats reuse the syntax from the [ALIGN](https://github.com/ALIGN-analoglayout/ALIGN-public) project. A generic version using standard LEF/DEF for the netlist/design rules is a work in progress.


# Usage

```
hanan_router -d <layers.json> -p <netlist.json> -l <lef file> [options]
```

| Option | Required | Description |
|--------|----------|-------------|
| `-d <layers.json>` | yes | Abstracted information for each metal/via layer of the technology. |
| `-p <netlist.json>` | yes | Netlist and placement in JSON: instances with their transforms, and nets (the format of ALIGN's placement output). |
| `-l <lef file>` | yes | LEF with the pin/blockage (OBS) information for each leaf cell. |
| `-ndr <ndr.json>` | no | User-specified non-default rules (see [NDR constraints](#ndr-constraints)). |
| `-o <output dir>` | no | Output directory for the generated LEF/DEF (default `./`). |
| `-r <precision>` | no | Coordinate precision / rounding (default `1`). |
| `-reorder <N>` | no | Max alternate net-ordering passes to try when nets remain unrouted (default `10`; `0` disables the search). |
| `-reorderbudget <N>` | no | Cap a block's reorder passes so `base-route expansions x passes` stays under `N` (default `15000000`; `0` = uncapped). A block whose first route is already expensive cannot afford ten more of them. See [Search effort](#search-effort). |
| `-maxexp <N>` | no | A\* node-expansion budget per search stage (default `100000`). Lower is faster and leaves more nets open. |
| `-escapepitch <N>` | no | Thin pin-escape points to one per `N` wire pitches, keeping the one nearest each pin rectangle's centre (default `1`; `0` seeds every candidate). See [Pin escape points](#pin-escape-points). |
| `-keepblockedescapes` | no | Seed every pin-escape point, including ones sitting inside a bloated obstacle that no wire can start from. Default is to drop them. |
| `-satfirst` | no | Route the nets whose pins the pre-route escape check could not clear before the rest. Off by default; see [Search effort](#search-effort). |
| `-hopeless <N>` | no | Retire a net the escape check flagged once it has routed nothing in `N` whole attempts (default `3`; `0` never retires). |
| `-abutescape` | no | A shape already touching a pin does not block that pin's escape via on spacing; only a pad that would overlap it does. Off by default; see [Abutting shapes and pin escapes](#abutting-shapes-and-pin-escapes). |
| `-viaalign` | no | When more than one via rotation is legal, pick the one whose pads run along the wires they join rather than the first legal one. Off by default; see [Via selection](#via-selection). |
| `-seedpolys <N>` | no | For a pin split into more than `N` polygons, seed only the `N` nearest the other terminal (default `0` = seed all). |
| `-seedpolysalways` | no | Keep that limit on every attempt; by default the rest are restored for a net still open by attempt 3. |
| `-rsmt` | no | Confine each net to a Borah-Owens-Irwin Steiner corridor over its pins. |
| `-viacost <r\|P\|0>` | no | How a via is priced. `r` (default): the length of the narrower metal it joins that has the same resistance. `P`: at least `P` wire pitches of the cheaper metal. `0`: its bare layer-file resistance. |
| `-padhalo` | no | Also put grid lines where a via pad, not only a wire, clears each obstacle. Off by default; a hierarchy still open at the end is re-routed with them. |
| `-nohalofallback` | no | Do not re-route still-open hierarchies with `-padhalo` lines at the end. |
| `-admissible` | no | Use the admissible distance bound in every A\* search, not only under a corridor guide. |
| `-nopattern` | no | Skip L/Z pattern routing; every wire goes to A\*. |
| `-noviarotate` | no | Offer each single-cut via only as the layer file draws it, not also turned 90 degrees. |
| `-gridprune <pct>` | no | Collapse grid coordinates closer than `pct` percent of the pitch. |
| `-satpoint` | no | The pre-route escape check takes escapes from points along each pin, as the search does. |
| `-reserveexcluded` | no | Keep one escape clear for every pin of a `do_not_route` net, for a later pass. |
| `-noseed` | no | Do not seed grid lines inside corridor bands. |
| `-softwires` | no | Measurement only: other nets' wires are not obstacles, only their pins, so the result overlaps nets and is not legal. A net still open this way is limited by its pins or the grid, not by contention for routing space. |
| `-relaxvia` | no | In the final pass, retry a still-unrouted net's escape via with spacing relaxed to as close as 5 to (never on) a shape -- source pins first, then target pins too. |
| `-threads <N>` | no | Route non-overlapping nets in parallel using `N` worker threads (default `1` = sequential). See [Parallel routing](#parallel-routing). |
| `-uu <scale>` | no | User-units scaling for the placement file (e.g. nm/um). |
| `-s` | no | Treat the LEF as scaled by `-uu`. |
| `-uil <dir>` | no | Reuse previously-generated interim LEFs from `<dir>` for hierarchical blocks. |
| `-sep <str>` | no | Hierarchy name separator. |
| `-log <file>` | no | Log file name (default `route.log`). |
| `-v [N]` | no | Log verbosity (default `0`). See [Log verbosity](#log-verbosity). |
| `-replay <ATTEMPT_*.lef>` | no | Re-route one wire from a `HANAN_DEBUG_WIRE` dump; needs `-d` only. |
| `-detour` | no | With `-replay`: allow a large detour even without NDR saying so. |


## Output

The router generates one LEF and one DEF for each hierarchy in the placement; the interim per-block LEF has the suffix `_interim_hier.lef`. A `route.log` with a detailed activity log is also written.

For every routed module the log contains an authoritative summary line:
```
ROUTE_SUMMARY module=<name> nets=<routable nets> unrouted=<count>
```
`unrouted=0` means the module routed completely. Single-pin nets (nothing to route) are excluded from both counts. This line is the recommended hook for scripts/CI; the smoke harness reads it directly.

The LEF/DEF can be visualized with [`klayout`](https://klayout.de/).


# Routing behavior

* **Pin-escape feasibility (SAT).** Before routing, the router proves each pin has at least one legal escape (a via up/down or a same-layer stub) without different nets' escapes clashing. Infeasible pins are reported up front (`pin escape SAT : ...`).
* **Net ordering search.** If a first pass leaves nets open, the router retries with the nets that were blocked promoted up the order (`promoting blocked nets up the routing order`), iterating up to `-reorder N` passes and keeping the order that routes the most nets. It re-routes the best order found if needed (`re-routing with best ordering`).
* **Adjacent-obstacle retry.** When a net's straight wire would cap another net's M1 pin, the blocking pin is projected onto the adjacent metal as an obstacle so the first net detours and leaves the escape free.
* **Coincident-pin merge.** Two different nets whose pins sit on the same point are physically one node; the router warns (`... has pin(s) coincident with net ...; merging them into one connected net`) and merges them so they route connected instead of shorting.
* **Unconnected-pin protection.** Instance pins not wired to any net are added as obstacles so no route lands on them.
* **Symmetric-net guiding.** Net pairs marked `symmetric_nets` are routed so they appear mirror-symmetric: the first net is routed, its solution mirrored across the pair's axis, and that mirror used as a soft *deviation-cost* guide for the second net's A\* search (see [Symmetric nets](#symmetric-nets)).
* **Obstacle locality.** A net's search never leaves its pin bounding box expanded by a bounded margin, so the module's obstacles are indexed in an R-tree and each net is given only the obstacles near it. This keeps per-net routing cost proportional to the *local* obstacle count rather than the whole design's, without changing the routed result.

* **Failure shortcuts.** A search that already failed is not run again. The exact problem -- obstacles, entry points and the flags that steer expansion -- is fingerprinted, and a repeat is skipped (`identical problem already failed`). A wire that has failed once in the block is also checked for reachability first: the free intervals of the Hanan grid are walked from the sources, and if no target segment can be reached the search is skipped outright (`no target is reachable from any source`). Both are exact -- A\* is deterministic, and the reachability walk over-approximates what the search can do, so only a negative answer is acted on.

## Search effort

The reorder search and the A\* budget are both bounded, because a block that is
expensive to route once is expensive to route ten times:

* `-reorderbudget <N>` scales a block's reorder passes by the search work its *first* attempt cost, so small blocks keep all ten and a large one gets fewer (`capping reorder at K pass(es) instead of 10`). It keys off expansions rather than wall time, so a run stays reproducible.
* Passes that stop improving on the best end the loop early (`no improvement in N pass(es); stopping at K/N`).
* Before every search, the free intervals of the Hanan grid are walked from the sources; if no target can be reached the search is skipped as provably hopeless (`no target is reachable from any source`). The walk over-approximates what the search can do, so only the negative answer is acted on.
* The pre-route pin-escape check names the pins with no guaranteed escape. A net it flagged that then routes nothing in `-hopeless N` whole attempts is retired from the reorder loop -- the two signals have to agree, since either alone retires nets that would have routed. `-satfirst` additionally routes the flagged nets first, while the block is empty; that is faster but tends to cost wirelength, so it is off by default.
* `-maxexp <N>` caps the node expansions one search stage may spend. A search that fails spends the whole budget, so this is the main lever on runtime -- and on how many nets are left open. Lowering it is a direct trade.

## Pin escape points

Each pin gets a set of candidate escape points, and the number of them drives
both the size of the Hanan grid and the cost of seeding the search:

* `-escapepitch <N>` drops escape points closer together than `N` wire pitches, keeping the one nearest each pin rectangle's centre (the cheapest under the off-centre escape cost). Every pin rectangle keeps at least one escape in every direction it had one. `-escapepitch 0` restores every candidate.
* Escape points sitting inside an already-bloated obstacle -- which no wire can start from and no route can enter -- are dropped before the search, and the grid is rebuilt without the lines they contributed (`pruned N blocked escape point(s)`). `-keepblockedescapes` turns that off.
* `-seedpolys <N>` limits how many polygons of a multi-polygon pin are seeded, keeping those nearest the other terminal.

## Via selection

Where a technology defines several vias for a layer pair -- typically the same
cut rotated 90 degrees -- the router has to pick one. A via that lands **on a
pin** picks the rotation whose pad overlaps the pin most (`VIAPAD kept/switched`
at `-v 2`). A via **away from a pin** takes the first legal candidate.

`-viaalign` scores the away-from-a-pin case too: +2 when the pad on the node's
own layer runs along the wire arriving there, +1 when the pad on the far layer
runs along that layer's preferred direction. Pin overlap still decides wherever
there is a pin; alignment only breaks its ties.

It is off by default because it has not been shown to pay -- see
`docs/ROUTING_NOTES.md`. On a technology with square cuts it is a no-op.

### Abutting shapes and pin escapes

A shape that already touches a pin has no spacing to that pin left to protect --
the pin is up against it whatever the via does -- yet the bloated obstacle it
becomes will cover the pin and block every escape via there. `-abutescape` judges
such a shape on overlap alone: the escape pad may sit beside it, but must not run
into it. Abutment is tested against the drawn geometry rather than an un-bloated
copy of the obstacle, because the obstacle rects are merged and split as they are
built and do not un-bloat back to the drawn shape.

It is off by default. On the design it was written for it routes one net more
than the default rule, runs 1.6x faster, and adds no short and no router-caused
DRC. See `docs/ROUTING_NOTES.md`.

### Blockage covered by a pin

An OBS rect that lies inside one of the macro's own pin polygons -- whether it
repeats the pin rect exactly or merely sits within it -- is the pin redrawn as
blockage. It is dropped as the cell's obstacles are read, unconditionally. It
protects nothing: a pin is already an obstacle to every other net in its own
right, and the pin shape is what the DRC check reports against. Kept, it would
only wall that pin's own escapes in.

## Log verbosity

`-v` takes a level rather than being a switch. `-v` alone means level 1; `-v <N>`
or `HANAN_VERBOSE=<N>` selects it.

| Level | Contents |
|-------|----------|
| `0` (default) | Results only: what routed, what is open, DRC, shorts, wirelengths, summaries. |
| `1` | Per-net detail: the pin pairs chosen, the via table, same-net obstacle filtering, where routes were attached. |
| `2` | Per-element dumps: every escape point, every via-pad decision, the per-layer cost and width tables. |
| `3` | Everything, including the per-layer expansion breakdown of each search. |

The level changes only what is written; the routed result is identical at every
level.

## Parallel routing

With `-threads N` (`N > 1`) the router routes nets concurrently. Routing order
still matters for nets that compete for the same space, so the router only
parallelizes nets that **cannot** interfere with one another:

* The ordered net list is split into consecutive **batches** of nets whose
  (inflated) pin bounding boxes are mutually disjoint. A net's A\* search stays
  within its pin box expanded by ~50% (clamped to the block), so disjoint
  inflated boxes guarantee disjoint search regions — the routes cannot overlap.
* Batches run one after another (preserving order); the nets **within** a batch
  are routed in parallel by a pool of `N` worker threads that pull nets off a
  shared queue under a mutex, each thread using its own private router state.
* Nets that need full sequential context — pinned `routing_order` nets, excluded
  (`do_not_route`) nets and `large_detour` nets — are never parallelized.

The result is identical to a sequential run: the routed geometry does not depend
on `N`, and `checkShort()` still verifies the final layout. `-threads 1` (the
default) is exactly the original sequential behavior.


# NDR constraints

The optional `-ndr <file>` is a JSON array of per-module objects. Module-level keys apply to all of a module's nets; a `nets` list overrides per net.

```json
[
  {
    "module": "BLOCK_CONC_0",
    "directions":   { "M3": "H", "M4": "V" },
    "use_pin_width": 1,
    "routing_order": ["CLK", "D"],
    "obstacles": [ { "shapes": { "M2": [[100, 0, 160, 400]] } } ],
    "nets": [
      {
        "name": "CLK",
        "widths":           { "M3": 60, "M4": 72 },
        "spaces":           { "M3": 60, "M4": 72 },
        "preferred_layers": ["M3", "M4"],
        "large_detour":     "allowed",
        "virtual_pins":     [ { "M3": [[1250, 190, 1282, 222]] } ]
      },
      { "name": "D", "do_not_route": 1 }
    ],
    "clock_nets": [ { "name": "CLK", "driver": "U1/Y" } ],
    "symmetric_nets": [ ["INP", "INM"], ["OUTP", "OUTM", { "V": 1250 }] ]
  }
]
```

| Key | Scope | Meaning |
|-----|-------|---------|
| `widths` / `spaces` | module / net | Per-layer non-default metal width / spacing. |
| `preferred_layers` | module / net | Restrict routing to these layers (raises the cost of others). |
| `directions` | module / net | Per-layer routing direction (`H`/`V`/`O`). |
| `large_detour` | net | `"allowed"` lets the net route well outside its pin bounding box. |
| `routing_order` | module | Pin/route these nets first, in the given order. |
| `use_pin_width` | module | Match wire width to the pin width where it helps escape. |
| `do_not_route` | net | Exclude the net from routing (kept as an obstacle). |
| `obstacles` | module / net | Extra routing blockages, as per-layer rectangle lists. |
| `virtual_pins` | net | Additional routing targets (e.g. a clock spine entry). |
| `clock_nets` | module | Mark a net as a clock with a named driver for ordering. |
| `symmetric_nets` | module | Pairs of nets to route mirror-symmetrically (see [Symmetric nets](#symmetric-nets)). |
| `deviation_cost` | module | Weight of the symmetry guide penalty (default `4`); higher hugs the mirror harder. |


## Symmetric nets

`symmetric_nets` takes a list of net-name pairs that should be routed as mirror
images of one another (e.g. the two halves of a differential pair). Each entry is
`["net1", "net2"]`, optionally followed by an explicit mirror axis
`{"V": x}` (a vertical line at `x`) or `{"H": y}` (a horizontal line at `y`),
given in the same user units as the placement:

```json
"symmetric_nets": [
  ["INP", "INM"],                 // axis auto-detected from the pins
  ["OUTP", "OUTM", { "V": 1250 }] // explicit vertical mirror axis at x = 1250
]
```

For each pair the router routes `net1` first, mirrors its solution across the
axis, and uses that mirrored route as a **guide** for `net2`: a *deviation cost*
is added to `net2`'s A\* search that penalises every node by its distance from the
guide, so the search is pulled toward the mirror. The two nets therefore come out
mirror-symmetric wherever the design allows it, and degrade gracefully (routing a
shortest path *near* the mirror) where obstacles make an exact mirror impossible.

* If no axis is given it is **auto-detected**: the orientation (vertical or
  horizontal) and position are taken from the two nets' pin bounding-box centres.
* Symmetric-pair nets are always routed sequentially (`net1` before `net2`) and
  are never parallelized, regardless of `-threads`.
* The guide is a soft bias, not a hard constraint, so a symmetric net never fails
  to route because of it. For every pair the log reports
  `SYMMETRY module=<m> pair=<n1>,<n2> axis=<V|H>:<pos> maxdev=<d> meandev=<d>`,
  where `maxdev`/`meandev` are the residual distance of `net2`'s route from the
  mirror (`0` = a perfect mirror).


# Tests

A smoke-test suite under `test/` exercises the example configurations: basic routing, NDR widths/spaces/directions/preferred-layers, virtual pins, clock nets, do-not-route, routing order, pin-width matching, large detours, NDR and LEF-OBS obstacles, custom via arrays, precision rounding, mirrored placements, debug dumps, interim-LEF reuse, the pin-escape SAT check, the net-ordering reorder search, coincident-pin merging, symmetric-net mirror routing (including an S-shaped serpentine pair), CLI/error-handling paths and a 30-net throughput case, plus the search-effort and
escape-point switches (`-maxexp`, `-escapepitch`, `-keepblockedescapes`,
`-seedpolys`, `-reorderbudget`), the reorder convergence stop, the failure
shortcuts, the verbosity levels, and two determinism guards -- run-to-run DEF
identity, and DEF identity across verbosity levels.

Run it with:
```
make test
```
Each case runs in `test/smoke_out/<case>/`. The harness checks the exit code, that the expected DEFs contain routes, that there are **no shorts**, and the per-module `ROUTE_SUMMARY` unrouted count, plus case-specific log assertions.

A slow (~25 s) 30-net **maze** stress case is gated off by default; enable it with:
```
MAZE_STRESS=1 make test
```
`test/maze_quality.py <module.def> <placement.json>` reports solution quality (completion, wirelength, via count and detour ratio) for a routed DEF.

A **parallel-routing** case (also off by default) generates a batch
of many disjoint, individually-expensive nets, routes it sequentially and with
several worker threads, and asserts the threaded run is both substantially faster
and produces identical wires. Enable it with:
```
PERF_STRESS=1 make test
```
It prints the measured speedup, e.g. `seq=0.30s 4-thread=0.16s 1.86x on 8 cores`.

Code coverage of the smoke suite (Clang source-based instrumentation):
```
make coverage
```
This builds an instrumented `hanan_router_cov`, runs the suite with it, and writes reports under `coverage/`: `summary.txt` (per file), `functions.txt` (per routine) and `html/index.html` (annotated sources).


# Visualization and GDSII

A GDSII generator is available in `bin/`. It is written in Python and requires `numpy`, `gdspy`, `json` and `argparse` (`pip install <package>` for any missing one):

```
usage: gen_rt_gds.py [-h] [-p PL_FILE] [-g GDS_DIR] [-t TOP_CELL] [-u UNITS] [-s SCALE] [-l LAYERS] [-d DEFF]
```

For example:
```
../bin/gen_rt_gds.py -p test.netlist.json -g . -t TEST_CONC_0 -l layers.json -d TEST_CONC_0.def
```
The generated GDSII can be viewed in klayout and streamed into commercial P&R tools.
