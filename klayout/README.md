# KLayout integration

Two Python macros that route from inside the KLayout GUI with `hanan_router`:

- `pymacros/hanan_complete_net.lym` -- **Complete Net** (Ctrl+Shift+M): select a
  shape on a broken net and route that net. Optionally follows a corridor drawn
  as a Path on layer 996/0, with a pitch (corridor width) and a guide weight
  (pull toward the drawn line).
- `pymacros/hanan_complete_all_nets.lym` -- **Route All Nets**: every broken net
  in the open hierarchy, each picking the nearest drawn corridor, if any.

Both are thin: the logic is `bin/klayout_autocomplete.py` in this repo, which
is also what `test/test_klayout_gui_complete.py` tests headlessly.

## Install

    ./klayout/install.sh          # symlinks the macros into ~/.klayout/pymacros
    ./klayout/install.sh --copy   # or copy them instead

Then restart KLayout (or Macros > Macro Development > reload). The macros appear
under the **Sky130** menu; open the layout in editor mode (`klayout -e`).

## Locating the router

The macros do not hard-code any path. In the environment KLayout is launched
from:

- `hanan_router` on `$PATH` is used if found; otherwise
- `$HANAN_ROUTER_DIR` must name the directory containing `hanan_router`
  (the repo root after `make`, or wherever it was installed).

`klayout_autocomplete.py` is looked up in `$HANAN_ROUTER_DIR/bin`, then next to
the router binary (`<dir>/bin/` or `<dir>/`).

`$HANAN_LAYERS_JSON` names the technology's layers.json (the file `-d` takes);
it defaults to the sky130 one used by the benchmarks.

Set these where KLayout will see them, e.g. in the shell that starts it, or a
wrapper:

    export HANAN_ROUTER_DIR=/path/to/hanan_router
    export HANAN_LAYERS_JSON=/path/to/sky130.layers.json
    klayout -e design.gds
