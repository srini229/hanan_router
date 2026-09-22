"""Drawn corridors, corridor pitch, the guide weight, nearest-path selection,
and the KLayout macros' router lookup -- all headless (klayout.db only).

Runs under `make test KLAYOUT=1`; see test_klayout_gui_complete.py for the
basic complete-net cases this builds on."""
import json
import os
import re
import subprocess
import sys
import tempfile
import unittest

import klayout.db as db

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
ROUTER = os.path.join(REPO, "hanan_router")
sys.path.insert(0, os.path.join(REPO, "bin"))
import klayout_autocomplete as ka  # noqa: E402
from test_klayout_gui_complete import LAYERS, net_span  # noqa: E402

# net B's two fragments and the zigzag drawn for it, in dbu (= router units)
PIN_A = db.Box(0, 20000, 1000, 21000)
PIN_B = db.Box(20000, 20000, 21000, 21000)
ZIGZAG = [(20500, 20500), (20500, 28800), (15295, 28800), (11655, 28800),
          (11655, 24875), (500, 24875), (500, 20500)]
STRAIGHT_FAR = [(500, 500), (20500, 500)]          # a corridor for some other net
STRAY_FAR = [(5000, 43000), (11600, 43000), (11600, 37000)]  # nowhere near either


def build_fixture(path, paths):
    """Two M1 fragments of net B (and, with STRAIGHT_FAR drawn, two of net A
    along y=500), plus the given Path shapes on the waypoint layer."""
    ly = db.Layout()
    ly.dbu = 0.001
    top = ly.create_cell("TOP")
    m1, m1lbl = ly.layer(68, 20), ly.layer(68, 5)
    wp = ly.layer(*ka.WAYPOINT_LAYER)
    for box in (PIN_A, PIN_B):
        top.shapes(m1).insert(box)
        top.shapes(m1lbl).insert(db.Text("B", db.Trans(box.center().x, box.center().y)))
    if STRAIGHT_FAR in paths:
        for x in (0, 20000):
            top.shapes(m1).insert(db.Box(x, 0, x + 1000, 1000))
            top.shapes(m1lbl).insert(db.Text("A", db.Trans(x + 500, 500)))
    for p in paths:
        top.shapes(wp).insert(db.Path([db.Point(*q) for q in p], 10))
    ly.write(path)


def seg_dist(px, py, a, b):
    (x1, y1), (x2, y2) = a, b
    if x1 == x2:
        return abs(px - x1) + max(0, min(y1, y2) - py, py - max(y1, y2))
    return abs(py - y1) + max(0, min(x1, x2) - px, px - max(x1, x2))


def mean_deviation(layout, net_name, polyline):
    """Mean Manhattan distance of the net's routed wire from `polyline`,
    sampled along every wire rectangle longer than a pin."""
    metals, labels, vias, _d, _w = ka.load_layers(LAYERS_PATH[0])
    islands, _ = ka.net_islands(layout, layout.top_cell(), metals, labels, vias, net_name)
    ds = []
    for island in islands:
        for lname, region in island.items():
            for poly in region.each():
                b = poly.bbox()
                if max(b.width(), b.height()) < 1500:
                    continue
                pts = ([(x, b.center().y) for x in range(b.left, b.right + 1, 100)]
                       if b.width() >= b.height() else
                       [(b.center().x, y) for y in range(b.bottom, b.top + 1, 100)])
                for (px, py) in pts:
                    ds.append(min(seg_dist(px, py, polyline[i], polyline[i + 1])
                                  for i in range(len(polyline) - 1)))
    return sum(ds) / len(ds) if ds else float("inf")


LAYERS_PATH = [None]


class DrawnCorridor(unittest.TestCase):
    def setUp(self):
        if not os.access(ROUTER, os.X_OK):
            self.skipTest(f"{ROUTER} not built (run make first)")
        self.d = tempfile.mkdtemp()
        self.layers = os.path.join(self.d, "layers.json")
        with open(self.layers, "w") as f:
            json.dump(LAYERS, f)
        LAYERS_PATH[0] = self.layers

    def route(self, gds, **kw):
        ly = db.Layout()
        ly.read(gds)
        res = ka.complete_net_on_layout(ly, ly.top_cell(), self.layers, "B", ROUTER, self.d,
                                        waypoints_layer_spec=ka.WAYPOINT_LAYER, **kw)
        self.assertEqual(res["status"], "completed", res.get("log", "")[-800:])
        self.assertEqual(net_span(ly, ly.top_cell(), self.layers, "B"), 1)
        return ly, res

    def test_drawn_corridor_is_followed(self):
        """Pins on one horizontal line, corridor drawn as a zigzag: the route
        must go up through the zigzag, not straight across."""
        gds = os.path.join(self.d, "f.gds")
        build_fixture(gds, [ZIGZAG])
        ly, _ = self.route(gds)
        # a straight route never leaves y~21000; the corridor's lower run is at
        # y=24875, so anything above 23000 went up through the drawn detour
        metals, labels, vias, _d, _w = ka.load_layers(self.layers)
        islands, _ = ka.net_islands(ly, ly.top_cell(), metals, labels, vias, "B")
        top = max(poly.bbox().top for island in islands for region in island.values()
                  for poly in region.each())
        self.assertGreater(top, 23000, "route did not follow the drawn corridor")

    def test_pitch_and_guide_weight_reach_the_ndr(self):
        gds = os.path.join(self.d, "f.gds")
        build_fixture(gds, [ZIGZAG])
        self.route(gds, corridor_pitch=2, corridor_guide_weight=1.5)
        with open(os.path.join(self.d, "autocomplete_ndr.json")) as f:
            net = [n for n in json.load(f)[0]["nets"] if n["name"] == "B"][0]
        self.assertEqual(net["corridor_pitch"], 2)
        self.assertEqual(net["corridor_guide_weight"], 1.5)
        self.assertEqual(len(net["corridor_topology"]), len(ZIGZAG))

    def test_guide_weight_pulls_the_route_onto_the_line(self):
        gds = os.path.join(self.d, "f.gds")
        build_fixture(gds, [ZIGZAG])
        ly0, _ = self.route(gds, corridor_pitch=4)
        ly1, _ = self.route(gds, corridor_pitch=4, corridor_guide_weight=1)
        d0, d1 = mean_deviation(ly0, "B", ZIGZAG), mean_deviation(ly1, "B", ZIGZAG)
        self.assertLess(d1, d0, f"guided {d1} not closer to the line than unguided {d0}")

    def test_single_net_picks_the_nearest_of_several_paths(self):
        """With a stray path drawn far away *first*, Complete Net must still
        take the zigzag that actually runs between B's pins."""
        gds = os.path.join(self.d, "f.gds")
        build_fixture(gds, [STRAY_FAR, ZIGZAG])
        ly = db.Layout()
        ly.read(gds)
        metals, labels, vias, _d, _w = ka.load_layers(self.layers)
        islands, _ = ka.net_islands(ly, ly.top_cell(), metals, labels, vias, "B")
        self.assertEqual(ka.nearest_waypoint_path(ly, ly.top_cell(), islands), ZIGZAG)
        self.assertEqual(ka.read_waypoints(ly, ly.top_cell()), STRAY_FAR)  # the old rule
        ly, _ = self.route(gds)   # and routing really uses the zigzag
        with open(os.path.join(self.d, "autocomplete_ndr.json")) as f:
            net = [n for n in json.load(f)[0]["nets"] if n["name"] == "B"][0]
        self.assertEqual([tuple(round(v * 1000) for v in p) for p in net["corridor_topology"]], ZIGZAG)

    def test_batch_matches_each_net_to_its_own_path(self):
        gds = os.path.join(self.d, "f.gds")
        build_fixture(gds, [STRAIGHT_FAR, ZIGZAG])
        ly = db.Layout()
        ly.read(gds)
        res = ka.complete_nets_on_layout(ly, ly.top_cell(), self.layers, ["A", "B"], ROUTER, self.d)
        for n in ("A", "B"):
            self.assertEqual(res[n]["status"], "completed", res[n])
            self.assertTrue(res[n]["had_corridor"], n)
        with open(os.path.join(self.d, "autocomplete_ndr.json")) as f:
            nets = {n["name"]: n for n in json.load(f)[0]["nets"]}
        self.assertEqual(len(nets["A"]["corridor_topology"]), len(STRAIGHT_FAR))
        self.assertEqual(len(nets["B"]["corridor_topology"]), len(ZIGZAG))


class MacroRouterLookup(unittest.TestCase):
    """The KLayout macros must compile and must find the router the documented
    way, without a KLayout GUI: their lookup block is run under controlled
    environments."""
    MACROS = [os.path.join(REPO, "klayout", "pymacros", n)
              for n in ("hanan_complete_net.lym", "hanan_complete_all_nets.lym")]

    def body(self, path):
        src = open(path).read()
        return re.search(r"<!\[CDATA\[(.*?)\]\]>", src, re.S).group(1)

    def test_macros_compile(self):
        for m in self.MACROS:
            compile(self.body(m), m, "exec")

    def lookup(self, env):
        code = self.body(self.MACROS[0])
        block = code[code.index("def _find_router"):code.index("LAYERS_JSON = os.environ")]
        prog = "import os, shutil, sys\n" + block + "\nprint(router_bin, bindir, ka is not None)"
        r = subprocess.run([sys.executable, "-c", prog], env=env, capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        return r.stdout.split()

    def test_router_from_path(self):
        if not os.access(ROUTER, os.X_OK):
            self.skipTest("router not built")
        out = self.lookup({"PATH": REPO + os.pathsep + "/usr/bin:/bin", "HOME": os.environ.get("HOME", "/")})
        self.assertEqual(os.path.realpath(out[0]), os.path.realpath(ROUTER))
        self.assertEqual(out[1], os.path.join(REPO, "bin"))
        self.assertEqual(out[2], "True")

    def test_router_from_hanan_router_dir(self):
        if not os.access(ROUTER, os.X_OK):
            self.skipTest("router not built")
        out = self.lookup({"PATH": "/usr/bin:/bin", "HANAN_ROUTER_DIR": REPO, "HOME": os.environ.get("HOME", "/")})
        self.assertEqual(os.path.realpath(out[0]), os.path.realpath(ROUTER))
        self.assertEqual(out[2], "True")

    def test_nothing_set_is_reported_not_guessed(self):
        out = self.lookup({"PATH": "/usr/bin:/bin", "HOME": os.environ.get("HOME", "/")})
        self.assertEqual(out, ["None", "None", "False"])


if __name__ == "__main__":
    unittest.main()
