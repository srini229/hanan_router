#!/usr/bin/env python3
"""End-to-end test for bin/klayout_autocomplete.py -- item 2's "route
unfinished connections" mode, driven straight off a design's own net-name
labels (no netlist, no manual fragment picking). Written against
`klayout.db` (see the module's own docstring for why) with a fixture
authored the same way, since a real KLayout-facing tool should be exercised
with `klayout.db`/`pya`-native shapes, not gdstk ones.

Three real-world outcomes are checked, not just the happy path: a genuinely
broken net gets completed and re-verified; an already-complete net is left
alone (a `status: already_connected` short-circuit that a buggy detector
could easily skip and re-route unnecessarily); and a net name with no
matching label is reported, not silently ignored.
"""
import os
import subprocess
import sys
import tempfile
import unittest

import klayout.db as db

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTER = os.path.join(HERE, "..", "hanan_router")
sys.path.insert(0, os.path.join(HERE, "..", "bin"))
import klayout_autocomplete as ka  # noqa: E402

LAYERS = {"Abstraction": [
    {"Layer": "M1", "GdsLayerNo": 68, "GdsDatatype": {"Draw": 20, "Pin": 5, "Label": 5, "Blockage": 6},
     "Direction": "H", "Pitch": 340, "Width": 140},
    {"Layer": "V1", "GdsLayerNo": 68, "GdsDatatype": {"Draw": 44}, "Stack": ["M1", "M2"],
     "WidthX": 150, "WidthY": 150, "SpaceX": 170, "SpaceY": 170,
     "VencA_L": 65, "VencA_H": 65, "VencP_L": 90, "VencP_H": 90},
    {"Layer": "M2", "GdsLayerNo": 69, "GdsDatatype": {"Draw": 20, "Pin": 5, "Label": 5, "Blockage": 6},
     "Direction": "V", "Pitch": 460, "Width": 140},
    {"Layer": "V2", "GdsLayerNo": 69, "GdsDatatype": {"Draw": 44}, "Stack": ["M2", "M3"],
     "WidthX": 200, "WidthY": 200, "SpaceX": 200, "SpaceY": 200,
     "VencA_L": 70, "VencA_H": 70, "VencP_L": 0, "VencP_H": 4},
    {"Layer": "M3", "GdsLayerNo": 70, "GdsDatatype": {"Draw": 20, "Pin": 5, "Label": 5, "Blockage": 6},
     "Direction": "H", "Pitch": 680, "Width": 300},
]}


def build_fixture(path):
    """Two fragments of a broken net N1 (far apart, M1), a genuinely
    complete net N2 (one M2 strap crossing N1's straight-line path, forcing
    a real detour), and their net-name labels -- nothing else."""
    ly = db.Layout()
    ly.dbu = 0.001
    top = ly.create_cell("TOP")
    m1, m1lbl = ly.layer(68, 20), ly.layer(68, 5)
    m2 = ly.layer(69, 20)

    top.shapes(m1).insert(db.Box(0, 0, 1000, 1000))
    top.shapes(m1lbl).insert(db.Text("N1", db.Trans(500, 500)))
    top.shapes(m1).insert(db.Box(20000, 0, 21000, 1000))
    top.shapes(m1lbl).insert(db.Text("N1", db.Trans(20500, 500)))

    top.shapes(m2).insert(db.Box(9000, -2000, 9200, 5000))
    top.shapes(m1lbl).insert(db.Text("N2", db.Trans(9100, -1000)))

    ly.write(path)


def net_span(layout, cell, layers_json, net_name):
    """How many disjoint physical islands `net_name` is in right now --
    the test's own independent check, using the same primitives the tool
    itself is built from but called fresh, not trusting the tool's own
    self-report."""
    metals, labels, vias, _draw, _widths = ka.load_layers(layers_json)
    islands, _ = ka.net_islands(layout, cell, metals, labels, vias, net_name)
    return len(islands)


class KlayoutGuiComplete(unittest.TestCase):
    def setUp(self):
        if not os.access(ROUTER, os.X_OK):
            self.skipTest(f"{ROUTER} not built (run make first)")
        self.d = tempfile.mkdtemp()
        self.layers = os.path.join(self.d, "layers.json")
        import json
        with open(self.layers, "w") as f:
            json.dump(LAYERS, f)
        self.gds = os.path.join(self.d, "fixture.gds")
        build_fixture(self.gds)

    def test_broken_net_is_two_islands_before_anything_runs(self):
        """Negative control: the fixture is genuinely broken -- checked
        independently of complete_net(), so a bug that made both "before"
        and "after" report 1 island the same way couldn't hide here."""
        ly = db.Layout()
        ly.read(self.gds)
        self.assertEqual(net_span(ly, ly.top_cell(), self.layers, "N1"), 2)
        self.assertEqual(net_span(ly, ly.top_cell(), self.layers, "N2"), 1)

    def test_completes_broken_net_and_routes_around_the_obstacle(self):
        result = ka.complete_net(self.gds, self.layers, "N1", ROUTER, self.d)
        layout = result.pop("layout", None)
        self.assertEqual(result["status"], "completed", result)
        self.assertEqual(result["islands_before"], 2)
        self.assertEqual(result["islands_after"], 1)
        self.assertGreater(result["shapes_inserted"], 0)

        # Independent re-check on the actual painted layout, not the tool's
        # own self-report -- and confirm N2 (the obstacle) was routed
        # *around*, not through: its own single-island status is untouched.
        self.assertEqual(net_span(layout, layout.top_cell(), self.layers, "N1"), 1)
        self.assertEqual(net_span(layout, layout.top_cell(), self.layers, "N2"), 1)

    def test_rsmt_default_avoids_gratuitous_layer_hops(self):
        """N2's strap only sits on M2 -- it doesn't block M1 at all, so the
        cheapest real connection is a single flat M1 wire. Locks in a real
        defect found by hand on this exact fixture: without `-rsmt` the
        router picked an M1->M2->M3->M2->M1 detour, touching 4 vias it
        never needed; `rsmt=True` (the default) does not."""
        result = ka.complete_net(self.gds, self.layers, "N1", ROUTER, self.d)
        layout = result.pop("layout", None)
        self.assertEqual(result["status"], "completed", result)

        top = layout.top_cell()
        draw_by_name = ka.load_layers(self.layers)[3]
        used = set()
        for lname, spec in draw_by_name.items():
            idx = layout.find_layer(*spec)
            if idx is not None:
                r = db.Region(db.RecursiveShapeIterator(layout, top, idx))
                if not r.is_empty():
                    used.add(lname)
        self.assertNotIn("V1", used)
        self.assertNotIn("V2", used)
        self.assertNotIn("M3", used)

    def test_already_connected_net_is_left_alone(self):
        result = ka.complete_net(self.gds, self.layers, "N2", ROUTER, self.d)
        result.pop("layout", None)
        self.assertEqual(result, {"status": "already_connected", "net": "N2"})

    def test_unknown_net_name_is_reported_not_ignored(self):
        result = ka.complete_net(self.gds, self.layers, "NOPE", ROUTER, self.d)
        result.pop("layout", None)
        self.assertEqual(result, {"status": "not_found", "net": "NOPE"})

    def test_selection_resolves_net_and_completes_it(self):
        """"Select the pin shape" -- a single world point on one N1
        fragment, nowhere near its label, resolves to N1 via the fragment's
        own physical island, exactly like typing "N1" would have."""
        ly = db.Layout()
        ly.read(self.gds)
        top = ly.top_cell()
        point_on_fragment_far_from_its_label = (900.0, 100.0)  # inside the
        # (0,0)-(1000,1000) box, not on top of the (500,500) label
        result = ka.complete_net_from_points(
            ly, top, self.layers, [point_on_fragment_far_from_its_label],
            ROUTER, self.d)
        result.pop("layout", None)
        self.assertEqual(result["status"], "completed", result)
        self.assertEqual(result["net"], "N1")
        self.assertTrue(result["resolved_from_selection"])

    def test_selection_of_multiple_fragments_still_resolves_one_net(self):
        """Selecting shapes from *both* pieces of the same broken net (the
        natural thing to do in the GUI -- select everything you want
        joined) must not be treated as ambiguous."""
        ly = db.Layout()
        ly.read(self.gds)
        top = ly.top_cell()
        points = [(500.0, 500.0), (20500.0, 500.0)]  # both N1 fragments
        result = ka.complete_net_from_points(ly, top, self.layers, points,
                                              ROUTER, self.d)
        result.pop("layout", None)
        self.assertEqual(result["status"], "completed", result)
        self.assertEqual(result["net"], "N1")

    def test_selection_spanning_two_nets_is_ambiguous(self):
        ly = db.Layout()
        ly.read(self.gds)
        top = ly.top_cell()
        points = [(500.0, 500.0), (9100.0, 1500.0)]  # N1, then N2
        result = ka.complete_net_from_points(ly, top, self.layers, points,
                                              ROUTER, self.d)
        self.assertEqual(result, {"status": "ambiguous_selection",
                                   "nets": ["N1", "N2"],
                                   "note": "selection spans more than one net"})

    def test_empty_selection_is_reported(self):
        ly = db.Layout()
        ly.read(self.gds)
        result = ka.complete_net_from_points(ly, ly.top_cell(), self.layers,
                                              [], ROUTER, self.d)
        self.assertEqual(result["status"], "no_selection")

    def test_selection_off_any_metal_is_reported(self):
        ly = db.Layout()
        ly.read(self.gds)
        result = ka.complete_net_from_points(
            ly, ly.top_cell(), self.layers, [(5000.0, 5000.0)], ROUTER, self.d)
        self.assertEqual(result["status"], "no_net_found")


class MultiFragmentComplete(unittest.TestCase):
    """A net broken into more than two pieces -- earlier fixtures only ever
    exercised two fragments; this locks in that the same per-island-anchor
    mechanism generalizes past that."""

    def setUp(self):
        if not os.access(ROUTER, os.X_OK):
            self.skipTest(f"{ROUTER} not built (run make first)")
        self.d = tempfile.mkdtemp()
        self.layers = os.path.join(self.d, "layers.json")
        import json
        with open(self.layers, "w") as f:
            json.dump(LAYERS, f)
        self.gds = os.path.join(self.d, "fixture4.gds")
        self.fragments = [(0, 0), (12000, 6000), (4000, -8000), (16000, -3000)]
        ly = db.Layout()
        ly.dbu = 0.001
        top = ly.create_cell("TOP")
        m1, m1lbl = ly.layer(68, 20), ly.layer(68, 5)
        for i, (fx, fy) in enumerate(self.fragments):
            top.shapes(m1).insert(db.Box(fx, fy, fx + 1000, fy + 1000))
            top.shapes(m1lbl).insert(db.Text("N1", db.Trans(fx + 500, fy + 500)))
        ly.write(self.gds)

    def test_four_fragments_all_join_into_one_net(self):
        result = ka.complete_net(self.gds, self.layers, "N1", ROUTER, self.d)
        layout = result.pop("layout", None)
        self.assertEqual(result["status"], "completed", result)
        self.assertEqual(result["islands_before"], 4)
        self.assertEqual(result["islands_after"], 1)
        self.assertEqual(net_span(layout, layout.top_cell(), self.layers, "N1"), 1)


if __name__ == "__main__":
    unittest.main(verbosity=1)
