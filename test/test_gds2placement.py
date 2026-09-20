#!/usr/bin/env python3
"""Unit tests for bin/gds2placement.py's pin-labeling fix.

Real hand-drawn analog layouts (the bandgap/OTA hierarchy this was written
against) label a net only where it surfaces in the *parent* cell that
instantiates a device, never on the device leaf itself. The original script
only ever looked for a label in the same cell as the pin geometry, so every
such leaf came out with zero terminals and every such instance came out with
an empty fa_map -- a placement with devices in it but no declared
connectivity.

These fixtures are hand-built with independently-computed expected output
(never derived by running the tool itself), in the style of
test_check_nets.py: build the fixture and its ground truth first, then
trust the tool only once it reproduces it.
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest

import gdstk

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, '..', 'bin', 'gds2placement.py')

LAYERS = {'Abstraction': [
    {'Layer': 'M1', 'GdsLayerNo': 68, 'GdsDatatype': {'Draw': 20, 'Pin': 5, 'Label': 16}},
]}


def write_layers(path):
    with open(path, 'w') as f:
        json.dump(LAYERS, f)


def run_tool(gds, layers, out, netlist=None):
    cmd = [sys.executable, TOOL, '-g', gds, '-l', layers, '-o', out]
    if netlist:
        cmd += ['-n', netlist]
    p = subprocess.run(cmd, capture_output=True, text=True)
    return p.returncode, p.stdout + p.stderr


class Gds2Placement(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.layers = os.path.join(self.d, 'layers.json')
        write_layers(self.layers)

    def test_local_label_still_wins(self):
        """Backward compatibility: a leaf that labels its own pin (the
        ALIGN/gds2lef.py convention) must keep using that label's text, not
        a synthetic name -- the fix must not disturb the case that already
        worked."""
        lib = gdstk.Library(name='fixture', unit=1e-6, precision=1e-9)
        dev = lib.new_cell('DEV')
        dev.add(gdstk.rectangle((0, 0), (1, 1), layer=68, datatype=5))
        dev.add(gdstk.Label('D', (0.5, 0.5), layer=68, texttype=16))
        gds = os.path.join(self.d, 'a.gds')
        lib.write_gds(gds)

        out = os.path.join(self.d, 'a.placement_verilog.json')
        rc, log = run_tool(gds, self.layers, out)
        self.assertEqual(rc, 0, log)
        with open(out) as f:
            data = json.load(f)
        self.assertEqual(len(data['leaves']), 1)
        names = {t['name'] for t in data['leaves'][0]['terminals']}
        self.assertEqual(names, {'D'})

    def test_unlabeled_leaf_gets_synthetic_pin(self):
        """A leaf with a Pin-purpose shape but no label of its own must not
        vanish -- it gets a generic PIN<n> name instead of zero terminals."""
        lib = gdstk.Library(name='fixture', unit=1e-6, precision=1e-9)
        dev = lib.new_cell('DEV')
        dev.add(gdstk.rectangle((0, 0), (1, 1), layer=68, datatype=5))
        gds = os.path.join(self.d, 'b.gds')
        lib.write_gds(gds)

        out = os.path.join(self.d, 'b.placement_verilog.json')
        rc, log = run_tool(gds, self.layers, out)
        self.assertEqual(rc, 0, log)
        with open(out) as f:
            data = json.load(f)
        terms = data['leaves'][0]['terminals']
        self.assertEqual(len(terms), 1)
        self.assertEqual(terms[0]['name'], 'PIN1')
        self.assertEqual(terms[0]['rect'], [0, 0, 1, 1])

    def test_via_leaf_gets_no_bogus_terminals(self):
        """A pure via cell (Draw-only cut geometry, no Pin-purpose shape at
        all) must not be swept into the synthetic-PIN fallback -- that
        fallback is scoped to Pin-purpose geometry only."""
        lib = gdstk.Library(name='fixture', unit=1e-6, precision=1e-9)
        via = lib.new_cell('VIA')
        via.add(gdstk.rectangle((0, 0), (1, 1), layer=68, datatype=20))  # Draw, no Pin
        gds = os.path.join(self.d, 'v.gds')
        lib.write_gds(gds)

        out = os.path.join(self.d, 'v.placement_verilog.json')
        rc, log = run_tool(gds, self.layers, out)
        self.assertEqual(rc, 0, log)
        with open(out) as f:
            data = json.load(f)
        self.assertEqual(data['leaves'][0]['terminals'], [])

    def test_ancestor_label_completes_fa_map_all_four_orientations(self):
        """The real-world case: a leaf with no label of its own, placed four
        times inside a module with N/mirror-X/mirror-Y/180-degree
        orientations, each net named by a label that lives only in the
        module's own frame (the bg__se_folded_cascode_p pattern). Each
        instance's fa_map must recover its own correct net, and the
        transformation each instance is placed with must reflect its real
        rotation/reflection -- not the pre-fix hardcoded (1, 1)."""
        lib = gdstk.Library(name='fixture', unit=1e-6, precision=1e-9)
        dev = lib.new_cell('DEV')
        dev.add(gdstk.rectangle((0, 0), (1, 1), layer=68, datatype=5))

        mod = lib.new_cell('MOD')
        # (net name, origin, rotation, x_reflection) -- ground truth chosen
        # to cover all four Manhattan orientations the router supports
        # (Geom::Transform::orient(): N, FN, FS, S).
        placements = [
            ('net_a', (10, 10), 0.0,       False),
            ('net_b', (20, 10), 0.0,       True),
            ('net_c', (30, 10), math.pi,   True),
            ('net_d', (40, 10), math.pi,   False),
        ]
        expect_sxsy = {
            'net_a': (1, 1), 'net_b': (1, -1), 'net_c': (-1, 1), 'net_d': (-1, -1),
        }
        for net, origin, rot, xrefl in placements:
            mod.add(gdstk.Reference(dev, origin=origin, rotation=rot, x_reflection=xrefl))
        for net, (ox, oy), rot, xrefl in placements:
            # Independently compute the pin's true world location for this
            # orientation and place the label exactly there -- ground truth,
            # not derived from the tool under test.
            lx, ly = 0.5, 0.5
            if xrefl:
                ly = -ly
            if abs(rot - math.pi) < 1e-9:
                lx, ly = -lx, -ly
            mod.add(gdstk.Label(net, (ox + lx, oy + ly), layer=68, texttype=16))

        gds = os.path.join(self.d, 'c.gds')
        lib.write_gds(gds)

        out = os.path.join(self.d, 'c.placement_verilog.json')
        rc, log = run_tool(gds, self.layers, out)
        self.assertEqual(rc, 0, log)
        with open(out) as f:
            data = json.load(f)

        module = next(m for m in data['modules'] if m['concrete_name'] == 'MOD')
        self.assertEqual(len(module['instances']), 4)

        # All four placements share one physical local pin, so the leaf's
        # aggregate terminal list -- necessarily shared across every
        # instance of a leaf type -- has exactly one entry, named after
        # whichever placement's ancestor label was found first (net_a, the
        # lowest-origin occurrence). That name is just a shared handle:
        # what must be correct per instance is fa_map's "actual", checked
        # below against each placement's real, independently-computed net.
        terms = next(l for l in data['leaves'] if l['abstract_name'] == 'DEV')['terminals']
        self.assertEqual(len(terms), 1)
        formal = terms[0]['name']
        self.assertEqual(formal, 'net_a')

        got = {}
        for inst in module['instances']:
            tr = inst['transformation']
            self.assertEqual(len(inst['fa_map']), 1, inst)
            self.assertEqual(inst['fa_map'][0]['formal'], formal)
            got[inst['fa_map'][0]['actual']] = (tr['sX'], tr['sY'])
        self.assertEqual(got, expect_sxsy)

    def test_ancestor_label_on_draw_only_leaf(self):
        """The exact real-world shape found in the bandgap/OTA hierarchy:
        the leaf has no dedicated "Pin"-datatype geometry at all, only plain
        "Draw" metal -- the synthetic-PIN fallback (scoped to Pin-purpose
        shapes, to keep via cells from getting bogus terminals) cannot help
        here; only an ancestor label falling on the Draw shape can name it."""
        lib = gdstk.Library(name='fixture', unit=1e-6, precision=1e-9)
        dev = lib.new_cell('DEV')
        dev.add(gdstk.rectangle((0, 0), (1, 1), layer=68, datatype=20))  # Draw only
        mod = lib.new_cell('MOD')
        mod.add(gdstk.Reference(dev, origin=(10, 10)))
        mod.add(gdstk.Label('vout', (10.5, 10.5), layer=68, texttype=16))
        gds = os.path.join(self.d, 'e.gds')
        lib.write_gds(gds)

        out = os.path.join(self.d, 'e.placement_verilog.json')
        rc, log = run_tool(gds, self.layers, out)
        self.assertEqual(rc, 0, log)
        with open(out) as f:
            data = json.load(f)

        terms = data['leaves'][0]['terminals']
        self.assertEqual(terms, [{'name': 'vout', 'rect': [0, 0, 1, 1]}])
        module = next(m for m in data['modules'] if m['concrete_name'] == 'MOD')
        self.assertEqual(module['instances'][0]['fa_map'],
                         [{'actual': 'vout', 'formal': 'vout'}])

    def test_netlist_fa_map_takes_priority_over_labels(self):
        """When a netlist genuinely covers an instance, its (verified,
        schematic-derived) fa_map must win -- the label-derived fallback
        must only fill gaps, never override real netlist connectivity."""
        lib = gdstk.Library(name='fixture', unit=1e-6, precision=1e-9)
        dev = lib.new_cell('DEV')
        dev.add(gdstk.rectangle((0, 0), (1, 1), layer=68, datatype=5))
        mod = lib.new_cell('MOD')
        mod.add(gdstk.Reference(dev, origin=(10, 10)))
        # A label that -- if the fallback wrongly ran anyway -- would name
        # the net "wrong_net"; the netlist's "right_net" must win instead.
        mod.add(gdstk.Label('wrong_net', (10.5, 10.5), layer=68, texttype=16))
        gds = os.path.join(self.d, 'd.gds')
        lib.write_gds(gds)

        netlist = os.path.join(self.d, 'd.scs')
        with open(netlist, 'w') as f:
            f.write('subckt MOD\n')
            f.write('I0 (right_net) DEV\n')
            f.write('ends\n')
            f.write('subckt DEV PIN1\n')
            f.write('ends\n')

        out = os.path.join(self.d, 'd.placement_verilog.json')
        rc, log = run_tool(gds, self.layers, out, netlist=netlist)
        self.assertEqual(rc, 0, log)
        with open(out) as f:
            data = json.load(f)
        module = next(m for m in data['modules'] if m['concrete_name'] == 'MOD')
        self.assertEqual(module['instances'][0]['fa_map'],
                         [{'actual': 'right_net', 'formal': 'PIN1'}])


if __name__ == '__main__':
    unittest.main(verbosity=1)
