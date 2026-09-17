#!/usr/bin/env python3
"""Unit tests for bin/check_nets.py, the routed-GDS connectivity check."""
import json
import os
import subprocess
import sys
import tempfile
import unittest

import gdstk

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, '..', 'bin', 'check_nets.py')

# M1 67/20, V1 67/44, M2 68/20, V2 68/44, M3 69/20. V2 shares 68/44 with a
# second abstraction so the layer-collision path is exercised the way sky130's
# V4/CapMIMContact pair exercises it on a real PDK.
LAYERS = {'Abstraction': [
    {'Layer': 'M1', 'GdsLayerNo': 67, 'GdsDatatype': {'Draw': 20}},
    {'Layer': 'V1', 'GdsLayerNo': 67, 'GdsDatatype': {'Draw': 44},
     'Stack': ['M1', 'M2']},
    {'Layer': 'M2', 'GdsLayerNo': 68, 'GdsDatatype': {'Draw': 20}},
    {'Layer': 'V2', 'GdsLayerNo': 68, 'GdsDatatype': {'Draw': 44},
     'Stack': ['M2', 'M3']},
    {'Layer': 'M3', 'GdsLayerNo': 69, 'GdsDatatype': {'Draw': 20}},
    {'Layer': 'CapContact', 'GdsLayerNo': 68, 'GdsDatatype': {'Draw': 44}},
]}
GDS = {'M1': (67, 20), 'V1': (67, 44), 'M2': (68, 20),
       'V2': (68, 44), 'M3': (69, 20)}


def write_gds(path, top, rects):
    lib = gdstk.Library(name=top, unit=1e-6, precision=1e-9)
    cell = lib.new_cell(top)
    for layer, x0, y0, x1, y1 in rects:
        g, d = GDS[layer]
        cell.add(gdstk.rectangle((x0, y0), (x1, y1), layer=g, datatype=d))
    lib.write_gds(path)


def write_def(path, nets, dbu=1000):
    with open(path, 'w') as f:
        f.write('VERSION 5.8 ;\nUNITS DISTANCE MICRONS %d ;\n' % dbu)
        f.write('NETS %d ;\n' % len(nets))
        for net, rects in nets.items():
            f.write('- %s\n' % net)
            for layer, x0, y0, x1, y1 in rects:
                f.write('  + RECT %s ( %d %d ) ( %d %d )\n'
                        % (layer, x0 * dbu, y0 * dbu, x1 * dbu, y1 * dbu))
            f.write(' ;\n')
        f.write('END NETS\nEND DESIGN\n')


class CheckNets(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.layers = os.path.join(self.d, 'layers.json')
        with open(self.layers, 'w') as f:
            json.dump(LAYERS, f)

    def run_tool(self, gds, deffile, stack=''):
        cmd = [sys.executable, TOOL, gds, '--top', 'TOP',
               '--layers', self.layers, '--def', deffile]
        if stack:
            cmd += ['--stack', stack]
        p = subprocess.run(cmd, capture_output=True, text=True)
        return p.returncode, p.stdout + p.stderr

    def test_connected_net_passes(self):
        """One net, two metals bridged by a cut: one conductor, check passes."""
        g = os.path.join(self.d, 'a.gds')
        write_gds(g, 'TOP', [('M1', 0, 0, 10, 1), ('V1', 9, 0, 10, 1),
                             ('M2', 9, 0, 10, 12)])
        dfl = os.path.join(self.d, 'a.def')
        write_def(dfl, {'N1': [('M1', 0, 0, 10, 1), ('M2', 9, 0, 10, 12)]})
        rc, out = self.run_tool(g, dfl)
        self.assertEqual(rc, 0, out)
        self.assertIn('1 net(s), one conductor each', out)

    def test_missing_cut_is_an_open(self):
        """Drop the via and the same two wires must be reported as two pieces."""
        g = os.path.join(self.d, 'b.gds')
        write_gds(g, 'TOP', [('M1', 0, 0, 10, 1), ('M2', 9, 3, 10, 12)])
        dfl = os.path.join(self.d, 'b.def')
        write_def(dfl, {'N1': [('M1', 0, 0, 10, 1), ('M2', 9, 3, 10, 12)]})
        rc, out = self.run_tool(g, dfl)
        self.assertEqual(rc, 1, out)
        self.assertIn('OPEN  N1 is in 2 pieces', out)

    def test_two_nets_stay_separate(self):
        """Two disjoint nets are two conductors, and that is not an open."""
        g = os.path.join(self.d, 'c.gds')
        write_gds(g, 'TOP', [('M1', 0, 0, 5, 1), ('M1', 0, 5, 5, 6)])
        dfl = os.path.join(self.d, 'c.def')
        write_def(dfl, {'N1': [('M1', 0, 0, 5, 1)], 'N2': [('M1', 0, 5, 5, 6)]})
        rc, out = self.run_tool(g, dfl)
        self.assertEqual(rc, 0, out)
        self.assertIn('2 net(s), one conductor each', out)

    def test_via_wins_a_shared_gds_layer(self):
        """V2 and CapContact share 68/44: the via must still bridge M2 to M3.

        This is the sky130 V4/CapMIMContact collision. A map that lets the last
        abstraction win loses the via and reports every strap as floating.
        """
        g = os.path.join(self.d, 'd.gds')
        write_gds(g, 'TOP', [('M2', 0, 0, 10, 1), ('V2', 9, 0, 10, 1),
                             ('M3', 9, 0, 10, 12)])
        dfl = os.path.join(self.d, 'd.def')
        write_def(dfl, {'N1': [('M2', 0, 0, 10, 1), ('M3', 9, 0, 10, 12)]})
        rc, out = self.run_tool(g, dfl)
        self.assertEqual(rc, 0, out)
        rc, out = self.run_tool(g, dfl, stack='M2,V2,M3')
        self.assertEqual(rc, 0, out)

    def test_net_named_by_same_layer_overlap_only(self):
        """A wire crossing over another net must not be tagged with it.

        N2's M3 wire passes over N1's M1 wire with no cut. Tagging on geometry
        alone would put both names on one conductor and hide the split.
        """
        g = os.path.join(self.d, 'e.gds')
        write_gds(g, 'TOP', [('M1', 0, 0, 10, 1), ('M3', 4, -5, 5, 8)])
        dfl = os.path.join(self.d, 'e.def')
        write_def(dfl, {'N1': [('M1', 0, 0, 10, 1)], 'N2': [('M3', 4, -5, 5, 8)]})
        rc, out = self.run_tool(g, dfl)
        self.assertEqual(rc, 0, out)
        self.assertIn('2 net(s), one conductor each', out)

    def test_net_with_no_metal_is_reported(self):
        """A net the GDS never received is called out, not silently counted."""
        g = os.path.join(self.d, 'f.gds')
        write_gds(g, 'TOP', [('M1', 0, 0, 10, 1)])
        dfl = os.path.join(self.d, 'f.def')
        write_def(dfl, {'N1': [('M1', 0, 0, 10, 1)], 'N2': [('M2', 0, 5, 5, 6)]})
        rc, out = self.run_tool(g, dfl)
        self.assertEqual(rc, 1, out)
        self.assertIn('ABSENT N2', out)

    def test_device_internal_conductor_is_not_an_open(self):
        """Metal no DEF wire lands on is counted apart, and does not fail."""
        g = os.path.join(self.d, 'g.gds')
        write_gds(g, 'TOP', [('M1', 0, 0, 10, 1), ('M1', 0, 20, 2, 21)])
        dfl = os.path.join(self.d, 'g.def')
        write_def(dfl, {'N1': [('M1', 0, 0, 10, 1)]})
        rc, out = self.run_tool(g, dfl)
        self.assertEqual(rc, 0, out)
        self.assertIn('1 conductor(s) carry no DEF wire', out)


    def test_two_nets_on_one_conductor_is_a_short(self):
        """Two nets whose wires land on the same metal must be called a short.

        Naming a conductor by its commonest net would drop the other name and
        report it as having no metal at all, which is what it looked like on
        current_mirror_ota.
        """
        g = os.path.join(self.d, 'h.gds')
        write_gds(g, 'TOP', [('M1', 0, 0, 10, 1)])
        dfl = os.path.join(self.d, 'h.def')
        write_def(dfl, {'N1': [('M1', 0, 0, 6, 1)], 'N2': [('M1', 5, 0, 10, 1)]})
        rc, out = self.run_tool(g, dfl)
        self.assertEqual(rc, 1, out)
        self.assertIn('SHORT N1 + N2', out)
        self.assertNotIn('ABSENT', out)


if __name__ == '__main__':
    unittest.main(verbosity=1)
