#!/usr/bin/env python3
"""Tests for bin/gen_pwr_grid.py.

Everything the generator decides comes out of the layer abstraction, so the
tests build tiny abstractions with known numbers and check the geometry against
them: straps on the layer's pitch and width, stitch cuts inside both straps and
on the cut layer's own pitch, and straps cut back around metal already there.

    python3 test/test_gen_pwr_grid.py
"""
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
GEN = os.path.join(os.path.dirname(HERE), 'bin', 'gen_pwr_grid.py')

spec = importlib.util.spec_from_file_location('gen_pwr_grid', GEN)
pg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pg)


def abstraction(**kw):
    """A minimal layers.json: two metals and the cut between them."""
    m4 = {'Layer': 'M4', 'Direction': 'H', 'Pitch': 1260, 'Width': 800, 'Offset': 0}
    m5 = {'Layer': 'M5', 'Direction': 'V', 'Pitch': 1480, 'Width': 1180, 'Offset': 0}
    v4 = {'Layer': 'V4', 'Stack': ['M4', 'M5'], 'WidthX': 200, 'WidthY': 200,
          'SpaceX': 200, 'SpaceY': 200,
          'VencA_L': 65, 'VencA_H': 65, 'VencP_L': 300, 'VencP_H': 440}
    m4.update(kw.get('m4', {}))
    m5.update(kw.get('m5', {}))
    v4.update(kw.get('v4', {}))
    return {'design_info': {'bottom_power_grid_layer': 'M4',
                            'top_power_grid_layer': 'M5'},
            'Abstraction': [m4, m5, v4]}


def placement(bbox=(0, 0, 20000, 20000), globals_=None, instances=None):
    return {
        'global_signals': globals_ if globals_ is not None else
            [{'actual': 'VDDA', 'formal': 'supply1', 'prefix': 'global_power'},
             {'actual': 'GNDA', 'formal': 'supply0', 'prefix': 'global_power'}],
        'leaves': [],
        'modules': [{'abstract_name': 'TOP', 'concrete_name': 'TOP_0',
                     'bbox': list(bbox), 'instances': instances or []}],
    }


class Straps(unittest.TestCase):
    def setUp(self):
        self.layers = pg.layer_table(abstraction())

    def test_on_the_layer_pitch_and_width(self):
        got = pg.straps((0, 0, 10000, 10000), self.layers['M4'], ['A', 'B'], 1, 1.0)
        self.assertTrue(got)
        for _, r in got:
            self.assertEqual(r[3] - r[1], 800)          # the layer's own width
            self.assertEqual((r[0], r[2]), (0, 10000))  # spans the block
        ys = [r[1] for _, r in got]
        self.assertTrue(all(b - a == 1260 for a, b in zip(ys, ys[1:])))

    def test_stay_inside_the_block(self):
        bbox = (0, 0, 10000, 10000)
        for name in ('M4', 'M5'):
            for _, r in pg.straps(bbox, self.layers[name], ['A'], 1, 1.0):
                self.assertGreaterEqual(r[0], bbox[0])
                self.assertGreaterEqual(r[1], bbox[1])
                self.assertLessEqual(r[2], bbox[2])
                self.assertLessEqual(r[3], bbox[3])

    def test_nets_alternate(self):
        got = pg.straps((0, 0, 10000, 10000), self.layers['M4'], ['A', 'B'], 1, 1.0)
        self.assertEqual([n for n, _ in got][:4], ['A', 'B', 'A', 'B'])

    def test_stride_thins_and_widen_fattens(self):
        one = pg.straps((0, 0, 10000, 10000), self.layers['M4'], ['A'], 1, 1.0)
        two = pg.straps((0, 0, 10000, 10000), self.layers['M4'], ['A'], 2, 1.0)
        self.assertLess(len(two), len(one))
        wide = pg.straps((0, 0, 10000, 10000), self.layers['M4'], ['A'], 1, 1.5)
        self.assertEqual(wide[0][1][3] - wide[0][1][1], 1200)

    def test_a_block_too_small_gets_no_straps(self):
        self.assertEqual(pg.straps((0, 0, 100, 100), self.layers['M4'], ['A'], 1, 1.0), [])


class GridLayers(unittest.TestCase):
    def test_default_range_comes_from_design_info(self):
        js = abstraction()
        self.assertEqual(pg.grid_layers(js, pg.layer_table(js)), ['M4', 'M5'])

    def test_unknown_layer_is_refused(self):
        js = abstraction()
        with self.assertRaises(SystemExit):
            pg.grid_layers(js, pg.layer_table(js), 'M9', 'M5')


class Stitch(unittest.TestCase):
    """Crossings must be tied, or the grid is a pile of separate straps."""

    def setUp(self):
        self.js = abstraction()
        self.layers = pg.layer_table(self.js)
        self.vias = pg.via_layers(self.js, self.layers)
        self.pins = {'A': [('M4', (0, 0, 10000, 800)), ('M5', (2000, 0, 3180, 10000))]}

    def test_cuts_land_inside_both_straps(self):
        # stitch() returns the cuts; it no longer adds them to the routing
        # shapes, because the router never routes through a cut
        added = pg.stitch(self.pins, self.vias, self.layers)
        self.assertTrue(added.get('A'), 'the crossing was left untied')
        self.assertEqual([r for l, r in self.pins['A'] if l == 'V4'], [],
                         'cuts must stay out of the routing shapes')
        cuts = [r for l, r in added['A']]
        for c in cuts:
            self.assertTrue(2000 <= c[0] and c[2] <= 3180, f'{c} outside the M5 strap')
            self.assertTrue(0 <= c[1] and c[3] <= 800, f'{c} outside the M4 strap')
            self.assertEqual((c[2] - c[0], c[3] - c[1]), (200, 200))

    def test_cuts_keep_the_across_strap_enclosure(self):
        added = pg.stitch(self.pins, self.vias, self.layers)
        for _, c in added['A']:
            self.assertGreaterEqual(c[0] - 2000, 65)   # VencA of the vertical layer
            self.assertGreaterEqual(3180 - c[2], 65)
            self.assertGreaterEqual(c[1] - 0, 65)      # VencA of the horizontal one
            self.assertGreaterEqual(800 - c[3], 65)

    def test_cuts_are_spaced_on_the_cut_pitch(self):
        added = pg.stitch(self.pins, self.vias, self.layers)
        xs = sorted({c[0] for l, c in added['A']})
        self.assertTrue(all(b - a == 400 for a, b in zip(xs, xs[1:])),
                        'cuts must sit on width + space')

    def test_a_strap_too_narrow_for_a_cut_is_left_alone(self):
        thin = {'A': [('M4', (0, 0, 10000, 100)), ('M5', (2000, 0, 2100, 10000))]}
        self.assertEqual(pg.stitch(thin, self.vias, self.layers), {})

    def test_different_nets_are_never_tied(self):
        pins = {'A': [('M4', (0, 0, 10000, 800))],
                'B': [('M5', (2000, 0, 3180, 10000))]}
        self.assertEqual(pg.stitch(pins, self.vias, self.layers), {})


class Carve(unittest.TestCase):
    """A strap laid over metal already there is a short nothing can undo."""

    def setUp(self):
        self.layers = pg.layer_table(abstraction())

    def test_strap_is_split_around_a_blocker(self):
        pins = {'A': [('M4', (0, 0, 10000, 800))]}
        blockers = [('M4', (4000, 0, 5000, 800))]
        pg.carve(pins, blockers, self.layers, keep_min=500)
        pieces = [r for l, r in pins['A'] if l == 'M4']
        self.assertEqual(len(pieces), 2)
        space = self.layers['M4']['pitch'] - self.layers['M4']['width']
        for r in pieces:
            self.assertTrue(r[2] <= 4000 - space or r[0] >= 5000 + space)

    def test_a_fully_covered_strap_is_dropped(self):
        pins = {'A': [('M4', (0, 0, 10000, 800))]}
        trimmed, dropped = pg.carve(pins, [('M4', (-1000, 0, 11000, 800))],
                                    self.layers, keep_min=500)
        self.assertEqual((trimmed, dropped), (1, 1))
        self.assertEqual(pins['A'], [])

    def test_short_fragments_are_dropped_not_left_as_stubs(self):
        pins = {'A': [('M4', (0, 0, 10000, 800))]}
        pg.carve(pins, [('M4', (200, 0, 9800, 800))], self.layers, keep_min=500)
        self.assertEqual(pins['A'], [])

    def test_a_blocker_on_another_layer_is_ignored(self):
        pins = {'A': [('M4', (0, 0, 10000, 800))]}
        pg.carve(pins, [('M2', (4000, 0, 5000, 800))], self.layers, keep_min=500)
        self.assertEqual(len(pins['A']), 1)

    def test_a_blocker_beside_the_strap_is_ignored(self):
        pins = {'A': [('M4', (0, 0, 10000, 800))]}
        pg.carve(pins, [('M4', (4000, 5000, 5000, 5800))], self.layers, keep_min=500)
        self.assertEqual(len(pins['A']), 1)


class PlacedShapes(unittest.TestCase):
    def test_instance_mirroring_is_applied(self):
        macros = {'CELL': [('M4', (0, 0, 100, 50))]}
        pl = placement(instances=[
            {'concrete_template_name': 'CELL', 'instance_name': 'X0',
             'transformation': {'oX': 1000, 'oY': 2000, 'sX': 1, 'sY': 1}},
            {'concrete_template_name': 'CELL', 'instance_name': 'X1',
             'transformation': {'oX': 1000, 'oY': 2000, 'sX': -1, 'sY': -1}},
        ])
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, 'p.json')
            with open(p, 'w') as fh:
                json.dump(pl, fh)
            got = pg.placed_shapes(p, macros, {'M4'})
        self.assertIn(('M4', (1000, 2000, 1100, 2050)), got)
        self.assertIn(('M4', (900, 1950, 1000, 2000)), got)

    def test_layers_not_asked_for_are_skipped(self):
        macros = {'CELL': [('M1', (0, 0, 10, 10))]}
        pl = placement(instances=[{'concrete_template_name': 'CELL',
                                   'transformation': {'oX': 0, 'oY': 0}}])
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, 'p.json')
            with open(p, 'w') as fh:
                json.dump(pl, fh)
            self.assertEqual(pg.placed_shapes(p, macros, {'M4'}), [])


class EndToEnd(unittest.TestCase):
    """The CLI, as the flow actually calls it."""

    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.d, ignore_errors=True)
        self.write('layers.json', abstraction())
        self.write('place.json', placement())

    def write(self, name, obj):
        with open(os.path.join(self.d, name), 'w') as fh:
            json.dump(obj, fh)

    def read(self, name):
        with open(os.path.join(self.d, name)) as fh:
            return fh.read()

    def run_gen(self, *args):
        cmd = [sys.executable, GEN, '-l', 'layers.json', '-p', 'place.json'] + list(args)
        return subprocess.run(cmd, cwd=self.d, capture_output=True, text=True)

    def test_net_names_come_from_the_placement(self):
        r = self.run_gen('--lef', 'g.lef')
        self.assertEqual(r.returncode, 0, r.stderr)
        lef = self.read('g.lef')
        self.assertIn('PIN VDDA', lef)
        self.assertIn('PIN GNDA', lef)
        self.assertNotIn('PIN VDD\n', lef)

    def test_falls_back_when_the_placement_names_no_supplies(self):
        self.write('place.json', placement(globals_=[]))
        self.assertEqual(self.run_gen('--lef', 'g.lef').returncode, 0)
        lef = self.read('g.lef')
        self.assertIn('PIN VDD', lef)
        self.assertIn('PIN VSS', lef)

    def test_explicit_nets_win(self):
        self.run_gen('--nets', 'PWR,GND', '--lef', 'g.lef')
        lef = self.read('g.lef')
        self.assertIn('PIN PWR', lef)
        self.assertNotIn('VDDA', lef)

    def test_lef_is_well_formed_and_carries_every_shape(self):
        self.run_gen('--lef', 'g.lef')
        lef = self.read('g.lef')
        self.assertEqual(lef.count('MACRO PGRID'), 1)
        self.assertEqual(lef.count('END PGRID'), 1)
        self.assertEqual(lef.count('PIN '), lef.count('END VDDA') + lef.count('END GNDA'))
        self.assertIn('USE POWER ;', lef)
        for line in lef.splitlines():
            if line.strip().startswith('RECT'):
                v = [int(x) for x in line.split()[1:5]]
                self.assertLess(v[0], v[2])
                self.assertLess(v[1], v[3])

    def test_placement_gains_one_instance_of_the_grid(self):
        self.run_gen('--lef', 'g.lef', '--placement-out', 'out.json')
        d = json.loads(self.read('out.json'))
        self.assertEqual([l['concrete_name'] for l in d['leaves']], ['PGRID'])
        insts = [i for i in d['modules'][0]['instances']
                 if i['concrete_template_name'] == 'PGRID']
        self.assertEqual(len(insts), 1)
        self.assertEqual({f['actual'] for f in insts[0]['fa_map']}, {'VDDA', 'GNDA'})

    def test_ndr_taps_every_pin_to_the_grid(self):
        self.run_gen('--ndr-out', 'ndr.json')
        d = json.loads(self.read('ndr.json'))
        self.assertEqual(d[0]['module'], 'TOP_0')
        self.assertEqual({c['driver'] for c in d[0]['clock_nets']},
                         {'X_PGRID/VDDA', 'X_PGRID/GNDA'})

    def test_cuts_are_kept_out_of_the_routing_lef(self):
        r = self.run_gen('--lef', 'g.lef', '--cuts-out', 'cuts.json')
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn('LAYER V4 ;', self.read('g.lef'),
                         'cuts must not reach the router')
        cuts = json.loads(self.read('cuts.json'))
        self.assertTrue(sum(len(v) for v in cuts.values()),
                        'cuts must still be emitted for the post-route merge')
        for v in cuts.values():
            for c in v:
                self.assertEqual(c[0], 'V4')
                self.assertEqual((c[3] - c[1], c[4] - c[2]), (200, 200))

    def test_cuts_in_lef_puts_them_back(self):
        self.run_gen('--cuts-in-lef', '--lef', 'g.lef')
        self.assertIn('LAYER V4 ;', self.read('g.lef'))

    def test_no_stitch_emits_no_cuts_at_all(self):
        self.run_gen('--no-stitch', '--lef', 'bare.lef', '--cuts-out', 'c2.json')
        self.assertNotIn('LAYER V4 ;', self.read('bare.lef'))
        self.assertFalse(os.path.exists(os.path.join(self.d, 'c2.json')))

    def test_bbox_can_replace_the_placement(self):
        cmd = [sys.executable, GEN, '-l', 'layers.json',
               '--bbox', '0,0,20000,20000', '--lef', 'g.lef']
        self.assertEqual(subprocess.run(cmd, cwd=self.d, capture_output=True).returncode, 0)

    def test_no_extent_is_an_error(self):
        cmd = [sys.executable, GEN, '-l', 'layers.json', '--lef', 'g.lef']
        self.assertNotEqual(subprocess.run(cmd, cwd=self.d, capture_output=True).returncode, 0)

    def test_avoid_cuts_the_grid_back(self):
        with open(os.path.join(self.d, 'cells.lef'), 'w') as fh:
            fh.write('MACRO CELL\n  SIZE 400 BY 400 ;\n  PIN A\n    PORT\n'
                     '      LAYER M4 ;\n        RECT 0 0 4000 800 ;\n'
                     '    END\n  END A\nEND CELL\n')
        self.write('place.json', placement(instances=[
            {'concrete_template_name': 'CELL', 'instance_name': 'X0',
             'transformation': {'oX': 0, 'oY': 0}}]))
        def metal_area(name):
            # carving shortens straps rather than removing them, so compare the
            # area covered, not the rectangle count
            total = 0
            for line in self.read(name).splitlines():
                t = line.split()
                if t and t[0] == 'RECT':
                    v = [int(x) for x in t[1:5]]
                    total += (v[2] - v[0]) * (v[3] - v[1])
            return total
        self.run_gen('--lef', 'a.lef')
        cut = self.run_gen('--avoid', 'cells.lef', '--lef', 'b.lef').stdout
        self.assertIn('avoiding', cut)
        self.assertLess(metal_area('b.lef'), metal_area('a.lef'),
                        'carving removed nothing')


if __name__ == '__main__':
    unittest.main(verbosity=2)
