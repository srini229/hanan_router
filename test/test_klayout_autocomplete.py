#!/usr/bin/env python3
"""Item 2 (KLayout front-end) acceptance test: autocomplete a broken net.

Prototype for the eventual KLayout-driven autocomplete: a net whose drawn
metal is in more than one connected component is "unfinished". This builds
a minimal hand-made fixture with exactly that shape -- two placed devices
and a net whose connecting wire was never drawn, only two disconnected fragments --
declares the fragments as `virtual_pins` in the NDR, routes with
hanan_router, paints the routed DEF's own geometry into a fresh GDS, and
verifies the result with check_nets.py. In the style of test_check_nets.py.

Exercises the router side of both unit-conversion boundaries the front-end
will have to get right: NDR virtual_pins/obstacles want microns while
placement/LEF want DEF-style database units (gotcha 1 in the plan), and a
routed DEF is DBU while a gdstk GDS written unit=1e-6 is microns, so painting
DEF geometry back into a GDS is a divide-by-DBU that a units mixup would get
silently wrong in the other direction. The `test_broken_net_is_reported_open`
case exists so this file can't pass by having done nothing -- see it for why.
"""
import json
import os
import re
import subprocess
import sys
import tempfile
import unittest

import gdstk

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTER = os.path.join(HERE, '..', 'hanan_router')
CHECK_NETS = os.path.join(HERE, '..', 'bin', 'check_nets.py')
DBU = 1000  # database units per micron, the sky130 convention

# M1/M2/M3 + V1/V2, values straight from sky130.layers.json (the layer set
# item 1's routing work already exercised).
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
GDS_LAYER = {"M1": (68, 20), "M2": (69, 20), "M3": (70, 20),
             "V1": (68, 44), "V2": (69, 44)}

LEF = """MACRO PAD
  CLASS BLOCK ;
  ORIGIN 0 0 ;
  UNITS
    DATABASE MICRONS UNITS 1000 ;
  END UNITS
  SIZE 2000 BY 2000 ;
  PIN A
    DIRECTION INOUT ;
    PORT
      LAYER M1 ;
        RECT 900 900 1100 1100 ;
    END
  END A
END PAD
"""

PLACEMENT = {
    "global_signals": [],
    "leaves": [{
        "abstract_name": "PAD", "bbox": [0, 0, 2000, 2000], "concrete_name": "PAD",
        "terminals": [{"name": "A", "rect": [900, 900, 1100, 1100]}],
    }],
    "modules": [{
        "abstract_name": "TOP", "bbox": [0, 0, 30000, 4000], "concrete_name": "TOP_CONC_0",
        "instances": [
            {"abstract_template_name": "PAD", "concrete_template_name": "PAD",
             "fa_map": [{"actual": "N1", "formal": "A"}], "instance_name": "I_0",
             "transformation": {"oX": 0, "oY": 0, "sX": 1, "sY": 1}},
            {"abstract_template_name": "PAD", "concrete_template_name": "PAD",
             "fa_map": [{"actual": "N1", "formal": "A"}], "instance_name": "I_1",
             "transformation": {"oX": 26000, "oY": 0, "sX": 1, "sY": 1}},
        ],
        "parameters": ["N1"],
    }],
}

# The two hand-drawn fragments (DBU), deliberately touching neither each
# other nor either device pin -- this is the "broken net" to complete.
FRAG1_DBU = (10000, 2400, 10200, 2600)
FRAG2_DBU = (16000, 2400, 16200, 2600)
PIN0_DBU = (900, 900, 1100, 1100)
PIN1_DBU = (26900, 900, 27100, 1100)


def dbu_to_um(rect):
    return tuple(v / DBU for v in rect)


def write_gds(path, top, rects):
    """rects: [(layer, x0, y0, x1, y1)] in DBU -> written in microns."""
    lib = gdstk.Library(name=top, unit=1e-6, precision=1e-9)
    cell = lib.new_cell(top)
    for layer, x0, y0, x1, y1 in rects:
        g, d = GDS_LAYER[layer]
        x0, y0, x1, y1 = dbu_to_um((x0, y0, x1, y1))
        cell.add(gdstk.rectangle((x0, y0), (x1, y1), layer=g, datatype=d))
    lib.write_gds(path)


def write_def(path, nets, top, die=(0, 0, 30000, 4000)):
    with open(path, 'w') as f:
        f.write('VERSION 5.8 ;\nDIVIDERCHAR "/" ;\nBUSBITCHARS "[]" ;\n')
        f.write(f'DESIGN {top} ;\nUNITS DISTANCE MICRONS {DBU} ;\n')
        f.write(f'DIEAREA ( {die[0]} {die[1]} ) ( {die[2]} {die[3]} ) ;\n')
        f.write(f'NETS {len(nets)} ;\n')
        for net, rects in nets.items():
            f.write(f' - {net}\n')
            for layer, x0, y0, x1, y1 in rects:
                f.write(f'  + RECT {layer} ( {x0} {y0} ) ( {x1} {y1} )\n')
            f.write(' ;\n')
        f.write('END NETS\nEND DESIGN\n')


def def_rects_from_routed_def(path):
    """[(layer, net, x0, y0, x1, y1)] parsed out of a routed DEF, DBU."""
    out, innets, net = [], False, None
    for line in open(path, errors='ignore'):
        if line.startswith('NETS'):
            innets = True
            continue
        if line.startswith('END NETS'):
            break
        if not innets:
            continue
        m = re.match(r'\s*- (\S+)', line)
        if m:
            net = m.group(1)
            continue
        m = re.match(r'\s*\+ RECT (\w+) \( (-?\d+) (-?\d+) \) \( (-?\d+) (-?\d+) \)', line)
        if m and m.group(1) in GDS_LAYER:
            v = [int(x) for x in m.groups()[1:]]
            out.append((m.group(1), net, *v))
    return out


class AutocompleteFixture(unittest.TestCase):
    def setUp(self):
        if not os.access(ROUTER, os.X_OK):
            self.skipTest(f'{ROUTER} not built (run make first)')
        self.d = tempfile.mkdtemp()
        self.layers = os.path.join(self.d, 'layers.json')
        with open(self.layers, 'w') as f:
            json.dump(LAYERS, f)
        self.lef = os.path.join(self.d, 'pad.lef')
        with open(self.lef, 'w') as f:
            f.write(LEF)
        self.placement = os.path.join(self.d, 'top.placement_verilog.json')
        with open(self.placement, 'w') as f:
            json.dump(PLACEMENT, f)

    def run_check_nets(self, gds, deffile):
        cmd = [sys.executable, CHECK_NETS, gds, '--top', 'TOP_CONC_0',
               '--layers', self.layers, '--def', deffile]
        p = subprocess.run(cmd, capture_output=True, text=True)
        return p.returncode, p.stdout + p.stderr

    def test_broken_net_is_reported_open(self):
        """The fixture is genuinely broken before routing -- a negative
        control, so a router/paint-back bug that leaves N1 broken can't
        make this file report a false pass."""
        gds = os.path.join(self.d, 'before.gds')
        write_gds(gds, 'TOP_CONC_0',
                  [('M1', *PIN0_DBU), ('M1', *PIN1_DBU),
                   ('M2', *FRAG1_DBU), ('M2', *FRAG2_DBU)])
        deffile = os.path.join(self.d, 'before.def')
        write_def(deffile, {'N1': [('M1', *PIN0_DBU), ('M1', *PIN1_DBU),
                                    ('M2', *FRAG1_DBU), ('M2', *FRAG2_DBU)]},
                  top='TOP_CONC_0')
        rc, out = self.run_check_nets(gds, deffile)
        self.assertEqual(rc, 1, out)
        self.assertIn('OPEN', out)
        self.assertIn('N1', out)

    def test_virtual_pins_complete_the_net(self):
        """Route with the fragments declared as virtual_pins, paint the
        routed DEF back into a GDS, and confirm one conductor."""
        ndr = os.path.join(self.d, 'ndr.json')
        with open(ndr, 'w') as f:
            json.dump([{
                'module': 'TOP_CONC_0',
                'nets': [{'name': 'N1', 'virtual_pins': [
                    {'M2': [list(dbu_to_um(FRAG1_DBU))]},
                    {'M2': [list(dbu_to_um(FRAG2_DBU))]},
                ]}],
            }], f)

        out_dir = os.path.join(self.d, 'route_out')
        os.makedirs(out_dir, exist_ok=True)
        p = subprocess.run([ROUTER, '-d', self.layers, '-p', self.placement,
                             '-l', self.lef, '-uu', str(DBU), '-ndr', ndr,
                             '-o', out_dir],
                            capture_output=True, text=True)
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)

        routed_def = os.path.join(out_dir, 'TOP_CONC_0.def')
        self.assertTrue(os.path.exists(routed_def), 'router produced no DEF')

        # Paint the router's new metal into the *original broken layout*,
        # not a fresh GDS built only from the router's own output -- the
        # thing under test is that the new metal actually lands on the
        # pre-existing fragments, not merely that the router's output is
        # self-connected.
        after_gds = os.path.join(self.d, 'after.gds')
        rects = def_rects_from_routed_def(routed_def)
        write_gds(after_gds, 'TOP_CONC_0',
                  [('M1', *PIN0_DBU), ('M1', *PIN1_DBU),
                   ('M2', *FRAG1_DBU), ('M2', *FRAG2_DBU)]
                  + [(l, x0, y0, x1, y1) for l, _n, x0, y0, x1, y1 in rects])

        rc, out = self.run_check_nets(after_gds, routed_def)
        self.assertEqual(rc, 0, out)
        self.assertIn('1 net(s), one conductor each', out)
        # rc==0 alone would also pass if a fragment landed disjoint from all
        # router metal (check_nets.py counts that as "no DEF wire", not an
        # open) -- pin the conductor count explicitly so that failure mode
        # can't hide.
        self.assertIn('-> 1 conductor(s)', out)


if __name__ == '__main__':
    unittest.main(verbosity=1)
