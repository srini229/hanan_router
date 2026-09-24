"""check_samenet.py's three rules on hand-drawn shapes."""
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
from check_samenet import check

RULES = {'M3': ('metal', 300, 300, 0, 0), 'V2': ('cut', 200, 200, 200, 200)}


def kinds(shapes):
    return sorted(b.split()[0] for b in check({'N': shapes}, RULES))


def test_gap_below_spacing_and_its_fill():
    a, b = ('M3', (0, 0, 1000, 400)), ('M3', (1100, 0, 2000, 400))
    assert kinds([a, b]) == ['gap']
    assert kinds([a, b, ('M3', (1000, 0, 1100, 400))]) == []
    assert kinds([a, ('M3', (1300, 0, 2000, 400))]) == []


def test_pad_cross_notch():
    v2pad, v3pad = ('M3', (57235, 17035, 57565, 17365)), ('M3', (57210, 17040, 57590, 17360))
    wire = ('M3', (57577, 17000, 59800, 17400))
    assert kinds([v2pad, v3pad, wire]) == ['gap']
    assert kinds([v2pad, v3pad, wire, ('M3', (57210, 17035, 57590, 17365))]) == []


def test_l_joint_neck():
    vert, horz = ('M3', (40000, 35400, 40400, 41400)), ('M3', (28000, 41200, 40200, 41600))
    assert kinds([vert, horz]) == ['neck']
    assert kinds([vert, ('M3', (28000, 41200, 40400, 41600))]) == []
    assert kinds([vert, horz, ('M3', (40200, 41400, 40400, 41600))]) == []


def test_cuts():
    assert kinds([('V2', (0, 0, 200, 200)), ('V2', (260, 0, 460, 200))]) == ['cut']
    assert kinds([('V2', (0, 0, 200, 200)), ('V2', (400, 0, 600, 200))]) == []
    assert kinds([('V2', (0, 0, 200, 200)), ('V2', (0, 0, 200, 200))]) == []
    assert kinds([('V2', (0, 0, 290, 200))]) == ['cut']
