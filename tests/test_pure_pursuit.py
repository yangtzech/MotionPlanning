import importlib
import os
import sys

# ensure repository root is on sys.path for pytest
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def test_pure_pursuit_straight():
    mod = importlib.import_module('pypnc.controllers.pure_pursuit')
    cx = [0.0, 10.0, 20.0]
    cy = [0.0, 0.0, 0.0]
    path = mod.PATH(cx, cy)
    # small lateral offset
    node = mod.Node(1.0, 0.1, 0.0, v=1.0, direct=1.0)
    delta, ind = mod.pure_pursuit(node, path, 0)
    assert abs(delta) < 0.2


def test_pid_control_basic():
    mod = importlib.import_module('pypnc.controllers.pure_pursuit')
    a = mod.pid_control(5.0, 1.0, 100.0, 1.0)
    assert a > 0
