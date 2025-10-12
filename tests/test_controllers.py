import importlib

from pypnc.controllers import Node, generate_path

pp = importlib.import_module("pypnc.controllers.pure_pursuit")


def test_generate_path_smoke():
    # only a smoke test: ensure function runs and returns expected tuple length
    states = [(0, 0, 0), (5, 2, 30)]
    out = generate_path(states)
    assert isinstance(out, tuple)
    assert len(out) == 6


def test_pure_pursuit_interface():
    # minimal interface smoke test for pure_pursuit
    node = Node(0.0, 0.0, 0.0, 0.0, 1.0)
    # build a trivial path
    cx = [0.0, 5.0]
    cy = [0.0, 2.0]
    path = pp.PATH(cx, cy)
    delta, ind = pp.pure_pursuit(node, path, 0)
    assert isinstance(delta, float)
    assert isinstance(ind, int)
