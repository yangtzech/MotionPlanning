import math
from pypnc import utils


def test_pi_2_pi_positive():
    assert abs(utils.pi_2_pi(math.pi + 0.1) - (-math.pi + 0.1)) < 1e-9


def test_pi_2_pi_negative():
    assert abs(utils.pi_2_pi(-math.pi - 0.2) - (math.pi - 0.2)) < 1e-9
