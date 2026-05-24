"""Tests for the ROS-free SLAM-divergence decision.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test_divergence_core.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from divergence_core import is_diverged


def test_in_bounds_is_not_diverged():
    assert not is_diverged(3.9, 2.0, bound=60.0)
    assert not is_diverged(-50.0, 59.9, bound=60.0)


def test_out_of_bounds_x_is_diverged():
    assert is_diverged(-21133.6, 0.0, bound=60.0)


def test_out_of_bounds_y_is_diverged():
    assert is_diverged(0.0, 75.0, bound=60.0)


def test_nan_and_inf_are_diverged():
    assert is_diverged(float("nan"), 0.0, bound=60.0)
    assert is_diverged(0.0, float("inf"), bound=60.0)
    assert is_diverged(-math.inf, 0.0, bound=60.0)


def test_boundary_exactly_at_bound_is_not_diverged():
    assert not is_diverged(60.0, -60.0, bound=60.0)
