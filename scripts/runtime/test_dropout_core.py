"""Tests for the ROS-free probabilistic drop logic.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test_dropout_core.py -v
(from scripts/runtime/)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dropout_core import DropCounter


def test_prob_zero_never_drops():
    dc = DropCounter(drop_prob=0.0, seed=1)
    assert all(not dc.should_drop() for _ in range(1000))
    assert dc.dropped == 0 and dc.passed == 1000


def test_prob_one_always_drops():
    dc = DropCounter(drop_prob=1.0, seed=1)
    assert all(dc.should_drop() for _ in range(1000))
    assert dc.passed == 0 and dc.dropped == 1000


def test_prob_half_is_approximately_half():
    dc = DropCounter(drop_prob=0.5, seed=42)
    n = 20000
    for _ in range(n):
        dc.should_drop()
    frac = dc.dropped / n
    assert 0.47 < frac < 0.53


def test_seed_is_reproducible():
    a = DropCounter(drop_prob=0.5, seed=7)
    b = DropCounter(drop_prob=0.5, seed=7)
    seq_a = [a.should_drop() for _ in range(500)]
    seq_b = [b.should_drop() for _ in range(500)]
    assert seq_a == seq_b


def test_prob_clamped_to_unit_interval():
    assert DropCounter(drop_prob=5.0, seed=1).drop_prob == 1.0
    assert DropCounter(drop_prob=-2.0, seed=1).drop_prob == 0.0
