"""Unit tests for the architecture-agnostic union-coverage helper.

Run from this directory: ``python -m pytest test_coverage_util.py -v``
(The helper lives beside this file because go2w_observability is an
ament_cmake scripts-only package with no importable module.)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from coverage_util import union_known_cell_count


def test_union_counts_distinct_cells():
    a = {(0, 0), (1, 0), (2, 0)}
    b = {(2, 0), (3, 0)}
    # union = {(0,0),(1,0),(2,0),(3,0)} -> 4 distinct
    assert union_known_cell_count([a, b]) == 4


def test_union_empty_inputs_is_zero():
    assert union_known_cell_count([]) == 0
    assert union_known_cell_count([set(), set()]) == 0


def test_union_single_robot_passthrough():
    a = {(0, 0), (1, 1)}
    assert union_known_cell_count([a]) == 2
