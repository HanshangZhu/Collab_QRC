"""Tests for the dropout-benchmark summarizer metric helpers.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test_summarize_dropout_benchmark.py -v
(from scripts/bench/)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from summarize_dropout_benchmark import time_to_coverage, final_coverage


def test_final_coverage_takes_last_row():
    rows = [{"t_sim": "0", "global_coverage_ratio": "0.1"},
            {"t_sim": "10", "global_coverage_ratio": "0.4"},
            {"t_sim": "20", "global_coverage_ratio": "0.55"}]
    assert final_coverage(rows) == 0.55


def test_final_coverage_empty_is_zero():
    assert final_coverage([]) == 0.0


def test_time_to_coverage_returns_first_crossing():
    rows = [{"t_sim": "0", "global_coverage_ratio": "0.1"},
            {"t_sim": "10", "global_coverage_ratio": "0.4"},
            {"t_sim": "20", "global_coverage_ratio": "0.55"}]
    assert time_to_coverage(rows, 0.5) == 20.0


def test_time_to_coverage_none_if_never_reached():
    rows = [{"t_sim": "0", "global_coverage_ratio": "0.1"}]
    assert time_to_coverage(rows, 0.9) is None
