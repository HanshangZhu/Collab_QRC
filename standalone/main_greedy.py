#!/usr/bin/env python3
"""Standalone MuJoCo exploration — greedy nearest-frontier baseline.

Identical nav/sim/CHAMP stack as main_vlm.py, but goal selection uses
the nearest frontier heuristic: at each step pick the A*-validated
frontier candidate with the smallest Euclidean distance to the robot.

This corresponds to the basic "nearest unexplored frontier" variant of
the Yamauchi (1997) frontier-based exploration algorithm, without the
information-gain weighting that CFPA2 adds.

No API key or network connection required.

Usage:
    mjpython standalone/main_greedy.py [--config path] [--headless] [--no-map]

This is one of four baselines for the exploration comparison:
    main_random.py      — random frontier (lower bound)
    main_greedy.py      — greedy nearest frontier (Yamauchi 1997 simplified)
    main_champ.py       — CFPA2 info-gain frontier (Yamauchi 1997 full)
    main_vlm.py         — VLM-guided exploration (this paper)
"""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT))
sys.path.insert(0, str(_REPO_ROOT / "standalone"))

# Reuse main_vlm.main() — inject fixed CLI flags before argparse runs.
# --provider mock          → no API calls, LangGraph bypassed
# --strategy greedy_nearest → pick closest frontier by Euclidean distance
# --cycle-sec 2            → fast decision loop (no network latency)
_injected = ["--provider", "mock", "--strategy", "greedy_nearest",
             "--cycle-sec", "2.0"]
for flag in reversed(_injected):
    if flag not in sys.argv:
        sys.argv.insert(1, flag)

# Override run_name so the timeseries CSV is labelled "greedy_nearest"
import standalone.observability.metrics_logger as _ml
_orig_init = _ml.ExplorationMetricsLogger.__init__


def _patched_init(self, *args, run_name: str = "run", **kwargs):
    if run_name.startswith("vlm_mock_"):
        run_name = run_name.replace("vlm_mock_", "")
    _orig_init(self, *args, run_name=run_name, **kwargs)


_ml.ExplorationMetricsLogger.__init__ = _patched_init  # type: ignore[method-assign]

import standalone.main_vlm as _main_vlm

if __name__ == "__main__":
    _main_vlm.main()
