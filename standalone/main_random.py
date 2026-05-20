#!/usr/bin/env python3
"""Standalone MuJoCo exploration — random frontier baseline.

Identical nav/sim/CHAMP stack as main_vlm.py, but goal selection is
purely random: at each decision step the robot picks one of the
A*-validated frontier candidates uniformly at random.

No API key or network connection required.

Usage:
    mjpython standalone/main_random.py [--config path] [--headless] [--no-map]

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
# --provider mock   → no API calls, LangGraph bypassed
# --strategy random → pick uniformly at random from A*-validated frontiers
# --cycle-sec 2     → fast decision loop (no network latency)
_injected = ["--provider", "mock", "--strategy", "random", "--cycle-sec", "2.0"]
for flag in reversed(_injected):
    if flag not in sys.argv:
        sys.argv.insert(1, flag)

# Override run_name so the timeseries CSV is labelled "random" not "vlm_mock_random"
import standalone.observability.metrics_logger as _ml
_orig_init = _ml.ExplorationMetricsLogger.__init__


def _patched_init(self, *args, run_name: str = "run", **kwargs):
    if run_name.startswith("vlm_mock_"):
        run_name = run_name.replace("vlm_mock_", "")
    _orig_init(self, *args, run_name=run_name, **kwargs)


_ml.ExplorationMetricsLogger.__init__ = _patched_init  # type: ignore[method-assign]

import standalone.main_vlm as _main_vlm

import logging
logging.getLogger("main_vlm").name  # ensure logger exists
import logging as _logging
_logging.getLogger("main_vlm").name  # no-op; label in logs still says main_vlm

if __name__ == "__main__":
    _main_vlm.main()
