"""Exploration metrics logger — stripped port of exploration_metrics_logger.py.

Monitors exploration progress and triggers stop condition when:
  - N consecutive 'no_reachable' status ticks, OR
  - known-area growth stagnates over a rolling window.

Publishes /<ns>/exploration_complete when done.

Timeseries CSV
--------------
Pass ``run_name`` and ``results_dir`` to the constructor.  Every
``timeseries_interval_sec`` sim-seconds the logger appends one row
``(sim_time_sec, known_m2)`` to an in-memory list and flushes a CSV to
``<results_dir>/coverage_<run_name>_<timestamp>.csv`` at stop / flush.
Call ``flush_timeseries()`` from the main script if you want an early save.
"""
from __future__ import annotations

import csv
import logging
import os
import time
from collections import deque
from pathlib import Path
from typing import List, Optional, Tuple

from ..core.bus import BUS
from ..core.ros_compat import OccupancyGrid, String, Twist

log = logging.getLogger("metrics_logger")


class ExplorationMetricsLogger:
    """Lightweight exploration stop trigger + progress log."""

    def __init__(
        self,
        namespace: str = "robot",
        *,
        consec_no_reachable_threshold: int = 3,
        coverage_stagnant_threshold_m2: float = 0.5,
        coverage_stagnant_window_sec: float = 30.0,
        enable_stop_trigger: bool = True,
        summary_interval_sec: float = 30.0,
        # Timeseries CSV parameters
        run_name: str = "run",
        results_dir: Optional[str] = None,
        timeseries_interval_sec: float = 5.0,
    ) -> None:
        self._ns = namespace
        self._consec_thr = consec_no_reachable_threshold
        self._stagnant_thr = coverage_stagnant_threshold_m2
        self._stagnant_window = coverage_stagnant_window_sec
        self._stop_enabled = enable_stop_trigger
        self._summary_interval = summary_interval_sec

        self._status: str = "searching"
        self._consec_no_reachable: int = 0
        self._last_status: str = ""
        self._coverage_history: deque = deque()  # (wall_t, known_m2)
        self._exploration_complete: bool = False
        self._last_summary_t: float = 0.0
        self._start_t: float = time.monotonic()

        # Timeseries recording: (sim_time_sec, known_m2) sampled every N sim-sec
        self._timeseries: List[Tuple[float, float]] = []
        self._ts_interval = timeseries_interval_sec
        self._last_ts_sim_t: float = -1e9
        self._run_name = run_name
        ts_stamp = time.strftime("%Y%m%d_%H%M%S")
        _results_dir = Path(results_dir) if results_dir else Path(
            os.environ.get("STANDALONE_RESULTS_DIR", "/tmp/standalone_results")
        )
        _results_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = _results_dir / f"coverage_{run_name}_{ts_stamp}.csv"

        BUS.subscribe(f"/{namespace}/exploration_status", self._on_status)
        BUS.subscribe(f"/{namespace}/map", self._on_map)

        self._complete_topic = f"/{namespace}/exploration_complete"
        self._cmd_vel_topic = f"/{namespace}/cmd_vel"

    def _on_status(self, msg: String) -> None:
        self._status = msg.data if hasattr(msg, "data") else str(msg)
        if self._status == "no_reachable":
            self._consec_no_reachable += 1
        else:
            self._consec_no_reachable = 0

        if self._status != self._last_status:
            elapsed = time.monotonic() - self._start_t
            log.info(f"[{self._ns}] exploration_status → {self._status}  "
                     f"(t={elapsed:.1f}s)")
            self._last_status = self._status

    def _on_map(self, grid: OccupancyGrid) -> None:
        if not grid.data:
            return
        import numpy as np
        data = np.array(grid.data, dtype=np.int8)
        known = float(np.sum(data >= 0)) * grid.info.resolution ** 2
        self._coverage_history.append((time.monotonic(), known))
        # Prune old entries
        cutoff = time.monotonic() - self._stagnant_window
        while self._coverage_history and self._coverage_history[0][0] < cutoff:
            self._coverage_history.popleft()

    # ── Timeseries helpers ────────────────────────────────────────────────────

    def record(self, sim_time: float) -> None:
        """Sample coverage at the given sim_time and append to timeseries.

        Called from the main loop every ``timeseries_interval_sec`` sim-seconds.
        """
        if sim_time - self._last_ts_sim_t < self._ts_interval:
            return
        known_m2 = self._coverage_history[-1][1] if self._coverage_history else 0.0
        self._timeseries.append((round(sim_time, 2), round(known_m2, 3)))
        self._last_ts_sim_t = sim_time

    def flush_timeseries(self) -> str:
        """Write timeseries to CSV and return the path."""
        with open(self._csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["sim_time_sec", "known_m2"])
            writer.writerows(self._timeseries)
        return str(self._csv_path)

    # ── Called from main loop ─────────────────────────────────────────────────

    def tick(self, sim_time: float = 0.0) -> bool:
        """Returns True when exploration is complete.

        Pass the current MuJoCo sim_time for timeseries recording.
        """
        if self._exploration_complete:
            return True

        self.record(sim_time)

        if self._stop_enabled:
            if self._consec_no_reachable >= self._consec_thr:
                self._trigger_stop(f"consec_no_reachable={self._consec_no_reachable}")
                return True

            if len(self._coverage_history) >= 2:
                oldest_known = self._coverage_history[0][1]
                latest_known = self._coverage_history[-1][1]
                delta = latest_known - oldest_known
                span = self._coverage_history[-1][0] - self._coverage_history[0][0]
                if span >= self._stagnant_window * 0.9 and delta < self._stagnant_thr:
                    self._trigger_stop(
                        f"coverage_stagnant Δ={delta:.2f}m² over {span:.0f}s")
                    return True

        t = time.monotonic()
        if t - self._last_summary_t >= self._summary_interval:
            self._log_summary()
            self._last_summary_t = t

        return False

    def _trigger_stop(self, reason: str) -> None:
        self._exploration_complete = True
        elapsed = time.monotonic() - self._start_t
        log.info(f"[{self._ns}] EXPLORATION COMPLETE: {reason}  "
                 f"(t={elapsed:.1f}s)")
        msg = String(data=reason)
        BUS.publish(self._complete_topic, msg, latch=True)
        # Send zero cmd_vel stop pulse
        stop = Twist()
        BUS.publish(self._cmd_vel_topic, stop)
        # Flush timeseries CSV
        try:
            csv_path = self.flush_timeseries()
            log.info(f"[{self._ns}] Timeseries CSV → {csv_path}  "
                     f"({len(self._timeseries)} rows)")
        except Exception as exc:
            log.warning(f"[{self._ns}] Could not write timeseries CSV: {exc}")

    def _log_summary(self) -> None:
        elapsed = time.monotonic() - self._start_t
        known_m2 = 0.0
        if self._coverage_history:
            known_m2 = self._coverage_history[-1][1]
        log.info(
            f"[{self._ns}] summary t={elapsed:.0f}s  "
            f"status={self._status}  "
            f"known={known_m2:.1f}m²  "
            f"consec_no_reachable={self._consec_no_reachable}"
        )
