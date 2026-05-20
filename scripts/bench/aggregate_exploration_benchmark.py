#!/usr/bin/env python3
"""Aggregate exploration benchmark trial outputs.

Expected layout:

    <root>/<env>/<planner>/trial_<n>/
        robot_a.json
        robot_b.json
        metrics.csv or exploration_*.csv

The per-robot JSON files come from ``session_reporter.py``.  The CSV file comes
from ``exploration_metrics_logger.py`` and carries global merged-map coverage.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROBOTS = ("robot_a", "robot_b")
CHECKPOINT_SECONDS = (120, 240, 360, 480)
CHECKPOINT_MAX_SAMPLE_AGE_SEC = 5.0


@dataclass(frozen=True)
class TrialRecord:
    env: str
    planner: str
    trial: str
    path: Path
    robots: dict[str, dict[str, Any]]
    global_metrics: dict[str, float]
    timeseries: list[dict[str, float]]
    nav2_plan_ms: list[float]
    nav2_plan_success_count: int
    nav2_plan_failure_count: int


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except Exception:
        return {}


def _float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _find_metrics_csv(trial_dir: Path) -> Path | None:
    exact = trial_dir / "metrics.csv"
    if exact.exists():
        return exact
    matches = sorted(trial_dir.glob("exploration_*.csv"))
    return matches[-1] if matches else None


def _load_csv_rows(path: Path | None) -> list[dict[str, float]]:
    if path is None or not path.exists():
        return []
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return []
    return [
        {k: _float(v) for k, v in row.items() if k}
        for row in rows
    ]


def _load_last_csv_row(rows: list[dict[str, float]]) -> dict[str, float]:
    return rows[-1] if rows else {}


_PLAN_EVENT_RE = re.compile(r"PLAN_(RETURNED|FAILED): .*? t=([0-9.]+)ms")


def _find_event_log(trial_dir: Path) -> Path | None:
    matches = sorted(trial_dir.glob("exploration_events_*.log"))
    return matches[-1] if matches else None


def _load_nav2_plan_events(path: Path | None) -> tuple[list[float], int, int]:
    if path is None or not path.exists():
        return [], 0, 0
    plan_ms: list[float] = []
    success = 0
    failure = 0
    try:
        for line in path.read_text(errors="replace").splitlines():
            match = _PLAN_EVENT_RE.search(line)
            if not match:
                continue
            kind, value = match.groups()
            if kind == "RETURNED":
                plan_ms.append(_float(value))
                success += 1
            else:
                failure += 1
    except OSError:
        return [], 0, 0
    return plan_ms, success, failure


def collect_trial_records(root: Path) -> list[TrialRecord]:
    root = Path(root)
    records: list[TrialRecord] = []
    for trial_dir in sorted(root.glob("*/*/trial_*")):
        if not trial_dir.is_dir():
            continue
        rel = trial_dir.relative_to(root)
        if len(rel.parts) < 3:
            continue
        env, planner, trial = rel.parts[:3]
        robots = {ns: _load_json(trial_dir / f"{ns}.json") for ns in ROBOTS}
        timeseries = _load_csv_rows(_find_metrics_csv(trial_dir))
        global_metrics = _load_last_csv_row(timeseries)
        nav2_plan_ms, nav2_success, nav2_failure = _load_nav2_plan_events(
            _find_event_log(trial_dir)
        )
        records.append(TrialRecord(
            env=env,
            planner=planner,
            trial=trial,
            path=trial_dir,
            robots=robots,
            global_metrics=global_metrics,
            timeseries=timeseries,
            nav2_plan_ms=nav2_plan_ms,
            nav2_plan_success_count=nav2_success,
            nav2_plan_failure_count=nav2_failure,
        ))
    return records


def _mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def _stdev(values: list[float]) -> float:
    return statistics.stdev(values) if len(values) >= 2 else 0.0


def _mean_or_none(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def _stdev_or_none(values: list[float]) -> float | None:
    if not values:
        return None
    return statistics.stdev(values) if len(values) >= 2 else 0.0


def _percentile(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    k = (len(ordered) - 1) * p
    lo = int(k)
    hi = min(lo + 1, len(ordered) - 1)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (k - lo)


def _robot_value(record: TrialRecord, ns: str, section: str, key: str) -> float:
    data = record.robots.get(ns, {})
    return _float(data.get(section, {}).get(key))


def _robot_nested_value(
    record: TrialRecord,
    ns: str,
    section: str,
    subsection: str,
    key: str,
) -> float:
    data = record.robots.get(ns, {})
    return _float(data.get(section, {}).get(subsection, {}).get(key))


def _robot_bool(record: TrialRecord, ns: str, section: str, key: str) -> float:
    data = record.robots.get(ns, {})
    return 1.0 if bool(data.get(section, {}).get(key)) else 0.0


def _record_wall_time_sec(record: TrialRecord) -> float:
    durations = []
    for robot in record.robots.values():
        start = _float(robot.get("started_at_unix"))
        end = _float(robot.get("ended_at_unix"))
        if end > start > 0.0:
            durations.append(end - start)
    return max(durations) if durations else 0.0


def _record_elapsed_sec(record: TrialRecord) -> float:
    elapsed = [_float(robot.get("elapsed_sec")) for robot in record.robots.values()]
    return max(elapsed) if elapsed else 0.0


def _record_rtf(record: TrialRecord) -> float:
    wall = _record_wall_time_sec(record)
    elapsed = _record_elapsed_sec(record)
    return (elapsed / wall) if wall > 0.0 else 0.0


def _timed_rows(record: TrialRecord) -> list[tuple[float, dict[str, float]]]:
    rows = [
        row for row in record.timeseries
        if "t_sim" in row or "t_wall" in row
    ]
    if not rows:
        return []
    first = rows[0]
    use_sim = "t_sim" in first
    time_key = "t_sim" if use_sim else "t_wall"
    start_t = first.get(time_key, 0.0)
    timed = [
        (max(0.0, row.get(time_key, 0.0) - start_t), row)
        for row in rows
        if time_key in row
    ]
    return sorted(timed, key=lambda item: item[0])


def _time_to_coverage_sec(record: TrialRecord, threshold: float) -> float | None:
    for elapsed, row in _timed_rows(record):
        if row.get("global_coverage_ratio", 0.0) >= threshold:
            return elapsed
    return None


def _checkpoint_row(record: TrialRecord, checkpoint_sec: int) -> dict[str, float] | None:
    timed = _timed_rows(record)
    if not timed or timed[-1][0] < checkpoint_sec:
        return None
    selected_elapsed = None
    selected_row = None
    for elapsed, row in timed:
        if elapsed <= checkpoint_sec:
            selected_elapsed = elapsed
            selected_row = row
        else:
            break
    if selected_elapsed is None or selected_row is None:
        return None
    if checkpoint_sec - selected_elapsed > CHECKPOINT_MAX_SAMPLE_AGE_SEC:
        return None
    return selected_row


def _row_values(rows: list[dict[str, float]], key: str) -> list[float]:
    return [_float(row.get(key)) for row in rows if key in row]


def summarise_records(records: list[TrialRecord]) -> dict[tuple[str, str], dict[str, Any]]:
    grouped: dict[tuple[str, str], list[TrialRecord]] = {}
    for record in records:
        grouped.setdefault((record.env, record.planner), []).append(record)

    summary: dict[tuple[str, str], dict[str, Any]] = {}
    for key, group in sorted(grouped.items()):
        global_area = [
            _float(r.global_metrics.get("global_explored_area_m2"))
            for r in group
            if "global_explored_area_m2" in r.global_metrics
        ]
        global_cov = [
            _float(r.global_metrics.get("global_coverage_ratio"))
            for r in group
            if "global_coverage_ratio" in r.global_metrics
        ]
        overlap = [
            _float(r.global_metrics.get("overlap_pct"))
            for r in group
            if "overlap_pct" in r.global_metrics
        ]
        wall_times = [_record_wall_time_sec(r) for r in group if _record_wall_time_sec(r) > 0.0]
        rtfs = [_record_rtf(r) for r in group if _record_rtf(r) > 0.0]
        total_distances = [
            sum(_robot_value(r, ns, "progress", "distance_travelled_m") for ns in ROBOTS)
            for r in group
        ]
        global_efficiencies = [
            area / dist
            for area, dist in zip(global_area, total_distances)
            if dist > 0.0
        ]
        nav2_plan_ms = [value for r in group for value in r.nav2_plan_ms]
        nav2_success = sum(r.nav2_plan_success_count for r in group)
        nav2_failure = sum(r.nav2_plan_failure_count for r in group)
        nav2_total = nav2_success + nav2_failure
        entry: dict[str, Any] = {
            "trial_count": float(len(group)),
            "completed_trial_count": float(
                sum(
                    1 for r in group
                    if all(r.robots.get(ns, {}).get("outcome") == "completed"
                           for ns in ROBOTS)
                )
            ),
            "global_explored_area_m2_mean": _mean(global_area),
            "global_explored_area_m2_std": _stdev(global_area),
            "global_coverage_ratio_mean": _mean(global_cov),
            "global_coverage_ratio_std": _stdev(global_cov),
            "overlap_pct_mean": _mean(overlap),
            "overlap_pct_std": _stdev(overlap),
            "wall_time_sec_mean": _mean(wall_times),
            "wall_time_sec_std": _stdev(wall_times),
            "real_time_factor_mean": _mean(rtfs),
            "real_time_factor_std": _stdev(rtfs),
            "global_efficiency_m2_per_m_mean": _mean(global_efficiencies),
            "nav2_plan_ms_mean": _mean(nav2_plan_ms),
            "nav2_plan_ms_p95": _percentile(nav2_plan_ms, 0.95),
            "nav2_plan_success_rate": (
                float(nav2_success) / float(nav2_total) if nav2_total else 0.0
            ),
        }
        for threshold in (0.25, 0.50, 0.75, 0.90):
            pct = int(round(threshold * 100.0))
            times = [
                t for t in (_time_to_coverage_sec(r, threshold) for r in group)
                if t is not None
            ]
            entry[f"time_to_coverage_{pct}pct_sec_mean"] = _mean_or_none(times)
            entry[f"coverage_{pct}pct_reach_rate"] = (
                float(len(times)) / float(len(group)) if group else 0.0
            )
        for checkpoint_sec in CHECKPOINT_SECONDS:
            prefix = f"at_{checkpoint_sec}s"
            rows = [
                row for row in (_checkpoint_row(r, checkpoint_sec) for r in group)
                if row is not None
            ]
            entry[f"{prefix}_sample_count"] = float(len(rows))
            entry[f"{prefix}_sample_rate"] = (
                float(len(rows)) / float(len(group)) if group else 0.0
            )
            global_area_at = _row_values(rows, "global_explored_area_m2")
            global_cov_at = _row_values(rows, "global_coverage_ratio")
            entry[f"{prefix}_global_explored_area_m2_mean"] = _mean_or_none(global_area_at)
            entry[f"{prefix}_global_explored_area_m2_std"] = _stdev_or_none(global_area_at)
            entry[f"{prefix}_global_coverage_ratio_mean"] = _mean_or_none(global_cov_at)
            entry[f"{prefix}_global_coverage_ratio_std"] = _stdev_or_none(global_cov_at)
            for ns in ROBOTS:
                distances_at = _row_values(rows, f"{ns}_trajectory_m")
                areas_at = _row_values(rows, f"{ns}_coverage_area_m2")
                entry[f"{prefix}_{ns}_distance_m_mean"] = _mean_or_none(distances_at)
                entry[f"{prefix}_{ns}_distance_m_std"] = _stdev_or_none(distances_at)
                entry[f"{prefix}_{ns}_explored_area_m2_mean"] = _mean_or_none(areas_at)
                entry[f"{prefix}_{ns}_explored_area_m2_std"] = _stdev_or_none(areas_at)
        for ns in ROBOTS:
            distances = [_robot_value(r, ns, "progress", "distance_travelled_m") for r in group]
            areas = [_robot_value(r, ns, "coverage", "explored_area_m2") for r in group]
            contacts = [_robot_value(r, ns, "safety", "wall_contact_count") for r in group]
            obstacle_contacts = [
                _robot_value(r, ns, "safety", "obstacle_contact_count")
                for r in group
            ]
            ever_touched = [_robot_bool(r, ns, "safety", "ever_touched") for r in group]
            tipped = [_robot_bool(r, ns, "progress", "tipped_over") for r in group]
            degraded_entries = [
                _robot_nested_value(r, ns, "progress", "degraded_tilt", "entry_count")
                for r in group
            ]
            degraded = [1.0 if count > 0.0 else 0.0 for count in degraded_entries]
            slam_mean = [
                _robot_value(r, ns, "slam", "trans_error_mean_m")
                for r in group
                if "trans_error_mean_m" in r.robots.get(ns, {}).get("slam", {})
            ]
            slam_final = [
                _robot_value(r, ns, "slam", "trans_error_final_m")
                for r in group
                if "trans_error_final_m" in r.robots.get(ns, {}).get("slam", {})
            ]
            entry[f"{ns}_distance_m_mean"] = _mean(distances)
            entry[f"{ns}_distance_m_std"] = _stdev(distances)
            entry[f"{ns}_explored_area_m2_mean"] = _mean(areas)
            entry[f"{ns}_explored_area_m2_std"] = _stdev(areas)
            entry[f"{ns}_wall_contacts_mean"] = _mean(contacts)
            entry[f"{ns}_obstacle_contacts_mean"] = _mean(obstacle_contacts)
            entry[f"{ns}_ever_touched_rate"] = _mean(ever_touched)
            entry[f"{ns}_tipped_over_rate"] = _mean(tipped)
            entry[f"{ns}_tipped_events_mean"] = _mean(tipped)
            entry[f"{ns}_degraded_tilt_rate"] = _mean(degraded)
            entry[f"{ns}_degraded_tilt_events_mean"] = _mean(degraded_entries)
            entry[f"{ns}_slam_trans_error_mean_m_mean"] = _mean(slam_mean)
            entry[f"{ns}_slam_trans_error_final_m_mean"] = _mean(slam_final)
            entry[f"{ns}_slam_translation_error_m_mean"] = _mean(slam_mean)
            entry[f"{ns}_slam_translation_error_final_m_mean"] = _mean(slam_final)
        summary[key] = entry
    return summary


def write_summary(root: Path, summary: dict[tuple[str, str], dict[str, Any]]) -> tuple[Path, Path]:
    json_path = root / "summary.json"
    csv_path = root / "summary.csv"

    json_payload = {
        f"{env}/{planner}": metrics
        for (env, planner), metrics in summary.items()
    }
    json_path.write_text(json.dumps(json_payload, indent=2) + "\n")

    fieldnames = ["env", "planner"]
    for metrics in summary.values():
        for key in metrics:
            if key not in fieldnames:
                fieldnames.append(key)
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for (env, planner), metrics in summary.items():
            row = {"env": env, "planner": planner}
            row.update(metrics)
            writer.writerow(row)
    return json_path, csv_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="Benchmark output root.")
    args = parser.parse_args()

    records = collect_trial_records(args.root)
    summary = summarise_records(records)
    json_path, csv_path = write_summary(args.root, summary)
    print(f"records={len(records)} summary_json={json_path} summary_csv={csv_path}")
    return 0 if records else 1


if __name__ == "__main__":
    raise SystemExit(main())
