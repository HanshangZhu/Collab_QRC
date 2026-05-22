#!/usr/bin/env python3
"""Validate one exploration benchmark trial directory."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


ROBOTS = ("robot_a", "robot_b")
DEFAULT_CHECKPOINT_SECONDS = (120, 240, 360, 480)
CHECKPOINT_MAX_SAMPLE_AGE_SEC = 5.0
REQUIRED_CSV_COLUMNS = (
    "global_explored_area_m2",
    "global_coverage_ratio",
    "robot_a_trajectory_m",
    "robot_a_coverage_area_m2",
    "robot_b_trajectory_m",
    "robot_b_coverage_area_m2",
)


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


def _load_csv_rows(path: Path | None) -> list[dict[str, str]]:
    if path is None or not path.exists():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _timed_rows(rows: list[dict[str, str]]) -> list[tuple[float, dict[str, str]]]:
    if not rows:
        return []
    first = rows[0]
    time_key = "t_sim" if "t_sim" in first else "t_wall" if "t_wall" in first else ""
    if not time_key:
        return []
    start_t = _float(first.get(time_key))
    return sorted(
        (
            max(0.0, _float(row.get(time_key)) - start_t),
            row,
        )
        for row in rows
        if row.get(time_key) not in (None, "")
    )


def _checkpoint_available(rows: list[dict[str, str]], checkpoint_sec: int) -> bool:
    timed = _timed_rows(rows)
    if not timed or timed[-1][0] < checkpoint_sec:
        return False
    selected_elapsed: float | None = None
    for elapsed, _row in timed:
        if elapsed <= checkpoint_sec:
            selected_elapsed = elapsed
        else:
            break
    if selected_elapsed is None:
        return False
    return checkpoint_sec - selected_elapsed <= CHECKPOINT_MAX_SAMPLE_AGE_SEC


def _load_robot_json(path: Path) -> tuple[dict[str, Any], str | None]:
    try:
        data = json.loads(path.read_text())
    except Exception as exc:
        return {}, f"{path.name} is not valid JSON: {exc}"
    if not isinstance(data, dict):
        return {}, f"{path.name} is not a JSON object"
    return data, None


def validate_trial(
    trial_dir: Path,
    *,
    duration_sec: float,
    checkpoint_seconds: tuple[int, ...] = DEFAULT_CHECKPOINT_SECONDS,
) -> dict[str, Any]:
    trial_dir = Path(trial_dir)
    errors: list[str] = []
    warnings: list[str] = []

    exit_code_path = trial_dir / "exit_code.txt"
    if not exit_code_path.exists():
        errors.append("exit_code.txt is missing")
    else:
        exit_code = exit_code_path.read_text(errors="replace").strip()
        if exit_code != "0":
            errors.append(f"launch exit code is {exit_code!r}, expected '0'")

    robot_payloads: dict[str, dict[str, Any]] = {}
    for ns in ROBOTS:
        robot_json = trial_dir / f"{ns}.json"
        if not robot_json.exists():
            errors.append(f"{robot_json.name} is missing")
            continue
        payload, error = _load_robot_json(robot_json)
        if error is not None:
            errors.append(error)
            continue
        robot_payloads[ns] = payload
        outcome = payload.get("outcome")
        if outcome != "completed":
            errors.append(f"{robot_json.name} outcome is {outcome!r}, expected 'completed'")
        distance = _float(payload.get("progress", {}).get("distance_travelled_m"))
        if distance < 0.5:
            warnings.append(f"{ns} travelled only {distance:.3f} m")

    metrics_csv = _find_metrics_csv(trial_dir)
    rows = _load_csv_rows(metrics_csv)
    if metrics_csv is None:
        errors.append("metrics CSV is missing")
    elif len(rows) < 2:
        errors.append(f"{metrics_csv.name} has fewer than 2 data rows")
    else:
        fieldnames = rows[0].keys()
        if "t_sim" not in fieldnames and "t_wall" not in fieldnames:
            errors.append(f"{metrics_csv.name} has neither t_sim nor t_wall")
        for column in REQUIRED_CSV_COLUMNS:
            if column not in fieldnames:
                errors.append(f"{metrics_csv.name} missing required column {column}")
        final_row = rows[-1]
        if _float(final_row.get("global_explored_area_m2"), -1.0) < 0.0:
            errors.append(f"{metrics_csv.name} final global_explored_area_m2 is invalid")
        if _float(final_row.get("global_coverage_ratio"), -1.0) < 0.0:
            errors.append(f"{metrics_csv.name} final global_coverage_ratio is invalid")
        for checkpoint_sec in checkpoint_seconds:
            if checkpoint_sec <= duration_sec and not _checkpoint_available(rows, checkpoint_sec):
                errors.append(
                    f"{metrics_csv.name} has no usable {checkpoint_sec}s checkpoint sample"
                )

    event_logs = sorted(trial_dir.glob("exploration_events_*.log"))
    if not event_logs:
        errors.append("exploration_events_*.log is missing")
    else:
        latest_event_log = event_logs[-1]
        if latest_event_log.stat().st_size == 0:
            warnings.append(f"{latest_event_log.name} is empty")

    result = {
        "valid": not errors,
        "trial_dir": str(trial_dir),
        "metrics_csv": str(metrics_csv) if metrics_csv else "",
        "event_log": str(event_logs[-1]) if event_logs else "",
        "errors": errors,
        "warnings": warnings,
        "robots": sorted(robot_payloads),
    }
    (trial_dir / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def _parse_checkpoints(text: str) -> tuple[int, ...]:
    values: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if part:
            values.append(int(part))
    return tuple(values)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial_dir", type=Path)
    parser.add_argument("--duration-sec", type=float, required=True)
    parser.add_argument(
        "--checkpoint-seconds",
        default=",".join(str(x) for x in DEFAULT_CHECKPOINT_SECONDS),
        help="Comma-separated checkpoint seconds to require when <= duration.",
    )
    args = parser.parse_args()

    result = validate_trial(
        args.trial_dir,
        duration_sec=args.duration_sec,
        checkpoint_seconds=_parse_checkpoints(args.checkpoint_seconds),
    )
    print(json.dumps(result, indent=2))
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
