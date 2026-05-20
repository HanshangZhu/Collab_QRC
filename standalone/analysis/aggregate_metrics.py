#!/usr/bin/env python3
"""aggregate_metrics.py — collect per-condition stats across all VLM/baseline runs.

Walks the logs/ tree, finds all *_report_*.json + coverage_*.csv files, groups
by condition (run_label inferred from path or report['run']), computes summary
statistics (final coverage mean/std/min/max, sim_time mean, goals_published,
artifacts_found, decision latency from per-cycle decision.json files), and
writes a single aggregated CSV + JSON.

Output:
    logs/_report_notes/metrics_aggregate.csv     ← per-trial rows
    logs/_report_notes/metrics_summary.json      ← per-condition summary
    logs/_report_notes/coverage_curves.json      ← per-condition median+IQR
                                                   (for plot generator)

Usage:
    python3 standalone/analysis/aggregate_metrics.py
    python3 standalone/analysis/aggregate_metrics.py --logs-root logs --out-dir logs/_report_notes
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics as stats
from pathlib import Path
from typing import Dict, List, Optional


# Old contaminated benchmark dirs from before timestamp-filter fix
# (per logs/_report_notes/SESSION_NOTES.md). Skip on aggregation.
_SKIP_DIRS = {
    "bench_20260519",
    "vlm_mock_variations",     # old May-19 fast bench
    "vlm_groq_attempts",       # all failed (mock fallback)
    "cfpa2",                   # pre-existing CFPA2 reports (use main_champ.py fresh ones instead)
    "greedy_nearest",          # pre-existing (use new mock_greedy_nearest/)
    "random",                  # pre-existing (use new mock_random/)
    "info_gain",               # pre-existing (use new mock_info_gain/)
}


def _path_contaminated(path: Path) -> bool:
    return any(p in _SKIP_DIRS for p in path.parts)


def find_reports(logs_root: Path) -> List[Path]:
    """Find all *_report_*.json files under logs/ tree (excluding contaminated)."""
    all_files = list(logs_root.rglob("*_report_*.json")) + list(logs_root.rglob("vlm_report_*.json"))
    # Deduplicate (a file can match both patterns)
    return sorted(set(p for p in all_files if not _path_contaminated(p)))


def infer_condition(report_path: Path, report_dict: dict) -> str:
    """Derive the condition label. Prefers directory-based grouping (newer layout)
    over the report['run'] field (older runs have hardcoded 'vlm').

    Layout convention: logs/<group>/<condition>/reports/<file>.json
    where <condition> starts with 'mock_' or 'vlm_' (e.g. 'mock_random',
    'vlm_groq_scout'). The umbrella group 'vlm_variations' is excluded.
    """
    UMBRELLA = {"vlm_variations", "vlm_gemini_real"}
    # 1. Innermost DIRECTORY part (deepest dir, excluding filename).
    for part in reversed(report_path.parent.parts):
        if part in UMBRELLA:
            continue
        if part == "cfpa2_champ":
            return "cfpa2"
        if part.startswith("mock_") or part.startswith("vlm_"):
            return part
    # 2. Explicit 'run' field (new code writes provider-aware label)
    run = report_dict.get("run", "")
    if run and run not in ("vlm", "run"):
        return run
    # 3. Provider-only fallback
    prov = report_dict.get("provider", "")
    if prov:
        return f"vlm_{prov}"
    return "unknown"


def find_timeseries(logs_root: Path) -> Dict[str, List[Path]]:
    """Map condition label → list of coverage_*.csv files.
    Skips legacy contaminated dirs. Prefers directory-based label
    (innermost dir starting w/ mock_ / vlm_, excluding umbrella names)."""
    UMBRELLA = {"vlm_variations", "vlm_gemini_real"}
    csvs = list(logs_root.rglob("coverage_*.csv"))
    out: Dict[str, List[Path]] = {}
    for f in csvs:
        if _path_contaminated(f):
            continue
        # Path-based inference (same convention as reports)
        label = None
        for part in reversed(f.parent.parts):
            if part in UMBRELLA:
                continue
            if part.startswith("mock_") or part.startswith("vlm_") or part == "cfpa2_champ":
                label = "cfpa2" if part == "cfpa2_champ" else part
                break
        if label is None:
            # Fallback: filename inference (legacy)
            stem = f.stem
            parts = stem[len("coverage_"):].rsplit("_", 2)
            label = parts[0] if parts else "unknown"
        out.setdefault(label, []).append(f)
    return out


def parse_csv(path: Path) -> List[tuple[float, float]]:
    """Parse coverage_*.csv → [(sim_time_sec, known_m2), ...]."""
    rows: List[tuple[float, float]] = []
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                rows.append((float(row[0]), float(row[1])))
            except ValueError:
                continue
    return rows


def summarize_condition(
    label: str,
    reports: List[dict],
    ts_files: List[Path],
) -> dict:
    """Compute summary stats for one condition."""
    n = len(reports)
    final_cov = [r.get("known_m2", 0.0) for r in reports]
    sim_times = [r.get("sim_time_sec", 0.0) for r in reports]
    goals = [r.get("goals_published", 0) for r in reports]
    cycles = [r.get("vlm_cycles", 0) for r in reports]
    arts = [r.get("artifacts_found", 0) for r in reports]

    def _stat(xs: List[float]) -> dict:
        if not xs:
            return {"n": 0}
        return {
            "n": len(xs),
            "mean": round(stats.mean(xs), 2),
            "std": round(stats.stdev(xs) if len(xs) > 1 else 0.0, 2),
            "min": round(min(xs), 2),
            "max": round(max(xs), 2),
            "median": round(stats.median(xs), 2),
        }

    return {
        "condition": label,
        "n_trials": n,
        "final_coverage_m2": _stat(final_cov),
        "sim_time_sec": _stat(sim_times),
        "goals_published": _stat(goals),
        "vlm_cycles": _stat(cycles),
        "artifacts_found": _stat(arts),
        "provider": reports[0].get("provider", "?") if reports else "?",
        "model": reports[0].get("model", "?") if reports else "?",
        "ts_csv_count": len(ts_files),
    }


def build_coverage_curves(ts_files: List[Path],
                           time_step: float = 5.0,
                           max_time: float = 2500.0) -> dict:
    """Interpolate all trial coverage curves onto common grid, return mean/p25/p75/p95."""
    if not ts_files:
        return {}
    # Common grid 0..max_time @ time_step
    import math
    grid = [i * time_step for i in range(int(max_time / time_step) + 1)]
    series = []
    for f in ts_files:
        rows = parse_csv(f)
        if not rows:
            continue
        # Forward-fill onto common grid
        vals = []
        j = 0
        last = 0.0
        for t in grid:
            while j < len(rows) and rows[j][0] <= t:
                last = rows[j][1]
                j += 1
            vals.append(last)
        series.append(vals)
    if not series:
        return {}
    # Pointwise median + p25 + p75
    out_grid = []
    medians = []
    p25 = []
    p75 = []
    for i, t in enumerate(grid):
        col = [s[i] for s in series if i < len(s)]
        if not col:
            continue
        col.sort()
        out_grid.append(t)
        medians.append(round(stats.median(col), 2))
        p25.append(round(col[max(0, len(col) // 4)], 2))
        p75.append(round(col[min(len(col) - 1, (3 * len(col)) // 4)], 2))
    return {
        "t_sec": out_grid,
        "median_m2": medians,
        "p25_m2": p25,
        "p75_m2": p75,
        "n_trials": len(series),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs-root", default="logs")
    ap.add_argument("--out-dir", default="logs/_report_notes")
    args = ap.parse_args()

    logs_root = Path(args.logs_root)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Group reports
    reports_by_cond: Dict[str, List[tuple[Path, dict]]] = {}
    for f in find_reports(logs_root):
        try:
            d = json.loads(f.read_text())
        except Exception:
            continue
        cond = infer_condition(f, d)
        reports_by_cond.setdefault(cond, []).append((f, d))

    # 2. Group timeseries CSVs
    ts_by_cond = find_timeseries(logs_root)

    # 3. Per-trial CSV
    trial_rows = []
    for cond, items in sorted(reports_by_cond.items()):
        for path, d in items:
            trial_rows.append({
                "condition": cond,
                "report_path": str(path),
                "provider": d.get("provider", "?"),
                "model": d.get("model", "?"),
                "strategy": d.get("strategy", ""),
                "scene": d.get("scene", "?"),
                "known_m2": d.get("known_m2", 0.0),
                "sim_time_sec": d.get("sim_time_sec", 0.0),
                "goals_published": d.get("goals_published", 0),
                "vlm_cycles": d.get("vlm_cycles", 0),
                "artifacts_found": d.get("artifacts_found", 0),
            })

    trial_csv = out_dir / "metrics_aggregate.csv"
    if trial_rows:
        with open(trial_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(trial_rows[0].keys()))
            w.writeheader()
            w.writerows(trial_rows)
    print(f"[aggregate] wrote {len(trial_rows)} trial rows → {trial_csv}")

    # 4. Per-condition summary
    summary = {}
    for cond, items in sorted(reports_by_cond.items()):
        reports = [d for _, d in items]
        summary[cond] = summarize_condition(cond, reports, ts_by_cond.get(cond, []))

    summary_json = out_dir / "metrics_summary.json"
    summary_json.write_text(json.dumps(summary, indent=2))
    print(f"[aggregate] wrote {len(summary)} conditions → {summary_json}")

    # 5. Coverage curves (for plot generator)
    curves = {}
    for cond, files in sorted(ts_by_cond.items()):
        curve = build_coverage_curves(files)
        if curve:
            curves[cond] = curve

    curves_json = out_dir / "coverage_curves.json"
    curves_json.write_text(json.dumps(curves, indent=2))
    print(f"[aggregate] wrote {len(curves)} curves → {curves_json}")

    # 6. Pretty print summary to stdout
    print("\n=== CONDITION SUMMARY ===")
    print(f"{'condition':<28s} {'n':>3s} {'cov mean±std':>16s} {'sim mean':>10s} {'goals mean':>11s}")
    for cond, s in sorted(summary.items()):
        cm = s["final_coverage_m2"]
        sm = s["sim_time_sec"]
        gm = s["goals_published"]
        if cm.get("n", 0) > 0:
            print(f"{cond:<28s} {cm['n']:>3d} "
                  f"{cm['mean']:>7.1f}±{cm['std']:<6.1f} "
                  f"{sm.get('mean', 0):>10.1f} "
                  f"{gm.get('mean', 0):>11.1f}")


if __name__ == "__main__":
    main()
