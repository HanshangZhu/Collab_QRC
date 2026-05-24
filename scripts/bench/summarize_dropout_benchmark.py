#!/usr/bin/env python3
"""Aggregate dropout-benchmark trial CSVs into a per-condition table + plots.

Layout expected:  OUT_DIR/<mode>/d<pct>/trial_<n>/metrics.csv
Each metrics.csv has at least columns: t_sim, global_coverage_ratio,
global_explored_area_m2 (produced by exploration_metrics_logger.py in
global_coverage_source=union mode).

Only directories named trial_* that contain a metrics.csv are counted;
diverged trials the driver discarded leave no metrics.csv (or are moved aside).
"""
from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path


def _rows(csv_path: Path) -> list[dict]:
    with open(csv_path) as f:
        return list(csv.DictReader(f))


def final_coverage(rows: list[dict]) -> float:
    if not rows:
        return 0.0
    return float(rows[-1]["global_coverage_ratio"])


def time_to_coverage(rows: list[dict], threshold: float):
    for r in rows:
        if float(r["global_coverage_ratio"]) >= threshold:
            return float(r["t_sim"])
    return None


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return statistics.mean(xs) if xs else None


def _std(xs):
    xs = [x for x in xs if x is not None]
    return statistics.pstdev(xs) if len(xs) > 1 else 0.0


def _dropout_value(name: str) -> float:
    """'d50' -> 0.5 ; '0.5' -> 0.5."""
    s = name[1:] if name.startswith("d") else name
    try:
        v = float(s)
    except ValueError:
        return 0.0
    return v / 100.0 if v > 1.0 else v


def summarize(out_dir: Path, thresholds=(0.5, 0.7, 0.9)):
    summary = []
    for mode_dir in sorted(p for p in out_dir.iterdir() if p.is_dir()):
        for drop_dir in sorted(p for p in mode_dir.iterdir() if p.is_dir()):
            trials = sorted(drop_dir.glob("trial_*/metrics.csv"))
            finals, ttc = [], {t: [] for t in thresholds}
            for csv_path in trials:
                rows = _rows(csv_path)
                finals.append(final_coverage(rows))
                for t in thresholds:
                    ttc[t].append(time_to_coverage(rows, t))
            entry = {
                "mode": mode_dir.name,
                "dropout": drop_dir.name,
                "dropout_val": _dropout_value(drop_dir.name),
                "n_trials": len(trials),
                "final_cov_mean": _mean(finals),
                "final_cov_std": _std(finals),
            }
            for t in thresholds:
                pct = int(t * 100)
                entry[f"ttc_{pct}_mean"] = _mean(ttc[t])
                reached = [x for x in ttc[t] if x is not None]
                entry[f"ttc_{pct}_reach_rate"] = (
                    len(reached) / len(trials)) if trials else 0.0
            summary.append(entry)
    return summary


def _plot(summary, out_dir: Path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib unavailable; skipping plots")
        return
    modes = sorted({e["mode"] for e in summary})
    fig, ax = plt.subplots()
    for mode in modes:
        pts = sorted((e["dropout_val"], e["final_cov_mean"] or 0.0)
                     for e in summary if e["mode"] == mode)
        if not pts:
            continue
        ax.plot([p[0] for p in pts], [p[1] for p in pts], marker="o", label=mode)
    ax.set_xlabel("comms dropout probability")
    ax.set_ylabel("final coverage ratio (mean)")
    ax.set_title("Exploration coverage vs comms dropout")
    ax.grid(True, alpha=0.3)
    ax.legend()
    out = out_dir / "final_coverage_vs_dropout.png"
    fig.savefig(out, dpi=120, bbox_inches="tight")
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", type=Path)
    args = ap.parse_args()
    summary = summarize(args.out_dir)
    if not summary:
        print("no trial metrics found under", args.out_dir)
        return
    cols = list(summary[0].keys())
    print("\t".join(cols))
    for e in summary:
        print("\t".join(str(e[c]) for c in cols))
    with open(args.out_dir / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(summary)
    print(f"wrote {args.out_dir/'summary.csv'}")
    _plot(summary, args.out_dir)


if __name__ == "__main__":
    main()
