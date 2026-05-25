#!/usr/bin/env python3
"""Coverage-over-time curves for the dropout benchmark.

Each trial's metrics.csv is a 1 Hz time-series of global_coverage_ratio. This
plots coverage vs elapsed exploration time, averaged across trials, one line per
(mode, dropout) condition: colour = dropout level, linestyle = mode
(centralised solid, decentralised dashed). Reveals exploration *speed* and how
comms dropout slows it per architecture.

Layout: OUT_DIR/<mode>/d<pct>/trial_*/metrics.csv
Usage:  plot_coverage_curves.py OUT_DIR [--grid-step 5] [--horizon 300]
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path


def trial_series(csv_path: Path):
    """Return (t_rel[], coverage[]) for one trial, time relative to logger start."""
    ts, cov = [], []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            try:
                ts.append(float(row["t_sim"]))
                cov.append(float(row["global_coverage_ratio"]))
            except (KeyError, ValueError):
                continue
    if not ts:
        return [], []
    t0 = ts[0]
    return [t - t0 for t in ts], cov


def resample(t_rel, cov, grid):
    """Step-interpolate coverage onto grid; hold last value past the trial end."""
    out = []
    for g in grid:
        if not t_rel or g < t_rel[0]:
            out.append(0.0)
            continue
        v = cov[0]
        for t, c in zip(t_rel, cov):
            if t <= g:
                v = c
            else:
                break
        out.append(v)
    return out


def condition_mean(trial_csvs, grid):
    series = []
    for p in trial_csvs:
        t_rel, cov = trial_series(p)
        if t_rel:
            series.append(resample(t_rel, cov, grid))
    if not series:
        return None
    n = len(series)
    return [sum(s[i] for s in series) / n for i in range(len(grid))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--grid-step", type=float, default=5.0)
    ap.add_argument("--horizon", type=float, default=300.0)
    args = ap.parse_args()

    grid = [i * args.grid_step for i in range(int(args.horizon / args.grid_step) + 1)]

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib unavailable")
        return

    drop_colors = {"d00": "#1b9e77", "d30": "#d95f02", "d50": "#7570b3", "d80": "#e7298a"}
    mode_style = {"centralised": "-", "decentralised": "--"}

    fig, ax = plt.subplots(figsize=(8, 5))
    plotted = 0
    for mode_dir in sorted(p for p in args.out_dir.iterdir() if p.is_dir()):
        mode = mode_dir.name
        if mode not in mode_style:
            continue
        for drop_dir in sorted(p for p in mode_dir.iterdir() if p.is_dir()):
            trials = sorted(drop_dir.glob("trial_*/metrics.csv"))
            mean = condition_mean(trials, grid)
            if mean is None:
                continue
            ax.plot(grid, mean, mode_style[mode],
                    color=drop_colors.get(drop_dir.name, "#666666"),
                    label=f"{mode[:5]} {drop_dir.name} (n={len(trials)})")
            plotted += 1

    if plotted == 0:
        print("no trial data found under", args.out_dir)
        return
    ax.set_xlabel("elapsed exploration time (sim s)")
    ax.set_ylabel("global coverage ratio")
    ax.set_title("Coverage over time — centralised (solid) vs decentralised (dashed)")
    ax.set_ylim(0, 1.0)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2)
    out = args.out_dir / "coverage_over_time.png"
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
