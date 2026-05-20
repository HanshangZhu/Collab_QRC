#!/usr/bin/env python3
"""Generate publication-quality comparison plots from benchmark results.

Reads timeseries CSVs and end-of-run JSON reports produced by
benchmark_standalone.sh (or individual main_*.py runs) and produces:

  Fig 1 — coverage_vs_time.pdf   Coverage (m²) vs simulation time, 4 methods
                                  with mean line + ±1σ shaded band per method.
  Fig 2 — final_coverage.pdf     Bar chart: final known_m2 per method (mean ± σ).
  Fig 3 — goals_efficiency.pdf   Goals published vs. final coverage scatter.
  Fig 4 — artifacts.pdf          Artifact detection count per method (VLM only).
  table.tex                      LaTeX table: mean ± σ for all metrics.
  summary_plots.png              Single-page 2×2 figure combining figs 1–4.

Usage:
    mjpython standalone/analysis/generate_plots.py <results_dir> [--out <out_dir>]
    mjpython standalone/analysis/generate_plots.py /tmp/standalone_bench_20260519

    <results_dir>   Directory containing coverage_*.csv and *_report_*.json files.
                    Accepts multiple directories: --dirs dir1 dir2 ...
    --out           Where to write the output figures (default: <results_dir>/plots)
    --scene-area    Total navigable area in m² for normalisation (default: auto)
    --no-pdf        Write PNG instead of PDF (better for quick preview)
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.ticker import MaxNLocator

# ── Style ─────────────────────────────────────────────────────────────────────

# Colour palette: colourblind-safe (Wong 2011).
PALETTE: Dict[str, str] = {
    "random":         "#E69F00",   # orange
    "greedy_nearest": "#56B4E9",   # sky blue
    "cfpa2":          "#009E73",   # green
    "vlm_groq":       "#CC79A7",   # purple
    "vlm_mock":       "#0072B2",   # blue
    "vlm":            "#0072B2",   # blue (alias)
}
LABELS: Dict[str, str] = {
    "random":         "Random Frontier",
    "greedy_nearest": "Greedy Nearest (Yamauchi'97)",
    "cfpa2":          "CFPA2 Info-Gain (Yamauchi'97)",
    "vlm_mock":       "VLM Mock (info-gain)",
    "vlm_groq":       "VLM (Groq Llama-4-Scout)",
    "vlm":            "VLM-Guided (ours)",
}
LINE_STYLES: Dict[str, str] = {
    "random":         ":",
    "greedy_nearest": "--",
    "cfpa2":          "-.",
    "vlm_mock":       "-",
    "vlm_groq":       "-",
    "vlm":            "-",
}

_PREFERRED_ORDER = ["random", "greedy_nearest", "cfpa2", "vlm_mock", "vlm_groq", "vlm"]


def _label(method: str) -> str:
    return LABELS.get(method, method.replace("_", " ").title())


def _colour(method: str) -> str:
    for key in PALETTE:
        if method.startswith(key):
            return PALETTE[key]
    # Fallback: cycle through tableau colours
    _fallback = list(plt.rcParams["axes.prop_cycle"].by_key()["color"])
    idx = abs(hash(method)) % len(_fallback)
    return _fallback[idx]


def _ls(method: str) -> str:
    for key in LINE_STYLES:
        if method.startswith(key):
            return LINE_STYLES[key]
    return "-"


def _sort_methods(methods: List[str]) -> List[str]:
    order = {m: i for i, m in enumerate(_PREFERRED_ORDER)}
    return sorted(methods, key=lambda m: order.get(m, 99))


# ── Data loading ──────────────────────────────────────────────────────────────

def load_timeseries(results_dir: Path) -> Dict[str, List[np.ndarray]]:
    """Return {method: [array(T,2), ...]} where columns are [sim_time, known_m2]."""
    data: Dict[str, List[np.ndarray]] = defaultdict(list)
    for f in sorted(results_dir.glob("coverage_*.csv")):
        # Filename: coverage_<method>_<timestamp>.csv
        stem = f.stem  # coverage_cfpa2_20260519_123456
        parts = stem.split("_", 1)  # ["coverage", "cfpa2_20260519_123456"]
        if len(parts) < 2:
            continue
        method_ts = parts[1]
        # Method name ends before the first 8-digit date token
        tokens = method_ts.split("_")
        date_idx = next(
            (i for i, t in enumerate(tokens) if len(t) == 8 and t.isdigit()),
            len(tokens),
        )
        method = "_".join(tokens[:date_idx])
        if not method:
            method = "unknown"
        try:
            rows = []
            with open(f, newline="") as fh:
                reader = csv.DictReader(fh)
                for row in reader:
                    rows.append([float(row["sim_time_sec"]),
                                  float(row["known_m2"])])
            if rows:
                data[method].append(np.array(rows))
        except Exception as exc:
            print(f"[plots] Warning: could not read {f}: {exc}", file=sys.stderr)
    return dict(data)


def load_reports(results_dir: Path) -> Dict[str, List[dict]]:
    """Return {method: [report_dict, ...]}."""
    reports: Dict[str, List[dict]] = defaultdict(list)
    for f in sorted(results_dir.glob("*_report_*.json")):
        try:
            d = json.loads(f.read_text())
        except Exception:
            continue
        method = d.get("run", f.stem.split("_")[0])
        if not method:
            method = f.stem.split("_report")[0]
        reports[method].append(d)
    # Also parse from benchmark output naming: <method>_trial*_*report*.json
    for f in sorted(results_dir.glob("*_trial*_*report*.json")):
        try:
            d = json.loads(f.read_text())
        except Exception:
            continue
        method = f.stem.split("_trial")[0]
        d.setdefault("run", method)
        reports[method].append(d)
    # Deduplicate
    seen: Dict[str, set] = defaultdict(set)
    deduped: Dict[str, List[dict]] = defaultdict(list)
    for method, reps in reports.items():
        for r in reps:
            key = (r.get("known_m2"), r.get("sim_time_sec"))
            if key not in seen[method]:
                seen[method].add(key)
                deduped[method].append(r)
    return dict(deduped)


# ── Interpolation helper ──────────────────────────────────────────────────────

def interpolate_to_common_grid(
    trials: List[np.ndarray],
    t_max: Optional[float] = None,
    n_points: int = 300,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Interpolate multiple timeseries to a common time grid.

    Returns (t_grid, mean_curve, std_curve).
    """
    if not trials:
        return np.array([]), np.array([]), np.array([])
    t_end = t_max or max(tr[-1, 0] for tr in trials)
    t_grid = np.linspace(0.0, t_end, n_points)
    interp = []
    for tr in trials:
        ts, ks = tr[:, 0], tr[:, 1]
        # Forward-fill: after last point, keep final value
        yi = np.interp(t_grid, ts, ks, left=0.0, right=ks[-1])
        interp.append(yi)
    mat = np.stack(interp, axis=0)
    return t_grid, mat.mean(axis=0), mat.std(axis=0)


# ── Figure 1: Coverage vs. time ───────────────────────────────────────────────

def plot_coverage_vs_time(
    ts_data: Dict[str, List[np.ndarray]],
    out_path: Path,
) -> None:
    if not ts_data:
        print("[plots] No timeseries data found — skipping Fig 1.", file=sys.stderr)
        return

    t_max = max(tr[-1, 0] for trials in ts_data.values() for tr in trials)
    fig, ax = plt.subplots(figsize=(7, 4.5))

    for method in _sort_methods(list(ts_data.keys())):
        trials = ts_data[method]
        t_grid, mean, std = interpolate_to_common_grid(trials, t_max=t_max)
        if len(t_grid) == 0:
            continue
        colour = _colour(method)
        ls = _ls(method)
        ax.plot(t_grid, mean, label=_label(method),
                color=colour, linestyle=ls, linewidth=2.0)
        if len(trials) > 1:
            ax.fill_between(t_grid, mean - std, mean + std,
                            alpha=0.18, color=colour)

    ax.set_xlabel("Simulation time (s)", fontsize=12)
    ax.set_ylabel("Explored area (m²)", fontsize=12)
    ax.set_title("Coverage vs. Simulation Time", fontsize=13, fontweight="bold")
    ax.legend(fontsize=9, loc="lower right", framealpha=0.9)
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.grid(True, linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[plots] Fig 1 saved → {out_path}")


# ── Figure 2: Final coverage bar chart ───────────────────────────────────────

def plot_final_coverage(
    reports: Dict[str, List[dict]],
    out_path: Path,
) -> None:
    if not reports:
        print("[plots] No reports found — skipping Fig 2.", file=sys.stderr)
        return

    methods = _sort_methods(list(reports.keys()))
    means, stds, labels, colours = [], [], [], []
    for method in methods:
        vals = [r.get("known_m2", 0.0) for r in reports[method]]
        means.append(float(np.mean(vals)))
        stds.append(float(np.std(vals)) if len(vals) > 1 else 0.0)
        labels.append(_label(method))
        colours.append(_colour(method))

    fig, ax = plt.subplots(figsize=(7, 4))
    x = np.arange(len(methods))
    bars = ax.bar(x, means, yerr=stds, capsize=5, color=colours,
                  edgecolor="black", linewidth=0.7, width=0.55,
                  error_kw={"elinewidth": 1.5, "ecolor": "#333333"})

    for bar, mean, std in zip(bars, means, stds):
        label = f"{mean:.1f}"
        if std > 0:
            label += f"\n±{std:.1f}"
        ax.text(bar.get_x() + bar.get_width() / 2.0,
                bar.get_height() + std + 2,
                label, ha="center", va="bottom", fontsize=8.5, fontweight="bold")

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=15, ha="right", fontsize=9)
    ax.set_ylabel("Final explored area (m²)", fontsize=12)
    ax.set_title("Final Coverage by Method", fontsize=13, fontweight="bold")
    ax.set_ylim(bottom=0)
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.grid(True, axis="y", linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[plots] Fig 2 saved → {out_path}")


# ── Figure 3: Goals published vs. final coverage scatter ─────────────────────

def plot_goals_efficiency(
    reports: Dict[str, List[dict]],
    out_path: Path,
) -> None:
    if not reports:
        return

    fig, ax = plt.subplots(figsize=(5.5, 4))
    for method in _sort_methods(list(reports.keys())):
        goals = [r.get("goals_published", 0) for r in reports[method]]
        known = [r.get("known_m2", 0.0) for r in reports[method]]
        colour = _colour(method)
        ax.scatter(goals, known, label=_label(method),
                   color=colour, s=60, edgecolors="black", linewidths=0.6,
                   zorder=5)

    ax.set_xlabel("Goals published", fontsize=12)
    ax.set_ylabel("Final explored area (m²)", fontsize=12)
    ax.set_title("Navigation Efficiency", fontsize=13, fontweight="bold")
    ax.legend(fontsize=8.5, framealpha=0.9)
    ax.grid(True, linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[plots] Fig 3 saved → {out_path}")


# ── Figure 4: Artifacts detected (VLM only) ──────────────────────────────────

def plot_artifacts(
    reports: Dict[str, List[dict]],
    out_path: Path,
) -> None:
    vlm_methods = [m for m in reports if "vlm" in m]
    if not vlm_methods:
        print("[plots] No VLM reports — skipping artifact plot.", file=sys.stderr)
        return

    fig, ax = plt.subplots(figsize=(5, 3.5))
    methods = _sort_methods(vlm_methods)
    for i, method in enumerate(methods):
        counts = [r.get("artifacts_found", 0) for r in reports[method]]
        # Jittered scatter + mean bar
        colour = _colour(method)
        jitter = np.random.uniform(-0.15, 0.15, size=len(counts))
        ax.scatter([i + j for j in jitter], counts,
                   color=colour, alpha=0.75, s=45,
                   edgecolors="black", linewidths=0.5, zorder=5)
        ax.barh(i, float(np.mean(counts)), height=0.25,
                color=colour, alpha=0.4, left=0)

    ax.set_yticks(range(len(methods)))
    ax.set_yticklabels([_label(m) for m in methods], fontsize=9)
    ax.set_xlabel("Artifacts detected", fontsize=12)
    ax.set_title("Artifact Detection (VLM methods)", fontsize=13, fontweight="bold")
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_xlim(left=0)
    ax.grid(True, axis="x", linestyle=":", alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[plots] Fig 4 saved → {out_path}")


# ── Combined 2×2 figure ───────────────────────────────────────────────────────

def plot_summary(
    ts_data: Dict[str, List[np.ndarray]],
    reports: Dict[str, List[dict]],
    out_path: Path,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5))
    fig.suptitle("Exploration Method Comparison — MuJoCo Go2W",
                 fontsize=14, fontweight="bold", y=1.01)

    # Panel A: coverage vs time
    ax = axes[0, 0]
    if ts_data:
        t_max = max(tr[-1, 0] for trials in ts_data.values() for tr in trials)
        for method in _sort_methods(list(ts_data.keys())):
            trials = ts_data[method]
            t_grid, mean, std = interpolate_to_common_grid(trials, t_max=t_max)
            if len(t_grid) == 0:
                continue
            colour = _colour(method)
            ax.plot(t_grid, mean, label=_label(method),
                    color=colour, linestyle=_ls(method), linewidth=1.8)
            if len(trials) > 1:
                ax.fill_between(t_grid, mean - std, mean + std,
                                alpha=0.15, color=colour)
        ax.set_xlabel("Sim time (s)", fontsize=10)
        ax.set_ylabel("Explored area (m²)", fontsize=10)
        ax.set_title("(a) Coverage vs. Time", fontsize=11)
        ax.legend(fontsize=7.5, loc="lower right")
        ax.set_xlim(left=0); ax.set_ylim(bottom=0)
        ax.grid(True, linestyle=":", alpha=0.4)

    # Panel B: final coverage bar
    ax = axes[0, 1]
    if reports:
        methods = _sort_methods(list(reports.keys()))
        x = np.arange(len(methods))
        means = [np.mean([r.get("known_m2", 0) for r in reports[m]]) for m in methods]
        stds  = [np.std( [r.get("known_m2", 0) for r in reports[m]])
                 if len(reports[m]) > 1 else 0.0 for m in methods]
        colours = [_colour(m) for m in methods]
        ax.bar(x, means, yerr=stds, capsize=4, color=colours,
               edgecolor="black", linewidth=0.6, width=0.55,
               error_kw={"elinewidth": 1.2, "ecolor": "#333333"})
        ax.set_xticks(x)
        ax.set_xticklabels([_label(m) for m in methods],
                            rotation=18, ha="right", fontsize=7.5)
        ax.set_ylabel("Final area (m²)", fontsize=10)
        ax.set_title("(b) Final Coverage", fontsize=11)
        ax.set_ylim(bottom=0)
        ax.grid(True, axis="y", linestyle=":", alpha=0.4)

    # Panel C: efficiency scatter
    ax = axes[1, 0]
    if reports:
        for method in _sort_methods(list(reports.keys())):
            goals = [r.get("goals_published", 0) for r in reports[method]]
            known = [r.get("known_m2", 0.0) for r in reports[method]]
            ax.scatter(goals, known, label=_label(method),
                       color=_colour(method), s=50,
                       edgecolors="black", linewidths=0.5, zorder=5)
        ax.set_xlabel("Goals published", fontsize=10)
        ax.set_ylabel("Final area (m²)", fontsize=10)
        ax.set_title("(c) Navigation Efficiency", fontsize=11)
        ax.legend(fontsize=7.5)
        ax.grid(True, linestyle=":", alpha=0.4)

    # Panel D: artifacts (VLM)
    ax = axes[1, 1]
    vlm_methods = [m for m in reports if "vlm" in m]
    if vlm_methods:
        sorted_vlm = _sort_methods(vlm_methods)
        for i, method in enumerate(sorted_vlm):
            counts = [r.get("artifacts_found", 0) for r in reports[method]]
            colour = _colour(method)
            jitter = np.random.uniform(-0.15, 0.15, size=len(counts))
            ax.scatter([i + j for j in jitter], counts,
                       color=colour, s=40, edgecolors="black",
                       linewidths=0.4, zorder=5)
            ax.plot([i - 0.25, i + 0.25],
                    [np.mean(counts)] * 2, color=colour,
                    linewidth=2.5, solid_capstyle="round", zorder=6)
        ax.set_xticks(range(len(sorted_vlm)))
        ax.set_xticklabels([_label(m) for m in sorted_vlm],
                            rotation=15, ha="right", fontsize=8)
        ax.set_ylabel("Artifacts detected", fontsize=10)
        ax.set_title("(d) Artifact Detection", fontsize=11)
        ax.yaxis.set_major_locator(MaxNLocator(integer=True))
        ax.set_ylim(bottom=-0.2)
        ax.grid(True, axis="y", linestyle=":", alpha=0.4)
    else:
        ax.set_visible(False)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[plots] Summary figure saved → {out_path}")


# ── LaTeX table ───────────────────────────────────────────────────────────────

def write_latex_table(
    reports: Dict[str, List[dict]],
    ts_data: Dict[str, List[np.ndarray]],
    out_path: Path,
) -> None:
    methods = _sort_methods(list(reports.keys()))
    lines = [
        r"\begin{table}[h]",
        r"\centering",
        r"\caption{Exploration performance comparison — MuJoCo Go2W, demo3 scene "
        r"(28$\times$20\,m). Values are mean $\pm$ std across "
        + str(max(len(reports[m]) for m in methods) if methods else 0)
        + r" trials.}",
        r"\label{tab:exploration_comparison}",
        r"\resizebox{\columnwidth}{!}{%",
        r"\begin{tabular}{lcccc}",
        r"\toprule",
        r"\textbf{Method} & \textbf{Coverage (m²)} & "
        r"\textbf{Sim Time (s)} & "
        r"\textbf{Goals} & \textbf{Artifacts} \\",
        r"\midrule",
    ]
    for method in methods:
        reps = reports[method]
        known = [r.get("known_m2", 0.0) for r in reps]
        sim_t = [r.get("sim_time_sec", 0.0) for r in reps]
        goals = [r.get("goals_published", 0) for r in reps]
        arts  = [r.get("artifacts_found", 0) for r in reps]
        n = len(reps)

        def fmt(vals: list) -> str:
            m = float(np.mean(vals))
            s = float(np.std(vals)) if n > 1 else 0.0
            return f"{m:.1f} $\\pm$ {s:.1f}"

        lbl = _label(method).replace("&", r"\&")
        row = (f"{lbl} & {fmt(known)} & {fmt(sim_t)} & "
               f"{fmt(goals)} & {fmt(arts)} \\\\")
        lines.append(row)

    lines += [
        r"\bottomrule",
        r"\end{tabular}%",
        r"}",
        r"\end{table}",
    ]
    out_path.write_text("\n".join(lines) + "\n")
    print(f"[plots] LaTeX table saved → {out_path}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate comparison plots from standalone benchmark results"
    )
    parser.add_argument("results_dir", nargs="?", default=None,
                        help="Directory containing coverage_*.csv and *_report_*.json")
    parser.add_argument("--dirs", nargs="+", default=None,
                        help="Multiple result directories to merge")
    parser.add_argument("--out", default=None,
                        help="Output directory for figures (default: <results_dir>/plots)")
    parser.add_argument("--no-pdf", action="store_true",
                        help="Save PNG instead of PDF (faster preview)")
    args = parser.parse_args()

    # Collect result directories
    result_dirs: List[Path] = []
    if args.dirs:
        result_dirs = [Path(d) for d in args.dirs]
    elif args.results_dir:
        result_dirs = [Path(args.results_dir)]
    else:
        # Try the default benchmark output location
        candidates = sorted(Path("/tmp").glob("standalone_bench_*"), reverse=True)
        if candidates:
            result_dirs = [candidates[0]]
            print(f"[plots] Auto-detected results dir: {result_dirs[0]}")
        else:
            print("[plots] No results directory found. Pass as argument or run "
                  "benchmark_standalone.sh first.", file=sys.stderr)
            sys.exit(1)

    for d in result_dirs:
        if not d.exists():
            print(f"[plots] Directory not found: {d}", file=sys.stderr)
            sys.exit(1)

    # Load data from all directories
    ts_data: Dict[str, List[np.ndarray]] = defaultdict(list)
    reports: Dict[str, List[dict]] = defaultdict(list)
    for d in result_dirs:
        for method, trials in load_timeseries(d).items():
            ts_data[method].extend(trials)
        for method, reps in load_reports(d).items():
            reports[method].extend(reps)

    if not ts_data and not reports:
        print("[plots] No data found in the specified directories.", file=sys.stderr)
        print("[plots] Expected files: coverage_*.csv, *_report_*.json")
        sys.exit(1)

    print(f"[plots] Loaded methods: {sorted(ts_data.keys() | reports.keys())}")
    for m in sorted(ts_data.keys()):
        print(f"  {m}: {len(ts_data[m])} timeseries trial(s)")
    for m in sorted(reports.keys()):
        print(f"  {m}: {len(reports[m])} report(s)")

    # Output directory
    out_dir = Path(args.out) if args.out else (result_dirs[0] / "plots")
    out_dir.mkdir(parents=True, exist_ok=True)
    ext = "png" if args.no_pdf else "pdf"

    # Generate figures
    plot_coverage_vs_time(dict(ts_data), out_dir / f"coverage_vs_time.{ext}")
    plot_final_coverage(dict(reports), out_dir / f"final_coverage.{ext}")
    plot_goals_efficiency(dict(reports), out_dir / f"goals_efficiency.{ext}")
    plot_artifacts(dict(reports), out_dir / f"artifacts.{ext}")
    plot_summary(dict(ts_data), dict(reports), out_dir / "summary_plots.png")
    write_latex_table(dict(reports), dict(ts_data), out_dir / "table.tex")

    print(f"\n[plots] All outputs written to {out_dir}/")


if __name__ == "__main__":
    main()
