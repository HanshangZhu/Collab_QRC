#!/usr/bin/env python3
"""generate_report_plots.py — produce report-ready figures from aggregated metrics.

Consumes outputs of aggregate_metrics.py:
    logs/_report_notes/metrics_summary.json
    logs/_report_notes/coverage_curves.json
    logs/_report_notes/metrics_aggregate.csv
    logs/vlm_variations/vlm_*/sessions/.../cycle_*_decision.json  (latency)

Writes (PNG + PDF for IEEEtran embed):
    logs/_report_notes/figs/fig_coverage_curves.{png,pdf}
    logs/_report_notes/figs/fig_final_coverage_bar.{png,pdf}
    logs/_report_notes/figs/fig_latency_cdf.{png,pdf}
    logs/_report_notes/figs/fig_efficiency_scatter.{png,pdf}
    logs/_report_notes/figs/table_results.tex

Usage:
    python3 standalone/analysis/generate_report_plots.py
"""
from __future__ import annotations

import argparse
import json
import statistics as stats
from pathlib import Path
from typing import Dict, List, Optional

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


# Display order (worst → best on coverage). Bibtex-ready labels.
_CONDITION_ORDER = [
    ("mock_random",         "Random",             "Yamauchi'97",  "#a02020"),
    ("mock_greedy_nearest", "Greedy nearest",     "Yamauchi'97",  "#d07020"),
    ("vlm_groq_scout",      "VLM (Groq Scout)",   "ours",         "#705020"),
    ("cfpa2",               "CFPA2",              "Burgard'05",   "#7060a0"),
    ("mock_info_gain",      "Info-gain",          "Bircher'16",   "#20a0a0"),
    ("vlm_google",          "VLM (Gemini Flash)", "ours",         "#2070d0"),
]


def _styled_axes(ax) -> None:
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(True, alpha=0.25, linewidth=0.5, linestyle="--")
    ax.set_axisbelow(True)


def plot_coverage_curves(curves: dict, out_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(7.0, 4.0), dpi=120)
    plotted = 0
    for cond, label, _src, color in _CONDITION_ORDER:
        if cond not in curves:
            continue
        c = curves[cond]
        t = c["t_sec"]
        med = c["median_m2"]
        p25 = c["p25_m2"]
        p75 = c["p75_m2"]
        n = c["n_trials"]
        ax.fill_between(t, p25, p75, color=color, alpha=0.18, linewidth=0)
        ax.plot(t, med, color=color, linewidth=1.8,
                label=f"{label}  (n={n})")
        plotted += 1
    ax.set_xlabel("Simulation time (s)", fontsize=10)
    ax.set_ylabel("Observed area, $\\mathrm{m}^2$", fontsize=10)
    ax.set_title("Coverage vs. time on demo3 (median + IQR)",
                 fontsize=11)
    ax.legend(loc="lower right", fontsize=8, framealpha=0.95)
    # Cap x at reasonable extent (max sim time observed)
    xmax = max((max(c["t_sec"]) for c in curves.values()
                if c.get("t_sec")), default=1500)
    ax.set_xlim(0, min(xmax, 2400))
    _styled_axes(ax)
    plt.tight_layout()
    base = out_dir / "fig_coverage_curves"
    plt.savefig(f"{base}.png", dpi=180, bbox_inches="tight")
    plt.savefig(f"{base}.pdf", bbox_inches="tight")
    plt.close()
    print(f"[plot] coverage curves → {base}.{{png,pdf}} ({plotted} conditions)")


def plot_final_coverage_bar(summary: dict, out_dir: Path,
                              logs_root: Optional[Path] = None) -> None:
    """Bar chart. Filters out quota-failed trials (cov < 60 m²) from real-VLM
    means so the bar reflects policy performance, not API availability.
    Footnote in caption should note how many trials were quota-failures."""
    fig, ax = plt.subplots(figsize=(8.0, 3.8), dpi=120)
    labels, means, stds, colors, ns = [], [], [], [], []
    failure_notes = []
    # Reload per-trial rows to allow filtering
    from pathlib import Path as _P
    import csv as _csv
    trial_rows = []
    if logs_root is not None:
        agg = _P("logs/_report_notes/metrics_aggregate.csv")
        if agg.exists():
            with open(agg) as fh:
                trial_rows = list(_csv.DictReader(fh))
    for cond, label, src, color in _CONDITION_ORDER:
        s = summary.get(cond)
        if not s:
            continue
        cm = s["final_coverage_m2"]
        if cm.get("n", 0) == 0:
            continue
        # Filter quota-failed trials for real-VLM conditions
        if trial_rows and cond.startswith("vlm_") and not cond.startswith("vlm_groq_scout"):
            success = [float(r["known_m2"]) for r in trial_rows
                       if r["condition"] == cond and float(r["known_m2"]) > 60.0]
            failed = sum(1 for r in trial_rows
                         if r["condition"] == cond and float(r["known_m2"]) <= 60.0)
            if success:
                import statistics as _st
                mean = round(_st.mean(success), 1)
                std = round(_st.stdev(success) if len(success) > 1 else 0.0, 1)
                n = len(success)
                if failed > 0:
                    failure_notes.append(f"{label}: {failed} quota-failed trial(s) excluded")
                labels.append(f"{label}\n({src}, n={n}/{n+failed})")
                means.append(mean)
                stds.append(std)
                colors.append(color)
                ns.append(n)
                continue
        labels.append(f"{label}\n({src}, n={cm['n']})")
        means.append(cm["mean"])
        stds.append(cm.get("std", 0.0))
        colors.append(color)
        ns.append(cm["n"])
    x = np.arange(len(labels))
    ax.bar(x, means, yerr=stds, capsize=4, color=colors,
           edgecolor="black", linewidth=0.6, alpha=0.92,
           error_kw=dict(linewidth=1.0, ecolor="#202020"))
    for i, (m, s) in enumerate(zip(means, stds)):
        ax.text(i, m + s + 5, f"{m:.0f}", ha="center", fontsize=9,
                fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8, rotation=0)
    ax.set_ylabel("Final coverage, $\\mathrm{m}^2$", fontsize=10)
    ax.set_title("Final observed area on demo3 (mean $\\pm$ std)",
                 fontsize=11)
    _styled_axes(ax)
    ymax = max((m + s for m, s in zip(means, stds)), default=100)
    ax.set_ylim(0, ymax * 1.15)
    plt.tight_layout()
    base = out_dir / "fig_final_coverage_bar"
    plt.savefig(f"{base}.png", dpi=180, bbox_inches="tight")
    plt.savefig(f"{base}.pdf", bbox_inches="tight")
    plt.close()
    print(f"[plot] final coverage bar → {base}.{{png,pdf}} ({len(labels)} bars)")


def collect_latencies(logs_root: Path) -> Dict[str, List[float]]:
    """Return {condition: [latency_s, ...]} from real-VLM cycle decisions."""
    out: Dict[str, List[float]] = {}
    # Walk new variations layout AND legacy Gemini layout (pre-reorg smoke runs)
    pattern_sources = [
        ("vlm_variations/vlm_*/sessions/session_*", lambda d: d.parent.parent.name),
        # Legacy: vlm_gemini_real/sessions/* — treat as vlm_google for grouping
        ("vlm_gemini_real/sessions/*", lambda d: "vlm_google"),
    ]
    for pat, name_fn in pattern_sources:
        for sess_dir in logs_root.glob(pat):
            if not sess_dir.is_dir():
                continue
            cond = name_fn(sess_dir)
            for f in sess_dir.glob("cycle_*_decision.json"):
                try:
                    d = json.loads(f.read_text())
                except Exception:
                    continue
                if d.get("goal") and not str(d.get("reason", "")).startswith("agent_error"):
                    dur = d.get("duration_sec")
                    if dur is not None and dur > 0:
                        out.setdefault(cond, []).append(float(dur))
    return out


def plot_latency_cdf(logs_root: Path, out_dir: Path) -> None:
    lats = collect_latencies(logs_root)
    if not lats:
        print("[plot] no latency data, skipping CDF")
        return
    fig, ax = plt.subplots(figsize=(6.5, 3.6), dpi=120)
    for cond, label, _src, color in _CONDITION_ORDER:
        ds = sorted(lats.get(cond, []))
        if not ds:
            continue
        y = [(i + 1) / len(ds) for i in range(len(ds))]
        ax.step(ds, y, where="post", linewidth=1.6, color=color,
                label=f"{label}  (n={len(ds)})")
        med = stats.median(ds)
        ax.axvline(med, color=color, linestyle=":", linewidth=0.7, alpha=0.5)
    ax.set_xlabel("VLM decision latency (s)", fontsize=10)
    ax.set_ylabel("CDF (fraction of cycles)", fontsize=10)
    ax.set_title("Real-VLM cycle latency CDF (successful cycles only)",
                 fontsize=10)
    ax.set_xscale("log")
    ax.legend(loc="lower right", fontsize=8, framealpha=0.95)
    _styled_axes(ax)
    plt.tight_layout()
    base = out_dir / "fig_latency_cdf"
    plt.savefig(f"{base}.png", dpi=180, bbox_inches="tight")
    plt.savefig(f"{base}.pdf", bbox_inches="tight")
    plt.close()
    print(f"[plot] latency CDF → {base}.{{png,pdf}}")


def plot_efficiency_scatter(summary: dict, out_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(6.8, 4.0), dpi=120)
    for cond, label, _src, color in _CONDITION_ORDER:
        s = summary.get(cond)
        if not s:
            continue
        cm = s["final_coverage_m2"]
        sm = s["sim_time_sec"]
        if cm.get("n", 0) == 0:
            continue
        ax.scatter(sm["mean"], cm["mean"],
                   s=80 + 18 * cm["n"], color=color, alpha=0.85,
                   edgecolor="black", linewidth=0.6,
                   label=f"{label}  (n={cm['n']})")
        ax.errorbar(sm["mean"], cm["mean"],
                    xerr=sm.get("std", 0), yerr=cm.get("std", 0),
                    color=color, alpha=0.5, linewidth=0.8, capsize=3)
    ax.set_xlabel("Mean sim time per trial (s)", fontsize=10)
    ax.set_ylabel("Mean final coverage, $\\mathrm{m}^2$", fontsize=10)
    ax.set_title("Coverage vs. sim time on demo3 (point area $\\propto$ n)",
                 fontsize=10)
    ax.legend(loc="lower right", fontsize=8, framealpha=0.95)
    _styled_axes(ax)
    plt.tight_layout()
    base = out_dir / "fig_efficiency_scatter"
    plt.savefig(f"{base}.png", dpi=180, bbox_inches="tight")
    plt.savefig(f"{base}.pdf", bbox_inches="tight")
    plt.close()
    print(f"[plot] efficiency scatter → {base}.{{png,pdf}}")


def write_latex_table(summary: dict, out_dir: Path,
                        logs_root: Optional[Path] = None) -> None:
    """Emit a tabular row per condition. Real-VLM rows use success-only stats
    (cov > 60 m²) so the table is consistent with the bar chart.
    Quota-failure trial counts are reported separately."""
    import csv as _csv, statistics as _st
    from pathlib import Path as _P
    trial_rows: List[dict] = []
    if logs_root is not None:
        agg = _P("logs/_report_notes/metrics_aggregate.csv")
        if agg.exists():
            with open(agg) as fh:
                trial_rows = list(_csv.DictReader(fh))

    def _filtered(cond: str):
        return [r for r in trial_rows
                if r["condition"] == cond and float(r["known_m2"]) > 60.0]

    def _failed(cond: str):
        return sum(1 for r in trial_rows
                   if r["condition"] == cond and float(r["known_m2"]) <= 60.0)

    lines: List[str] = []
    lines.append("% Auto-generated by generate_report_plots.py — do not edit by hand.")
    lines.append("\\begin{tabular}{@{}lcccc@{}}")
    lines.append("\\toprule")
    lines.append("\\textbf{Method} & \\textbf{n} & \\textbf{Coverage (m$^2$)} "
                 "& \\textbf{Sim time (s)} & \\textbf{Goals} \\\\")
    lines.append("\\midrule")
    for cond, label, src, _ in _CONDITION_ORDER:
        s = summary.get(cond)
        if not s:
            continue
        cm = s["final_coverage_m2"]
        sm = s["sim_time_sec"]
        gm = s["goals_published"]
        if cm.get("n", 0) == 0:
            continue
        # Real-VLM rows: filter quota-fail trials
        if trial_rows and cond.startswith("vlm_") and not cond.startswith("vlm_groq_scout"):
            succ = _filtered(cond)
            failed = _failed(cond)
            if succ:
                cov = [float(r["known_m2"]) for r in succ]
                stt = [float(r["sim_time_sec"]) for r in succ]
                gls = [float(r["goals_published"]) for r in succ]
                cov_m = _st.mean(cov); cov_s = _st.stdev(cov) if len(cov) > 1 else 0
                stt_m = _st.mean(stt); stt_s = _st.stdev(stt) if len(stt) > 1 else 0
                glm = _st.mean(gls)
                n_label = f"{len(succ)}{f' / {len(succ)+failed}' if failed else ''}"
                lines.append(
                    f"{label} ({src}) & {n_label} & "
                    f"{cov_m:.1f} $\\pm$ {cov_s:.1f} & "
                    f"{stt_m:.0f} $\\pm$ {stt_s:.0f} & "
                    f"{glm:.1f} \\\\"
                )
                continue
        lines.append(
            f"{label} ({src}) & {cm['n']} & "
            f"{cm['mean']:.1f} $\\pm$ {cm.get('std', 0):.1f} & "
            f"{sm['mean']:.0f} $\\pm$ {sm.get('std', 0):.0f} & "
            f"{gm['mean']:.1f} \\\\"
        )
    lines.append("\\bottomrule")
    lines.append("\\end{tabular}")
    out_path = out_dir / "table_results.tex"
    out_path.write_text("\n".join(lines) + "\n")
    print(f"[plot] latex table → {out_path}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs-root", default="logs")
    ap.add_argument("--in-dir", default="logs/_report_notes")
    ap.add_argument("--out-dir", default="logs/_report_notes/figs")
    args = ap.parse_args()

    in_dir = Path(args.in_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    summary_path = in_dir / "metrics_summary.json"
    curves_path = in_dir / "coverage_curves.json"
    if not summary_path.exists() or not curves_path.exists():
        print("[plot] run aggregate_metrics.py first")
        return

    summary = json.loads(summary_path.read_text())
    curves = json.loads(curves_path.read_text())

    plot_coverage_curves(curves, out_dir)
    plot_final_coverage_bar(summary, out_dir, logs_root=Path(args.logs_root))
    plot_latency_cdf(Path(args.logs_root), out_dir)
    plot_efficiency_scatter(summary, out_dir)
    write_latex_table(summary, out_dir, logs_root=Path(args.logs_root))

    print(f"\nDone. Figures + table → {out_dir}/")


if __name__ == "__main__":
    main()
