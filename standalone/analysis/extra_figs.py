#!/usr/bin/env python3
"""extra_figs.py — supplementary report figures.

Produces:
  fig_storyboard.{png,pdf}          — Gemini cycles 1/5/10/20 progression
  fig_provider_compare.{png,pdf}    — same cycle id from Gemini vs Scout
  fig_anchoring_evidence.{png,pdf}  — Scout WITH vs WITHOUT info_gain reasoning
  fig_hallucination.{png,pdf}       — Scout camera frames + reported artefact pos
  fig_goal_scatter.{png,pdf}        — where each method picks goals on the map
  fig_latency_timeline.{png,pdf}    — per-cycle latency series, quota cascade visual
  fig_decision_diversity.{png,pdf}  — unique goals / total goals per method

All consume the existing per-cycle artefacts in logs/.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = Path("logs/_report_notes/figs")
OUT.mkdir(parents=True, exist_ok=True)

# Gemini smoke 2 = best long Gemini session
GEMINI_SESS = Path("logs/vlm_gemini_real/sessions/session_20260519_165732")
SCOUT_SESS = Path("logs/vlm_variations/vlm_groq_scout/sessions/session_20260520_055415")
NOIG_SESS = sorted(Path("logs/vlm_variations/vlm_groq_noig/sessions").iterdir())[0]


def _styled(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(True, alpha=0.25, linewidth=0.5, linestyle="--")
    ax.set_axisbelow(True)


def storyboard_gemini():
    """4 cycles from Gemini showing map building up."""
    cycles = ["0001", "0005", "0010", "0020"]
    avail = [c for c in cycles if (GEMINI_SESS / f"cycle_{c}_map.png").exists()]
    if not avail:
        print("[storyboard] no cycles found"); return
    fig, axes = plt.subplots(1, len(avail), figsize=(3.0 * len(avail), 3.0), dpi=120)
    if len(avail) == 1:
        axes = [axes]
    for ax, c in zip(axes, avail):
        img = plt.imread(str(GEMINI_SESS / f"cycle_{c}_map.png"))
        ax.imshow(img)
        d = json.loads((GEMINI_SESS / f"cycle_{c}_decision.json").read_text())
        s = json.loads((GEMINI_SESS / f"cycle_{c}_scene.json").read_text())
        known = s["coverage"]["known_m2"]
        ax.set_title(f"cycle {int(c)} — known {known:.0f} m²", fontsize=9)
        ax.set_xticks([]); ax.set_yticks([])
        for sp in ax.spines.values(): sp.set_visible(False)
    fig.suptitle("Gemini VLM exploration progress on demo3 (map sent to VLM at each cycle)",
                 fontsize=10, y=1.02)
    plt.tight_layout()
    for ext in ["png", "pdf"]:
        plt.savefig(OUT / f"fig_storyboard.{ext}", dpi=180, bbox_inches="tight")
    plt.close()
    print("[storyboard] saved")


def provider_compare():
    """Same approximate map state, Gemini vs Scout side-by-side."""
    # Use cycle 1 from each (both start fresh, same spawn)
    cyc = "0001"
    fig, axes = plt.subplots(2, 2, figsize=(9.0, 7.0), dpi=120,
                              gridspec_kw={"height_ratios": [3.0, 1.6], "hspace": 0.35})
    for col, (sess, label, color) in enumerate([
        (GEMINI_SESS, "Gemini 2.5 Flash-Lite", "#2070d0"),
        (SCOUT_SESS,  "Groq Llama-4-Scout 17B", "#705020"),
    ]):
        ax_map = axes[0, col]
        ax_txt = axes[1, col]
        img = plt.imread(str(sess / f"cycle_{cyc}_map.png"))
        ax_map.imshow(img)
        ax_map.set_title(f"{label} — cycle 1 occupancy input", fontsize=10, color=color)
        ax_map.set_xticks([]); ax_map.set_yticks([])
        for sp in ax_map.spines.values(): sp.set_visible(False)
        d = json.loads((sess / f"cycle_{cyc}_decision.json").read_text())
        txt = []
        txt.append(f"goal: ({d['goal'][0]:.2f}, {d['goal'][1]:.2f})")
        txt.append(f"latency: {d.get('duration_sec', 0):.2f}s")
        txt.append(f"artefact_seen: {d.get('artifact_seen')}")
        if d.get("artifact_pos"):
            txt.append(f"artefact_pos: ({d['artifact_pos'][0]:.1f}, {d['artifact_pos'][1]:.1f})")
        txt.append("")
        txt.append("reason:")
        reason = (d.get("reason") or "")[:240]
        # wrap to ~50 chars
        import textwrap as tw
        txt.extend(["  " + l for l in tw.wrap(reason, width=58)])
        ax_txt.axis("off")
        ax_txt.text(0.0, 1.0, "\n".join(txt), family="monospace", fontsize=8,
                    verticalalignment="top", transform=ax_txt.transAxes,
                    bbox=dict(boxstyle="round,pad=0.5", facecolor="#fafafa",
                              edgecolor=color, linewidth=1.0))
    fig.suptitle("Same scene, different VLM — cycle 1 decision comparison",
                 fontsize=11, y=0.99)
    for ext in ["png", "pdf"]:
        plt.savefig(OUT / f"fig_provider_compare.{ext}", dpi=180, bbox_inches="tight")
    plt.close()
    print("[provider_compare] saved")


def anchoring_evidence():
    """Side-by-side Scout WITH vs WITHOUT info_gain — sample reasons + maps."""
    # Use cycle 1 from Scout baseline (WITH info_gain) and Scout-noig (WITHOUT)
    cyc = "0001"
    fig, axes = plt.subplots(2, 2, figsize=(9.0, 6.5), dpi=120,
                              gridspec_kw={"height_ratios": [2.6, 1.6], "hspace": 0.35})
    for col, (sess, title, color) in enumerate([
        (SCOUT_SESS, "WITH info_gain field in scene JSON", "#205070"),
        (NOIG_SESS,  "WITHOUT info_gain field (ablation)",  "#a04040"),
    ]):
        ax_map = axes[0, col]
        ax_txt = axes[1, col]
        map_png = sess / f"cycle_{cyc}_map.png"
        if map_png.exists():
            ax_map.imshow(plt.imread(str(map_png)))
        ax_map.set_title(title, fontsize=10, color=color)
        ax_map.set_xticks([]); ax_map.set_yticks([])
        for sp in ax_map.spines.values(): sp.set_visible(False)
        # Compile first 3 reasons from this session
        decs = sorted(sess.glob("cycle_*_decision.json"))[:3]
        lines = []
        for f in decs:
            d = json.loads(f.read_text())
            g = d.get("goal")
            if g is None: continue
            r = (d.get("reason") or "")[:110]
            lines.append(f"c{f.stem[6:10]} → ({g[0]:.2f}, {g[1]:.2f})")
            lines.append(f"   {r}")
            lines.append("")
        ax_txt.axis("off")
        ax_txt.text(0.0, 1.0, "\n".join(lines), family="monospace", fontsize=8,
                    verticalalignment="top", transform=ax_txt.transAxes,
                    bbox=dict(boxstyle="round,pad=0.5", facecolor="#fafafa",
                              edgecolor=color, linewidth=1.0))
    fig.suptitle("Prompt anchoring ablation: Groq Scout with vs without the info_gain field",
                 fontsize=11, y=0.99)
    for ext in ["png", "pdf"]:
        plt.savefig(OUT / f"fig_anchoring_evidence.{ext}", dpi=180, bbox_inches="tight")
    plt.close()
    print("[anchoring_evidence] saved")


def hallucination_evidence():
    """Scout camera frames where it reports artefact near spawn (false positives)."""
    import math
    # Find 4 cycles with artifact_seen=True and pos near (4, 2)
    candidates = []
    for sess in sorted(Path("logs/vlm_variations/vlm_groq_scout/sessions").iterdir()):
        for f in sorted(sess.glob("cycle_*_decision.json")):
            d = json.loads(f.read_text())
            ap = d.get("artifact_pos")
            if ap and math.hypot(ap[0]-4.0, ap[1]-2.0) < 1.5:
                cam = sess / f"cycle_{f.stem[6:10]}_camera.png"
                if cam.exists():
                    candidates.append((cam, ap, d.get("reason", "")[:60], sess.name))
    if not candidates:
        print("[hallucination] no candidates"); return
    show = candidates[:4]
    fig, axes = plt.subplots(1, len(show), figsize=(3.0 * len(show), 3.0), dpi=120)
    if len(show) == 1: axes = [axes]
    for ax, (cam, ap, reason, sess) in zip(axes, show):
        ax.imshow(plt.imread(str(cam)))
        ax.set_title(f"reported: ({ap[0]:.1f}, {ap[1]:.1f})\n[spawn at ~(4, 2)]",
                     fontsize=8, color="#a04040")
        ax.set_xticks([]); ax.set_yticks([])
        for sp in ax.spines.values(): sp.set_visible(False)
    fig.suptitle("Hallucinated artefacts (Scout 17B) — camera frames where VLM reported "
                 "an artefact within 1.5 m of robot spawn, although none exists there",
                 fontsize=9, y=1.01)
    plt.tight_layout()
    for ext in ["png", "pdf"]:
        plt.savefig(OUT / f"fig_hallucination.{ext}", dpi=180, bbox_inches="tight")
    plt.close()
    print(f"[hallucination] saved ({len(show)} examples)")


def goal_scatter():
    """Where on demo3 did each method pick goals? Spatial bias."""
    methods = [
        ("mock_random",         "Random",          "#a02020"),
        ("mock_greedy_nearest", "Greedy nearest",  "#d07020"),
        ("mock_info_gain",      "Info-gain",       "#20a0a0"),
        ("vlm_groq_scout",      "Scout",           "#705020"),
        ("vlm_google",          "Gemini",          "#2070d0"),
    ]
    fig, axes = plt.subplots(1, len(methods), figsize=(2.8 * len(methods), 3.4), dpi=120,
                              sharex=True, sharey=True)
    for ax, (cond, label, color) in zip(axes, methods):
        ax.set_facecolor("#f5f5f5")
        # Plot scene boundary
        ax.add_patch(plt.Rectangle((0, -8), 24, 16, fill=False, edgecolor="black", linewidth=0.8))
        # Get goals from sessions if VLM, from logs if mock (mock doesn't save sessions same way)
        goals = []
        # Try VLM session goals first
        for sess_root in [Path(f"logs/vlm_variations/{cond}/sessions"), Path(f"logs/vlm_gemini_real/sessions") if cond == "vlm_google" else None]:
            if sess_root is None or not sess_root.exists(): continue
            for sess in sess_root.iterdir():
                if not sess.is_dir(): continue
                for f in sess.glob("cycle_*_decision.json"):
                    d = json.loads(f.read_text())
                    g = d.get("goal")
                    if g: goals.append(g)
        # For mocks no goal extraction needed since mock = info_gain heuristic = exposed
        if not goals and cond.startswith("mock"):
            # Use scene JSON files instead (mock's frontier picks are saved as scene_json + decision)
            for sess in Path("logs/vlm_variations").rglob(f"{cond}/**/sessions/*/cycle_*_decision.json"):
                d = json.loads(sess.read_text())
                g = d.get("goal")
                if g: goals.append(g)
        if goals:
            xs = [g[0] for g in goals]
            ys = [g[1] for g in goals]
            ax.scatter(xs, ys, s=18, color=color, alpha=0.5, edgecolor="black",
                       linewidth=0.3)
        ax.set_xlim(0, 24); ax.set_ylim(-8, 8); ax.set_aspect("equal")
        ax.set_title(f"{label}\n(n={len(goals)} goals)", fontsize=9)
        if ax == axes[0]:
            ax.set_xlabel("x (m)", fontsize=8); ax.set_ylabel("y (m)", fontsize=8)
        else:
            ax.set_yticklabels([])
        for sp in ax.spines.values(): sp.set_linewidth(0.5)
    fig.suptitle("Spatial distribution of goals picked by each method on demo3 (24 × 16 m)",
                 fontsize=10, y=1.03)
    plt.tight_layout()
    for ext in ["png", "pdf"]:
        plt.savefig(OUT / f"fig_goal_scatter.{ext}", dpi=180, bbox_inches="tight")
    plt.close()
    print("[goal_scatter] saved")


def latency_timeline():
    """Per-cycle latency timeline showing quota cascade pattern."""
    fig, axes = plt.subplots(2, 1, figsize=(8, 5), dpi=120, sharex=False,
                              gridspec_kw={"hspace": 0.4})
    # Top: Gemini smoke 2 (shows quota wall partway through)
    sess = GEMINI_SESS
    decs = sorted(sess.glob("cycle_*_decision.json"))
    cycles = []; durs = []; cols = []
    for f in decs:
        d = json.loads(f.read_text())
        cycles.append(int(f.stem[6:10]))
        dur = d.get("duration_sec", 0)
        durs.append(dur if dur is not None else 0)
        is_err = "agent_error" in (d.get("reason") or "")
        cols.append("#a04040" if is_err else "#2070d0")
    ax = axes[0]
    ax.bar(cycles, durs, width=0.9, color=cols, edgecolor="black", linewidth=0.3)
    ax.set_xlabel("cycle index", fontsize=9)
    ax.set_ylabel("latency (s)", fontsize=9)
    ax.set_title("Gemini Flash-Lite — single trial latency timeline "
                 "(blue=success, red=429 quota error)", fontsize=10)
    _styled(ax)
    # Bottom: Scout long session
    sess = SCOUT_SESS
    decs = sorted(sess.glob("cycle_*_decision.json"))
    cycles = []; durs = []; cols = []
    for f in decs:
        d = json.loads(f.read_text())
        cycles.append(int(f.stem[6:10]))
        dur = d.get("duration_sec", 0)
        durs.append(dur if dur is not None else 0)
        is_err = "agent_error" in (d.get("reason") or "")
        cols.append("#a04040" if is_err else "#705020")
    ax = axes[1]
    ax.bar(cycles, durs, width=0.9, color=cols, edgecolor="black", linewidth=0.3)
    ax.set_xlabel("cycle index", fontsize=9)
    ax.set_ylabel("latency (s)", fontsize=9)
    ax.set_title("Groq Scout — single trial latency timeline (consistent low latency)",
                 fontsize=10)
    _styled(ax)
    fig.suptitle("Cycle-by-cycle decision latency: Gemini's quota wall vs Scout's stable throughput",
                 fontsize=10, y=1.00)
    for ext in ["png", "pdf"]:
        plt.savefig(OUT / f"fig_latency_timeline.{ext}", dpi=180, bbox_inches="tight")
    plt.close()
    print("[latency_timeline] saved")


def decision_diversity():
    """Unique goals / total goals per method — quantifies repetition."""
    method_diversity = []
    for cond, label in [
        ("mock_random", "Random"),
        ("mock_greedy_nearest", "Greedy"),
        ("mock_info_gain", "Info-gain"),
        ("vlm_groq_scout", "Scout"),
        ("vlm_google", "Gemini"),
    ]:
        goals = []
        for sess_root in [Path(f"logs/vlm_variations/{cond}/sessions"),
                          Path("logs/vlm_gemini_real/sessions") if cond == "vlm_google" else None]:
            if sess_root is None or not sess_root.exists(): continue
            for sess in sess_root.iterdir():
                if not sess.is_dir(): continue
                for f in sess.glob("cycle_*_decision.json"):
                    d = json.loads(f.read_text())
                    g = d.get("goal")
                    if g: goals.append(tuple(round(x, 1) for x in g))
        if not goals:
            method_diversity.append((label, 0, 0)); continue
        uniq = len(set(goals))
        total = len(goals)
        method_diversity.append((label, uniq, total))
    labels = [m[0] for m in method_diversity]
    ratios = [m[1] / m[2] if m[2] else 0 for m in method_diversity]
    counts = [f"{m[1]}/{m[2]}" for m in method_diversity]
    fig, ax = plt.subplots(figsize=(7.0, 3.6), dpi=120)
    x = np.arange(len(labels))
    colors = ["#a02020", "#d07020", "#20a0a0", "#705020", "#2070d0"]
    bars = ax.bar(x, ratios, color=colors[:len(labels)],
                  edgecolor="black", linewidth=0.6, alpha=0.92)
    for i, (r, c) in enumerate(zip(ratios, counts)):
        ax.text(i, r + 0.02, c, ha="center", fontsize=9)
    ax.set_xticks(x); ax.set_xticklabels(labels)
    ax.set_ylabel("unique goals / total goals", fontsize=10)
    ax.set_ylim(0, 1.1)
    ax.set_title("Goal diversity — fraction of cycles selecting a previously-unseen goal",
                 fontsize=10)
    _styled(ax)
    for ext in ["png", "pdf"]:
        plt.savefig(OUT / f"fig_decision_diversity.{ext}", dpi=180, bbox_inches="tight")
    plt.close()
    print("[decision_diversity] saved")


def main():
    storyboard_gemini()
    provider_compare()
    anchoring_evidence()
    hallucination_evidence()
    goal_scatter()
    latency_timeline()
    decision_diversity()
    print(f"\nDone. Extra figures → {OUT}/")


if __name__ == "__main__":
    main()
