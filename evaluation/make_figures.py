#!/usr/bin/env python3
"""Figures for the README and papers, drawn from a run's files and the published numbers in benchmarks.py.

  python -m evaluation.make_figures runs/<run_id>     # writes docs/images/*.png
"""
import os
import shutil
import sys

import matplotlib
import matplotlib.ticker  # noqa: F401
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

from evaluation import benchmarks as B  # noqa: E402
from evaluation.baseline_pdf import ROOT, derived, figures, load_run  # noqa: E402

OURS, THEIRS, GRID = "#1f6feb", "#8b949e", "#d0d7de"
OUT = os.path.join(ROOT, "docs", "images")


def style(ax):
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(axis="x", color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    ax.tick_params(labelsize=8)


def architecture(d):
    rates = d["man"]["config"]["agent_rates_hz"]
    fig, ax = plt.subplots(figsize=(9.5, 4.3), dpi=170)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 46)
    ax.axis("off")

    def box(x, y, w, h, title, sub, fc="#eaf2ff", ec=OURS):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4,rounding_size=1.2", fc=fc, ec=ec, lw=1.2))
        ax.text(x + w / 2, y + h * 0.66, title, ha="center", va="center", fontsize=8.5, fontweight="bold")
        ax.text(x + w / 2, y + h * 0.28, sub, ha="center", va="center", fontsize=6.8, color="#444")

    ax.text(50, 44, "CoppeliaSim: quadruped, forward RGB-D camera, gyro, gait odometry", ha="center", fontsize=8.5, color="#444")
    names = [("Perception", "PerceptionAgent", "depth to 2D scan"), ("SLAM", "SLAMAgent", "scan match, loops"),
             ("Mapping", "MappingAgent", "grid + rooms"), ("Frontier", "FrontierAgent", "next target"),
             ("Planning", "PlanningAgent", "A* path"), ("Locomotion", "LocomotionAgent", "gait, steering")]
    w, gap, x0, y0, h = 14.2, 2.1, 1.5, 24, 11
    for i, (t, k, sub) in enumerate(names):
        x = x0 + i * (w + gap)
        box(x, y0, w, h, t, f"{sub}\n{rates[k]:g} Hz")
        if i:
            ax.annotate("", xy=(x - 0.2, y0 + h / 2), xytext=(x - gap + 0.2, y0 + h / 2), arrowprops=dict(arrowstyle="->", color="#444", lw=1.1))
    ax.annotate("", xy=(x0 + w / 2, 38), xytext=(x0 + w / 2, y0 + h + 0.6), arrowprops=dict(arrowstyle="<-", color="#444", lw=1.1))
    ax.annotate("", xy=(x0 + 5 * (w + gap) + w / 2, 38), xytext=(x0 + 5 * (w + gap) + w / 2, y0 + h + 0.6), arrowprops=dict(arrowstyle="->", color="#444", lw=1.1))
    box(34, 3.5, 32, 7, "Supervisor", f"knowledge graph + event log, {rates['SupervisorAgent']:g} Hz", fc="#f6f1ff", ec="#8250df")
    ax.add_patch(FancyBboxPatch((1.5, 15), 97, 4.2, boxstyle="round,pad=0.3,rounding_size=1", fc="#f3f4f6", ec="#9aa0a6", lw=1))
    ax.text(50, 17.1, "Message bus (in-process publish / subscribe): sensor/scan, pose/slam, map/grid, frontier/target, plan/path, control/actuation, mission/event",
            ha="center", va="center", fontsize=6.8, color="#333")
    ax.annotate("", xy=(50, 10.8), xytext=(50, 14.6), arrowprops=dict(arrowstyle="<->", color="#8250df", lw=1.1))
    fig.savefig(os.path.join(OUT, "architecture.png"), bbox_inches="tight")
    plt.close(fig)


def latency(d, x):
    ag = d["m"]["agents"]
    names = [a["agent"].replace("Agent", "") for a in ag]
    fig, ax = plt.subplots(figsize=(7.2, 3.0), dpi=170)
    y = np.arange(len(ag))
    ax.barh(y + 0.18, [a["p50_ms"] for a in ag], 0.34, color=OURS, label="median")
    ax.barh(y - 0.18, [a["p95_ms"] for a in ag], 0.34, color="#79a8f7", label="95th percentile")
    ax.set_yticks(y, names)
    ax.invert_yaxis()
    ax.set_xscale("log")
    ax.set_xlabel("time of one step() call (ms, log scale)", fontsize=8)
    ax.set_title("Processing time per agent", fontsize=9)
    ax.legend(fontsize=7, frameon=False)
    style(ax)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "agent_latency.png"))
    plt.close(fig)


def ablation(d):
    rows = d["abl"]["rows"][:-1]
    names = [r["variant"].replace("without ", "without\n").replace(" (odometry + gyro only)", "") for r in rows]
    fig, ax = plt.subplots(figsize=(7.2, 3.0), dpi=170)
    y = np.arange(len(rows))
    vals = [r["ate_rmse_cm"] for r in rows]
    ax.barh(y, vals, color=[OURS] + ["#e0746b"] * (len(rows) - 1))
    for yi, v in zip(y, vals):
        ax.text(v + 3, yi, f"{v:.1f}", va="center", fontsize=7.5)
    ax.set_yticks(y, names, fontsize=7.5)
    ax.invert_yaxis()
    ax.set_xlabel("ATE RMSE (cm), offline replay of the same recording", fontsize=8)
    ax.set_title("Effect of removing one SLAM component", fontsize=9)
    style(ax)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "ablation.png"))
    plt.close(fig)


def comparison(d, x):
    loc, t = d["m"]["localization"], d.get("truth")
    mean_dist = sum(B.QUAD_MISSION_DIST_M) / len(B.QUAD_MISSION_DIST_M)
    ate = [("Ours (simulation)", loc["ate_rmse_m"] * 100, True)]
    ate += [(n + " (Replica)", next(r for r in B.MNE_REPLICA if r[0] == n)[4], False) for n in ("MNE-SLAM", "Loopy-SLAM", "Point-SLAM")]
    for meth, sens, a, _, _, _ in B.QUAD_SLAM:
        if (meth, sens) in (("FAST-LIVO2", "LiDAR + camera + IMU (Honeywell)"), ("ORB-SLAM3", "stereo (global shutter)")):
            ate.append((f"{meth} (ANYmal D)", a * 100, False))
    acc = [("Ours (vs real scene)", t["map_vs_scene"]["accuracy_meters"] * 100 if t else d["m"]["mapping"]["wall_accuracy_m"] * 100, True)]
    acc += [(n + " (Replica)", next(r for r in B.MNE_REPLICA if r[0] == n)[1], False) for n in ("MNE-SLAM", "Loopy-SLAM", "Point-SLAM")]
    fig, axs = plt.subplots(1, 2, figsize=(9.5, 3.0), dpi=170)
    for ax, data, title, unit in ((axs[0], ate, "Pose error (ATE RMSE)", "cm, log scale"), (axs[1], acc, "Map accuracy", "cm, log scale")):
        y = np.arange(len(data))
        ax.barh(y, [v for _, v, _ in data], color=[OURS if o else THEIRS for _, _, o in data])
        for yi, (_, v, _) in zip(y, data):
            ax.text(v * 1.12, yi, f"{v:.2f}" if v < 10 else f"{v:.1f}", va="center", fontsize=7)
        ax.set_yticks(y, [n for n, _, _ in data], fontsize=7)
        ax.invert_yaxis()
        ax.set_xscale("log")
        ax.set_xticks([0.3, 1, 3, 10, 30, 100], ["0.3", "1", "3", "10", "30", "100"])
        ax.xaxis.set_minor_locator(matplotlib.ticker.NullLocator())
        ax.set_xlim(0.25, 150)
        ax.set_xlabel(unit, fontsize=8)
        ax.set_title(title, fontsize=9)
        style(ax)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.text(0.5, 0.015, "Published numbers on other scenes and sensors: shows the gap, not a ranking", ha="center", fontsize=8, color="#555")
    fig.savefig(os.path.join(OUT, "comparison.png"))
    plt.close(fig)


def build(run_dir):
    os.makedirs(OUT, exist_ok=True)
    d = load_run(run_dir)
    x = derived(d)
    figs = figures(d, x)
    shutil.copy(figs["progress"], os.path.join(OUT, "exploration_progress.png"))
    shutil.copy(figs["error"], os.path.join(OUT, "localization_error.png"))
    shutil.copy(os.path.join(run_dir, "map_overlay.png"), os.path.join(OUT, "map_overlay.png"))
    architecture(d)
    latency(d, x)
    if d.get("abl"):
        ablation(d)
    comparison(d, x)
    print(f"  [SAVED] figures in {OUT}: {sorted(os.listdir(OUT))}")


if __name__ == "__main__":
    build(sys.argv[1])
