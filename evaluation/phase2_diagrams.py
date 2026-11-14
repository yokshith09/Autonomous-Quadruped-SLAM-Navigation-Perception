"""Architecture and flow diagrams for the Phase II report: python -m evaluation.phase2_diagrams <out_dir>"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

C = {"sim": "#dfe9f7", "agent": "#e3f4e1", "core": "#fff3d6", "eval": "#f3e3f4", "todo": "#f2f2f2", "edge": "#36454f"}


def box(ax, x, y, w, h, title, sub="", color="#ffffff", dashed=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12", fc=color, ec=C["edge"],
                                lw=1.2, ls="--" if dashed else "-"))
    ax.text(x + w / 2, y + h * (0.62 if sub else 0.5), title, ha="center", va="center", fontsize=9.5, weight="bold")
    if sub:
        ax.text(x + w / 2, y + h * 0.3, sub, ha="center", va="center", fontsize=7.3, color="#333333")


def arrow(ax, a, b, label="", rad=0.0, lx=0.0, ly=0.0):
    ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="-|>", lw=1.1, color=C["edge"],
                                                    connectionstyle=f"arc3,rad={rad}"))
    if label:
        ax.text((a[0] + b[0]) / 2 + lx, (a[1] + b[1]) / 2 + ly, label, fontsize=6.8, color="#1f3b73", ha="center",
                va="center", bbox=dict(fc="white", ec="none", pad=0.6))


def architecture(path):
    fig, ax = plt.subplots(figsize=(12, 8.2))
    ax.set_xlim(0, 12); ax.set_ylim(0, 8.2); ax.axis("off")
    # layer bands
    for y, h, name, col in ((6.7, 1.38, "Simulation / hardware interface", C["sim"]),
                            (2.0, 4.5, "Agents on a publish-subscribe message bus (one process, 20 Hz control loop)", "#f7fbf6"),
                            (0.1, 1.7, "Evaluation, storage and planned stages", "#fbf7fc")):
        ax.add_patch(FancyBboxPatch((0.1, y), 11.8, h, boxstyle="round,pad=0.02,rounding_size=0.15", fc=col, ec="#c8ccd2",
                                    lw=0.8))
        ax.text(0.25, y + h - 0.05, name, fontsize=8.5, style="italic", color="#555555", va="top")
    box(ax, 0.4, 6.85, 3.2, 0.85, "Actuators", "12 joints, trot gait (core/robot.py)", "#ffffff")
    box(ax, 4.2, 6.85, 3.6, 0.85, "Sensors", "depth camera 60 deg, gyro, leg odometry", "#ffffff")
    box(ax, 8.4, 6.85, 3.2, 0.85, "CoppeliaSim quadruped", "ZMQ remote API, scene loaded per environment", "#ffffff")

    box(ax, 0.8, 4.75, 2.4, 1.05, "Perception agent", "depth -> 2D scan, thumbnails", C["agent"])
    box(ax, 3.5, 4.75, 2.6, 1.05, "SLAM agent", "scan-to-submap ICP, gyro calib.,\nloop closure, robust pose graph", C["agent"])
    box(ax, 6.5, 4.75, 2.4, 1.05, "Mapping agent", "occupancy grid 5 cm,\nrooms, doorways", C["agent"])
    box(ax, 9.3, 4.75, 2.3, 1.05, "Supervisor agent", "mission state, knowledge\ngraph, episodic memory", C["agent"])
    box(ax, 6.5, 2.35, 2.4, 1.05, "Frontier agent", "viewpoints, operator route,\nnarrow gaps, completion", C["agent"])
    box(ax, 3.5, 2.35, 2.6, 1.05, "Planning agent", "A* on inflated grid,\npath smoothing", C["agent"])
    box(ax, 0.8, 2.35, 2.4, 1.05, "Locomotion agent", "path following, watchdog,\nstuck recovery, drop check", C["agent"])
    box(ax, 9.3, 2.35, 2.3, 1.05, "Safety monitor", "contact check (collisions),\ntip-over detection", C["core"])

    arrow(ax, (5.2, 6.85), (2.0, 5.8), "sensor/rgbd", lx=-0.3, ly=0.12)
    arrow(ax, (3.2, 5.27), (3.5, 5.27), "sensor/scan", ly=0.22)
    arrow(ax, (6.1, 5.27), (6.5, 5.27), "pose/slam", ly=0.22)
    arrow(ax, (8.9, 5.27), (9.3, 5.27), "map/grid", ly=0.22)
    arrow(ax, (7.7, 4.75), (7.7, 3.4), "map/grid", lx=0.42)
    arrow(ax, (6.5, 2.87), (6.1, 2.87), "frontier/target", ly=0.22)
    arrow(ax, (3.5, 2.87), (3.2, 2.87), "plan/path", ly=0.22)
    ax.plot([0.8, 0.45, 0.45], [2.87, 2.87, 6.5], color=C["edge"], lw=1.1)
    arrow(ax, (0.45, 6.5), (0.45, 6.85))
    ax.text(0.52, 4.1, "joint\ntargets", fontsize=6.8, color="#1f3b73", ha="left")
    arrow(ax, (2.6, 3.4), (4.2, 4.75), "control/actuation\n(odometry, gyro)", rad=-0.1, lx=0.75, ly=-0.05)
    arrow(ax, (10.45, 4.75), (8.9, 3.15), "mission/event", rad=0.1, lx=0.3)
    arrow(ax, (10.45, 3.4), (10.45, 4.75), "events", lx=0.35)

    box(ax, 0.4, 0.3, 2.7, 1.1, "Run storage", "runs/<id>: metrics, trajectory,\nmaps, recording, REPORT.md", C["eval"])
    box(ax, 3.4, 0.3, 2.9, 1.1, "Evaluation", "ATE vs ground truth, map F1 vs\nscene model, replay, baselines", C["eval"])
    box(ax, 6.6, 0.3, 2.5, 1.1, "Event trigger +\nteam's Mamba model", "[planned, not integrated]", C["todo"],
        dashed=True)
    box(ax, 9.4, 0.3, 2.3, 1.1, "Forensic store + sync", "SHA-256 logs, AES-256\n[planned, not integrated]", C["todo"],
        dashed=True)
    arrow(ax, (3.1, 0.85), (3.4, 0.85))
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def flow(path):
    fig, ax = plt.subplots(figsize=(9.2, 11))
    ax.set_xlim(-0.6, 8.9); ax.set_ylim(0, 11); ax.axis("off")
    steps = [
        ("Start", "load scene profile (odometry scale, route, map size)", C["sim"]),
        ("Calibrate", "stand still: estimate gyro bias (62 s rescue)", C["core"]),
        ("Sense", "depth frame -> 2D scan; leg odometry + gyro", C["agent"]),
        ("Localise", "odometry predict, scan-to-submap ICP, loop closure + pose graph", C["agent"]),
        ("Map", "update occupancy grid; segment rooms, doorways", C["agent"]),
        ("Decide", "next operator-route waypoint, else best frontier viewpoint", C["agent"]),
        ("Plan", "A* path on the inflated grid (narrow-gap mode per scene)", C["agent"]),
        ("Act", "trot gait follows the path; collision and drop checks", C["agent"]),
    ]
    y = 10.2
    for i, (t, s, col) in enumerate(steps):
        box(ax, 1.3, y - 0.75, 6.0, 0.75, t, s, col)
        if i < len(steps) - 1:
            arrow(ax, (4.3, y - 0.75), (4.3, y - 0.98))
        y -= 1.0
    # decision diamond
    ax.add_patch(plt.Polygon([(4.3, 2.0), (6.0, 1.45), (4.3, 0.9), (2.6, 1.45)], fc="#fff7e0", ec=C["edge"], lw=1.2))
    ax.text(4.3, 1.45, "Progress?\nFrontiers left?", ha="center", va="center", fontsize=8, weight="bold")
    arrow(ax, (4.3, 2.45), (4.3, 2.0))
    ax.plot([2.6, 0.9], [1.45, 1.45], color=C["edge"], lw=1.0)
    ax.text(0.8, 1.45, "no progress:\nwatchdog ->\nturn-away,\nskip blocked\ntarget", ha="right", va="center", fontsize=7)
    ax.plot([0.9, 0.9], [1.45, 7.85], color=C["edge"], lw=1.0)
    arrow(ax, (0.9, 7.85), (1.3, 7.85))
    ax.plot([6.0, 7.8, 7.8], [1.45, 1.45, 7.65], color=C["edge"], lw=1.0)
    arrow(ax, (7.8, 7.65), (7.3, 7.65))
    ax.text(7.9, 4.4, "frontiers left:\nnext step\n(20 Hz loop)", fontsize=7, ha="left")
    arrow(ax, (4.3, 0.9), (4.3, 0.45))
    box(ax, 2.3, -0.25, 4.0, 0.7, "Done", "save run, report, scores", C["eval"])
    ax.set_ylim(-0.4, 11)
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "docs/report_phase2"
    os.makedirs(out, exist_ok=True)
    architecture(os.path.join(out, "architecture.png"))
    flow(os.path.join(out, "flow.png"))
    print("ok", out)
