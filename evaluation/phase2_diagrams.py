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
    """Top-down hierarchy of the robot's own pipeline: supervisor above, six agents from sensing to acting."""
    fig, ax = plt.subplots(figsize=(10, 9.4))
    ax.set_xlim(0, 10); ax.set_ylim(0, 9.4); ax.axis("off")
    box(ax, 0.6, 8.25, 8.8, 0.9, "Supervisor agent  (mission level)",
        "mission state, knowledge graph of rooms, event timeline, completion", C["core"])
    tiers = [("1  Sense", "Perception agent", "depth frame -> 2D obstacle scan of the band above the floor"),
             ("2  Localise", "SLAM agent", "odometry + gyro prediction, scan-to-submap ICP,\nloop closure in a robust pose graph -> pose"),
             ("3  Map", "Mapping agent", "occupancy grid (5 cm), rooms and doorways"),
             ("4  Decide", "Frontier agent", "next operator-route waypoint, else the best\nfrontier viewpoint 0.5 m from unknown space"),
             ("5  Plan", "Planning agent", "A* path on the inflated grid, narrow-gap mode"),
             ("6  Act", "Locomotion agent", "trot gait follows the path; stall watchdog,\nfloor-edge and collision checks")]
    topics = ["sensor/scan", "pose/slam", "map/grid", "frontier/target", "plan/path"]
    y = 7.35
    for i, (tier, name, sub) in enumerate(tiers):
        ax.text(0.6, y - 0.36, tier, fontsize=10, weight="bold", color="#1f3b73", va="center")
        box(ax, 2.3, y - 0.8, 6.0, 0.8, name, sub, C["agent"])
        if i < len(tiers) - 1:
            arrow(ax, (5.3, y - 0.8), (5.3, y - 1.12))
            ax.text(5.45, y - 0.96, topics[i], fontsize=7.5, color="#1f3b73", va="center")
        y -= 1.12
    # inputs / outputs
    ax.text(5.3, 7.75, "depth frame", fontsize=7.5, ha="center", color="#555555")
    arrow(ax, (5.3, 7.65), (5.3, 7.37))
    arrow(ax, (5.3, 0.95), (5.3, 0.6))
    ax.text(5.3, 0.38, "joint commands to the legs", fontsize=7.5, ha="center", color="#555555")
    # feedback: odometry from Act to Localise
    ax.plot([8.3, 9.2, 9.2], [1.35, 1.35, 5.83], color=C["edge"], lw=1.1)
    arrow(ax, (9.2, 5.83), (8.3, 5.83))
    ax.text(9.3, 3.7, "leg odometry\n+ gyro", fontsize=7.5, color="#1f3b73", va="center")
    # supervisor link
    ax.plot([1.9, 1.9], [8.25, 0.95], color="#8a8a8a", lw=1.0, ls="--")
    ax.text(1.75, 4.7, "mission events (all agents)", fontsize=7.5, color="#555555", rotation=90, va="center", ha="center")
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
