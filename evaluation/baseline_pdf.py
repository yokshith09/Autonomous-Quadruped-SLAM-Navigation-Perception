#!/usr/bin/env python3
"""Baseline PDF: our measured results, what each value means and why it came out that way, a Good / Improve / Poor
class for each metric, and a comparison with published systems.

  python -m evaluation.baseline_pdf runs/<run_id> [out.pdf]

Our numbers are read from the run's files; published numbers come from evaluation/benchmarks.py (each tied to its
source). Classes are computed from the thresholds in METRICS below; each threshold states its basis: `published`
(anchored on a number in a cited paper or product page), `budget` (a real-time budget this system sets itself) or
`target` (an engineering target proposed here, not a literature value).
"""
import csv
import html
import json
import os
import re
import sys

import pymupdf as fitz  # PyMuPDF
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from evaluation import benchmarks as B  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLASS_CSS = {"Good": "good", "Improve": "improve", "Poor": "poor", "Helps": "good", "Weak": "improve",
             "Not shown": "poor", "Info": "info", "n/a": "info"}


# ------------------------------------------------------------------------------------------------ data
def load_run(run_dir):
    j = lambda n: json.load(open(os.path.join(run_dir, n), encoding="utf-8")) if os.path.exists(os.path.join(run_dir, n)) else None
    d = {"m": j("metrics.json"), "man": j("manifest.json"), "abl": j("ablation.json"), "dir": run_dir}
    d["traj"] = np.loadtxt(os.path.join(run_dir, "trajectory.csv"), delimiter=",", skiprows=1)
    prog = os.path.join(run_dir, "progress.csv")
    log = os.path.join(run_dir, "terminal.log")
    if not os.path.exists(prog) and os.path.exists(log):  # derive the 30 s progress samples from the terminal log
        rows = []
        for line in open(log, encoding="utf-8", errors="replace"):
            g = re.match(r"\s*\[t=\s*(\d+)s\] state (\S+) \| rooms cleared (\d+) \| explored ([\d.]+)% \| ATE (\d+) cm \| "
                         r"err now (\d+) cm \| yaw err ([\d.]+) deg \| F1 (\d+)%", line)
            if g:
                rows.append(g.groups())
        with open(prog, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "state", "rooms_cleared", "explored_pct", "ate_cm", "err_now_cm", "yaw_err_deg", "f1_pct"])
            w.writerows(rows)
    d["prog"] = [dict(zip(("t", "state", "rooms", "explored", "ate", "err", "yaw", "f1"), r))
                 for r in csv.reader(open(prog, encoding="utf-8"))][1:] if os.path.exists(prog) else []
    for r in d["prog"]:
        r["t"], r["explored"], r["rooms"] = float(r["t"]), float(r["explored"]), int(r["rooms"])
    return d


def derived(d):
    m, t = d["m"], d["traj"]
    o, loc, mp, c = m["overall"], m["localization"], m["mapping"], m["compute"]
    seg = np.hypot(*np.diff(t[:, 1:3], axis=0).T)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    out = {"dist": float(cum[-1]), "speed": float(cum[-1] / t[-1, 0])}
    if d["prog"]:
        final = d["prog"][-1]["explored"]
        for frac, key in ((0.95, "t95"), (0.97, "t97")):
            out[key] = next(r["t"] for r in d["prog"] if r["explored"] >= frac * final)
        i = int(np.searchsorted(t[:, 0], out["t97"]))
        out["dist_after_plateau"] = float(cum[-1] - cum[min(i, len(cum) - 1)])
        out["waste_pct"] = 100.0 * out["dist_after_plateau"] / out["dist"]
        out["coverage_gain_after_plateau"] = final - next(r["explored"] for r in d["prog"] if r["t"] == out["t97"])
    ev = m["data_quality"]["events"]
    out["stuck_per_100m"] = o["stuck_events"] / out["dist"] * 100.0
    out["room_entries_per_room"] = ev.get("ROOM_ENTERED", 0) / max(1, o["rooms_discovered"])
    out["ate_pct_path"] = loc["ate_rmse_m"] / out["dist"] * 100.0
    agents = {a["agent"]: a for a in m["agents"]}
    out["agents"] = agents
    out["agent_share"] = sum(a["total_s"] for a in m["agents"]) / o["wall_time_s"] * 100.0
    out["control_p95"] = m["agents_extra"]["control_loop_p95_ms"]
    out["control_max"] = m["agents_extra"]["control_loop_max_ms"]
    gait = re.search(r"GAIT_SPEED_CURVE = \(\((.*?)\)\)", open(os.path.join(ROOT, "robot.py"), encoding="utf-8").read())
    out["gait_max"] = float(gait.group(1).split("(")[-1].split(",")[-1]) if gait else None
    out["overhead_pct"] = 100.0 * c["telemetry_overhead_ms_per_step"] / m["agents_extra"]["control_loop_p50_ms"]
    if d["abl"]:
        rows = d["abl"]["rows"]
        out["abl"] = {r["variant"]: 100.0 * (r["ate_rmse_cm"] / rows[0]["ate_rmse_cm"] - 1.0) for r in rows[1:-1]}
    return out


def abl_val(d, prefix, key):
    return next(r[key] for r in d["abl"]["rows"] if r["variant"].startswith(prefix)) if d.get("abl") else float("nan")


# ------------------------------------------------------------------------------------------------ classification
def classify(v, good, poor, lower=True):
    if v is None:
        return "n/a"
    if lower:
        return "Good" if v <= good else ("Poor" if v > poor else "Improve")
    return "Good" if v >= good else ("Poor" if v < poor else "Improve")


def metric_rows(d, x):
    """[(group, metric, shown_value, unit, class, good-rule, basis, what, why)]"""
    m = d["m"]
    o, loc, mp, c = m["overall"], m["localization"], m["mapping"], m["compute"]
    ag, ev = x["agents"], m["data_quality"]["events"]
    cm = lambda v: v * 100.0
    R = []

    def add(group, metric, value, unit, good, poor, lower, basis, what, why, d_=1, cls=None):
        rule = f"{'<=' if lower else '>='} {good:g} good, {'>' if lower else '<'} {poor:g} poor" if good is not None else "-"
        cl = cls or classify(value, good, poor, lower)
        shown = "n/a" if value is None else (f"{value:.{d_}f}" if isinstance(value, float) else str(value))
        R.append((group, metric, f"{shown} {unit}".strip(), cl, rule, basis, what, why))

    G = "Exploration (Frontier, Planning, Locomotion agents)"
    add(G, "Task completed", "yes" if o["task_success"] else "no", "", None, None, True, "task",
        "The mission reported that every reachable frontier was explored.",
        f"It ended at the {o['sim_time_s']:.0f} s cap with {o['rooms_completed']} of {o['rooms_discovered']} rooms completed. The interior was "
        f"already mapped around t = {x.get('t97', float('nan')):.0f} s; the {mp['frontiers_remaining']} frontiers left are mostly not real "
        f"(outside or at the outer wall) and cannot be cleared, so completion is never reported.",
        cls="Good" if o["task_success"] else "Poor")
    add(G, "Average speed", x["speed"], "m/s", 0.5, 0.2, False, "published",
        "Ground-truth distance divided by simulated time.",
        f"The calibrated gait peaks near {x['gait_max']:.3f} m/s in the simulator, and the robot also turns, stops and revisits. "
        f"Reference: ANYmal 0.75 m/s normal walking, Spot 1.6 m/s max (product pages); FARE's indoor runs average "
        f"{1048 / 590:.2f} m/s (distance/time, derived).", 3)
    if "waste_pct" in x:
        add(G, "Distance walked after coverage plateaued", x["waste_pct"], "% of path", 10, 25, True, "target",
            f"Share of the path walked after the explored area reached 97 % of its final value (t = {x['t97']:.0f} s).",
            f"{x['dist_after_plateau']:.1f} m were walked after the plateau for only {x['coverage_gain_after_plateau']:.1f} percentage points "
            f"more coverage: the robot kept chasing frontiers that cannot be cleared (fixes F1, F2).", 0)
    add(G, "Room entries per room", x["room_entries_per_room"], "", 1.25, 2.0, True, "target",
        "Room-entered events divided by rooms found. About 1 means each room is entered once.",
        f"{ev.get('ROOM_ENTERED', 0)} entries for {o['rooms_discovered']} rooms: the room labels change as the map grows, so rooms are "
        f"'entered' again and re-swept (fix F3).", 2)
    add(G, "Stuck events", x["stuck_per_100m"], "per 100 m", 1, 3, True, "target",
        "Times the anti-stuck watchdog fired.", f"{o['stuck_events']} events in {x['dist']:.0f} m; each cost a reverse step and a dropped target.", 1)
    add(G, "Unreachable targets / paths planned", 100.0 * ev.get("PATH_UNREACHABLE", 0) / max(1, ag["PlanningAgent"]["calls"]), "% of planner calls",
        2, 5, True, "target", "Targets the A* planner could not reach.",
        f"{ev.get('PATH_UNREACHABLE', 0)} unreachable targets; they come from phantom frontiers beyond walls.", 2)

    G = "Localization (SLAM agent)"
    add(G, "ATE as % of path length", x["ate_pct_path"], "%", 0.3, 1.0, True, "published",
        "ATE RMSE divided by distance walked. Lets runs of different length be compared.",
        f"Published ANYmal D missions (145-332 m): ORB-SLAM3 stereo {0.330 / (sum(B.QUAD_MISSION_DIST_M) / 7) * 100:.2f} %, FAST-LIVO2 "
        f"{0.105 / (sum(B.QUAD_MISSION_DIST_M) / 7) * 100:.2f} % (derived from mean mission length). Our path is long because the "
        f"robot revisits a small arena, which flatters this ratio.", 2)
    add(G, "ATE RMSE", cm(loc["ate_rmse_m"]), "cm", 15, 50, True, "published",
        "Typical position error over the run, against simulator ground truth, no alignment.",
        f"Odometry error accumulates and {loc['loop_closures']} loop closures pull it back; the largest error was "
        f"{cm(loc['ate_max_m']):.0f} cm. It exceeds the 15 cm map tolerance. Published long-mission ATE: 11-33 cm (ANYmal D, outdoors).")
    add(G, "Final drift", loc["final_drift_pct_of_distance"], "% of path", 0.5, 2.0, True, "target",
        "Position error at the end of the run divided by the distance walked.",
        f"{cm(loc['final_position_error_m']):.1f} cm after {o['sim_time_s'] / 60:.0f} minutes: no runaway drift.", 2)
    add(G, "Largest position error", cm(loc["ate_max_m"]), "cm", 50, 100, True, "target",
        "Worst single-step error.", "Never above 1 m, so the estimate stayed usable for navigation throughout.", 0)
    add(G, "Mean heading error", loc["heading_err_mean_deg"], "deg", 2, 5, True, "target",
        "Orientation error against ground truth.", "Gyro-led yaw with a wall-direction (Manhattan) prior keeps heading small; "
        f"removing the prior raises the error to {abl_val(d, 'without Manhattan', 'heading_err_deg'):.1f} deg (section 2b).", 2)
    add(G, "Relative error per metre (RPE)", loc["rpe_trans_pct"], "% of 1 m", 2, 5, True, "target",
        "Error of the motion over each 1 m of true path, independent of accumulated drift.",
        f"Odometry plus gyro alone scores {next(r['rpe_trans_pct'] for r in d['abl']['rows'] if r['variant'].startswith('without scan matching')):.1f} %, "
        "so scan matching adds local jitter while fixing global drift (fix F7).")
    add(G, "Scan-match acceptance (diagnostic)", loc["icp_acceptance_pct"], "%", 80, 50, False, "target",
        "Share of SLAM updates whose scan match passed its gates; the rest fall back to odometry.",
        "The gates against odometry are tight (0.01 m plus the step length). Rejection reasons are not recorded, so this is an "
        "indicator, not a failure rate (fix F16).", 1)

    G = "Mapping (Mapping agent)"
    add(G, "Wall accuracy", cm(mp["wall_accuracy_m"]), "cm", 5, 10, True, "published",
        "Mean distance from each SLAM wall cell to the nearest reference wall cell.",
        f"Walls are thick: {mp['occupied_cells']} occupied cells against {mp['reference_occupied_cells']} in the reference built from the same "
        f"rays with the true pose. Pose error (ATE {cm(loc['ate_rmse_m']):.0f} cm) smears walls over several 5 cm cells. "
        "Published neural 3D maps reach 1.6-2.9 cm, a different representation.")
    add(G, "Wall completion", cm(mp["wall_completion_m"]), "cm", 5, 10, True, "published",
        "Mean distance from each reference wall cell to the nearest SLAM wall cell.", "Walls are present; the error is thickness, not missing walls.")
    add(G, "Completion ratio (5 cm)", mp["completion_ratio_5cm_pct"], "%", 90, 70, False, "published",
        "Share of reference wall cells matched within 5 cm (one cell).",
        "Low because walls are offset by more than one cell. Published neural SLAM on Replica: 90-97 % (different scenes and definition).")
    add(G, "Precision", mp["precision_pct"], "%", 85, 70, False, "target", "Share of SLAM occupied cells that match the reference within 15 cm.",
        "Drops when walls are doubled or smeared and when free-space rays leak past walls.")
    add(G, "Recall", mp["recall_pct"], "%", 85, 70, False, "target", "Share of reference occupied cells found by SLAM within 15 cm.",
        "The robot finds almost every wall it saw.")
    add(G, "F1", mp["f1_pct"], "%", 85, 70, False, "target", "Balance of precision and recall.", "Held back by precision.")
    add(G, "Free-space IoU", mp["free_space_iou_pct"], "%", 85, 70, False, "target", "Overlap of free cells with the reference.",
        "Free space is mostly right; leaks beyond walls reduce it.")
    add(G, "Map update time (p95)", mp["map_update_p95_ms"], "ms", 100, 200, True, "budget", "Time to integrate one depth frame (10 Hz budget 100 ms).",
        "From the agent's own timing of each frame update on the 220 x 220 grid.")
    add(G, "Map memory", mp["map_memory_mb"], "MB", 25, 100, True, "published", "Occupancy grid size.",
        "A 2D grid is tiny; published neural scene models use 20-40 MB (MNE-SLAM Table 5; not like-for-like).", 2)

    G = "Agents and real-time operation"
    add(G, "SLAM update time (median)", ag["SLAMAgent"]["p50_ms"], "ms", 20, 50, True, "published", "Time per SLAM update (scan match and loop checks).",
        f"CPU only. Published tracking: ORB-SLAM2 RGB-D {B.ORB2_TRACK_MS} ms/frame, MNE-SLAM 22.1 ms (different tasks and hardware).")
    add(G, "Frontier selection (median)", ag["FrontierAgent"]["p50_ms"], "ms", 100, 300, True, "budget", "Time per target selection (1 Hz).",
        "A geodesic search in pure Python; the most expensive per call (fix F14).")
    add(G, "Planner time (p95)", ag["PlanningAgent"]["p95_ms"], "ms", 100, 250, True, "budget", "A* path planning at 5 Hz.",
        f"Median {ag['PlanningAgent']['p50_ms']:.1f} ms but p95 {ag['PlanningAgent']['p95_ms']:.0f} ms: the calls that run a full A* search are slow.")
    add(G, "Slowest single agent call", max(a["max_ms"] for a in m["agents"]), "ms", 200, 1000, True, "budget",
        "Longest stall of any agent.", f"Perception reached {ag['PerceptionAgent']['max_ms']:.0f} ms and Planning {ag['PlanningAgent']['max_ms']:.0f} ms; "
        "causes not yet traced (fix F13).", 0)
    add(G, "Control-loop step (p95)", x["control_p95"], "ms", 50, 100, True, "budget", "Wall time of one whole control step (budget 50 ms at 20 Hz).",
        f"The agents use {x['agent_share']:.0f} % of wall time; the rest is simulator stepping, image transfer, HUD and per-step map queries (split not measured).", 0)
    add(G, "Real-time factor", o["real_time_factor"], "x", 1.0, 0.5, False, "budget", "Simulated seconds per wall second; 1 or more is real time.",
        "Not real time on this laptop (fix F12).", 2)
    add(G, "Process CPU (mean)", c["process_cpu_pct_mean"], "% of one core", 100, 200, True, "budget", "CPU use of the Python process (simulator excluded).",
        "One thread of Python work. Published for reference: ORB-SLAM3 stereo 290 %.", 0)
    add(G, "Memory (peak)", c["rss_mb_peak"], "MB", 1000, 4000, True, "budget", "Peak resident memory.",
        "Includes the in-memory recording buffer, which grows with run length (fix F15).", 0)
    add(G, "Telemetry overhead", x["overhead_pct"], "% of loop step", 1, 5, True, "budget", "Estimated cost of the measurement itself.",
        "A timing wrapper per agent call; estimated by micro-benchmark, not an uninstrumented A/B run.", 3)
    return R


def ablation_rows(d, x):
    if not d["abl"]:
        return []
    rows, out = d["abl"]["rows"], []
    why = {"without scan matching": f"Without it the heading drifts to {abl_val(d, 'without scan matching', 'heading_err_deg'):.1f} deg and the position error grows to metres.",
           "without loop closure": "Closures pull accumulated drift back; without them error grows between revisits.",
           "without degeneracy filter": "No measurable effect on this run (final error was slightly lower without it).",
           "without Manhattan": f"The wall-direction prior cancels gyro heading drift; without it heading error reaches {abl_val(d, 'without Manhattan', 'heading_err_deg'):.1f} deg."}
    for r in rows[1:-1]:
        dlt = x["abl"][r["variant"]]
        cl = "Helps" if dlt >= 25 else ("Weak" if dlt >= 10 else "Not shown")
        out.append((r["variant"].replace("without ", "Remove: "), f"{r['ate_rmse_cm']:.1f}", f"{dlt:+.0f} %", f"{r['heading_err_deg']:.2f}", cl,
                    next(v for k, v in why.items() if r["variant"].startswith(k))))
    return out


# ------------------------------------------------------------------------------------------------ comparison data
def comparison(d, x):
    loc, c = d["m"]["localization"], d["m"]["compute"]
    mean_dist = sum(B.QUAD_MISSION_DIST_M) / len(B.QUAD_MISSION_DIST_M)
    ours = (f"<b>Ours (this run)</b>", "forward RGB-D to 2D scan + gait odometry + gyro", "simulated 10 x 10 m, 8 rooms",
            f"{x['dist']:.0f}", f"{loc['ate_rmse_m'] * 100:.1f} cm", f"{x['ate_pct_path']:.2f} %")
    loc_rows = [ours]
    a1 = B.A1_SHORT_TRAJECTORIES
    loc_rows.append(("A1 SLAM paper (Unitree A1), 4 trajectories", "2D LiDAR + IMU + depth; motion-capture truth", "indoor lab",
                     f"{min(r[1] for r in a1):.1f}-{max(r[1] for r in a1):.1f}",
                     f"A1 SLAM {min(r[4] for r in a1) * 100:.1f}-{max(r[4] for r in a1) * 100:.1f} cm; Cartographer "
                     f"{min(r[3] for r in a1) * 100:.1f}-{max(r[3] for r in a1) * 100:.1f} cm",
                     f"{min(r[4] / r[1] for r in a1) * 100:.1f}-{max(r[4] / r[1] for r in a1) * 100:.1f} % (A1 SLAM)"))
    for meth, sens, ate, rpe, cpu, ram in B.QUAD_SLAM:
        if (meth, sens) in (("FAST-LIVO2", "LiDAR + camera + IMU (Honeywell)"), ("ORB-SLAM3", "stereo (global shutter)"),
                            ("RTAB-Map", "RGB-D (ZED2i)"), ("ORB-SLAM3", "RGB-D (ZED2i)")):
            loc_rows.append((f"{meth}, ANYmal D (GrandTour)", sens, "mostly outdoor, 7 missions", f"{min(B.QUAD_MISSION_DIST_M):.0f}-{max(B.QUAD_MISSION_DIST_M):.0f}",
                             f"{ate * 100:.1f} cm ({'mean over successful runs only (2 of 7 failed)' if meth == 'RTAB-Map' else 'mean over missions'})",
                             f"{ate / mean_dist * 100:.2f} % (derived)"))
    return loc_rows


# ------------------------------------------------------------------------------------------------ figures
def figures(d, x):
    paths = {}
    if d["prog"]:
        fig, ax = plt.subplots(figsize=(7.2, 2.8), dpi=170)
        t = [r["t"] for r in d["prog"]]
        ax.plot(t, [r["explored"] for r in d["prog"]], color="#1f6feb", lw=2, label="map explored (%)")
        ax.axvline(x["t97"], color="#b3261e", ls="--", lw=1)
        ax.text(x["t97"] + 15, 5, f"coverage 97 % of final at {x['t97']:.0f} s\n{x['waste_pct']:.0f} % of the path is walked after this", fontsize=7, color="#b3261e")
        ax.set_xlabel("simulated time (s)", fontsize=8)
        ax.set_ylabel("explored (% of map box)", fontsize=8, color="#1f6feb")
        ax2 = ax.twinx()
        ax2.step(t, [r["rooms"] for r in d["prog"]], color="#1b7f3b", lw=1.5, where="post")
        ax2.set_ylabel("rooms completed", fontsize=8, color="#1b7f3b")
        ax.tick_params(labelsize=7)
        ax2.tick_params(labelsize=7)
        ax.set_title("Exploration progress: coverage saturates long before the run ends", fontsize=9)
        fig.tight_layout()
        paths["progress"] = os.path.join(d["dir"], "figure_progress.png")
        fig.savefig(paths["progress"])
        plt.close(fig)
    tr = d["traj"]
    fig, ax = plt.subplots(figsize=(7.2, 2.4), dpi=170)
    err = np.hypot(tr[:, 4] - tr[:, 1], tr[:, 5] - tr[:, 2]) * 100
    ax.plot(tr[:, 0], err, color="#b36b00", lw=0.7)
    ax.set_xlabel("simulated time (s)", fontsize=8)
    ax.set_ylabel("position error (cm)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.set_title("Localization error over the run (drops are loop closures)", fontsize=9)
    fig.tight_layout()
    paths["error"] = os.path.join(d["dir"], "figure_error.png")
    fig.savefig(paths["error"])
    plt.close(fig)
    return paths


# ------------------------------------------------------------------------------------------------ HTML
CSS = """
body { font-family: sans-serif; font-size: 9pt; color: #1b1f24; }
h1 { font-size: 19pt; margin-bottom: 2pt; } h2 { font-size: 13pt; margin-top: 12pt; margin-bottom: 3pt; color: #0b3d91; }
h3 { font-size: 10.5pt; margin-top: 8pt; margin-bottom: 2pt; }
p { margin-top: 2pt; margin-bottom: 4pt; line-height: 1.25; } .note { font-size: 8pt; color: #555555; }
table { width: 100%; margin-top: 3pt; margin-bottom: 6pt; }
td.hdr { background-color: #e8eef9; font-weight: bold; font-size: 8pt; padding: 2pt; border: 0.5pt solid #b8c2d6; }
td { font-size: 8pt; padding: 2pt; border: 0.5pt solid #d5dbe6; vertical-align: top; }
.good { color: #1b7f3b; font-weight: bold; } .improve { color: #a35a00; font-weight: bold; }
.poor { color: #b3261e; font-weight: bold; } .info { color: #555555; }
"""


def code_label(man):
    import subprocess
    try:
        snap = subprocess.run(["git", "log", "--follow", "--diff-filter=A", "--format=%h", "--", "evaluation/run_telemetry.py"],
                              capture_output=True, text=True, cwd=ROOT, timeout=10).stdout.split()[-1]
    except Exception:
        snap = None
    g = man["git"]
    return (f"snapshot commit {snap}; the run recorded {g.get('commit')}" + (" plus uncommitted changes" if g.get("uncommitted_changes") else "")
            if snap else f"{g.get('commit')}")


def esc(s):
    return html.escape(str(s))


def tbl(headers, rows, widths=None, raw_cols=()):
    h = "<table><tr>" + "".join(f"<td class='hdr'{f' width={w}' if w else ''}>{esc(x)}</td>" for x, w in zip(headers, widths or [None] * len(headers))) + "</tr>"
    for r in rows:
        h += "<tr>" + "".join(f"<td>{c if i in raw_cols else esc(c)}</td>" for i, c in enumerate(r)) + "</tr>"
    return h + "</table>"


def badge(c):
    return f"<span class='{CLASS_CSS.get(c, 'info')}'>{esc(c)}</span>"


def build_html(d, x, figs):
    m, man = d["m"], d["man"]
    o, loc = m["overall"], m["localization"]
    R = metric_rows(d, x)
    groups = {}
    for r in R:
        groups.setdefault(r[0], []).append(r)
    cnt = lambda rows: {k: sum(1 for r in rows if r[3] == k) for k in ("Good", "Improve", "Poor")}
    h = [f"<h1>Baseline results</h1><p class='note'>Run {esc(m['run_id'])} | code: {esc(code_label(man))} | "
         f"{esc(man.get('cpu'))} | one run, simulation only | built by evaluation/baseline_pdf.py from the run's files</p>"]

    # 1. verdict
    h.append("<h2>1. Verdict at a glance</h2>")
    h.append("<p>The robot localizes well and maps the interior fully, but it cannot finish the mission, walks slowly, and "
             "does not run in real time on this machine. Every number below was measured in this run; a class (Good, Improve, "
             "Poor) is assigned from the thresholds in section 2.</p>")
    rows = [(g, *(str(cnt(rs)[k]) for k in ("Good", "Improve", "Poor"))) for g, rs in groups.items()]
    allc = cnt(R)
    rows.append(("<b>All metrics</b>", *(f"<b>{allc[k]}</b>" for k in ("Good", "Improve", "Poor"))))
    h.append(tbl(["Component", "Good", "Improve", "Poor"], rows, [300, 50, 50, 50], raw_cols=(0, 1, 2, 3)))
    h.append("<table><tr><td class='hdr'>Where we are</td><td class='hdr'>Evidence from this run</td></tr>"
             f"<tr><td class='good'>Good</td><td>No runaway drift: final error {loc['final_drift_pct_of_distance']:.2f} % of {x['dist']:.0f} m, never above "
             f"{loc['ate_max_m'] * 100:.0f} cm; heading error {loc['heading_err_mean_deg']:.2f} deg; wall recall {m['mapping']['recall_pct']:.0f} %; "
             f"SLAM update {x['agents']['SLAMAgent']['p50_ms']:.1f} ms; scan matching, loop closure and the heading prior each clearly help (section 2b).</td></tr>"
             f"<tr><td class='improve'>Can improve</td><td>Absolute error {loc['ate_rmse_m'] * 100:.0f} cm (target 15 cm); wall accuracy {m['mapping']['wall_accuracy_m'] * 100:.1f} cm; "
             "frontier selection time; memory use.</td></tr>"
             f"<tr><td class='poor'>Poor</td><td>Mission not completed; speed {x['speed']:.3f} m/s; {x.get('waste_pct', 0):.0f} % of the path walked after coverage plateaued; "
             f"rooms re-entered {x['room_entries_per_room']:.1f} times each; relative error per metre {loc['rpe_trans_pct']:.1f} %; real-time factor "
             f"{o['real_time_factor']:.2f}; control-loop stalls up to {x['control_max'] / 1000:.1f} s.</td></tr></table>")

    # 2. results
    h.append("<h2>2. Our results: what each value is, why it came out this way, and its class</h2>")
    h.append("<p>Each table lists one component. <b>Value</b> is measured. <b>Class rule</b> shows the limits: for example "
             "'&lt;= 15 good, &gt; 50 poor' means 15 or less is Good, above 50 is Poor, in between is Improve. <b>Basis</b> says where the limit "
             "comes from: <i>published</i> (anchored on a cited number), <i>budget</i> (a real-time budget of this system) or "
             "<i>target</i> (an engineering target proposed here, not a literature value).</p>")
    h.append(f"<p class='note'>Run: {esc(o['end_reason'])}; simulated {o['sim_time_s']:.0f} s in {o['wall_time_s']:.0f} s of wall time; "
             f"{x['dist']:.1f} m walked; {o['rooms_completed']} of {o['rooms_discovered']} rooms completed. Ground truth is the simulator pose; "
             "the map reference is a grid built with that pose, not an independent floor plan.</p>")
    for g, rs in groups.items():
        h.append(f"<h3>{esc(g)}</h3>")
        h.append(tbl(["Metric", "Value", "Class", "Class rule", "Basis", "What it is", "Why this value"],
                     [(esc(r[1]), esc(r[2]), badge(r[3]), esc(r[4]), esc(r[5]), esc(r[6]), esc(r[7])) for r in rs],
                     [78, 52, 36, 62, 40, 120, 170], raw_cols=(0, 1, 2, 3, 4, 5, 6)))
    if figs.get("progress"):
        h.append(f"<p><img src='{os.path.basename(figs['progress'])}' width='480'/></p>")
    h.append(f"<p><img src='{os.path.basename(figs['error'])}' width='480'/></p>")
    h.append("<p><img src='map_overlay.png' width='560'/></p><p class='note'>Left: SLAM map with the true path (blue), the estimated path (orange) and "
             "the 48 frontiers left at the end (red circles, most at or beyond the outer wall). Right: reference grid built with the true pose.</p>")

    # 2b ablation
    ab = ablation_rows(d, x)
    if ab:
        h.append("<h3>2b. Does each SLAM component help? (offline replay of this run's recording, one component removed)</h3>")
        h.append("<p>The same recorded scans and odometry are replayed with one component switched off, so any difference is caused by that "
                 f"component. Replay check: the full replay matches the live trajectory to {d['abl']['rows'][0]['replay_vs_live_max_cm']:.2f} cm and two "
                 "identical replays are identical. 'Helps' means ATE rises by 25 % or more when it is removed; 'Not shown' means under 10 %. One recording only.</p>")
        h.append(tbl(["Configuration", "ATE (cm)", "Change vs full", "Heading err (deg)", "Verdict", "Why"], [(a, b, c, e, badge(f), g) for a, b, c, e, f, g in ab],
                     [110, 40, 55, 55, 45, 190], raw_cols=(4,)))

    # 3. comparison
    h.append("<h2>3. Comparison with published systems</h2>")
    h.append("<p><b>Result of the search: no published system was evaluated under our conditions, so none is directly comparable.</b> "
             "The closest are partial matches: the A1 SLAM paper (same robot type, short paths), the ANYmal D evaluation (quadruped, long "
             "missions, real outdoors) and FARE (autonomous exploration, wheeled). Numbers below are quoted from the cited tables; "
             "'derived' means we divided two published numbers. Differences are listed so a reader can judge.</p>")
    h.append("<h3>3a. Localization (partially comparable)</h3>")
    h.append(tbl(["System", "Sensors", "Environment", "Path (m)", "ATE RMSE", "ATE / path"], comparison(d, x), [150, 120, 80, 50, 100, 60], raw_cols=(0,)))
    h.append(f"<p><b>Reading:</b> our drift per path length ({x['ate_pct_path']:.2f} %) is the same order as ORB-SLAM3 stereo (derived {0.330 / (sum(B.QUAD_MISSION_DIST_M) / 7) * 100:.2f} %) and above FAST-LIVO2 ({0.105 / (sum(B.QUAD_MISSION_DIST_M) / 7) * 100:.2f} %); we do not claim "
             "to beat them. Not like-for-like: ours is simulation with exact ground truth, a small arena that the robot loops through repeatedly "
             "(which helps loop closure), and a 2D estimate; theirs are real, mostly outdoor missions with 3D estimates. The RGB-D configurations in that "
             "study (closest to our depth sensor) performed worst (2.4-8.2 m), so sensor type alone does not decide the result.</p>")
    h.append("<h3>3b. Runtime and resources (reference only: different tasks and hardware)</h3>")
    h.append(tbl(["System", "Quantity", "Value", "Hardware / note"],
                 [("<b>Ours</b>", "SLAM update, median", f"{x['agents']['SLAMAgent']['p50_ms']:.1f} ms", f"CPU only, {esc(d['man'].get('cpu'))}; 2D scan matching"),
                  ("ORB-SLAM2 (RGB-D)", "tracking per frame", f"{B.ORB2_TRACK_MS} ms", "Intel Core i7-4790, TUM fr3_office, feature-based"),
                  ("MNE-SLAM", "tracking per frame", "22.1 ms", "authors' hardware, neural scene model"),
                  ("A1 SLAM / Cartographer", "optimization per scan", "1.65 / 0.96 ms", "optimizer call only, not full pipeline"),
                  ("ORB-SLAM3 stereo (ANYmal D)", "CPU / RAM", "290 % / 1.16 GB", "Corlito et al. Table V"),
                  ("<b>Ours</b>", "CPU / RAM", f"{d['m']['compute']['process_cpu_pct_mean']:.0f} % of one core / {d['m']['compute']['rss_mb_mean'] / 1000:.1f} GB mean "
                   f"({d['m']['compute']['rss_mb_peak'] / 1000:.1f} GB peak)", "includes the recording buffer; simulator excluded")], [110, 90, 130, 180], raw_cols=(0,)))
    h.append("<p><b>Reading:</b> our SLAM step is cheaper than the camera-based systems, which is expected for 2D scan matching against feature tracking "
             "and says nothing about accuracy. Our whole control loop is not real time (section 2).</p>")
    h.append("<h3>3c. Exploration speed and time (partially comparable: task similar, robot and environment differ)</h3>")
    fi = B.FARE_INDOOR
    rows = [("<b>Ours</b>", f"{x['dist']:.0f}", f"{o['sim_time_s']:.0f} (not finished)", f"{x['speed']:.3f}", "quadruped, forward RGB-D, 10 x 10 m arena")]
    rows += [(f"FARE paper: {n}", f"{dist}", f"{t}", f"{dist / t:.2f} (derived)", "wheeled, 16-channel 3D LiDAR, Gazebo indoor, max 2 m/s") for n, dist, _, t, _ in fi]
    rows += [("ANYmal (product page)", "-", "-", "0.75 normal walking", "inspection robot; routes, not open exploration"),
             ("Spot (product page)", "-", "-", "1.6 max", "needs a recorded map; Directed Exploration only reroutes"),
             ("Go2 (product page)", "-", "-", "5 max (lab)", "follows a user-specified path")]
    h.append(tbl(["System", "Distance (m)", "Time (s)", "Average speed (m/s)", "Robot / environment"], rows, [120, 50, 80, 80, 200], raw_cols=(0,)))
    h.append("<p><b>Reading:</b> distances and times cannot be compared (their environments are far larger and the robot differs), and we did not finish. "
             f"Speed is a property of the robot: ours ({x['speed']:.3f} m/s) is slower than every system listed. Spot and Go2 are documented as needing a recorded map "
             "or a user-given path; the ANYmal page does not say whether it explores unknown space. None of the three product pages gives exploration time or accuracy figures.</p>")
    h.append("<h3>3d. Mapping (not comparable)</h3>")
    mn = [r for r in B.MNE_REPLICA if r[0] == "MNE-SLAM"][0]
    h.append(tbl(["System", "Accuracy", "Completion", "Completion ratio", "Representation"],
                 [("<b>Ours</b>", f"{d['m']['mapping']['wall_accuracy_m'] * 100:.1f} cm", f"{d['m']['mapping']['wall_completion_m'] * 100:.1f} cm",
                   f"{d['m']['mapping']['completion_ratio_5cm_pct']:.1f} %", "2D occupancy grid, 5 cm cells, wall cells"),
                  (f"MNE-SLAM (Replica)", f"{mn[1]} cm", f"{mn[2]} cm", f"{mn[3]} %", "3D neural implicit map, mesh surfaces, GPU")], [110, 70, 70, 80, 200], raw_cols=(0,)))
    h.append("<p><b>Reading:</b> the metrics share names but not definitions or scenes. A 5 cm grid cannot resolve better than about 2.5 cm, so a "
             "cell-to-cell comparison with a 1.7 cm mesh error is not fair. We make no better/worse claim.</p>")
    h.append("<h3>3e. Experiments that would make a fair comparison possible</h3>")
    h.append("<p>(1) Run Cartographer or another open 2D SLAM on this run's recorded scans and odometry: identical data and ground truth, the only "
             "head-to-head available (F25). (2) Run a standard nearest-frontier explorer in the same arena with the same robot for time to complete (F26). "
             "(3) Repeat our run at least three times with different gyro seeds to give a spread (F18). (4) Hardware validation (F24).</p>")

    # 4. fixes
    h.append("<h2>4. Fixes needed</h2><p>Full register with evidence and acceptance checks: docs/FIXES.md. Highest priority first:</p>")
    h.append(tbl(["ID", "Fix", "Why (evidence in this run)"], [
        ("F1-F3", "Remove phantom frontiers, keep the blacklist, freeze room labels", f"{m['mapping']['frontiers_remaining']} frontiers left, most unclearable; "
                                                                                  f"{x.get('waste_pct', 0):.0f} % of the path wasted; {x['room_entries_per_room']:.1f} entries per room"),
        ("F4", "End runs on completion or no progress, not a fixed cap", "run ended by the cap"),
        ("F5", "Raise walking speed after F1-F3", f"{x['speed']:.3f} m/s"),
        ("F7-F9", "Reduce local jitter and wall smear; tighten ATE", f"RPE {loc['rpe_trans_pct']:.1f} %, wall accuracy {m['mapping']['wall_accuracy_m'] * 100:.1f} cm, ATE {loc['ate_rmse_m'] * 100:.0f} cm"),
        ("F12-F14", "Profile and remove stalls", f"real-time factor {o['real_time_factor']:.2f}; loop stalls to {x['control_max'] / 1000:.1f} s")], [40, 200, 250]))

    # 5. limits
    h.append("<h2>5. What this baseline can and cannot show</h2><p>It shows how the current system behaves in this arena in one run. It does not show "
             "repeatability (one run), real-robot performance (simulation), independent map accuracy (reference built from the same sensor), or the contribution "
             "of mapping, frontier selection and planning (only SLAM components were ablated). Scan-match acceptance and tracking-loss counts are "
             "diagnostic. Interventions and collisions were not measurable. Class limits marked <i>target</i> are our proposals, not published standards.</p>")
    h.append("<h2>Sources</h2><p class='note'>" + " | ".join(f"{esc(v[1].split(chr(34))[1] if chr(34) in v[1] else v[1])} ({esc(v[0])}) {esc(v[2])}" for v in B.SOURCES.values()) + "</p>")
    return "".join(h)


def build(run_dir, out=None):
    d = load_run(run_dir)
    x = derived(d)
    figs = figures(d, x)
    out = out or os.path.join(ROOT, "docs", "BASELINE_REPORT.pdf")
    story = fitz.Story(html=build_html(d, x, figs), user_css=CSS, archive=fitz.Archive(run_dir))
    raw = out + ".raw"
    writer = fitz.DocumentWriter(raw)
    page = fitz.paper_rect("a4")
    where = page + (34, 36, -34, -40)
    more = 1
    while more:
        dev = writer.begin_page(page)
        more, _ = story.place(where)
        story.draw(dev)
        writer.end_page()
    writer.close()
    doc = fitz.open(raw)
    for i, pg in enumerate(doc):
        pg.insert_text((34, 818), f"Baseline run {d['m']['run_id']}", fontsize=7, color=(0.4, 0.4, 0.4))
        pg.insert_text((520, 818), f"page {i + 1} / {len(doc)}", fontsize=7, color=(0.4, 0.4, 0.4))
    doc.save(out, garbage=3, deflate=True)
    doc.close()
    del writer, story
    import gc
    gc.collect()
    try:
        os.remove(raw)  # the writer can keep the handle on Windows; a leftover .raw is git-ignored
    except OSError:
        pass
    print(f"  [SAVED] {out}")
    return out


if __name__ == "__main__":
    build(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
