#!/usr/bin/env python3
"""SLAM, localization and exploration evaluator for a recorded CoppeliaSim run.

Every number for OUR system is recomputed from the raw matrices saved by main_agentic.py
(captured_matrices/). Published baselines are quoted verbatim from MNE-SLAM (Deng et al.,
CVPR 2025) with their table number and dataset, so a reader can check each value.
"""

import ast
import json
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from accuracy_evaluator import evaluate_grid_accuracy, evaluate_trajectory
from generate_separate_maps import generate_separate_maps
from benchmarks import MNE_APARTMENT, MNE_REPLICA, MNE_RUNTIME, MNE_SCANNET_AVG

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

MNE_REF = "Deng et al., MNE-SLAM, CVPR 2025"

GT_POSE_APIS = {"getObjectPosition", "getObjectOrientation", "getObjectMatrix", "getObjectPose", "getObjectVelocity"}
NAV_FILES = ["slam.py"] + [os.path.join("agents", f) for f in (
    "slam_agent.py", "mapping_agent.py", "perception_agent.py", "frontier_agent.py", "planning_agent.py")]


def compute_all_live_metrics(
    data_dir: str = "captured_matrices",
    results_dir: str = "accuracy_results",
    resolution: float = 0.05,
    save_plots: bool = True,
    plot_dir: str = None,
) -> dict:
    """Recompute all localization and mapping metrics directly from saved NumPy matrices."""
    os.makedirs(results_dir, exist_ok=True)
    plot_dir = plot_dir or results_dir
    combined = {}

    json_path = os.path.join(results_dir, "latest_metrics.json")
    if os.path.exists(json_path):
        with open(json_path, "r", encoding="utf-8") as f:
            combined.update(json.load(f))

    true_grid_path = os.path.join(data_dir, "true_grid.npy")
    slam_grid_path = os.path.join(data_dir, "slam_grid.npy")
    if os.path.exists(true_grid_path) and os.path.exists(slam_grid_path):
        true_grid = np.load(true_grid_path)
        slam_grid = np.load(slam_grid_path)
        combined.update(evaluate_grid_accuracy(
            true_grid, slam_grid, resolution=resolution, tolerance_m=0.15, output_dir=plot_dir, save_plots=save_plots
        ))
        explored_cells = int(np.count_nonzero((slam_grid <= -0.5) | (slam_grid >= 1.0)))
        combined["explored_pct"] = explored_cells / slam_grid.size * 100.0
        combined.setdefault("map_memory_mb", slam_grid.nbytes / (1024.0 * 1024.0))

    traj_path = os.path.join(data_dir, "trajectory_data.npz")
    if os.path.exists(traj_path):
        data = np.load(traj_path)
        timestamps = data["timestamps"] if "timestamps" in data else None
        combined.update(evaluate_trajectory(
            data["true_poses"], data["slam_poses"], timestamps=timestamps, output_dir=plot_dir, save_plots=save_plots
        ))

    kg_path = os.path.join(results_dir, "knowledge_graph.json")
    if os.path.exists(kg_path):
        with open(kg_path, "r", encoding="utf-8") as f:
            kg = json.load(f)
        combined["rooms_discovered"] = len(kg.get("rooms", {}))
        combined["doorways_discovered"] = len(kg.get("doorways", {}))
        combined["entities_mapped"] = len(kg.get("entities", {}))

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(combined, f, indent=2)
    return combined


def _gt_api_calls(tree, allowed=frozenset()):
    return [f"line {n.lineno}: {n.attr}" for n in ast.walk(tree)
            if isinstance(n, ast.Attribute) and n.attr in GT_POSE_APIS - allowed]


def verify_results_not_bypassed(data_dir: str = "captured_matrices", results_dir: str = "accuracy_results") -> list:
    """Integrity checks that the SLAM estimate never consumes simulator ground truth."""
    checks = []

    # 1. Static audit: no ground-truth pose API anywhere in the estimation/navigation path.
    leaks = []
    for fname in NAV_FILES:
        if os.path.exists(fname):
            with open(fname, "r", encoding="utf-8") as f:
                leaks += [f"{fname} {c}" for c in _gt_api_calls(ast.parse(f.read()))]
    # Odometry feeds SLAM: the simulated gyro may read orientation (per-step delta only), never position.
    with open("robot.py", "r", encoding="utf-8") as f:
        robot_tree = ast.parse(f.read())
    odom_fn = next((n for n in ast.walk(robot_tree)
                    if isinstance(n, ast.FunctionDef) and n.name == "get_odometry_imu_delta"), None)
    if odom_fn is None:
        leaks.append("robot.py: get_odometry_imu_delta not found")
    else:
        leaks += [f"robot.py get_odometry_imu_delta {c}" for c in _gt_api_calls(odom_fn, {"getObjectOrientation"})]
    checks.append((
        "Static audit: no GT pose API in SLAM, agents, or odometry",
        not leaks,
        "; ".join(leaks) if leaks else f"{len(NAV_FILES)} nav files + odometry clean (gyro = noisy orientation delta)",
    ))

    # 2. Runtime audit: in closed-loop mode SLAMAgent never subscribes to ground truth.
    from agents import MessageBus, SLAMAgent
    bus = MessageBus()
    SLAMAgent(bus, use_ground_truth_nav=False).initialize()
    subscribed = "pose/ground_truth" in bus._subscribers
    checks.append((
        "Runtime audit: SLAMAgent bus subscriptions (closed loop)",
        not subscribed,
        "SLAMAgent subscribes to: " + ", ".join(sorted(bus._subscribers)),
    ))

    # 3. Trajectories must physically diverge (a GT replay would give ~0 error).
    traj_ok, traj_detail = False, "trajectory_data.npz not found"
    traj_path = os.path.join(data_dir, "trajectory_data.npz")
    if os.path.exists(traj_path):
        data = np.load(traj_path)
        tp, sp = data["true_poses"][:, :2], data["slam_poses"][:, :2]
        n = min(len(tp), len(sp))
        if n > 5:
            err = np.linalg.norm(tp[:n] - sp[:n], axis=1)
            rmse_cm = float(np.sqrt(np.mean(err ** 2))) * 100.0
            traj_ok = rmse_cm > 0.1
            traj_detail = f"ATE RMSE {rmse_cm:.2f} cm, max {float(np.max(err)) * 100.0:.2f} cm over {n} poses"
    checks.append(("Trajectory independence (SLAM != ground truth)", traj_ok, traj_detail))

    # 4. The two occupancy grids must be independently built.
    grid_ok, grid_detail = False, "true_grid.npy / slam_grid.npy not found"
    tg_path, sg_path = os.path.join(data_dir, "true_grid.npy"), os.path.join(data_dir, "slam_grid.npy")
    if os.path.exists(tg_path) and os.path.exists(sg_path):
        diff_cells = int(np.count_nonzero(np.abs(np.load(tg_path) - np.load(sg_path)) > 1e-3))
        grid_ok = diff_cells > 50
        grid_detail = f"{diff_cells:,d} cells differ"
    checks.append(("Dual-grid independence (true grid != SLAM grid)", grid_ok, grid_detail))

    # 5. The run itself must have navigated on the SLAM pose.
    cl_ok = False
    json_path = os.path.join(results_dir, "latest_metrics.json")
    if os.path.exists(json_path):
        with open(json_path, "r", encoding="utf-8") as f:
            cl_ok = bool(json.load(f).get("closed_loop_slam", False))
    checks.append(("Run navigated on pose/slam (closed_loop_slam flag)", cl_ok,
                   "closed_loop_slam=True" if cl_ok else "flag missing or False"))
    return checks


def _fmt(value, scale=1.0, unit="", nd=2):
    return "n/a" if value is None else f"{value * scale:.{nd}f}{unit}"


def _rank_line(label, ours, rows, col):
    vals = sorted([r[col] for r in rows if r[col] is not None] + [ours])
    return f"  -> {label}: OURS would rank {vals.index(ours) + 1} of {len(vals)} (lower is better)"


def build_comparison_lines(m: dict) -> list:
    """Side-by-side tables against MNE-SLAM's published numbers, with our measured row appended."""
    ate = m.get("ate_rmse_meters")
    acc, comp, ratio = m.get("accuracy_meters"), m.get("completion_meters"), m.get("completion_ratio_5cm")
    ate_cm = ate * 100.0 if ate is not None else None
    lines = [
        f"  Source of every non-OURS value: {MNE_REF}. Datasets differ from ours, so read these as",
        "  'same metric, different scene' context, not a head-to-head on identical data:",
        "  - Baselines: dense 3D (NeRF / RGB-D) SLAM on Replica / Apartment / ScanNet, metrics on 3D meshes.",
        "  - OURS: 2D occupancy-grid SLAM, single agent, quadruped in CoppeliaSim (robot_working.ttt),",
        "    Acc/Comp computed on 2D wall cells against a grid raycast from the simulator's true pose.",
        "",
        "  [MNE-SLAM Table 2] Multi-room 'Apartment' scenes, global ATE RMSE (cm)",
        f"  {'Method':<14} | {'Apt-1':>7} | {'Apt-2':>7} | {'Apt-0':>7}",
        "  " + "-" * 44,
    ]
    for name, a1, a2, a0 in MNE_APARTMENT:
        lines.append(f"  {name:<14} | {_fmt(a1):>7} | {_fmt(a2):>7} | {_fmt(a0):>7}")
    lines.append(f"  {'OURS':<14} | {_fmt(ate_cm):>7} |  (our CoppeliaSim multi-room arena, single run)")
    if ate_cm is not None:
        lines.append(_rank_line("vs Apartment-1", ate_cm, MNE_APARTMENT, 1))

    lines += [
        "",
        "  [MNE-SLAM Table 1] Replica, global reconstruction + tracking",
        f"  {'Method':<14} | {'Acc cm':>7} | {'Comp cm':>7} | {'Ratio %':>7} | {'ATE cm':>7}",
        "  " + "-" * 56,
    ]
    for name, a, c, r, t in MNE_REPLICA:
        lines.append(f"  {name:<14} | {a:>7.3f} | {c:>7.3f} | {r:>7.3f} | {t:>7.3f}")
    lines.append(f"  {'OURS (2D)':<14} | {_fmt(acc, 100, '', 3):>7} | {_fmt(comp, 100, '', 3):>7} | "
                 f"{_fmt(ratio, 100, '', 3):>7} | {_fmt(ate_cm, 1, '', 3):>7}")

    lines += [
        "",
        "  [MNE-SLAM Table 3] ScanNet (real-world) average ATE RMSE (cm)",
        "  " + " | ".join(f"{n} {_fmt(v, nd=1)}" for n, v in MNE_SCANNET_AVG),
        f"  OURS (this run): {_fmt(ate_cm, nd=1)}",
        "",
        "  [MNE-SLAM Table 5] Runtime and scene-representation memory (Replica; their hardware)",
        f"  {'Method':<14} | {'Map/Fr':>9} | {'Track/Fr':>9} | {'Memory':>9}",
        "  " + "-" * 50,
    ]
    for name, mp, tr, mem in MNE_RUNTIME:
        lines.append(f"  {name:<14} | {mp:>8.2f}s | {tr:>7.1f}ms | {mem:>7.2f}MB")
    map_s = m.get("cpu_map_ms") / 1000.0 if m.get("cpu_map_ms") is not None else None
    lines.append(f"  {'OURS (CPU)':<14} | {_fmt(map_s, nd=4):>8}s | {_fmt(m.get('cpu_track_ms'), nd=1):>7}ms | "
                 f"{_fmt(m.get('map_memory_mb')):>7}MB   [{m.get('cpu_name', 'cpu unknown')}]")
    return lines


def print_full_evaluation_suite():
    os.makedirs("accuracy_results", exist_ok=True)
    m = compute_all_live_metrics()
    checks = verify_results_not_bypassed()

    w = 102
    report_lines = []

    def log_line(line=""):
        print(line)
        report_lines.append(line)

    def section(title):
        log_line("\n" + "=" * w)
        log_line(f"  {title}")
        log_line("=" * w)

    def rows(items):
        for name, val in items:
            log_line(f"  {name:<46} | {val}")

    section("1. LOCALIZATION & TRAJECTORY ACCURACY (this run)")
    rows([
        ("ATE RMSE",                         _fmt(m.get("ate_rmse_meters"), 100, " cm")),
        ("ATE mean / median / std",          f"{_fmt(m.get('ate_mean_meters'), 100, ' cm')} / "
                                             f"{_fmt(m.get('ate_median_meters'), 100, ' cm')} / "
                                             f"{_fmt(m.get('ate_std_meters'), 100, ' cm')}"),
        ("ATE max",                          _fmt(m.get("ate_max_meters"), 100, " cm")),
        ("RPE translational (per pose step)", _fmt(m.get("rpe_trans_rmse_meters"), 100, " cm", 3)),
        ("RPE rotational (per pose step)",   _fmt(m.get("rpe_rot_rmse_deg"), 1, " deg", 3)),
        ("Drift rate (ATE RMSE / path length)", _fmt(m.get("drift_rate_percent"), 1, " %")),
        ("Heading error mean / max",         f"{_fmt(m.get('mean_heading_error_deg'), 1, ' deg')} / "
                                             f"{_fmt(m.get('max_heading_error_deg'), 1, ' deg')}"),
        ("Path length (ground truth)",       _fmt(m.get("total_distance_traveled_m"), 1, " m")),
    ])

    section("2. MAPPING ACCURACY (SLAM grid vs grid raycast from true pose)")
    rows([
        ("Acc. (SLAM walls -> true walls)",  _fmt(m.get("accuracy_meters"), 100, " cm")),
        ("Comp. (true walls -> SLAM walls)", _fmt(m.get("completion_meters"), 100, " cm")),
        ("Chamfer (mean of Acc/Comp)",       _fmt(m.get("chamfer_distance_meters"), 100, " cm")),
        ("Hausdorff 95th percentile",        _fmt(m.get("hausdorff_95_meters"), 100, " cm")),
        ("Comp. ratio (< 5 cm)",             _fmt(m.get("completion_ratio_5cm"), 100, " %", 1)),
        ("Wall F1 / precision / recall (+/-15 cm)", f"{_fmt(m.get('f1_score'), 100, ' %', 1)} / "
                                             f"{_fmt(m.get('precision'), 100, ' %', 1)} / "
                                             f"{_fmt(m.get('recall'), 100, ' %', 1)}"),
        ("Obstacle IoU exact / tolerant",    f"{_fmt(m.get('obstacle_iou'), 100, ' %', 1)} / "
                                             f"{_fmt(m.get('tolerant_iou'), 100, ' %', 1)}"),
        ("Free-space IoU",                   _fmt(m.get("free_space_iou"), 100, " %", 1)),
    ])

    section("3. EXPLORATION MISSION & CPU RUNTIME (this run)")
    rows([
        ("Mission ended because",            m.get("end_reason", "n/a")),
        ("Mission time (sim)",               _fmt(m.get("mission_time_s"), 1, " s", 1)),
        ("Rooms segmented / rooms cleared",  f"{m.get('rooms_discovered', 'n/a')} / {m.get('rooms_cleared', 'n/a')}"),
        ("Doorways / semantic entities",     f"{m.get('doorways_discovered', 'n/a')} / {m.get('entities_mapped', 'n/a')}"),
        ("Frontiers remaining at end",       str(m.get("frontiers_remaining", "n/a"))),
        ("Map cells known (free + occupied)", _fmt(m.get("explored_pct"), 1, " % of 11x11 m grid", 1)),
        ("Tracking / mapping / planning latency", f"{_fmt(m.get('cpu_track_ms'), 1, ' ms', 1)} / "
                                             f"{_fmt(m.get('cpu_map_ms'), 1, ' ms', 1)} / "
                                             f"{_fmt(m.get('cpu_plan_ms'), 1, ' ms', 1)}"),
        ("Map memory",                       _fmt(m.get("map_memory_mb"), 1, " MB")),
        ("CPU",                              m.get("cpu_name", "n/a")),
    ])

    section(f"4. COMPARISON WITH PUBLISHED SLAM SYSTEMS ({MNE_REF})")
    for line in build_comparison_lines(m):
        log_line(line)

    section("5. RESULT VALIDITY AUDIT")
    for name, passed, detail in checks:
        log_line(f"  {'[PASS]' if passed else '[FAIL]':<8} {name:<56} -> {detail}")
    log_line("=" * w + "\n")

    report_path = os.path.join("accuracy_results", "full_evaluation_report.txt")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("\n".join(report_lines))
    print(f"  [SAVED] Full evaluation report -> {report_path}", flush=True)

    try:
        save_all_evaluation_figures(image_dir="evaluation_results_images")
        generate_separate_maps(data_dir="captured_matrices", results_dir="accuracy_results",
                               images_dir="evaluation_results_images")
    except Exception as e:
        print(f"  [WARN] Could not save figures: {e}", flush=True)


def save_all_evaluation_figures(
    image_dir: str = "evaluation_results_images",
    data_dir: str = "captured_matrices",
    results_dir: str = "accuracy_results",
    resolution: float = 0.05
):
    """Saves the quantitative evaluation figures for the last run into image_dir."""
    os.makedirs(image_dir, exist_ok=True)
    m = compute_all_live_metrics(data_dir=data_dir, results_dir=results_dir, resolution=resolution, plot_dir=image_dir)

    for old, new in [("numerical_map_accuracy_comparison.png", "01_numerical_matrix_map_comparison.png"),
                     ("robot_movement_trajectory_analysis.png", "02_robot_trajectory_and_drift_analysis.png")]:
        if os.path.exists(os.path.join(image_dir, old)):
            os.replace(os.path.join(image_dir, old), os.path.join(image_dir, new))

    true_grid_path = os.path.join(data_dir, "true_grid.npy")
    slam_grid_path = os.path.join(data_dir, "slam_grid.npy")
    traj_path = os.path.join(data_dir, "trajectory_data.npz")

    if os.path.exists(true_grid_path) and os.path.exists(slam_grid_path):
        true_grid, slam_grid = np.load(true_grid_path), np.load(slam_grid_path)
        h, w = true_grid.shape
        extent = [-w * resolution / 2, w * resolution / 2, -h * resolution / 2, h * resolution / 2]
        fig, ax = plt.subplots(figsize=(10, 8))
        im = ax.imshow(np.abs(true_grid - slam_grid), cmap='inferno', origin='lower', extent=extent)
        ax.set_title("Cell-by-cell log-odds discrepancy |l_true - l_slam|", fontsize=12, fontweight='bold')
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        fig.colorbar(im, ax=ax).set_label("|Δ log-odds|")
        plt.tight_layout()
        plt.savefig(os.path.join(image_dir, "05_map_cell_difference_heatmap.png"), dpi=200)
        plt.close()

    def pct(key):
        return _fmt(m.get(key), 100, " %", 1)

    def cm(key):
        return _fmt(m.get(key), 100, " cm")

    fig, axes = plt.subplots(1, 2, figsize=(18, 10))
    tables = [
        (axes[0], "LOCALIZATION", "#1f77b4", [
            ["ATE RMSE", cm("ate_rmse_meters")], ["ATE mean", cm("ate_mean_meters")],
            ["ATE median", cm("ate_median_meters")], ["ATE max", cm("ate_max_meters")],
            ["RPE trans / step", _fmt(m.get("rpe_trans_rmse_meters"), 100, " cm", 3)],
            ["RPE rot / step", _fmt(m.get("rpe_rot_rmse_deg"), 1, " deg", 3)],
            ["Drift rate", _fmt(m.get("drift_rate_percent"), 1, " %")],
            ["Heading error mean", _fmt(m.get("mean_heading_error_deg"), 1, " deg")],
            ["Path length", _fmt(m.get("total_distance_traveled_m"), 1, " m")],
        ]),
        (axes[1], "MAPPING", "#2ca02c", [
            ["Acc.", cm("accuracy_meters")], ["Comp.", cm("completion_meters")],
            ["Chamfer", cm("chamfer_distance_meters")], ["Hausdorff-95", cm("hausdorff_95_meters")],
            ["Comp. ratio < 5 cm", pct("completion_ratio_5cm")], ["Wall F1 (±15 cm)", pct("f1_score")],
            ["Precision / recall", f"{pct('precision')} / {pct('recall')}"],
            ["Free-space IoU", pct("free_space_iou")],
        ]),
    ]
    for ax, title, color, data in tables:
        ax.axis('off')
        tbl = ax.table(cellText=data, colLabels=[f"{title} METRIC", "MEASURED"], loc='center', cellLoc='left')
        tbl.auto_set_font_size(False)
        tbl.set_fontsize(11)
        tbl.scale(1.0, 1.8)
        for (row, _), cell in tbl.get_celld().items():
            if row == 0:
                cell.set_facecolor(color)
                cell.set_text_props(color='white', weight='bold')
        ax.set_title(title, fontsize=12, fontweight='bold', pad=15)
    plt.suptitle("Measured metrics for the last CoppeliaSim run", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(image_dir, "03_evaluation_metrics_scorecard.png"), dpi=200, bbox_inches='tight')
    plt.close()

    if os.path.exists(slam_grid_path):
        slam_grid = np.load(slam_grid_path)
        h, w = slam_grid.shape
        extent = [-w * resolution / 2, w * resolution / 2, -h * resolution / 2, h * resolution / 2]
        fig, ax = plt.subplots(figsize=(10, 8))
        im = ax.imshow(slam_grid, cmap='coolwarm', origin='lower', extent=extent, vmin=-3.0, vmax=3.0)
        if os.path.exists(traj_path):
            data = np.load(traj_path)
            ax.plot(data["true_poses"][:, 0], data["true_poses"][:, 1], color='#00ffcc', linewidth=2.0, label='Ground truth path')
            ax.plot(data["slam_poses"][:, 0], data["slam_poses"][:, 1], color='#ff00ff', linestyle='--', linewidth=1.5, label='SLAM path')
            ax.legend(loc='upper right')
        ax.set_title(f"Exploration coverage (known cells {_fmt(m.get('explored_pct'), 1, '%', 1)}) | "
                     f"rooms cleared {m.get('rooms_cleared', 'n/a')} | doorways {m.get('doorways_discovered', 'n/a')}",
                     fontsize=12, fontweight='bold')
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        plt.colorbar(im, ax=ax, label="Log-odds")
        plt.tight_layout()
        plt.savefig(os.path.join(image_dir, "06_room_exploration_coverage.png"), dpi=200)
        plt.close()

    print(f"  [SAVED] Evaluation figures -> {image_dir}/", flush=True)


if __name__ == "__main__":
    print_full_evaluation_suite()
