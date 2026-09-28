"""MNE-SLAM (CVPR 2025) vs. Autonomous Quadruped SLAM Pipeline Benchmark Comparison.

Reads live simulation metrics from accuracy_results/ and displays side-by-side
comparison against published MNE-SLAM benchmarks, our current live runs,
and target operational thresholds for closed-loop edge deployment (Raspberry Pi 5).
"""

import os
import re
import json
import numpy as np


def load_evaluation_report(report_path="accuracy_results/evaluation_summary_report.txt"):
    metrics = {}
    json_path = "accuracy_results/latest_metrics.json"
    if os.path.exists(json_path):
        try:
            with open(json_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass

    if not os.path.exists(report_path):
        return metrics
    try:
        with open(report_path, "r", encoding="utf-8") as f:
            for line in f:
                match = re.match(r"\s*([a-zA-Z0-9_]+):\s*([0-9\.\-e]+)", line)
                if match:
                    key = match.group(1)
                    val = float(match.group(2))
                    metrics[key] = val
    except Exception as e:
        print(f"  [WARN] Failed parsing {report_path}: {e}")
    return metrics


def load_knowledge_graph(kg_path="accuracy_results/knowledge_graph.json"):
    stats = {"rooms": 0, "doorways": 0, "entities": 0, "frontiers_cleared": 0}
    if not os.path.exists(kg_path):
        return stats
    try:
        with open(kg_path, "r", encoding="utf-8") as f:
            data = json.load(f)
            stats["rooms"] = len(data.get("rooms", {}))
            stats["doorways"] = len(data.get("doorways", {}))
            stats["entities"] = len(data.get("entities", {}))
            stats["frontiers_cleared"] = len(data.get("doorways", {})) + len(data.get("rooms", {}))
    except Exception:
        pass
    return stats


def load_trajectory_data(traj_path="captured_matrices/trajectory_data.npz"):
    stats = {"ate_rmse_cm": None, "max_ate_cm": None, "total_dist_m": None}
    if not os.path.exists(traj_path):
        return stats
    try:
        data = np.load(traj_path)
        true_poses = data["true_poses"]
        slam_poses = data["slam_poses"]
        min_len = min(len(true_poses), len(slam_poses))
        if min_len > 5:
            tp = true_poses[:min_len, :2]
            sp = slam_poses[:min_len, :2]
            errors = np.linalg.norm(tp - sp, axis=1)
            rmse_m = float(np.sqrt(np.mean(errors ** 2)))
            stats["ate_rmse_cm"] = rmse_m * 100.0
            stats["max_ate_cm"] = float(np.max(errors)) * 100.0
            diffs = np.linalg.norm(np.diff(tp, axis=0), axis=1)
            stats["total_dist_m"] = float(np.sum(diffs))
    except Exception:
        pass
    return stats


def print_comparison_benchmark():
    grid_metrics = load_evaluation_report()
    kg_stats = load_knowledge_graph()
    traj_stats = load_trajectory_data()

    dist_m = traj_stats.get("total_dist_m", grid_metrics.get("total_distance_traveled_m"))
    dist_str = f"{dist_m:.2f} m" if dist_m is not None else "N/A"

    explored_pct = grid_metrics.get("explored_pct")
    explored_str = f"{explored_pct:.1f} %" if explored_pct is not None else "N/A"

    ate_cm = traj_stats.get("ate_rmse_cm")
    if ate_cm is None and "ate_rmse_meters" in grid_metrics:
        ate_cm = grid_metrics["ate_rmse_meters"] * 100.0
    ate_str = f"{ate_cm:.2f} cm" if ate_cm is not None else "N/A"

    max_ate_cm = traj_stats.get("max_ate_cm")
    if max_ate_cm is None and "ate_max_meters" in grid_metrics:
        max_ate_cm = grid_metrics["ate_max_meters"] * 100.0
    max_ate_str = f"{max_ate_cm:.2f} cm" if max_ate_cm is not None else "N/A"

    chamfer_m = grid_metrics.get("chamfer_distance_meters")
    chamfer_str = f"{chamfer_m * 100.0:.2f} cm" if chamfer_m is not None else "N/A"

    f1_val = grid_metrics.get("f1_score")
    f1_str = f"{f1_val * 100.0:.1f} %" if f1_val is not None else "N/A"

    prec_val = grid_metrics.get("precision")
    rec_val = grid_metrics.get("recall")
    pr_str = f"{prec_val * 100.0:.1f} % / {rec_val * 100.0:.1f} %" if (prec_val is not None and rec_val is not None) else "N/A"

    iou_val = grid_metrics.get("obstacle_iou")
    iou_str = f"{iou_val * 100.0:.1f} %" if iou_val is not None else "N/A"

    rooms_str = f"{max(1, kg_stats['rooms'])} Rooms"
    doors_str = f"{kg_stats['doorways']} Doorways"
    entities_str = f"{kg_stats.get('entities', 0)} Detected (Chair/Wall)"

    w = 68
    print("\n" + "=" * w)
    print("         AUTONOMOUS SLAM & EXPLORATION: ACTIVE RUN RESULTS")
    print("=" * w)
    print(f"{'Metric':<40} | {'Active Run Result':<24}")
    print("-" * w)

    rows = [
        ("Total Distance Traveled",           dist_str),
        ("Explored Grid Coverage",            explored_str),
        ("Local ATE RMSE (Trajectory Error)", ate_str),
        ("Peak Trajectory Drift (Max ATE)",   max_ate_str),
        ("Mean Wall Chamfer Error",           chamfer_str),
        ("Obstacle Map F1 Score",             f1_str),
        ("Map Precision / Recall",            pr_str),
        ("Obstacle IoU",                      iou_str),
        ("Topological Rooms Discovered",      rooms_str),
        ("Doorway Chokepoints Identified",    doors_str),
        ("Semantic Entities Mapped",          entities_str),
    ]

    for name, val in rows:
        print(f"{name:<40} | {val:<24}")

    print("=" * w + "\n", flush=True)


if __name__ == "__main__":
    print_comparison_benchmark()
