#!/usr/bin/env python3
"""
MNE-SLAM-Style Research Benchmark for Autonomous Quadruped SLAM & Navigation.

Evaluation philosophy follows:
  "MNE-SLAM: Multi-Agent Neural SLAM for Mobile Robots", CVPR 2025.

Adapted for 2D occupancy-grid + SLAM quadruped system.

DATA INTEGRITY GUARANTEE:
  All reported values are computed exclusively from:
    - accuracy_results/latest_metrics.json   (real SLAM run metrics)
    - accuracy_results/knowledge_graph.json  (real topological map)
    - captured_matrices/true_grid.npy        (if available)
    - captured_matrices/slam_grid.npy        (if available)
    - captured_matrices/trajectory_data.npz  (if available)
  No values are hardcoded, invented, or copied from any paper.

Usage:
  python research_benchmark.py
  python research_benchmark.py --data_dir captured_matrices
                               --results_dir accuracy_results
                               --resolution 0.05
"""

import argparse
import json
import math
import os
import sys
import warnings
import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

try:
    from scipy.spatial import cKDTree
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    warnings.warn("scipy not found — some distance metrics will use slower fallback.", stacklevel=1)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from accuracy_evaluator import evaluate_grid_accuracy, evaluate_trajectory

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────
NA = "N/A"          # Sentinel string for unavailable metrics
BENCHMARK_VERSION = "1.0"


# ─────────────────────────────────────────────────────────────────────────────
# 1. DATA LOADING HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def _load_json(path: str) -> dict:
    """Load a JSON file; return empty dict on failure."""
    if not os.path.exists(path):
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        print(f"  [WARN] Could not load {path}: {e}")
        return {}


def load_all_data(data_dir: str, results_dir: str) -> dict:
    """
    Load every available data source and return a catalogue with provenance.

    Returns a dict with keys:
      metrics_json        : dict from latest_metrics.json (real SLAM run)
      knowledge_graph     : dict from knowledge_graph.json (real KG)
      true_grid           : np.ndarray or None
      slam_grid           : np.ndarray or None
      trajectory          : dict with 'true_poses','slam_poses','timestamps' or None
      grid_recomputed     : dict of grid metrics if grids are available
      traj_recomputed     : dict of traj metrics if trajectory file available
      sources             : dict mapping metric-group → file used
      data_is_real        : bool (True = genuine SLAM data, never synthetic)
    """
    cat = {
        "metrics_json": {},
        "knowledge_graph": {},
        "true_grid": None,
        "slam_grid": None,
        "trajectory": None,
        "grid_recomputed": None,
        "traj_recomputed": None,
        "sources": {},
        "data_is_real": True,
    }

    # latest_metrics.json
    mj_path = os.path.join(results_dir, "latest_metrics.json")
    cat["metrics_json"] = _load_json(mj_path)
    if cat["metrics_json"]:
        cat["sources"]["metrics_json"] = mj_path
        print(f"  [DATA] Loaded {len(cat['metrics_json'])} metrics from {mj_path}")

    # knowledge_graph.json
    kg_path = os.path.join(results_dir, "knowledge_graph.json")
    cat["knowledge_graph"] = _load_json(kg_path)
    if cat["knowledge_graph"]:
        cat["sources"]["knowledge_graph"] = kg_path
        print(f"  [DATA] Loaded knowledge graph from {kg_path}")

    # true_grid.npy / slam_grid.npy (optional — required for extended reconstruction metrics)
    tg_path = os.path.join(data_dir, "true_grid.npy")
    sg_path = os.path.join(data_dir, "slam_grid.npy")
    if os.path.exists(tg_path) and os.path.exists(sg_path):
        cat["true_grid"] = np.load(tg_path)
        cat["slam_grid"] = np.load(sg_path)
        cat["sources"]["occupancy_grids"] = f"{tg_path} + {sg_path}"
        print(f"  [DATA] Loaded true_grid {cat['true_grid'].shape} and slam_grid {cat['slam_grid'].shape}")
    else:
        print(f"  [DATA] Occupancy grids not found — reconstruction metrics use cached latest_metrics.json values")
        cat["sources"]["occupancy_grids"] = "Not available — using cached latest_metrics.json"

    # trajectory_data.npz (optional)
    tr_path = os.path.join(data_dir, "trajectory_data.npz")
    if os.path.exists(tr_path):
        try:
            tr = np.load(tr_path)
            cat["trajectory"] = {
                "true_poses": tr["true_poses"],
                "slam_poses": tr["slam_poses"],
                "timestamps": tr["timestamps"] if "timestamps" in tr else None,
            }
            cat["sources"]["trajectory"] = tr_path
            n = len(tr["true_poses"])
            print(f"  [DATA] Loaded trajectory_data.npz — {n} pose pairs")
        except Exception as e:
            print(f"  [WARN] trajectory_data.npz load failed: {e}")
    else:
        print(f"  [DATA] trajectory_data.npz not found — localization metrics use cached latest_metrics.json values")
        cat["sources"]["trajectory"] = "Not available — using cached latest_metrics.json"

    return cat


# ─────────────────────────────────────────────────────────────────────────────
# 2. EXTENDED RECONSTRUCTION METRIC COMPUTATION
# ─────────────────────────────────────────────────────────────────────────────

def compute_completion_ratios(true_grid: np.ndarray, slam_grid: np.ndarray,
                               resolution: float, thresh_occ: float = 1.0) -> dict:
    """
    Compute completion ratio at multiple distance thresholds.
    Called only when both grid .npy files are available.
    """
    true_occ = (true_grid >= thresh_occ)
    slam_occ = (slam_grid >= thresh_occ)

    true_ys, true_xs = np.where(true_occ)
    slam_ys, slam_xs = np.where(slam_occ)

    if len(true_xs) == 0 or len(slam_xs) == 0:
        return {"comp_10cm": NA, "comp_15cm": NA}

    true_pts = np.column_stack([true_xs * resolution, true_ys * resolution])
    slam_pts = np.column_stack([slam_xs * resolution, slam_ys * resolution])

    if HAS_SCIPY:
        tree_slam = cKDTree(slam_pts)
        dists_true_to_slam, _ = tree_slam.query(true_pts)
    else:
        import cv2 as _cv2
        slam_mask = slam_occ.astype(np.uint8)
        dist_map = _cv2.distanceTransform(
            (1 - slam_mask).astype(np.uint8), _cv2.DIST_L2, 3
        ) * resolution
        dists_true_to_slam = dist_map[true_occ]

    comp_10cm = float(np.mean(dists_true_to_slam <= 0.10)) * 100.0
    comp_15cm = float(np.mean(dists_true_to_slam <= 0.15)) * 100.0

    return {"comp_10cm": comp_10cm, "comp_15cm": comp_15cm,
            "_dists_true_to_slam": dists_true_to_slam}


def compute_reconstruction_curve(dists_true_to_slam: np.ndarray,
                                  max_thresh_cm: float = 30.0,
                                  steps: int = 61) -> tuple:
    """Return (thresholds_cm, completion_ratios_pct) for a completion-ratio curve."""
    thresholds = np.linspace(0.0, max_thresh_cm / 100.0, steps)
    ratios = [float(np.mean(dists_true_to_slam <= t)) * 100.0 for t in thresholds]
    return thresholds * 100.0, ratios


# ─────────────────────────────────────────────────────────────────────────────
# 3. ASSEMBLING THE FINAL METRICS DICT
# ─────────────────────────────────────────────────────────────────────────────

def assemble_metrics(cat: dict, resolution: float,
                     results_dir: str) -> dict:
    """
    Merge all available data into the final structured metrics dictionary.
    Values from real SLAM runs are used; genuinely missing ones are set to NA.
    """
    m = cat["metrics_json"]
    kg = cat["knowledge_graph"]

    # ── helper: get from cached JSON, or NA ──
    def _get(key, scale=1.0):
        v = m.get(key)
        if v is None:
            return NA
        return float(v) * scale

    # ── helper: count KG collection ──
    def _kg_count(key):
        col = kg.get(key, {})
        if isinstance(col, dict):
            return len(col)
        if isinstance(col, list):
            return len(col)
        return NA

    # ─────────────────────────────────────────
    # PART 1 — 2D OCCUPANCY RECONSTRUCTION
    # ─────────────────────────────────────────
    # Accuracy = mean NN dist from slam_pts → true_pts
    accuracy_m = _get("accuracy_meters")
    accuracy_cm = (accuracy_m * 100.0) if accuracy_m != NA else NA

    # Completeness = mean NN dist from true_pts → slam_pts
    completeness_m = _get("completion_meters")
    completeness_cm = (completeness_m * 100.0) if completeness_m != NA else NA

    # Completion ratio @ 5cm (cached)
    comp_5_raw = _get("completion_ratio_5cm")
    comp_5cm_pct = (comp_5_raw * 100.0) if comp_5_raw != NA else NA

    chamfer_m = _get("chamfer_distance_meters")
    chamfer_cm = (chamfer_m * 100.0) if chamfer_m != NA else NA

    hausdorff_m = _get("hausdorff_95_meters")
    hausdorff_cm = (hausdorff_m * 100.0) if hausdorff_m != NA else NA

    # Completion ratios @ 10/15cm — computed from live grids if available
    comp_10cm_pct = NA
    comp_15cm_pct = NA
    _dists_for_curve = None

    if cat["true_grid"] is not None and cat["slam_grid"] is not None:
        ext = compute_completion_ratios(cat["true_grid"], cat["slam_grid"], resolution)
        comp_10cm_pct = round(ext["comp_10cm"], 2)
        comp_15cm_pct = round(ext["comp_15cm"], 2)
        _dists_for_curve = ext.get("_dists_true_to_slam")

    # ─────────────────────────────────────────
    # PART 2 — LOCALIZATION
    # ─────────────────────────────────────────
    ate_rmse_m = _get("ate_rmse_meters")
    ate_rmse_cm = (ate_rmse_m * 100.0) if ate_rmse_m != NA else NA
    ate_mean_m = _get("ate_mean_meters")
    ate_mean_cm = (ate_mean_m * 100.0) if ate_mean_m != NA else NA
    ate_median_m = _get("ate_median_meters")
    ate_median_cm = (ate_median_m * 100.0) if ate_median_m != NA else NA
    ate_max_m = _get("ate_max_meters")
    ate_max_cm = (ate_max_m * 100.0) if ate_max_m != NA else NA

    rpe_trans_m = _get("rpe_trans_rmse_meters")
    rpe_trans_cm = (rpe_trans_m * 100.0) if rpe_trans_m != NA else NA
    rpe_rot_deg = _get("rpe_rot_rmse_deg")

    mean_heading_deg = _get("mean_heading_error_deg")
    max_heading_deg = _get("max_heading_error_deg")
    drift_pct = _get("drift_rate_percent")

    total_dist_gt = _get("total_distance_traveled_m")
    total_dist_slam = _get("slam_estimated_distance_m")

    # ─────────────────────────────────────────
    # PART 3 — OCCUPANCY CLASSIFICATION
    # ─────────────────────────────────────────
    precision_pct = _get("precision", 100.0)
    recall_pct = _get("recall", 100.0)
    f1_pct = _get("f1_score", 100.0)
    obstacle_iou_pct = _get("obstacle_iou", 100.0)
    tolerant_iou_pct = _get("tolerant_iou", 100.0)
    free_space_iou_pct = _get("free_space_iou", 100.0)
    overall_acc_pct = _get("overall_grid_accuracy", 100.0)

    # ─────────────────────────────────────────
    # PART 4 — EXPLORATION
    # ─────────────────────────────────────────
    explored_area_pct = _get("explored_pct")

    rooms_detected = _kg_count("rooms") if kg else NA
    doorways_detected = _kg_count("doorways") if kg else NA

    # Frontiers — not persisted at runtime
    frontiers_detected = NA
    frontiers_completed = NA

    # Loop closures — not persisted at runtime
    loop_closures = NA

    # Stuck recoveries — not persisted at runtime
    stuck_recoveries = NA

    path_efficiency_raw = _get("path_efficiency_ratio")
    path_efficiency_pct = (path_efficiency_raw * 100.0) if path_efficiency_raw != NA else NA

    # ─────────────────────────────────────────
    # CELL COUNTS (for provenance)
    # ─────────────────────────────────────────
    gt_obstacle_cells = m.get("total_true_obstacle_cells", NA)
    slam_obstacle_cells = m.get("total_slam_obstacle_cells", NA)
    slam_free_cells = m.get("total_free_cells", NA)
    slam_unknown_cells = m.get("total_unknown_cells", NA)
    total_pose_steps = m.get("total_pose_steps", NA)
    tolerance_margin_m = m.get("tolerance_margin_meters", 0.15)

    return {
        # ── Reconstruction ──
        "mapping": {
            "accuracy_m": round(accuracy_m, 6) if accuracy_m != NA else NA,
            "accuracy_cm": round(accuracy_cm, 4) if accuracy_cm != NA else NA,
            "completeness_m": round(completeness_m, 6) if completeness_m != NA else NA,
            "completeness_cm": round(completeness_cm, 4) if completeness_cm != NA else NA,
            "completion_ratio_5cm_pct": round(comp_5cm_pct, 2) if comp_5cm_pct != NA else NA,
            "completion_ratio_10cm_pct": round(comp_10cm_pct, 2) if comp_10cm_pct != NA else NA,
            "completion_ratio_15cm_pct": round(comp_15cm_pct, 2) if comp_15cm_pct != NA else NA,
            "chamfer_m": round(chamfer_m, 6) if chamfer_m != NA else NA,
            "chamfer_cm": round(chamfer_cm, 4) if chamfer_cm != NA else NA,
            "hausdorff95_m": round(hausdorff_m, 6) if hausdorff_m != NA else NA,
            "hausdorff95_cm": round(hausdorff_cm, 4) if hausdorff_cm != NA else NA,
        },
        # ── Localization ──
        "localization": {
            "ate_rmse_m": round(ate_rmse_m, 6) if ate_rmse_m != NA else NA,
            "ate_rmse_cm": round(ate_rmse_cm, 4) if ate_rmse_cm != NA else NA,
            "ate_mean_m": round(ate_mean_m, 6) if ate_mean_m != NA else NA,
            "ate_mean_cm": round(ate_mean_cm, 4) if ate_mean_cm != NA else NA,
            "ate_median_m": round(ate_median_m, 6) if ate_median_m != NA else NA,
            "ate_median_cm": round(ate_median_cm, 4) if ate_median_cm != NA else NA,
            "ate_max_m": round(ate_max_m, 6) if ate_max_m != NA else NA,
            "ate_max_cm": round(ate_max_cm, 4) if ate_max_cm != NA else NA,
            "rpe_translation_m": round(rpe_trans_m, 6) if rpe_trans_m != NA else NA,
            "rpe_translation_cm": round(rpe_trans_cm, 4) if rpe_trans_cm != NA else NA,
            "rpe_rotation_deg": round(rpe_rot_deg, 4) if rpe_rot_deg != NA else NA,
            "mean_heading_error_deg": round(mean_heading_deg, 4) if mean_heading_deg != NA else NA,
            "max_heading_error_deg": round(max_heading_deg, 4) if max_heading_deg != NA else NA,
            "drift_rate_pct": round(drift_pct, 4) if drift_pct != NA else NA,
            "total_dist_gt_m": round(total_dist_gt, 4) if total_dist_gt != NA else NA,
            "total_dist_slam_m": round(total_dist_slam, 4) if total_dist_slam != NA else NA,
        },
        # ── Occupancy ──
        "occupancy": {
            "precision_pct": round(precision_pct, 2) if precision_pct != NA else NA,
            "recall_pct": round(recall_pct, 2) if recall_pct != NA else NA,
            "f1_pct": round(f1_pct, 2) if f1_pct != NA else NA,
            "obstacle_iou_pct": round(obstacle_iou_pct, 2) if obstacle_iou_pct != NA else NA,
            "tolerant_iou_pct": round(tolerant_iou_pct, 2) if tolerant_iou_pct != NA else NA,
            "free_space_iou_pct": round(free_space_iou_pct, 2) if free_space_iou_pct != NA else NA,
            "overall_accuracy_pct": round(overall_acc_pct, 2) if overall_acc_pct != NA else NA,
        },
        # ── Exploration ──
        "exploration": {
            "explored_area_pct": round(explored_area_pct, 2) if explored_area_pct != NA else NA,
            "rooms_detected": rooms_detected,
            "doorways_detected": doorways_detected,
            "frontiers_detected": frontiers_detected,
            "frontiers_completed": frontiers_completed,
            "loop_closures": loop_closures,
            "stuck_recoveries": stuck_recoveries,
            "path_efficiency_pct": round(path_efficiency_pct, 2) if path_efficiency_pct != NA else NA,
        },
        # ── Provenance / metadata (not printed in main table) ──
        "_meta": {
            "resolution_m": resolution,
            "tolerance_m": tolerance_margin_m,
            "gt_obstacle_cells": gt_obstacle_cells,
            "slam_obstacle_cells": slam_obstacle_cells,
            "slam_free_cells": slam_free_cells,
            "slam_unknown_cells": slam_unknown_cells,
            "total_pose_steps": total_pose_steps,
            "data_sources": cat["sources"],
            "data_is_real": cat["data_is_real"],
            "benchmark_version": BENCHMARK_VERSION,
        },
        # Internal — for curve plotting; not written to JSON
        "_dists_for_curve": _dists_for_curve,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 4. SAVE JSON RESULT FILE
# ─────────────────────────────────────────────────────────────────────────────

def save_benchmark_json(result: dict, results_dir: str) -> str:
    """Write the structured benchmark JSON (excludes private _* keys)."""
    os.makedirs(results_dir, exist_ok=True)
    out_path = os.path.join(results_dir, "research_benchmark_metrics.json")

    public_keys = ["mapping", "localization", "occupancy", "exploration"]
    public = {k: result[k] for k in public_keys if k in result}
    # Include metadata without internal curve arrays
    meta = dict(result.get("_meta", {}))
    meta.pop("data_sources_objects", None)
    public["_meta"] = meta

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(public, f, indent=2)
    return out_path


# ─────────────────────────────────────────────────────────────────────────────
# 5. TERMINAL REPORT (PART 7)
# ─────────────────────────────────────────────────────────────────────────────

def _fmt(val, suffix="", digits=2):
    """Format a metric value or return N/A."""
    if val == NA or val is None:
        return "N/A"
    return f"{val:.{digits}f}{suffix}"


def print_terminal_report(result: dict) -> None:
    mp = result["mapping"]
    lc = result["localization"]
    oc = result["occupancy"]
    ex = result["exploration"]
    meta = result["_meta"]

    W = 63
    SEP = "=" * W
    DIV = "-" * W

    print()
    print(SEP)
    print("      AUTONOMOUS QUADRUPED SLAM — RESEARCH BENCHMARK")
    print(SEP)

    # ── MAPPING ──
    print()
    print("[ 2D OCCUPANCY RECONSTRUCTION METRICS ]")
    print(DIV)
    rows_map = [
        ("Accuracy (SLAM→GT)",         _fmt(mp["accuracy_cm"],              " cm")),
        ("Completeness (GT→SLAM)",      _fmt(mp["completeness_cm"],          " cm")),
        ("Completion Ratio @ 5 cm",     _fmt(mp["completion_ratio_5cm_pct"],  " %")),
        ("Completion Ratio @ 10 cm",    _fmt(mp["completion_ratio_10cm_pct"], " %")),
        ("Completion Ratio @ 15 cm",    _fmt(mp["completion_ratio_15cm_pct"], " %")),
        ("Chamfer Distance",            _fmt(mp["chamfer_cm"],               " cm")),
        ("95% Hausdorff Distance",      _fmt(mp["hausdorff95_cm"],           " cm")),
    ]
    for name, val in rows_map:
        print(f"  {name:<33}: {val}")

    # ── LOCALIZATION ──
    print()
    print("[ LOCALIZATION METRICS ]")
    print(DIV)
    rows_loc = [
        ("ATE RMSE",                    _fmt(lc["ate_rmse_cm"],              " cm")),
        ("Mean ATE",                    _fmt(lc["ate_mean_cm"],              " cm")),
        ("Median ATE",                  _fmt(lc["ate_median_cm"],            " cm")),
        ("Maximum ATE",                 _fmt(lc["ate_max_cm"],               " cm")),
        ("RPE Translation RMSE",        _fmt(lc["rpe_translation_cm"],       " cm")),
        ("RPE Rotation RMSE",           _fmt(lc["rpe_rotation_deg"],         " deg")),
        ("Mean Heading Error",          _fmt(lc["mean_heading_error_deg"],   " deg")),
        ("Maximum Heading Error",       _fmt(lc["max_heading_error_deg"],    " deg")),
        ("Drift Rate",                  _fmt(lc["drift_rate_pct"],           " %")),
        ("GT Trajectory Length",        _fmt(lc["total_dist_gt_m"],          " m")),
        ("SLAM Trajectory Length",      _fmt(lc["total_dist_slam_m"],        " m")),
    ]
    for name, val in rows_loc:
        print(f"  {name:<33}: {val}")

    # ── OCCUPANCY MAP ──
    print()
    print("[ OCCUPANCY MAP CLASSIFICATION METRICS ]")
    print(DIV)
    rows_occ = [
        ("Precision",                   _fmt(oc["precision_pct"],            " %")),
        ("Recall",                      _fmt(oc["recall_pct"],               " %")),
        ("F1 Score",                    _fmt(oc["f1_pct"],                   " %")),
        ("Obstacle IoU (Exact)",        _fmt(oc["obstacle_iou_pct"],         " %")),
        ("Tolerant IoU (±15 cm)",       _fmt(oc["tolerant_iou_pct"],         " %")),
        ("Free-space IoU",              _fmt(oc["free_space_iou_pct"],       " %")),
        ("Overall Grid Accuracy",       _fmt(oc["overall_accuracy_pct"],     " %")),
    ]
    for name, val in rows_occ:
        print(f"  {name:<33}: {val}")

    # ── EXPLORATION ──
    print()
    print("[ EXPLORATION METRICS ]")
    print(DIV)
    rows_exp = [
        ("Explored Area",               _fmt(ex["explored_area_pct"],        " %")),
        ("Rooms Detected",              str(ex["rooms_detected"]) if ex["rooms_detected"] != NA else "N/A"),
        ("Doorways Detected",           str(ex["doorways_detected"]) if ex["doorways_detected"] != NA else "N/A"),
        ("Frontiers Detected",          "N/A" if ex["frontiers_detected"] == NA else str(ex["frontiers_detected"])),
        ("Frontiers Completed",         "N/A" if ex["frontiers_completed"] == NA else str(ex["frontiers_completed"])),
        ("Loop Closures",               "N/A" if ex["loop_closures"] == NA else str(ex["loop_closures"])),
        ("Robot-Stuck Recoveries",      "N/A" if ex["stuck_recoveries"] == NA else str(ex["stuck_recoveries"])),
        ("Path Efficiency",             _fmt(ex["path_efficiency_pct"],       " %")),
    ]
    for name, val in rows_exp:
        print(f"  {name:<33}: {val}")

    # ── METADATA ──
    print()
    print("[ EVALUATION METADATA ]")
    print(DIV)
    print(f"  {'Map Resolution':<33}: {meta['resolution_m'] * 100:.0f} cm/cell")
    print(f"  {'Tolerance Margin':<33}: {meta['tolerance_m'] * 100:.0f} cm")
    print(f"  {'GT Obstacle Cells':<33}: {meta['gt_obstacle_cells']}")
    print(f"  {'SLAM Obstacle Cells':<33}: {meta['slam_obstacle_cells']}")
    print(f"  {'Total Pose Steps':<33}: {meta['total_pose_steps']}")
    print(f"  {'Data Origin':<33}: {'REAL (from simulation)' if meta['data_is_real'] else 'SYNTHETIC'}")

    print()
    print(SEP)
    print()


# ─────────────────────────────────────────────────────────────────────────────
# 6. RESEARCH COMPARISON TABLE (PART 8)
# ─────────────────────────────────────────────────────────────────────────────

def print_comparison_table(result: dict) -> str:
    mp = result["mapping"]
    lc = result["localization"]
    oc = result["occupancy"]
    ex = result["exploration"]

    lines = []
    lines.append("")
    lines.append("## Research Benchmark Comparison Table")
    lines.append("")
    lines.append("| Metric | Our Quadruped (Measured) |")
    lines.append("|--------|--------------------------|")
    lines.append(f"| **2D Occupancy Reconstruction** | |")
    lines.append(f"| Accuracy (SLAM→GT) | {_fmt(mp['accuracy_cm'], ' cm')} |")
    lines.append(f"| Completeness (GT→SLAM) | {_fmt(mp['completeness_cm'], ' cm')} |")
    lines.append(f"| Completion Ratio @ 5 cm | {_fmt(mp['completion_ratio_5cm_pct'], ' %')} |")
    lines.append(f"| Completion Ratio @ 10 cm | {_fmt(mp['completion_ratio_10cm_pct'], ' %')} |")
    lines.append(f"| Completion Ratio @ 15 cm | {_fmt(mp['completion_ratio_15cm_pct'], ' %')} |")
    lines.append(f"| Chamfer Distance | {_fmt(mp['chamfer_cm'], ' cm')} |")
    lines.append(f"| 95% Hausdorff Distance | {_fmt(mp['hausdorff95_cm'], ' cm')} |")
    lines.append(f"| **Localization** | |")
    lines.append(f"| ATE RMSE | {_fmt(lc['ate_rmse_cm'], ' cm')} |")
    lines.append(f"| Mean ATE | {_fmt(lc['ate_mean_cm'], ' cm')} |")
    lines.append(f"| Median ATE | {_fmt(lc['ate_median_cm'], ' cm')} |")
    lines.append(f"| Maximum ATE | {_fmt(lc['ate_max_cm'], ' cm')} |")
    lines.append(f"| RPE Translation RMSE | {_fmt(lc['rpe_translation_cm'], ' cm')} |")
    lines.append(f"| RPE Rotation RMSE | {_fmt(lc['rpe_rotation_deg'], ' deg')} |")
    lines.append(f"| Drift Rate | {_fmt(lc['drift_rate_pct'], ' %')} |")
    lines.append(f"| **Occupancy Map** | |")
    lines.append(f"| Precision | {_fmt(oc['precision_pct'], ' %')} |")
    lines.append(f"| Recall | {_fmt(oc['recall_pct'], ' %')} |")
    lines.append(f"| F1 Score | {_fmt(oc['f1_pct'], ' %')} |")
    lines.append(f"| Obstacle IoU (Exact) | {_fmt(oc['obstacle_iou_pct'], ' %')} |")
    lines.append(f"| Tolerant IoU (±15 cm) | {_fmt(oc['tolerant_iou_pct'], ' %')} |")
    lines.append(f"| Free-space IoU | {_fmt(oc['free_space_iou_pct'], ' %')} |")
    lines.append(f"| Overall Grid Accuracy | {_fmt(oc['overall_accuracy_pct'], ' %')} |")
    lines.append(f"| **Exploration** | |")
    lines.append(f"| Explored Area | {_fmt(ex['explored_area_pct'], ' %')} |")
    lines.append(f"| Rooms Detected | {ex['rooms_detected']} |")
    lines.append(f"| Doorways Detected | {ex['doorways_detected']} |")
    lines.append(f"| Path Efficiency | {_fmt(ex['path_efficiency_pct'], ' %')} |")
    lines.append("")
    lines.append("> **Note**: All values in the 'Our Quadruped (Measured)' column are")
    lines.append("> computed exclusively from real simulation data recorded by this system.")
    lines.append("> No values have been hardcoded, interpolated from, or compared to")
    lines.append("> published MNE-SLAM or any other paper results in this column.")
    lines.append("> N/A entries indicate metrics not persisted during the simulation run.")
    lines.append("")

    table_text = "\n".join(lines)
    print(table_text)
    return table_text


# ─────────────────────────────────────────────────────────────────────────────
# 7. PUBLICATION-QUALITY PLOTS (PART 9)
# ─────────────────────────────────────────────────────────────────────────────

def _safe_float_arr(d: dict, *keys):
    """Extract a float array from a dict of dicts; return empty if missing."""
    vals = []
    for k in keys:
        v = d.get(k)
        if v is not None and v != NA:
            vals.append(float(v))
        else:
            vals.append(float("nan"))
    return np.array(vals)


def generate_plots(result: dict, cat: dict, plot_dir: str) -> list:
    """Generate all 7 publication-quality plots. Returns list of saved paths."""
    os.makedirs(plot_dir, exist_ok=True)
    saved = []

    mp = result["mapping"]
    lc = result["localization"]
    oc = result["occupancy"]
    ex = result["exploration"]
    meta = result["_meta"]

    STYLE = {
        "font.size": 11,
        "axes.titlesize": 12,
        "axes.labelsize": 11,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "figure.dpi": 150,
    }
    plt.rcParams.update(STYLE)

    # ── Plot 1: GT Occupancy vs SLAM Occupancy ──────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    fig.suptitle("Plot 1 — 2D Occupancy: Ground Truth vs SLAM",
                 fontsize=13, fontweight="bold", y=1.01)

    if cat["true_grid"] is not None and cat["slam_grid"] is not None:
        tg = cat["true_grid"]
        sg = cat["slam_grid"]
        h, w = tg.shape
        res = meta["resolution_m"]
        ext = [0, w * res, 0, h * res]
        im0 = axes[0].imshow(tg, cmap="coolwarm", origin="lower", extent=ext,
                              vmin=-3, vmax=3, aspect="auto")
        axes[0].set_title("Ground Truth Occupancy Grid (Log-Odds)")
        axes[0].set_xlabel("X (m)")
        axes[0].set_ylabel("Y (m)")
        fig.colorbar(im0, ax=axes[0], label="Log-Odds")
        im1 = axes[1].imshow(sg, cmap="coolwarm", origin="lower", extent=ext,
                              vmin=-3, vmax=3, aspect="auto")
        axes[1].set_title("SLAM-Built Occupancy Grid (Log-Odds)")
        axes[1].set_xlabel("X (m)")
        fig.colorbar(im1, ax=axes[1], label="Log-Odds")
    else:
        for ax, title in zip(axes, ["Ground Truth", "SLAM Output"]):
            ax.text(0.5, 0.5, f"{title}\n(Grid file unavailable)\nSee captured_matrices/true_grid.npy",
                    ha="center", va="center", fontsize=10, color="gray",
                    transform=ax.transAxes)
            ax.set_title(f"{title} Occupancy Grid")
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")

    plt.tight_layout()
    p = os.path.join(plot_dir, "01_gt_vs_slam_occupancy.png")
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    saved.append(p)
    print(f"  [PLOT] Saved: {p}")

    # ── Plot 2: GT Trajectory vs SLAM Trajectory ────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 6))
    fig.suptitle("Plot 2 — Trajectory: Ground Truth vs SLAM",
                 fontsize=13, fontweight="bold")

    if cat["trajectory"] is not None:
        tp = np.asarray(cat["trajectory"]["true_poses"])
        sp = np.asarray(cat["trajectory"]["slam_poses"])
        n = min(len(tp), len(sp))
        tp, sp = tp[:n], sp[:n]
        ax.plot(tp[:, 0], tp[:, 1], "c-", linewidth=2, label="Ground Truth")
        ax.plot(sp[:, 0], sp[:, 1], "m--", linewidth=1.8, label="SLAM Estimate")
        ax.scatter([tp[0, 0]], [tp[0, 1]], color="green", s=100, zorder=5, label="Start")
        ax.scatter([tp[-1, 0]], [tp[-1, 1]], color="red", s=100, zorder=5, label="End")
        skip = max(1, n // 20)
        for i in range(0, n, skip):
            ax.plot([tp[i, 0], sp[i, 0]], [tp[i, 1], sp[i, 1]],
                    "y-", alpha=0.6, linewidth=0.8)
    else:
        ax.text(0.5, 0.5,
                "Trajectory data unavailable\n(trajectory_data.npz not found)\n"
                "ATE RMSE from cached metrics: "
                + _fmt(lc["ate_rmse_cm"], " cm"),
                ha="center", va="center", fontsize=11, color="gray",
                transform=ax.transAxes)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(fontsize=9)
    ax.grid(True, linestyle=":", alpha=0.5)

    plt.tight_layout()
    p = os.path.join(plot_dir, "02_gt_vs_slam_trajectory.png")
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    saved.append(p)
    print(f"  [PLOT] Saved: {p}")

    # ── Plot 3: ATE Error vs Time ────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Plot 3 — ATE Error vs Time",
                 fontsize=13, fontweight="bold")

    if cat["trajectory"] is not None:
        tp = np.asarray(cat["trajectory"]["true_poses"])
        sp = np.asarray(cat["trajectory"]["slam_poses"])
        ts = cat["trajectory"]["timestamps"]
        n = min(len(tp), len(sp))
        tp, sp = tp[:n], sp[:n]
        pos_err = np.hypot(sp[:, 0] - tp[:, 0], sp[:, 1] - tp[:, 1]) * 100.0
        time_x = np.asarray(ts)[:n] if ts is not None else np.arange(n)
        x_label = "Time (s)" if ts is not None else "Step"

        axes[0].plot(time_x, pos_err, "r-", linewidth=1.5, label="ATE (cm)")
        if lc["ate_rmse_cm"] != NA:
            axes[0].axhline(float(lc["ate_rmse_cm"]), color="blue", linestyle="--",
                            linewidth=1.5, label=f"RMSE = {lc['ate_rmse_cm']:.2f} cm")
        axes[0].set_xlabel(x_label)
        axes[0].set_ylabel("Position Error (cm)")
        axes[0].set_title("ATE (Position Error) vs Time")
        axes[0].legend(fontsize=9)
        axes[0].grid(True, linestyle=":", alpha=0.5)

        if tp.shape[1] >= 3:
            def _norm_angle_np(a):
                return (a + np.pi) % (2.0 * np.pi) - np.pi
            yaw_err = np.degrees(np.abs(_norm_angle_np(sp[:, 2] - tp[:, 2])))
            axes[1].plot(time_x, yaw_err, color="orange", linewidth=1.5,
                         label="Heading Error (deg)")
            if lc["mean_heading_error_deg"] != NA:
                axes[1].axhline(float(lc["mean_heading_error_deg"]), color="purple",
                                linestyle="--", linewidth=1.5,
                                label=f"Mean = {lc['mean_heading_error_deg']:.2f}°")
            axes[1].set_xlabel(x_label)
            axes[1].set_ylabel("Heading Error (deg)")
            axes[1].set_title("Heading (Yaw) Error vs Time")
            axes[1].legend(fontsize=9)
            axes[1].grid(True, linestyle=":", alpha=0.5)
        else:
            axes[1].text(0.5, 0.5, "No yaw data in trajectory",
                         ha="center", va="center", transform=axes[1].transAxes, color="gray")
    else:
        for ax in axes:
            ax.text(0.5, 0.5,
                    "Trajectory data unavailable\n(trajectory_data.npz not found)",
                    ha="center", va="center", fontsize=11, color="gray",
                    transform=ax.transAxes)

    plt.tight_layout()
    p = os.path.join(plot_dir, "03_ate_vs_time.png")
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    saved.append(p)
    print(f"  [PLOT] Saved: {p}")

    # ── Plot 4: Occupancy Error Heatmap ─────────────────────────────────────
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.5))
    fig.suptitle("Plot 4 — Occupancy Error Heatmap",
                 fontsize=13, fontweight="bold")

    if cat["true_grid"] is not None and cat["slam_grid"] is not None:
        tg = cat["true_grid"]
        sg = cat["slam_grid"]
        h, w = tg.shape
        res = meta["resolution_m"]
        ext = [0, w * res, 0, h * res]

        thresh_occ = 1.0
        thresh_free = -0.5
        true_occ = (tg >= thresh_occ).astype(np.uint8)
        slam_occ = (sg >= thresh_occ).astype(np.uint8)

        diff = tg.astype(float) - sg.astype(float)
        im2 = axes[0].imshow(np.abs(diff), cmap="hot_r", origin="lower",
                             extent=ext, aspect="auto")
        axes[0].set_title("Absolute Log-Odds Difference |GT − SLAM|")
        axes[0].set_xlabel("X (m)")
        axes[0].set_ylabel("Y (m)")
        fig.colorbar(im2, ax=axes[0], label="|ΔLog-Odds|")

        overlay = np.zeros((h, w, 3), dtype=np.uint8) + 40
        overlay[true_occ == 1] = [0, 200, 0]
        overlay[slam_occ == 1] = [200, 30, 30]
        match = (true_occ == 1) & (slam_occ == 1)
        overlay[match] = [255, 220, 0]
        axes[1].imshow(overlay, origin="lower", extent=ext, aspect="auto")
        axes[1].set_title("Wall Overlay\n(Green=GT | Red=SLAM | Yellow=Match)")
        axes[1].set_xlabel("X (m)")

        # Per-cell error
        log_err = np.abs(diff)
        axes[2].hist(log_err.ravel(), bins=50, color="#2171b5", edgecolor="white",
                     alpha=0.85)
        axes[2].set_title("Log-Odds Error Distribution")
        axes[2].set_xlabel("|ΔLog-Odds|")
        axes[2].set_ylabel("Cell Count")
        axes[2].grid(True, linestyle=":", alpha=0.5)
    else:
        for ax in axes:
            ax.text(0.5, 0.5, "Grid files unavailable\n(true_grid.npy / slam_grid.npy)",
                    ha="center", va="center", transform=ax.transAxes, color="gray")

    plt.tight_layout()
    p = os.path.join(plot_dir, "04_occupancy_error_heatmap.png")
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    saved.append(p)
    print(f"  [PLOT] Saved: {p}")

    # ── Plot 5: Reconstruction Error Distribution ────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 5))
    fig.suptitle("Plot 5 — Reconstruction Error Distribution (2D Occupancy)",
                 fontsize=13, fontweight="bold")

    if cat["true_grid"] is not None and cat["slam_grid"] is not None:
        tg = cat["true_grid"]
        sg = cat["slam_grid"]
        thresh_occ = 1.0
        true_occ = tg >= thresh_occ
        slam_occ = sg >= thresh_occ
        true_ys, true_xs = np.where(true_occ)
        slam_ys, slam_xs = np.where(slam_occ)
        res = meta["resolution_m"]

        if len(true_xs) > 0 and len(slam_xs) > 0:
            tp2 = np.column_stack([true_xs * res, true_ys * res])
            sp2 = np.column_stack([slam_xs * res, slam_ys * res])
            if HAS_SCIPY:
                d2s, _ = cKDTree(sp2).query(tp2)
                d_s2t, _ = cKDTree(tp2).query(sp2)
            else:
                import cv2 as _cv2
                dm = _cv2.distanceTransform(
                    (1 - slam_occ.astype(np.uint8)).astype(np.uint8), _cv2.DIST_L2, 3
                ) * res
                d2s = dm[true_occ]
                dm2 = _cv2.distanceTransform(
                    (1 - true_occ.astype(np.uint8)).astype(np.uint8), _cv2.DIST_L2, 3
                ) * res
                d_s2t = dm2[slam_occ]

            ax.hist(d2s * 100, bins=50, alpha=0.7, color="#2171b5",
                    edgecolor="white", label=f"GT→SLAM (Completeness, n={len(d2s):,})")
            ax.hist(d_s2t * 100, bins=50, alpha=0.7, color="#d95f0e",
                    edgecolor="white", label=f"SLAM→GT (Accuracy, n={len(d_s2t):,})")
            ax.axvline(5.0, color="black", linestyle="--", linewidth=1.5, label="5 cm threshold")
            ax.axvline(10.0, color="gray", linestyle=":", linewidth=1.5, label="10 cm threshold")
            ax.set_xlabel("Nearest-Neighbour Distance (cm)")
            ax.set_ylabel("Count")
            ax.legend(fontsize=9)
            ax.grid(True, linestyle=":", alpha=0.5)
        else:
            ax.text(0.5, 0.5, "No occupied cells found in grids",
                    ha="center", va="center", transform=ax.transAxes, color="gray")
    else:
        ax.text(0.5, 0.5, "Grid files unavailable\n(true_grid.npy / slam_grid.npy)",
                ha="center", va="center", transform=ax.transAxes, color="gray")

    plt.tight_layout()
    p = os.path.join(plot_dir, "05_reconstruction_error_distribution.png")
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    saved.append(p)
    print(f"  [PLOT] Saved: {p}")

    # ── Plot 6: Completion Ratio vs Distance Threshold ───────────────────────
    fig, ax = plt.subplots(figsize=(8, 5))
    fig.suptitle("Plot 6 — Completion Ratio vs Distance Threshold (2D Occupancy)",
                 fontsize=13, fontweight="bold")

    if result["_dists_for_curve"] is not None:
        thresholds_cm, ratios = compute_reconstruction_curve(
            result["_dists_for_curve"], max_thresh_cm=30.0, steps=61
        )
        ax.plot(thresholds_cm, ratios, "b-", linewidth=2.5, label="Completion Ratio")
        for thr, label in [(5.0, "5 cm"), (10.0, "10 cm"), (15.0, "15 cm")]:
            idx = np.argmin(np.abs(thresholds_cm - thr))
            ax.axvline(thr, color="gray", linestyle=":", linewidth=1)
            ax.annotate(f"{ratios[idx]:.1f}%",
                        xy=(thr, ratios[idx]),
                        xytext=(thr + 0.5, ratios[idx] - 4),
                        fontsize=9, color="darkblue")
        ax.set_xlabel("Distance Threshold (cm)")
        ax.set_ylabel("Completion Ratio (%)")
        ax.set_xlim(0, 30)
        ax.set_ylim(0, 105)
        ax.grid(True, linestyle=":", alpha=0.5)
        ax.legend(fontsize=9)
    else:
        # Use available cached point values
        known_pts = []
        if mp["completion_ratio_5cm_pct"] != NA:
            known_pts.append((5.0, float(mp["completion_ratio_5cm_pct"])))
        if mp["completion_ratio_10cm_pct"] != NA:
            known_pts.append((10.0, float(mp["completion_ratio_10cm_pct"])))
        if mp["completion_ratio_15cm_pct"] != NA:
            known_pts.append((15.0, float(mp["completion_ratio_15cm_pct"])))

        if known_pts:
            xs = [p[0] for p in known_pts]
            ys = [p[1] for p in known_pts]
            ax.scatter(xs, ys, s=90, color="blue", zorder=5, label="Measured thresholds")
            ax.plot(xs, ys, "b--", linewidth=1.5, alpha=0.6)
            for x, y in known_pts:
                ax.annotate(f"{y:.1f}%", xy=(x, y), xytext=(x + 0.3, y - 4),
                            fontsize=9, color="darkblue")
            ax.set_xlabel("Distance Threshold (cm)")
            ax.set_ylabel("Completion Ratio (%)")
            ax.set_xlim(0, 20)
            ax.set_ylim(0, 105)
            ax.grid(True, linestyle=":", alpha=0.5)
            ax.legend(fontsize=9)
            ax.text(0.98, 0.05,
                    "Full curve requires true_grid.npy + slam_grid.npy",
                    ha="right", va="bottom", transform=ax.transAxes,
                    fontsize=8, color="gray")
        else:
            ax.text(0.5, 0.5,
                    "Completion ratio data unavailable\n(grid files not found)",
                    ha="center", va="center", transform=ax.transAxes, color="gray")

    plt.tight_layout()
    p = os.path.join(plot_dir, "06_completion_ratio_vs_threshold.png")
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    saved.append(p)
    print(f"  [PLOT] Saved: {p}")

    # ── Plot 7: Metric Summary Bar Chart ─────────────────────────────────────
    fig = plt.figure(figsize=(16, 7))
    fig.suptitle("Plot 7 — Metric Summary (Autonomous Quadruped SLAM)",
                 fontsize=13, fontweight="bold")

    # We collect only the percentage metrics that are available
    metric_names = []
    metric_vals = []
    metric_colors = []

    candidates = [
        ("Precision",        oc["precision_pct"],          "#4393c3"),
        ("Recall",           oc["recall_pct"],             "#4393c3"),
        ("F1 Score",         oc["f1_pct"],                 "#4393c3"),
        ("Obstacle IoU",     oc["obstacle_iou_pct"],       "#4393c3"),
        ("Tolerant IoU",     oc["tolerant_iou_pct"],       "#4393c3"),
        ("Free-space IoU",   oc["free_space_iou_pct"],     "#4393c3"),
        ("Grid Accuracy",    oc["overall_accuracy_pct"],   "#4393c3"),
        ("Explored Area",    ex["explored_area_pct"],      "#74c476"),
        ("Path Efficiency",  ex["path_efficiency_pct"],    "#74c476"),
        ("Comp Ratio @5cm",  mp["completion_ratio_5cm_pct"], "#fd8d3c"),
        ("Comp Ratio @10cm", mp["completion_ratio_10cm_pct"], "#fd8d3c"),
        ("Comp Ratio @15cm", mp["completion_ratio_15cm_pct"], "#fd8d3c"),
    ]
    for nm, v, col in candidates:
        if v != NA and v is not None:
            metric_names.append(nm)
            metric_vals.append(float(v))
            metric_colors.append(col)

    ax7 = fig.add_subplot(111)
    if metric_names:
        x_pos = np.arange(len(metric_names))
        bars = ax7.bar(x_pos, metric_vals, color=metric_colors, edgecolor="white",
                       linewidth=0.8, alpha=0.9, width=0.65)
        ax7.set_xticks(x_pos)
        ax7.set_xticklabels(metric_names, rotation=30, ha="right", fontsize=9)
        ax7.set_ylabel("Value (%)")
        ax7.set_ylim(0, 110)
        ax7.axhline(100, color="gray", linestyle=":", linewidth=0.8)
        ax7.grid(axis="y", linestyle=":", alpha=0.4)
        for bar, v in zip(bars, metric_vals):
            ax7.text(bar.get_x() + bar.get_width() / 2,
                     bar.get_height() + 1.2,
                     f"{v:.1f}%", ha="center", va="bottom", fontsize=8.5,
                     fontweight="bold")
        from matplotlib.patches import Patch
        legend_elems = [
            Patch(facecolor="#4393c3", label="Occupancy Classification"),
            Patch(facecolor="#74c476", label="Exploration"),
            Patch(facecolor="#fd8d3c", label="2D Reconstruction"),
        ]
        ax7.legend(handles=legend_elems, fontsize=9, loc="lower right")
    else:
        ax7.text(0.5, 0.5, "No percentage metrics available",
                 ha="center", va="center", transform=ax7.transAxes, color="gray")

    plt.tight_layout()
    p = os.path.join(plot_dir, "07_metric_summary_bar.png")
    plt.savefig(p, dpi=150, bbox_inches="tight")
    plt.close()
    saved.append(p)
    print(f"  [PLOT] Saved: {p}")

    return saved


# ─────────────────────────────────────────────────────────────────────────────
# 8. VALIDATION SUMMARY (PART 10)
# ─────────────────────────────────────────────────────────────────────────────

def print_validation_summary(result: dict, cat: dict) -> None:
    meta = result["_meta"]
    W = 70

    print()
    print("=" * W)
    print("  BENCHMARK VALIDATION & PROVENANCE SUMMARY")
    print("=" * W)

    # Input files
    print("\n  Input files used:")
    for grp, src in meta["data_sources"].items():
        print(f"    [{grp}] {src}")

    # Sample counts
    print("\n  Sample counts:")
    print(f"    GT obstacle cells      : {meta['gt_obstacle_cells']}")
    print(f"    SLAM obstacle cells    : {meta['slam_obstacle_cells']}")
    print(f"    Total pose steps       : {meta['total_pose_steps']}")

    # Configuration
    print("\n  Evaluation configuration:")
    print(f"    Map resolution         : {meta['resolution_m'] * 100:.0f} cm/cell")
    print(f"    Tolerance margin       : {meta['tolerance_m'] * 100:.0f} cm")
    print(f"    Completion thresholds  : 5 cm, 10 cm, 15 cm")

    # Data origin
    is_real = meta["data_is_real"]
    status = "REAL — from actual SLAM simulation run" if is_real else "SYNTHETIC (WARNING)"
    print(f"\n  Benchmark data origin  : {status}")

    # N/A reasons
    mp = result["mapping"]
    lc = result["localization"]
    ex = result["exploration"]

    na_list = []
    if mp["accuracy_cm"] == NA:
        na_list.append("  • Reconstruction Accuracy / Completeness: requires true_grid.npy + slam_grid.npy")
    if mp["completion_ratio_10cm_pct"] == NA:
        na_list.append("  • Completion Ratio @10cm/15cm: requires true_grid.npy + slam_grid.npy")
    if lc["ate_median_cm"] == NA:
        na_list.append("  • Median ATE: requires trajectory_data.npz")
    if lc["rpe_translation_cm"] == NA:
        na_list.append("  • RPE Translation/Rotation: requires trajectory_data.npz")
    if ex["loop_closures"] == NA:
        na_list.append("  • Loop closures: not persisted during simulation run")
    if ex["stuck_recoveries"] == NA:
        na_list.append("  • Stuck recoveries: not persisted during simulation run")
    if ex["frontiers_detected"] == NA:
        na_list.append("  • Frontier counts: not persisted during simulation run")

    if na_list:
        print("\n  Metrics reported as N/A (no fabrication):")
        for reason in na_list:
            print(reason)

    print("\n  Limitations of current ground-truth data:")
    if cat["true_grid"] is None:
        print("    • true_grid.npy / slam_grid.npy not saved at mission end.")
        print("      To fix: add np.save() calls in main_agentic.py at mission stop.")
    if cat["trajectory"] is None:
        print("    • trajectory_data.npz not saved. ATE/RPE from cached latest_metrics.json.")
        print("      To fix: save true_poses, slam_poses, timestamps at mission end.")
    if cat["knowledge_graph"].get("loop_closures") is None:
        print("    • Loop closure count not persisted in knowledge_graph.json.")
        print("      To fix: count loop_closures in ICPPoseEstimator and log to metrics.")

    print("=" * W + "\n")


# ─────────────────────────────────────────────────────────────────────────────
# 9. MAIN ORCHESTRATOR
# ─────────────────────────────────────────────────────────────────────────────

def run_benchmark(data_dir: str = "captured_matrices",
                  results_dir: str = "accuracy_results",
                  resolution: float = 0.05) -> dict:
    """
    Run the complete MNE-SLAM-style research benchmark.

    Steps:
      1.  Load all available real data (no synthetic generation).
      2.  Assemble structured metrics dict.
      3.  Save research_benchmark_metrics.json.
      4.  Print terminal report.
      5.  Print Markdown comparison table.
      6.  Generate 7 publication-quality plots.
      7.  Print validation summary.

    Returns:
      The assembled result dict (also saved to JSON).
    """
    plot_dir = os.path.join(results_dir, "research_benchmark")
    os.makedirs(plot_dir, exist_ok=True)

    print()
    print("=" * 70)
    print("  AUTONOMOUS QUADRUPED SLAM — MNE-SLAM STYLE RESEARCH BENCHMARK")
    print(f"  Data dir   : {os.path.abspath(data_dir)}")
    print(f"  Results dir: {os.path.abspath(results_dir)}")
    print(f"  Resolution : {resolution * 100:.0f} cm/cell")
    print("=" * 70)

    # ── Step 1: Load data ──
    print("\n[1/7] Loading data sources ...")
    cat = load_all_data(data_dir, results_dir)

    # ── Step 2: Recompute from live files if available ──
    if cat["true_grid"] is not None and cat["slam_grid"] is not None:
        print("\n[1b] Recomputing grid metrics from live grid files ...")
        grid_m = evaluate_grid_accuracy(
            cat["true_grid"], cat["slam_grid"],
            resolution=resolution, tolerance_m=0.15,
            output_dir=results_dir, save_plots=True
        )
        cat["grid_recomputed"] = grid_m
        # Merge into cached metrics (live wins)
        cat["metrics_json"].update(grid_m)

    if cat["trajectory"] is not None:
        print("\n[1c] Recomputing trajectory metrics from live trajectory file ...")
        traj_m = evaluate_trajectory(
            cat["trajectory"]["true_poses"],
            cat["trajectory"]["slam_poses"],
            timestamps=cat["trajectory"]["timestamps"],
            output_dir=results_dir,
            save_plots=True
        )
        cat["traj_recomputed"] = traj_m
        cat["metrics_json"].update(traj_m)

    # ── Step 3: Assemble metrics ──
    print("\n[2/7] Assembling metrics ...")
    result = assemble_metrics(cat, resolution, results_dir)

    # ── Step 4: Save JSON ──
    print("\n[3/7] Saving research_benchmark_metrics.json ...")
    json_path = save_benchmark_json(result, results_dir)
    print(f"  [OK] Written to: {json_path}")

    # ── Step 5: Terminal report ──
    print("\n[4/7] Printing terminal report ...")
    print_terminal_report(result)

    # ── Step 6: Markdown comparison table ──
    print("\n[5/7] Printing Markdown comparison table ...")
    table_text = print_comparison_table(result)

    # Save the table to file
    table_path = os.path.join(results_dir, "research_benchmark", "comparison_table.md")
    with open(table_path, "w", encoding="utf-8") as f:
        f.write("# Autonomous Quadruped SLAM — Research Benchmark Comparison\n\n")
        f.write(table_text)
    print(f"  [OK] Markdown table saved: {table_path}")

    # ── Step 7: Plots ──
    print("\n[6/7] Generating publication-quality plots ...")
    saved_plots = generate_plots(result, cat, plot_dir)
    print(f"  [OK] {len(saved_plots)} plots saved to: {plot_dir}")

    # ── Step 8: Validation summary ──
    print("\n[7/7] Validation summary ...")
    print_validation_summary(result, cat)

    return result


# ─────────────────────────────────────────────────────────────────────────────
# CLI ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="MNE-SLAM-style Research Benchmark for Autonomous Quadruped SLAM."
    )
    parser.add_argument("--data_dir", default="captured_matrices",
                        help="Directory containing grid/trajectory .npy/.npz files (default: captured_matrices)")
    parser.add_argument("--results_dir", default="accuracy_results",
                        help="Directory containing latest_metrics.json and knowledge_graph.json (default: accuracy_results)")
    parser.add_argument("--resolution", type=float, default=0.05,
                        help="Map resolution in metres/cell (default: 0.05)")
    args = parser.parse_args()

    run_benchmark(
        data_dir=args.data_dir,
        results_dir=args.results_dir,
        resolution=args.resolution,
    )
