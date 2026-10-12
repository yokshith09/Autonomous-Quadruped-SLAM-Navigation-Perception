"""
Numerical Accuracy & Evaluation Engine for Autonomous Robot SLAM and Perception.

This module evaluates:
1. Occupancy Grid Matrix Accuracy (True log-odds vs SLAM log-odds, IoU, Precision, Recall, F1, Chamfer/Hausdorff wall distance in meters).
2. Robot Trajectory & Movement Fidelity (ATE RMSE, RPE Drift Rate, Heading Error, Path Efficiency).
3. Perception & Depth Heatmap Analysis (Depth matrix distribution, valid obstacle band %, sensor clipping, colorimetry).
"""

import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
from typing import Dict, Tuple, List, Optional, Union

try:
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree = None


# =====================================================================
# 1. OCCUPANCY GRID & WALL DETECTION EVALUATION (MATRICES, NOT SCREENSHOTS)
# =====================================================================

def evaluate_grid_accuracy(
    true_grid: np.ndarray,
    slam_grid: np.ndarray,
    resolution: float = 0.05,
    thresh_occ: float = 1.0,
    thresh_free: float = -0.5,
    tolerance_m: float = 0.15,
    output_dir: str = 'accuracy_results',
    save_plots: bool = True
) -> Dict[str, float]:
    """
    Computes rigorous numerical accuracy metrics between two 2D occupancy grid matrices.
    
    Args:
        true_grid: (H, W) float32 array of ground-truth log-odds or occupancy states.
        slam_grid: (H, W) float32 array of SLAM log-odds or occupancy states.
        resolution: Meters per cell (default 0.05m = 5cm).
        thresh_occ: Log-odds threshold for OCCUPIED cells.
        thresh_free: Log-odds threshold for FREE cells.
        tolerance_m: Spatial tolerance margin in meters for wall alignment (e.g. 0.15m = 3 cells).
        output_dir: Folder to save evaluation plots.
        save_plots: Whether to generate and save visualization plots.
        
    Returns:
        Dictionary of computed metric values.
    """
    os.makedirs(output_dir, exist_ok=True)
    
    if true_grid.shape != slam_grid.shape:
        # Resize SLAM grid to match True grid if shapes differ
        slam_grid = cv2.resize(slam_grid, (true_grid.shape[1], true_grid.shape[0]), interpolation=cv2.INTER_NEAREST)

    h, w = true_grid.shape
    total_cells = h * w
    
    # 1. Extract Binary Masks
    true_occ = (true_grid >= thresh_occ).astype(np.uint8)
    slam_occ = (slam_grid >= thresh_occ).astype(np.uint8)
    
    true_free = (true_grid <= thresh_free).astype(np.uint8)
    slam_free = (slam_grid <= thresh_free).astype(np.uint8)
    
    true_unk = ((true_grid > thresh_free) & (true_grid < thresh_occ)).astype(np.uint8)
    slam_unk = ((slam_grid > thresh_free) & (slam_grid < thresh_occ)).astype(np.uint8)

    # 2. Tolerance Dilation (Spatial error margin in grid cells)
    tol_cells = max(1, int(round(tolerance_m / resolution)))
    kernel_size = 2 * tol_cells + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    
    true_occ_dilated = cv2.dilate(true_occ, kernel, iterations=1)
    slam_occ_dilated = cv2.dilate(slam_occ, kernel, iterations=1)

    # 3. Obstacle / Wall Metrics
    total_true_occ = int(np.sum(true_occ))
    total_slam_occ = int(np.sum(slam_occ))
    
    # Direct Exact Match
    exact_intersection = int(np.sum((true_occ == 1) & (slam_occ == 1)))
    exact_union = int(np.sum((true_occ == 1) | (slam_occ == 1)))
    exact_iou = exact_intersection / exact_union if exact_union > 0 else 0.0
    
    # Tolerance-based Metrics
    slam_correct_occ = int(np.sum(np.logical_and(true_occ_dilated, slam_occ)))
    true_found_occ = int(np.sum(np.logical_and(true_occ, slam_occ_dilated)))
    
    precision = slam_correct_occ / total_slam_occ if total_slam_occ > 0 else 0.0
    recall = true_found_occ / total_true_occ if total_true_occ > 0 else 0.0
    f1_score = (2 * precision * recall) / (precision + recall) if (precision + recall) > 0 else 0.0
    
    # Tolerant IoU
    tolerant_union = total_true_occ + total_slam_occ - exact_intersection
    tolerant_iou = (slam_correct_occ + true_found_occ) / (2.0 * tolerant_union) if tolerant_union > 0 else 0.0

    # 4. Free Space Metrics
    free_intersection = int(np.sum((true_free == 1) & (slam_free == 1)))
    free_union = int(np.sum((true_free == 1) | (slam_free == 1)))
    free_iou = free_intersection / free_union if free_union > 0 else 0.0

    # 5. Overall Multi-class Classification Accuracy (Free, Occ, Unk)
    correct_classes = int(np.sum(
        ((true_occ == 1) & (slam_occ == 1)) |
        ((true_free == 1) & (slam_free == 1)) |
        ((true_unk == 1) & (slam_unk == 1))
    ))
    overall_accuracy = correct_classes / total_cells

    # 6. Physical Geometric Distance Error (Chamfer & Hausdorff in Meters)
    true_pts_y, true_pts_x = np.where(true_occ == 1)
    slam_pts_y, slam_pts_x = np.where(slam_occ == 1)
    
    chamfer_dist_m = 0.0
    acc_dist_m = 0.0
    comp_dist_m = 0.0
    comp_ratio_5cm = 0.0
    hausdorff_95_m = 0.0
    
    if len(true_pts_x) > 0 and len(slam_pts_x) > 0:
        true_pts = np.column_stack([true_pts_x * resolution, true_pts_y * resolution])
        slam_pts = np.column_stack([slam_pts_x * resolution, slam_pts_y * resolution])
        
        if cKDTree is not None:
            tree_true = cKDTree(true_pts)
            tree_slam = cKDTree(slam_pts)
            
            dists_slam_to_true, _ = tree_true.query(slam_pts)
            dists_true_to_slam, _ = tree_slam.query(true_pts)
            
            acc_dist_m = float(np.mean(dists_slam_to_true))
            comp_dist_m = float(np.mean(dists_true_to_slam))
            chamfer_dist_m = float(0.5 * (acc_dist_m + comp_dist_m))
            comp_ratio_5cm = float(np.mean(dists_true_to_slam <= 0.05))
            all_dists = np.concatenate([dists_slam_to_true, dists_true_to_slam])
            hausdorff_95_m = float(np.percentile(all_dists, 95))
        else:
            # Fallback distance transform
            dist_map_true = cv2.distanceTransform((1 - true_occ).astype(np.uint8), cv2.DIST_L2, 3) * resolution
            dist_map_slam = cv2.distanceTransform((1 - slam_occ).astype(np.uint8), cv2.DIST_L2, 3) * resolution
            acc_dist_m = float(np.mean(dist_map_true[slam_occ == 1]))
            comp_dist_m = float(np.mean(dist_map_slam[true_occ == 1]))
            chamfer_dist_m = float(0.5 * (acc_dist_m + comp_dist_m))
            comp_ratio_5cm = float(np.mean(dist_map_slam[true_occ == 1] <= 0.05))
            hausdorff_95_m = float(np.percentile(dist_map_true[slam_occ == 1], 95)) if np.any(slam_occ == 1) else 0.0

    metrics = {
        'f1_score': float(f1_score),
        'precision': float(precision),
        'recall': float(recall),
        'obstacle_iou': float(exact_iou),
        'tolerant_iou': float(tolerant_iou),
        'free_space_iou': float(free_iou),
        'overall_grid_accuracy': float(overall_accuracy),
        'accuracy_meters': float(acc_dist_m),
        'completion_meters': float(comp_dist_m),
        'completion_ratio_5cm': float(comp_ratio_5cm),
        'chamfer_distance_meters': float(chamfer_dist_m),
        'hausdorff_95_meters': float(hausdorff_95_m),
        'total_true_obstacle_cells': total_true_occ,
        'total_slam_obstacle_cells': total_slam_occ,
        'total_free_cells': int(np.sum(slam_free)),
        'total_unknown_cells': int(np.sum(slam_unk)),
        'tolerance_margin_meters': tolerance_m,
    }

    if save_plots:
        _plot_grid_accuracy_comparison(true_grid, slam_grid, true_occ, slam_occ, 
                                       true_occ_dilated, slam_occ_dilated, 
                                       metrics, resolution, output_dir)

    return metrics


def _plot_grid_accuracy_comparison(true_grid, slam_grid, true_occ, slam_occ, 
                                   true_occ_dilated, slam_occ_dilated, 
                                   metrics, resolution, output_dir):
    """Generates detailed 4-panel visual comparison and difference heatmaps."""
    h, w = true_grid.shape
    extent = [-w * resolution / 2, w * resolution / 2, -h * resolution / 2, h * resolution / 2]
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 14))
    
    # Panel 1: Ground Truth Log-Odds
    im0 = axes[0, 0].imshow(true_grid, cmap='coolwarm', origin='lower', extent=extent, vmin=-3.0, vmax=3.0)
    axes[0, 0].set_title("Ground Truth Bayesian Occupancy Grid (Log-Odds)", fontsize=13, fontweight='bold')
    axes[0, 0].set_xlabel("X (meters)")
    axes[0, 0].set_ylabel("Y (meters)")
    fig.colorbar(im0, ax=axes[0, 0], label="Log-Odds ($l$)")
    
    # Panel 2: SLAM Log-Odds
    im1 = axes[0, 1].imshow(slam_grid, cmap='coolwarm', origin='lower', extent=extent, vmin=-3.0, vmax=3.0)
    axes[0, 1].set_title("SLAM Built Occupancy Grid (Log-Odds with Drift)", fontsize=13, fontweight='bold')
    axes[0, 1].set_xlabel("X (meters)")
    axes[0, 1].set_ylabel("Y (meters)")
    fig.colorbar(im1, ax=axes[0, 1], label="Log-Odds ($l$)")
    
    # Panel 3: Color-Coded Geometric Wall Error Overlay
    overlay = np.zeros((h, w, 3), dtype=np.uint8) + 40 # Dark Gray background
    overlay[true_occ == 1] = [0, 220, 0]             # Green = Ground Truth Obstacle
    overlay[slam_occ == 1] = [220, 30, 30]           # Red = SLAM Detected Obstacle
    
    # Yellow = True Match within Tolerance
    matches = np.logical_and(true_occ_dilated, slam_occ)
    overlay[matches] = [255, 230, 0]
    
    axes[1, 0].imshow(overlay, origin='lower', extent=extent)
    axes[1, 0].set_title(f"Wall Matching Overlay (Tol: ±{metrics['tolerance_margin_meters']*100:.0f}cm)\n"
                         f"Green=Missed Wall | Red=False Wall | Yellow=Correct Match", 
                         fontsize=12, fontweight='bold')
    axes[1, 0].set_xlabel("X (meters)")
    axes[1, 0].set_ylabel("Y (meters)")
    
    # Panel 4: Metrics Scorecard & Confusion Matrix
    ax_card = axes[1, 1]
    ax_card.axis('off')
    
    card_text = (
        "=====================================================\n"
        "           NUMERICAL MAP ACCURACY REPORT             \n"
        "=====================================================\n\n"
        f"  * Obstacle F1 Accuracy Score:      {metrics['f1_score'] * 100:6.2f} %\n"
        f"  * Precision (SLAM Wall Accuracy):   {metrics['precision'] * 100:6.2f} %\n"
        f"  * Recall (True Wall Discovered):    {metrics['recall'] * 100:6.2f} %\n"
        f"  * Obstacle IoU (Jaccard Index):     {metrics['obstacle_iou'] * 100:6.2f} %\n"
        f"  * Tolerant IoU (with Margin):       {metrics['tolerant_iou'] * 100:6.2f} %\n"
        f"  * Free Space IoU:                   {metrics['free_space_iou'] * 100:6.2f} %\n"
        f"  * Overall Grid Classification Acc:  {metrics['overall_grid_accuracy'] * 100:6.2f} %\n\n"
        "-----------------------------------------------------\n"
        "  PHYSICAL GEOMETRIC DISTANCE ERRORS (Meters)\n"
        "-----------------------------------------------------\n"
        f"  * Mean Chamfer Wall Error:          {metrics['chamfer_distance_meters'] * 100:6.2f} cm\n"
        f"  * 95th-Percentile Hausdorff Error:  {metrics['hausdorff_95_meters'] * 100:6.2f} cm\n\n"
        "-----------------------------------------------------\n"
        "  CELL COUNT STATISTICS\n"
        "-----------------------------------------------------\n"
        f"  * Ground Truth Obstacle Cells:     {metrics['total_true_obstacle_cells']:6d}\n"
        f"  * SLAM Detected Obstacle Cells:     {metrics['total_slam_obstacle_cells']:6d}\n"
        f"  * Explored Free Space Cells:        {metrics['total_free_cells']:6d}\n"
        f"  * Remaining Unknown Cells:          {metrics['total_unknown_cells']:6d}\n"
        "====================================================="
    )
    
    ax_card.text(0.05, 0.95, card_text, transform=ax_card.transAxes,
                 fontsize=11, fontfamily='monospace', verticalalignment='top',
                 bbox=dict(boxstyle='round,pad=0.8', facecolor='#f4f7fb', edgecolor='#b0c4de', linewidth=1.5))
    
    plt.tight_layout()
    save_path = os.path.join(output_dir, 'numerical_map_accuracy_comparison.png')
    plt.savefig(save_path, dpi=200)
    plt.close()


# =====================================================================
# 2. ROBOT TRAJECTORY & MOVEMENT ACCURACY EVALUATION (ATE / RPE / DRIFT)
# =====================================================================

def evaluate_trajectory(
    true_poses: Union[np.ndarray, List[Tuple[float, float, float]]],
    slam_poses: Union[np.ndarray, List[Tuple[float, float, float]]],
    timestamps: Optional[Union[np.ndarray, List[float]]] = None,
    output_dir: str = 'accuracy_results',
    save_plots: bool = True
) -> Dict[str, float]:
    """
    Computes Absolute Trajectory Error (ATE) and Relative Pose Error (RPE)
    between Ground Truth movement and SLAM estimated movement.
    
    Args:
        true_poses: List or (N, 3) array of (x, y, yaw_rad) ground-truth positions.
        slam_poses: List or (N, 3) array of (x, y, yaw_rad) SLAM-estimated positions.
        timestamps: Optional (N,) array of time in seconds.
        output_dir: Folder to save trajectory evaluation plots.
        save_plots: Whether to save trajectory comparison plots.
        
    Returns:
        Dictionary of trajectory error metrics.
    """
    os.makedirs(output_dir, exist_ok=True)
    
    true_p = np.asarray(true_poses, dtype=np.float64)
    slam_p = np.asarray(slam_poses, dtype=np.float64)
    
    min_len = min(len(true_p), len(slam_p))
    if min_len < 2:
        return {'ate_rmse': 0.0, 'ate_mean': 0.0, 'ate_max': 0.0, 'drift_rate_pct': 0.0}
        
    true_p = true_p[:min_len]
    slam_p = slam_p[:min_len]
    if timestamps is not None:
        timestamps = np.asarray(timestamps, dtype=np.float64)[:min_len]
    
    # 1. Absolute Trajectory Error (ATE) in 2D Euclidean Distance
    pos_errors = np.hypot(slam_p[:, 0] - true_p[:, 0], slam_p[:, 1] - true_p[:, 1])
    ate_rmse = float(np.sqrt(np.mean(pos_errors ** 2)))
    ate_mean = float(np.mean(pos_errors))
    ate_median = float(np.median(pos_errors))
    ate_std = float(np.std(pos_errors))
    ate_max = float(np.max(pos_errors))
    
    # 2. Heading Error
    def norm_angle_np(a):
        return (a + np.pi) % (2.0 * np.pi) - np.pi
        
    yaw_errors_deg = np.degrees(np.abs(norm_angle_np(slam_p[:, 2] - true_p[:, 2])))
    mean_yaw_error_deg = float(np.mean(yaw_errors_deg))
    max_yaw_error_deg = float(np.max(yaw_errors_deg))
    
    # 3. Path Length & Cumulative Traveled Distance
    diffs_true = np.diff(true_p[:, :2], axis=0)
    step_dists_true = np.hypot(diffs_true[:, 0], diffs_true[:, 1])
    total_dist_true = float(np.sum(step_dists_true))
    
    diffs_slam = np.diff(slam_p[:, :2], axis=0)
    step_dists_slam = np.hypot(diffs_slam[:, 0], diffs_slam[:, 1])
    total_dist_slam = float(np.sum(step_dists_slam))
    
    # 4. Relative Pose Error (RPE) - Frame-to-frame local translational & rotational RMSE
    rpe_trans_errors = np.hypot(diffs_slam[:, 0] - diffs_true[:, 0], diffs_slam[:, 1] - diffs_true[:, 1])
    rpe_trans_rmse = float(np.sqrt(np.mean(rpe_trans_errors ** 2))) if len(rpe_trans_errors) > 0 else 0.0
    dyaw_true = norm_angle_np(np.diff(true_p[:, 2]))
    dyaw_slam = norm_angle_np(np.diff(slam_p[:, 2]))
    rpe_rot_errors_deg = np.degrees(np.abs(norm_angle_np(dyaw_slam - dyaw_true)))
    rpe_rot_rmse_deg = float(np.sqrt(np.mean(rpe_rot_errors_deg ** 2))) if len(rpe_rot_errors_deg) > 0 else 0.0

    # 5. Translational Drift Rate (% of total distance)
    drift_rate_pct = (ate_rmse / total_dist_true * 100.0) if total_dist_true > 0 else 0.0
    
    # 6. Path Efficiency (Direct Euclidean Start-to-End vs Traveled)
    net_displacement = float(np.hypot(true_p[-1, 0] - true_p[0, 0], true_p[-1, 1] - true_p[0, 1]))
    path_efficiency = (net_displacement / total_dist_true) if total_dist_true > 0 else 1.0

    metrics = {
        'ate_rmse_meters': ate_rmse,
        'ate_mean_meters': ate_mean,
        'ate_median_meters': ate_median,
        'ate_std_meters': ate_std,
        'ate_max_meters': ate_max,
        'rpe_trans_rmse_meters': rpe_trans_rmse,
        'rpe_rot_rmse_deg': rpe_rot_rmse_deg,
        'mean_heading_error_deg': mean_yaw_error_deg,
        'max_heading_error_deg': max_yaw_error_deg,
        'total_distance_traveled_m': total_dist_true,
        'slam_estimated_distance_m': total_dist_slam,
        'drift_rate_percent': drift_rate_pct,
        'path_efficiency_ratio': path_efficiency,
        'total_pose_steps': min_len
    }

    if save_plots:
        _plot_trajectory_analysis(true_p, slam_p, pos_errors, yaw_errors_deg, 
                                  timestamps, metrics, output_dir)

    return metrics


def _plot_trajectory_analysis(true_p, slam_p, pos_errors, yaw_errors_deg, 
                             timestamps, metrics, output_dir):
    """Generates trajectory comparison, drift vector fields, and error curves over time."""
    fig, axes = plt.subplots(1, 3, figsize=(21, 6.5))
    
    # Panel 1: Trajectory Overlay in Arena Plane (X vs Y)
    ax0 = axes[0]
    ax0.plot(true_p[:, 0], true_p[:, 1], 'c-', linewidth=2.5, label='Ground Truth Trajectory')
    ax0.plot(slam_p[:, 0], slam_p[:, 1], 'm--', linewidth=2.0, label='SLAM Estimated Trajectory')
    ax0.scatter([true_p[0, 0]], [true_p[0, 1]], color='green', s=100, zorder=5, label='Start')
    ax0.scatter([true_p[-1, 0]], [true_p[-1, 1]], color='red', s=100, zorder=5, label='Final Goal/Stop')
    
    # Draw drift vector lines every N steps
    step_skip = max(1, len(true_p) // 25)
    for i in range(0, len(true_p), step_skip):
        ax0.plot([true_p[i, 0], slam_p[i, 0]], [true_p[i, 1], slam_p[i, 1]], 'y-', alpha=0.7, linewidth=1.2)
        
    ax0.set_title("Robot Trajectory: Ground Truth vs SLAM", fontsize=12, fontweight='bold')
    ax0.set_xlabel("Arena X (meters)")
    ax0.set_ylabel("Arena Y (meters)")
    ax0.grid(True, linestyle=':', alpha=0.6)
    ax0.legend(loc='best', fontsize=9)
    
    # Panel 2: Position Drift Error (ATE) over Time / Steps
    ax1 = axes[1]
    time_x = timestamps if timestamps is not None else np.arange(len(pos_errors))
    x_label = "Simulation Time (s)" if timestamps is not None else "Trajectory Steps"
    
    ax1.plot(time_x, pos_errors * 100.0, 'r-', linewidth=2.0, label='Absolute Position Drift')
    ax1.axhline(metrics['ate_rmse_meters'] * 100.0, color='blue', linestyle='--', linewidth=1.5,
                label=f"ATE RMSE ({metrics['ate_rmse_meters']*100:.1f} cm)")
    ax1.set_title("Position Drift Error (ATE) vs Time", fontsize=12, fontweight='bold')
    ax1.set_xlabel(x_label)
    ax1.set_ylabel("Position Error (cm)")
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='best', fontsize=9)
    
    # Panel 3: Heading Orientation Error vs Time
    ax2 = axes[2]
    ax2.plot(time_x, yaw_errors_deg, 'orange', linewidth=2.0, label='Heading Angular Error')
    ax2.axhline(metrics['mean_heading_error_deg'], color='purple', linestyle='--', linewidth=1.5,
                label=f"Mean Error ({metrics['mean_heading_error_deg']:.1f}°)")
    ax2.set_title("Robot Heading (Yaw) Error vs Time", fontsize=12, fontweight='bold')
    ax2.set_xlabel(x_label)
    ax2.set_ylabel("Heading Error (degrees)")
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='best', fontsize=9)
    
    plt.tight_layout()
    save_path = os.path.join(output_dir, 'robot_movement_trajectory_analysis.png')
    plt.savefig(save_path, dpi=200)
    plt.close()


# =====================================================================
# 3. SENSOR DEPTH MATRICES & HEATMAP PERCEPTION FIDELITY
# =====================================================================
