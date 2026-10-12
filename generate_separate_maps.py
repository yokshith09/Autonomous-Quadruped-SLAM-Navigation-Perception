#!/usr/bin/env python3
"""Standalone Generator for Separate High-Resolution Maps:
1. localization_map.png: Dedicated strictly to Robot Pose Tracking, Trajectory Drift, Heading, and ATE.
2. slam_map.png: Dedicated strictly to SLAM Occupancy Grid, Obstacle Geometry, Rooms, and Doorways.
3. ground_truth_map.png: Dedicated strictly to Ground Truth Physics Grid for baseline comparison.
4. slam_vs_groundtruth_overlay_map.png: Direct color-coded overlay of SLAM reconstructed walls vs True walls.
"""

import os
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D


def generate_separate_maps(
    data_dir: str = "captured_matrices",
    results_dir: str = "accuracy_results",
    images_dir: str = "evaluation_results_images",
    resolution: float = 0.05
):
    os.makedirs(results_dir, exist_ok=True)
    os.makedirs(images_dir, exist_ok=True)

    # 1. Load metrics if available
    metrics = {}
    json_path = os.path.join(results_dir, "latest_metrics.json")
    if os.path.exists(json_path):
        try:
            with open(json_path, "r", encoding="utf-8") as f:
                metrics = json.load(f)
        except Exception:
            pass

    # 2. Load Trajectory Data for LOCALIZATION MAP
    traj_path = os.path.join(data_dir, "trajectory_data.npz")
    if os.path.exists(traj_path):
        data = np.load(traj_path)
        true_poses = data["true_poses"]
        slam_poses = data["slam_poses"]
        min_len = min(len(true_poses), len(slam_poses))
        true_p = true_poses[:min_len]
        slam_p = slam_poses[:min_len]

        _render_localization_map(true_p, slam_p, metrics, images_dir, results_dir)

    # 3. Load Grid Data for SLAM MAP and GROUND TRUTH MAP
    slam_grid_path = os.path.join(data_dir, "slam_grid.npy")
    true_grid_path = os.path.join(data_dir, "true_grid.npy")
    kg_path = os.path.join(results_dir, "knowledge_graph.json")

    kg = {}
    if os.path.exists(kg_path):
        try:
            with open(kg_path, "r", encoding="utf-8") as f:
                kg = json.load(f)
        except Exception:
            pass

    if os.path.exists(slam_grid_path):
        slam_grid = np.load(slam_grid_path)
        _render_slam_occupancy_map(slam_grid, kg, metrics, resolution, images_dir, results_dir)

    if os.path.exists(true_grid_path):
        true_grid = np.load(true_grid_path)
        _render_ground_truth_map(true_grid, resolution, images_dir, results_dir)

    if os.path.exists(slam_grid_path) and os.path.exists(true_grid_path):
        _render_overlay_comparison_map(true_grid, slam_grid, resolution, metrics, images_dir, results_dir)


def _render_localization_map(true_p, slam_p, metrics, images_dir, results_dir):
    """Generates a dedicated high-resolution localization map focusing on pose accuracy and drift."""
    fig, ax = plt.subplots(figsize=(12, 11), dpi=250)
    fig.patch.set_facecolor('#0f141c')
    ax.set_facecolor('#161d27')

    # Grid background styling
    ax.grid(True, color='#283344', linestyle='--', linewidth=0.7, alpha=0.7)

    # Compute Euclidean drift error per step
    pos_errors = np.hypot(slam_p[:, 0] - true_p[:, 0], slam_p[:, 1] - true_p[:, 1])
    ate_rmse = float(np.sqrt(np.mean(pos_errors ** 2))) if len(pos_errors) > 0 else 0.0
    ate_max = float(np.max(pos_errors)) if len(pos_errors) > 0 else 0.0
    ate_mean = float(np.mean(pos_errors)) if len(pos_errors) > 0 else 0.0

    # Heading error
    heading_errors_deg = np.degrees(np.abs(np.arctan2(
        np.sin(slam_p[:, 2] - true_p[:, 2]),
        np.cos(slam_p[:, 2] - true_p[:, 2])
    )))
    mean_yaw_err = float(np.mean(heading_errors_deg)) if len(heading_errors_deg) > 0 else 0.0

    # 1. Ground Truth Path
    ax.plot(true_p[:, 0], true_p[:, 1], color='#00e5ff', linewidth=3.0, alpha=0.9,
            label='Ground Truth Physics Path (True Odometry)', zorder=4)

    # 2. SLAM Estimated Path
    ax.plot(slam_p[:, 0], slam_p[:, 1], color='#ff007f', linestyle='--', linewidth=2.2, alpha=0.95,
            label='Estimated Closed-Loop SLAM Path (Point-to-Line ICP)', zorder=5)

    # 3. Draw Drift Error Vectors (every 80 steps)
    step_skip = max(1, len(true_p) // 50)
    for i in range(0, len(true_p), step_skip):
        ax.plot([true_p[i, 0], slam_p[i, 0]], [true_p[i, 1], slam_p[i, 1]],
                color='#ffe600', linewidth=1.2, alpha=0.75, zorder=6)

    # 4. Heading orientation quivers at key intervals
    q_skip = max(1, len(true_p) // 25)
    ax.quiver(true_p[::q_skip, 0], true_p[::q_skip, 1],
              np.cos(true_p[::q_skip, 2]), np.sin(true_p[::q_skip, 2]),
              color='#00e5ff', scale=22, width=0.005, alpha=0.8, zorder=7)
    ax.quiver(slam_p[::q_skip, 0], slam_p[::q_skip, 1],
              np.cos(slam_p[::q_skip, 2]), np.sin(slam_p[::q_skip, 2]),
              color='#ff007f', scale=22, width=0.005, alpha=0.8, zorder=8)

    # Start and Current position markers
    ax.scatter(true_p[0, 0], true_p[0, 1], s=180, color='#00ff66', edgecolors='white', linewidth=2.0,
               label='Mission Start (0, 0)', zorder=10)
    ax.scatter(slam_p[-1, 0], slam_p[-1, 1], s=180, color='#ffaa00', marker='s', edgecolors='white', linewidth=2.0,
               label=f'Latest Robot Pose ({slam_p[-1, 0]:.2f}, {slam_p[-1, 1]:.2f})', zorder=10)

    # Metric Scorecard Overlay Card
    card_text = (
        "╔═════════════════════════════════════════════╗\n"
        "║       ROBOT LOCALIZATION ACCURACY CARD      ║\n"
        "╠═════════════════════════════════════════════╣\n"
        f"║  * Absolute Trajectory Error (RMSE): {ate_rmse*100:5.2f} cm ║\n"
        f"║  * Mean Trajectory Error (Mean):     {ate_mean*100:5.2f} cm ║\n"
        f"║  * Peak Drift (Max ATE Error):       {ate_max*100:5.2f} cm ║\n"
        f"║  * Mean Heading (Yaw) Error:          {mean_yaw_err:5.2f}°  ║\n"
        f"║  * Path Distance Traveled:           {metrics.get('total_distance_traveled_m', 0.0):5.2f} m  ║\n"
        f"║  * Drift Rate (% of Path Length):     {metrics.get('drift_rate_percent', 0.0):5.2f} %  ║\n"
        "╚═════════════════════════════════════════════╝"
    )
    ax.text(0.03, 0.97, card_text, transform=ax.transAxes, fontsize=10, fontfamily='monospace',
            verticalalignment='top', color='#e0f2fe',
            bbox=dict(boxstyle='round,pad=0.6', facecolor='#1e293b', edgecolor='#0284c7', linewidth=1.5, alpha=0.92))

    ax.set_title("ROBOT LOCALIZATION & TRAJECTORY ACCURACY MAP\nClosed-Loop SLAM Pose Estimation vs Ground Truth Physics Pose",
                 fontsize=14, fontweight='bold', color='#ffffff', pad=15)
    ax.set_xlabel("X (meters in World Frame)", fontsize=11, fontweight='bold', color='#cbd5e1')
    ax.set_ylabel("Y (meters in World Frame)", fontsize=11, fontweight='bold', color='#cbd5e1')
    ax.tick_params(colors='#94a3b8', labelsize=10)

    legend = ax.legend(loc='lower right', facecolor='#1e293b', edgecolor='#334155', fontsize=10, labelcolor='white')
    legend.get_frame().set_alpha(0.9)

    plt.tight_layout()
    for dest in [images_dir, results_dir]:
        p = os.path.join(dest, "localization_map.png")
        plt.savefig(p, dpi=250, facecolor=fig.get_facecolor(), bbox_inches='tight')
    plt.close()
    print("  [SAVED] Standalone Localization Map -> localization_map.png", flush=True)


def _render_slam_occupancy_map(slam_grid, kg, metrics, resolution, images_dir, results_dir):
    """Generates a dedicated high-resolution SLAM map focusing strictly on geometry, rooms, and walls."""
    h, w = slam_grid.shape
    extent = [-w * resolution / 2, w * resolution / 2, -h * resolution / 2, h * resolution / 2]

    fig, ax = plt.subplots(figsize=(12, 11), dpi=250)
    fig.patch.set_facecolor('#ffffff')
    ax.set_facecolor('#ffffff')

    # Construct high-contrast 3-tone architectural display
    # 0 = Free Space (White #FFFFFF), 1 = Unknown Space (Light Gray #E5E7EB), 2 = Obstacles (Navy #1E293B)
    display_rgb = np.full((h, w, 3), 229, dtype=np.uint8)  # Default: Unknown Gray

    free_mask = slam_grid <= -0.5
    occ_mask = slam_grid >= 1.0

    display_rgb[free_mask] = [255, 255, 255]               # Free Space: Pure Crisp White
    display_rgb[occ_mask] = [20, 24, 33]                   # Obstacles: Deep Charcoal Black

    ax.imshow(display_rgb, origin='lower', extent=extent, zorder=1)

    # Overlay Room Annotations from Knowledge Graph
    rooms = kg.get("rooms", {})
    room_colors = ['#3b82f6', '#10b981', '#f59e0b', '#8b5cf6', '#ec4899', '#06b6d4']
    for idx, (rid_str, rdata) in enumerate(rooms.items()):
        cx = rdata.get("centroid_x", 0.0)
        cy = rdata.get("centroid_y", 0.0)
        color = room_colors[idx % len(room_colors)]
        ax.scatter(cx, cy, s=120, color=color, edgecolors='black', linewidth=1.5, zorder=5)
        ax.text(cx, cy + 0.22, f"Room {rid_str}", fontsize=11, fontweight='bold', ha='center',
                color=color, bbox=dict(boxstyle='round,pad=0.2', facecolor='white', edgecolor=color, alpha=0.85), zorder=6)

    # Overlay Discovered Doorways
    doorways = kg.get("doorways", [])
    if isinstance(doorways, dict):
        doorways = list(doorways.values())
    for dw in doorways:
        if isinstance(dw, (list, tuple)) and len(dw) >= 2:
            dx, dy = dw[0], dw[1]
        elif isinstance(dw, dict):
            dx, dy = dw.get("x", 0.0), dw.get("y", 0.0)
        else:
            continue
        ax.scatter(dx, dy, s=90, marker='D', color='#10b981', edgecolors='black', linewidth=1.2, zorder=7)

    # Architectural Grid & Scale
    ax.grid(True, color='#cbd5e1', linestyle=':', linewidth=0.8, alpha=0.8, zorder=2)
    ax.set_title("SLAM ENVIRONMENT OCCUPANCY MAP\n2D Multi-Room Bayesian Occupancy Grid (5cm Resolution, CPU Mapping)",
                 fontsize=14, fontweight='bold', color='#0f172a', pad=15)
    ax.set_xlabel("X (meters in Arena Frame)", fontsize=11, fontweight='bold', color='#1e293b')
    ax.set_ylabel("Y (meters in Arena Frame)", fontsize=11, fontweight='bold', color='#1e293b')

    # Mapping Metrics Card
    f1 = metrics.get('f1_score', 0.0) * 100.0
    chamfer = metrics.get('chamfer_distance_meters', 0.0) * 100.0
    explored = metrics.get('explored_pct', 0.0)
    mem_mb = metrics.get('map_memory_mb', slam_grid.nbytes / (1024.0 * 1024.0))
    free_cells = int(np.sum(free_mask))
    occ_cells = int(np.sum(occ_mask))

    card_text = (
        "╔═════════════════════════════════════════════╗\n"
        "║         SLAM RECONSTRUCTION ACCURACY        ║\n"
        "╠═════════════════════════════════════════════╣\n"
        f"║  * Wall Reconstruction F1 Score:   {f1:5.1f} %  ║\n"
        f"║  * Mean Wall Chamfer Distance:    {chamfer:5.2f} cm ║\n"
        f"║  * Arena Explored Coverage:       {explored:5.1f} %  ║\n"
        f"║  * Free Space Area Explored:      {free_cells * (resolution**2):5.1f} m² ║\n"
        f"║  * Occupied Obstacle Perimeter:   {occ_cells * resolution:5.1f} m  ║\n"
        f"║  * Grid Resolution:                5.0 cm/cell║\n"
        f"║  * Map RAM Memory Footprint:       {mem_mb:5.2f} MB  ║\n"
        "╚═════════════════════════════════════════════╝"
    )
    ax.text(0.03, 0.97, card_text, transform=ax.transAxes, fontsize=10, fontfamily='monospace',
            verticalalignment='top', color='#0f172a',
            bbox=dict(boxstyle='round,pad=0.6', facecolor='#f8fafc', edgecolor='#64748b', linewidth=1.5, alpha=0.95))

    # Clean Legend
    legend_elements = [
        mpatches.Patch(facecolor='#141821', edgecolor='black', label='Occupied Wall / Obstacle'),
        mpatches.Patch(facecolor='#ffffff', edgecolor='#94a3b8', label='Explored Navigable Free Space'),
        mpatches.Patch(facecolor='#e5e7eb', edgecolor='#94a3b8', label='Unexplored Unknown Space'),
        Line2D([0], [0], marker='D', color='w', markerfacecolor='#10b981', markeredgecolor='black', markersize=9, label='Discovered Doorway / Transit'),
    ]
    ax.legend(handles=legend_elements, loc='lower right', facecolor='#f8fafc', edgecolor='#94a3b8', fontsize=10)

    plt.tight_layout()
    for dest in [images_dir, results_dir]:
        p = os.path.join(dest, "slam_map.png")
        plt.savefig(p, dpi=250, facecolor=fig.get_facecolor(), bbox_inches='tight')
    plt.close()
    print("  [SAVED] Standalone SLAM Map -> slam_map.png", flush=True)


def _render_ground_truth_map(true_grid, resolution, images_dir, results_dir):
    """Generates the reference Ground Truth map from simulator physics raycasting."""
    h, w = true_grid.shape
    extent = [-w * resolution / 2, w * resolution / 2, -h * resolution / 2, h * resolution / 2]

    fig, ax = plt.subplots(figsize=(12, 11), dpi=250)
    fig.patch.set_facecolor('#ffffff')
    ax.set_facecolor('#ffffff')

    display_rgb = np.full((h, w, 3), 229, dtype=np.uint8)
    free_mask = true_grid <= -0.5
    occ_mask = true_grid >= 1.0

    display_rgb[free_mask] = [255, 255, 255]
    display_rgb[occ_mask] = [15, 23, 42]

    ax.imshow(display_rgb, origin='lower', extent=extent)
    ax.grid(True, color='#cbd5e1', linestyle=':', linewidth=0.8, alpha=0.8)
    ax.set_title("GROUND TRUTH REFERENCE MAP\nSimulator Physics Ground Truth Raycast Baseline (5cm Resolution)",
                 fontsize=14, fontweight='bold', color='#0f172a', pad=15)
    ax.set_xlabel("X (meters in Arena Frame)", fontsize=11, fontweight='bold', color='#1e293b')
    ax.set_ylabel("Y (meters in Arena Frame)", fontsize=11, fontweight='bold', color='#1e293b')

    plt.tight_layout()
    for dest in [images_dir, results_dir]:
        p = os.path.join(dest, "ground_truth_map.png")
        plt.savefig(p, dpi=250, facecolor=fig.get_facecolor(), bbox_inches='tight')
    plt.close()
    print("  [SAVED] Standalone Ground Truth Map -> ground_truth_map.png", flush=True)


def _render_overlay_comparison_map(true_grid, slam_grid, resolution, metrics, images_dir, results_dir):
    """Generates color-coded overlay map showing exact wall alignment between SLAM and Ground Truth."""
    h, w = true_grid.shape
    extent = [-w * resolution / 2, w * resolution / 2, -h * resolution / 2, h * resolution / 2]

    fig, ax = plt.subplots(figsize=(12, 11), dpi=250)
    fig.patch.set_facecolor('#0f172a')
    ax.set_facecolor('#0f172a')

    true_occ = (true_grid >= 1.0)
    slam_occ = (slam_grid >= 1.0)

    # Color code:
    # Background: Dark #0F172A
    # Green (#10B981) = Ground Truth Obstacle
    # Red (#EF4444)   = SLAM Detected Obstacle
    # Yellow (#FBBF24)= True Wall Match within tolerance
    import cv2
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    true_dilated = cv2.dilate(true_occ.astype(np.uint8), kernel, iterations=1) > 0
    slam_matches = np.logical_and(true_dilated, slam_occ)

    overlay = np.zeros((h, w, 3), dtype=np.uint8) + 20
    overlay[true_occ] = [0, 210, 110]         # Green: True Wall
    overlay[slam_occ] = [235, 45, 45]          # Red: SLAM Wall
    overlay[slam_matches] = [255, 215, 0]      # Yellow: Matching Wall Alignment

    ax.imshow(overlay, origin='lower', extent=extent)
    ax.grid(True, color='#334155', linestyle=':', linewidth=0.7, alpha=0.7)

    ax.set_title("SLAM vs GROUND TRUTH GEOMETRIC ALIGNMENT OVERLAY\nYellow = Verified Wall Match | Green = Missed True Wall | Red = Drift / False Wall",
                 fontsize=13, fontweight='bold', color='#ffffff', pad=15)
    ax.set_xlabel("X (meters)", fontsize=11, fontweight='bold', color='#cbd5e1')
    ax.set_ylabel("Y (meters)", fontsize=11, fontweight='bold', color='#cbd5e1')
    ax.tick_params(colors='#94a3b8', labelsize=10)

    legend_elements = [
        mpatches.Patch(facecolor='#ffd700', edgecolor='white', label='Accurate Wall Match (Yellow, ±15cm)'),
        mpatches.Patch(facecolor='#00d26e', edgecolor='white', label='Ground Truth Wall (Green)'),
        mpatches.Patch(facecolor='#eb2d2d', edgecolor='white', label='SLAM Reconstructed Wall (Red)'),
    ]
    ax.legend(handles=legend_elements, loc='lower right', facecolor='#1e293b', edgecolor='#475569', fontsize=10, labelcolor='white')

    plt.tight_layout()
    for dest in [images_dir, results_dir]:
        p = os.path.join(dest, "slam_vs_groundtruth_overlay_map.png")
        plt.savefig(p, dpi=250, facecolor=fig.get_facecolor(), bbox_inches='tight')
    plt.close()
    print("  [SAVED] Standalone SLAM vs GT Overlay Map -> slam_vs_groundtruth_overlay_map.png", flush=True)


if __name__ == "__main__":
    generate_separate_maps()
