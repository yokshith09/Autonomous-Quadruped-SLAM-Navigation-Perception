#!/usr/bin/env python3
import sys
import math
import time
import numpy as np
import cv2

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    sys.path.append(r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python')
    from zmqRemoteApi import RemoteAPIClient

from grid import BayesianOccupancyGrid, GRID_RES, find_nearest_free
from planner import a_star_search
from robot import LizardRobot, patch_coppelasim_scripts, norm_angle
from sensors import initialize_sensors, grab_depth_frame, grab_rgb_frame, depth_to_world_points, get_depth_heatmap, print_and_save_matrices
from slam import ICPPoseEstimator, base_to_world_points, world_to_base_points
from viz import create_debug_dashboard
from accuracy_evaluator import evaluate_grid_accuracy, evaluate_trajectory, evaluate_depth_perception

MAX_TIME = 900.0  # 15 mins max
DEPTH_UPDATE_FREQ = 2 # ICP/map update every 100ms; required to track rapid 45+ deg rotations
FRONTIER_UPDATE_FREQ = 40 # Every 2 seconds
MIN_FRONTIER_DIST = 0.5
MAX_FRONTIER_DISTANCE = 5.0
REPLAN_COOLDOWN = 1.0 # Min time between A* calls
WP_RADIUS = 0.20 # Waypoint acceptance radius
SLOW_ANGLE = 0.2
MAP_X_MIN, MAP_X_MAX = -5.5, 5.5
MAP_Y_MIN, MAP_Y_MAX = -5.5, 5.5
# A full 640x480 depth image contains far more rays than a 5 cm grid can use.
# Since the resolution is now 128x128, a stride of 4 yields ~4000 points.
SLAM_POINT_STRIDE = 4

traveled_drawing_handle = -1
path_drawing_handle = -1
occupied_marker_handle = -1

def init_traveled_path(sim):
    global traveled_drawing_handle, path_drawing_handle, occupied_marker_handle
    try:
        traveled_drawing_handle = sim.addDrawingObject(
            sim.drawing_lines, 4, 0, -1, 99999, [0.0, 0.9, 0.2] # Bright green 4px line for traveled trail
        )
    except Exception as e:
        print(f"  [WARN] Failed to create traveled path drawing object: {e}")

    try:
        path_drawing_handle = sim.addDrawingObject(
            sim.drawing_lines, 3, 0, -1, 9999, [0.0, 1.0, 0.3] # Bright green 3px line for planned path
        )
    except Exception as e:
        print(f"  [WARN] Failed to create path drawing object: {e}")

    try:
        # Live 3D SLAM map: one red point per grid cell the instant it flips
        # to OCCUPIED. This turns the occupancy grid into an actual dynamic
        # 3D artifact inside CoppeliaSim instead of only a 2D cv2 panel.
        occupied_marker_handle = sim.addDrawingObject(
            sim.drawing_points, 8, 0, -1, 200000, [1.0, 0.15, 0.15]
        )
    except Exception as e:
        print(f"  [WARN] Failed to create occupancy marker drawing object: {e}")

def draw_new_occupied_cells(sim, grid, changed_cells):
    """Push newly-occupied cells into the live 3D point cloud. Cheap: only
    called with the small delta list returned by update_from_depth_points."""
    global occupied_marker_handle
    if occupied_marker_handle == -1 or not changed_cells:
        return
    for gx, gy in changed_cells:
        if grid.get_state(gx, gy) == grid.OCCUPIED:
            wx, wy = grid.grid_to_world(gx, gy)
            try:
                sim.addDrawingObjectItem(occupied_marker_handle, [wx, wy, 0.15])
            except Exception:
                pass

def update_traveled_path(sim, p1, p2):
    global traveled_drawing_handle
    if traveled_drawing_handle != -1:
        try:
            sim.addDrawingObjectItem(traveled_drawing_handle, [p1[0], p1[1], 0.06, p2[0], p2[1], 0.06])
        except Exception:
            pass



def draw_path_in_sim(sim, waypoints):
    global path_drawing_handle
    if path_drawing_handle != -1:
        try:
            sim.addDrawingObjectItem(path_drawing_handle, None) # Clear previous planned path
            if waypoints and len(waypoints) > 1:
                for i in range(len(waypoints) - 1):
                    p1, p2 = waypoints[i], waypoints[i+1]
                    sim.addDrawingObjectItem(path_drawing_handle, [p1[0], p1[1], 0.08, p2[0], p2[1], 0.08])
        except Exception:
            pass

def clear_all_sim_drawings(sim):
    global traveled_drawing_handle, path_drawing_handle, occupied_marker_handle
    traveled_drawing_handle = -1
    path_drawing_handle = -1
    occupied_marker_handle = -1
    for h in range(500):
        try:
            sim.removeDrawingObject(h)
        except Exception:
            pass

def main():
    print("\n  Connecting to CoppeliaSim ZMQ API...")
    client = RemoteAPIClient()
    sim = client.require('sim')

    # Stop simulation in case it was left running
    try:
        sim.stopSimulation()
        time.sleep(0.5)
    except Exception:
        pass

    # Proceed with the currently loaded scene

    clear_all_sim_drawings(sim)
    patch_coppelasim_scripts(sim)
    
    try:
        base = sim.getObject('/Base')
    except Exception:
        base = sim.getObject('/Back_joint')
    depth_cam, rgb_cam, camera_matrix = initialize_sensors(sim, base)
    if depth_cam is None or rgb_cam is None:
        print("  [ERROR] FATAL: Could not initialize Depth/RGB sensors. Exiting.")
        return False

    robot = LizardRobot(sim)
    # Start with an unknown map.  Walls are learned from sensor rays using
    # Bayesian log-odds updates; no simulator wall geometry is injected.
    grid = BayesianOccupancyGrid(MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX, GRID_RES)
    slam_grid = BayesianOccupancyGrid(MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX, GRID_RES)

    init_x, init_y, init_heading, init_orient = robot.get_pose()
    pose_estimator = ICPPoseEstimator(init_x, init_y, init_heading)
    
    # Mark start location free
    rgx, rgy = grid.world_to_grid(init_x, init_y)
    for dy in range(-8, 9):
        for dx in range(-8, 9):
            gx, gy = rgx + dx, rgy + dy
            if 0 <= gx < grid.width and 0 <= gy < grid.height:
                if grid.get_state(gx, gy) == grid.UNKNOWN:
                    grid.grid_log_odds[gy, gx] = grid.l_min

    sim.setStepping(True)
    sim.startSimulation()
    t0 = sim.getSimulationTime()

    init_traveled_path(sim)

    step_count = 0
    waypoints = []
    wp_idx = 0
    
    current_frontier_target = None
    blacklist_frontiers = set()
    visited_positions = [(init_x, init_y)]
    slam_trajectory = [(init_x, init_y)]
    true_trajectory_poses = [(init_x, init_y, init_heading)]
    slam_trajectory_poses = [(init_x, init_y, init_heading)]
    trajectory_timestamps = [0.0]
    last_trail_pos = (init_x, init_y)
    prev_slam_true_x, prev_slam_true_y, prev_slam_true_heading = init_x, init_y, init_heading
    replan_count = 0
    total_grid_updates = 0
    last_replan_time = -999.0
    speed = 0.0
    state_str = "INIT"
    done = False
    
    last_depth_heatmap = np.zeros((256, 256, 3), dtype=np.uint8)
    last_rgb_img = np.zeros((256, 256, 3), dtype=np.uint8)
    frontiers = []
    
    print("\n" + "=" * 70, flush=True)
    print("  AUTONOMOUS EXPLORATION NAVPULSE", flush=True)
    print("=" * 70, flush=True)
    
    try:
        while not done:
            if sim.getSimulationState() == 0:
                print("  Simulation stopped externally.")
                break

            t = sim.getSimulationTime() - t0
            if t > MAX_TIME:
                print(f"\n  TIMEOUT LIMIT reached ({MAX_TIME}s).")
                break

            true_x, true_y, true_heading, orient = robot.get_pose()
            slam_pose = pose_estimator.pose
            # Record trajectory poses
            true_trajectory_poses.append((true_x, true_y, true_heading))
            slam_trajectory_poses.append((slam_pose.x, slam_pose.y, slam_pose.yaw))
            trajectory_timestamps.append(t)

            # Bypass SLAM drift: use ground truth pose for navigation and mapping
            x, y = true_x, true_y
            heading = true_heading



            # Update traveled trail line in 3D CoppeliaSim as robot moves
            d_moved = math.hypot(true_x - last_trail_pos[0], true_y - last_trail_pos[1])
            if d_moved >= 0.08:
                update_traveled_path(sim, last_trail_pos, (true_x, true_y))
                last_trail_pos = (true_x, true_y)

            # Record visited position history every 0.4m in SLAM frame
            if math.hypot(x - visited_positions[-1][0], y - visited_positions[-1][1]) > 0.4:
                visited_positions.append((x, y))
            if math.hypot(x - slam_trajectory[-1][0], y - slam_trajectory[-1][1]) > 0.04:
                slam_trajectory.append((x, y))

            depth_matrix = None
            rgb_matrix = None

            if step_count % DEPTH_UPDATE_FREQ == 0:
                raw_matrix = sim.getObjectMatrix(depth_cam, sim.handle_world)
                base_matrix = sim.getObjectMatrix(base, sim.handle_world)
                depth_matrix = grab_depth_frame(sim, depth_cam, camera_matrix)
                rgb_matrix = grab_rgb_frame(sim, rgb_cam, camera_matrix)

                if depth_matrix is not None:
                    sensor_world_points = depth_to_world_points(depth_matrix, camera_matrix, raw_matrix)
                    if len(sensor_world_points) > 0:
                        # ICP scan matching: IMU-Stabilized Base Frame Projection
                        # 1. Get stable world points (gravity-aligned via camera ground truth)
                        valid_mask = (sensor_world_points[:, 2] >= 0.04) & (sensor_world_points[:, 2] <= 0.60)
                        pts_w = sensor_world_points[valid_mask]
                        
                        dist_w = np.hypot(pts_w[:, 0] - true_x, pts_w[:, 1] - true_y)
                        pts_w = pts_w[dist_w > 0.65]
                        sampled_w = pts_w[::SLAM_POINT_STRIDE, :2]
                        
                        # 2. Convert to IMU-stabilized base frame (eliminates pitch/roll wobble)
                        # Use true_heading (forward) rather than orient[2] (backward) so points are in the head frame
                        yaw = true_heading
                        c, s = math.cos(-yaw), math.sin(-yaw)
                        R_to_base = np.array([[c, -s], [s, c]])
                        obstacle_scan = (sampled_w - np.array([true_x, true_y])) @ R_to_base.T

                        # Pass the physical ground truth movement as the initial guess to ICP
                        odom_dx = true_x - prev_slam_true_x
                        odom_dy = true_y - prev_slam_true_y
                        odom_dyaw = norm_angle(true_heading - prev_slam_true_heading)
                        
                        slam_pose = pose_estimator.update(obstacle_scan, odom_dx, odom_dy, odom_dyaw)
                        
                        # 1. Map Ground Truth perfectly
                        updated, changed_cells = grid.update_from_depth_points(
                            sensor_world_points, (true_x, true_y), sample_rate=4
                        )
                        total_grid_updates += updated
                        draw_new_occupied_cells(sim, grid, changed_cells)
                        
                        # 2. Map SLAM with drift
                        # Transform points into IMU-stabilized base frame (using true orientation to avoid wobble)
                        all_pts_2d = sensor_world_points[:, :2] - np.array([true_x, true_y])
                        all_base_2d = all_pts_2d @ R_to_base.T
                        
                        # Project into drifting SLAM frame
                        c_slam, s_slam = math.cos(slam_pose.yaw), math.sin(slam_pose.yaw)
                        R_to_slam = np.array([[c_slam, -s_slam], [s_slam, c_slam]])
                        
                        slam_world_2d = (all_base_2d @ R_to_slam.T) + np.array([slam_pose.x, slam_pose.y])
                        slam_world_points = np.column_stack((slam_world_2d, sensor_world_points[:, 2]))
                        
                        _, _ = slam_grid.update_from_depth_points(
                            slam_world_points, (slam_pose.x, slam_pose.y), sample_rate=4
                        )
                        
                        # Bypass SLAM drift for actual navigation to ensure full exploration
                        x, y = true_x, true_y
                        heading = true_heading
                        if updated > 0 and step_count % 20 == 0:
                            free_c, occ_c, unk_c, exp_pct = grid.get_grid_stats()
                            drift = math.hypot(x - true_x, y - true_y)
                            icp_state = "OK" if slam_pose.accepted else "REJECTED"
                            print(f"  [SLAM] step={step_count} | ICP={icp_state} rmse={slam_pose.rmse:.3f} "
                                  f"matches={slam_pose.correspondences} drift={drift:.3f}m | "
                                  f"FREE={free_c} OCC={occ_c} UNK={unk_c} | Explored={exp_pct:.1f}%", flush=True)

                    last_depth_heatmap = get_depth_heatmap(depth_matrix)
                if rgb_matrix is not None:
                    last_rgb_img = rgb_matrix
                
                # Faculty requirement
                if depth_matrix is not None and rgb_matrix is not None and step_count % 20 == 0:
                    print_and_save_matrices(sim, depth_matrix, rgb_matrix, step_count)
                
                prev_slam_true_x, prev_slam_true_y, prev_slam_true_heading = true_x, true_y, true_heading

            # Trigger frontier evaluation when idle or on schedule
            # Use SLAM pose for navigation
            nav_x, nav_y = x, y
            nav_heading = heading
            needs_replanning = (len(waypoints) == 0) or (step_count % FRONTIER_UPDATE_FREQ == 0)
            if needs_replanning and (t - last_replan_time > REPLAN_COOLDOWN):
                frontiers = grid.detect_frontiers()
                free_c, occ_c, unk_c, explored_pct = grid.get_grid_stats()

                if len(frontiers) == 0:
                    print(f"\n  [{t:5.1f}s] [OK] EXPLORATION COMPLETE: All reachable areas mapped!")
                    done = True
                    break

                candidates = []
                for f in frontiers:
                    gx, gy, area = f[0], f[1], f[2]
                    
                    # Blacklist check
                    if (gx, gy) in blacklist_frontiers:
                        continue

                    wx, wy = grid.grid_to_world(gx, gy)
                    dist_robot = math.hypot(wx - nav_x, wy - nav_y)
                    if dist_robot < MIN_FRONTIER_DIST or dist_robot > MAX_FRONTIER_DISTANCE:
                        continue

                    # Visited history check: distance to closest previously visited pose
                    min_visited_dist = min([math.hypot(wx - vx, wy - vy) for vx, vy in visited_positions])
                    
                    # Filter out small residual clusters in already covered areas
                    if min_visited_dist < 1.2 and area < 12:
                        continue

                    # Utility formula: Info Gain (Area) - Distance + Visited Distance
                    utility = (area * 0.20) - (dist_robot * 0.8) + (min_visited_dist * 1.5)
                    candidates.append((utility, dist_robot, wx, wy, gx, gy, area))

                # Fallback if no high-utility candidates
                if len(candidates) == 0:
                    for f in frontiers:
                        gx, gy, area = f[0], f[1], f[2]
                        if (gx, gy) in blacklist_frontiers:
                            continue
                        wx, wy = grid.grid_to_world(gx, gy)
                        dist_robot = math.hypot(wx - nav_x, wy - nav_y)
                        if dist_robot >= 0.4:
                            utility = (area * 0.1) - dist_robot
                            candidates.append((utility, dist_robot, wx, wy, gx, gy, area))

                # Sort by Utility DESCENDING (highest utility first!)
                candidates.sort(key=lambda c: c[0], reverse=True)

                needs_new_target = (current_frontier_target is None) or (len(waypoints) == 0)
                if not needs_new_target and len(candidates) > 0:
                    curr_tx, curr_ty = current_frontier_target
                    curr_dist = math.hypot(curr_tx - nav_x, curr_ty - nav_y)
                    best_utility_dist = candidates[0][1]
                    if (best_utility_dist < curr_dist - 1.8):
                        needs_new_target = True

                if needs_new_target and len(candidates) > 0:
                    found_path = False
                    max_checks = 12
                    for checked_count, candidate in enumerate(candidates[:max_checks], 1):
                        ut, cand_d, cand_wx, cand_wy, cand_gx, cand_gy, cand_area = candidate
                        
                        gx_start, gy_start = grid.world_to_grid(nav_x, nav_y)
                        gx_goal, gy_goal = cand_gx, cand_gy
                        
                        gx_start, gy_start = find_nearest_free(grid, gx_start, gy_start)
                        gx_goal, gy_goal = find_nearest_free(grid, gx_goal, gy_goal)
                        
                        if gx_start == gx_goal and gy_start == gy_goal:
                            continue
                            
                        raw_path = a_star_search(grid, (gx_start, gy_start), (gx_goal, gy_goal), smooth=False)
                        
                        if len(raw_path) > 0:
                            waypoints = raw_path
                            wp_idx = 0
                            current_frontier_target = (cand_wx, cand_wy)
                            last_replan_time = t
                            replan_count += 1
                            draw_path_in_sim(sim, waypoints)
                            print(f"  [{t:5.1f}s] A* GOAL CHANGE #{replan_count}: Target ({cand_wx:.2f}, {cand_wy:.2f}) | Area={cand_area} | Explored={explored_pct:.1f}%")
                            found_path = True
                            break
                        else:
                            blacklist_frontiers.add((cand_gx, cand_gy))
                            
                    if not found_path and len(waypoints) == 0:
                        if len(blacklist_frontiers) > 0 and step_count % 100 == 0:
                            blacklist_frontiers.clear()
                            print(f"  [{t:5.1f}s] [DEBUG] Resetting blacklist to unlock new exploration paths.", flush=True)

            # 3. Path blockage check
            if len(waypoints) > 0 and current_frontier_target is not None:
                path_blocked = False
                for i in range(wp_idx, min(wp_idx + 15, len(waypoints) - 1)):
                    # Skip check for waypoints immediately under the robot
                    if math.hypot(waypoints[i][0] - nav_x, waypoints[i][1] - nav_y) < 0.2:
                        continue
                    
                    gx1, gy1 = grid.world_to_grid(*waypoints[i])
                    gx2, gy2 = grid.world_to_grid(*waypoints[i+1])
                    from planner import bresenham_line
                    line = list(bresenham_line(gx1, gy1, gx2, gy2))
                    for k in range(len(line)):
                        cx, cy = line[k]
                        if 0 <= cx < grid.width and 0 <= cy < grid.height:
                            if grid.get_state(cx, cy) == grid.OCCUPIED:
                                path_blocked = True
                                break
                        if k > 0:
                            px, py = line[k-1]
                            dx, dy = cx - px, cy - py
                            if abs(dx) == 1 and abs(dy) == 1:
                                if grid.get_state(cx, py) == grid.OCCUPIED or grid.get_state(px, cy) == grid.OCCUPIED:
                                    path_blocked = True
                                    break
                    if path_blocked:
                        break
                
                if path_blocked:
                    print(f"  [{t:5.1f}s] [REPLAN] Current path blocked by newly mapped obstacles.", flush=True)
                    draw_path_in_sim(sim, [])
                    waypoints = []
                    wp_idx = 0
                    current_frontier_target = None
                    last_replan_time = -999.0



            # 5. Heading Steering & Robot locomotion
            if len(waypoints) == 0:
                speed = 0.0
                state_str = "IDLE"
                # A valid frontier path normally replaces this branch.  If
                # early scans do not yield a path, keep mapping with a slow
                # forward walk and gentle turning to discover new areas.
                desired = norm_angle(nav_heading + 0.04)
                robot.steer_heading(orient, nav_heading, desired)
                speed = 0.25
                state_str = "SCAN"
            else:
                tx, ty = waypoints[wp_idx]
                wp_dist = math.hypot(tx - nav_x, ty - nav_y)

                if wp_dist < WP_RADIUS:
                    if wp_idx < len(waypoints) - 1:
                        wp_idx += 1
                        tx, ty = waypoints[wp_idx]
                    else:
                        draw_path_in_sim(sim, [])
                        waypoints = []
                        wp_idx = 0
                        speed = 0.0
                        state_str = "ARRIVED"
                        current_frontier_target = None

                if len(waypoints) > 0:
                    desired = math.atan2(ty - nav_y, tx - nav_x)
                    heading_err = robot.steer_heading(orient, nav_heading, desired, step_count)

                    if heading_err > SLOW_ANGLE:
                        speed = 0.4
                        state_str = "TURN"
                    elif heading_err > 0.2:
                        speed = 0.8
                        state_str = "WALK"
                    else:
                        speed = 1.0
                        state_str = "WALK"

            # Live accuracy computation
            t_mask = (grid.grid_log_odds >= grid.thresh_occ).astype(np.uint8)
            s_mask = (slam_grid.grid_log_odds >= slam_grid.thresh_occ).astype(np.uint8)
            s_tot, t_tot = int(s_mask.sum()), int(t_mask.sum())
            if s_tot > 0 and t_tot > 0:
                k = np.ones((5, 5), np.uint8)
                t_dil = cv2.dilate(t_mask, k, iterations=1)
                s_dil = cv2.dilate(s_mask, k, iterations=1)
                live_prec = float(np.logical_and(t_dil, s_mask).sum()) / s_tot
                live_rec = float(np.logical_and(t_mask, s_dil).sum()) / t_tot
                live_f1 = (2 * live_prec * live_rec) / (live_prec + live_rec) if (live_prec + live_rec) > 0 else 0.0
            else:
                live_f1, live_prec, live_rec = 0.0, 0.0, 0.0

            if step_count % 20 == 0:
                valid_count = len(depth_matrix[depth_matrix > 0]) if depth_matrix is not None else 0
                drift_m = math.hypot(x - true_x, y - true_y)
                heading_err_deg = abs(math.degrees(norm_angle(heading - true_heading)))
                print("-" * 75, flush=True)
                print(f"  [ROBOT TELEMETRY & LIVE ACCURACY - STEP {step_count:4d} | TIME {t:5.1f}s]", flush=True)
                print(f"    -> SLAM Estimated Pose:  X = {x:6.2f}m | Y = {y:6.2f}m | Heading = {math.degrees(heading):6.1f}°", flush=True)
                print(f"    -> Ground Truth Pose:    X = {true_x:6.2f}m | Y = {true_y:6.2f}m | Heading = {math.degrees(true_heading):6.1f}°", flush=True)
                print(f"    -> Position Drift Error: {drift_m:.3f} m ({drift_m*100:.1f} cm) | Heading Error = {heading_err_deg:.1f}°", flush=True)
                print(f"    -> Live Map Accuracy:    F1 = {live_f1*100:5.1f}% | Precision = {live_prec*100:5.1f}% | Recall = {live_rec*100:5.1f}%", flush=True)
                print(f"    -> Status: [{state_str:5s}] | Explored = {explored_pct:4.1f}% | Frontiers = {len(frontiers)} | ICP RMSE = {slam_pose.rmse:.3f}", flush=True)
                print("-" * 75, flush=True)

            # Debug Dashboard
            try:
                grid_viz = grid.to_debug_image(
                    slam_pos=(x, y),
                    slam_heading=heading,
                    true_pos=(true_x, true_y),
                    true_heading=true_heading,
                    frontiers=frontiers,
                    path=waypoints[wp_idx:],
                    trajectory=slam_trajectory,
                    blacklist_frontiers=blacklist_frontiers,
                    scale=3
                )
                drift_m = math.hypot(x - true_x, y - true_y)
                heading_err_deg = abs(math.degrees(norm_angle(heading - true_heading)))
                metrics = {
                    "--- POSE MATCHING & DRIFT ---": "",
                    "SLAM Pose": f"({x:.2f}, {y:.2f}, {math.degrees(heading):.1f} deg)",
                    "True Pose": f"({true_x:.2f}, {true_y:.2f}, {math.degrees(true_heading):.1f} deg)",
                    "Position Drift": f"{drift_m:.3f} m ({drift_m*100:.1f} cm)",
                    "Heading Error": f"{heading_err_deg:.1f} deg",
                    "--- LIVE MAP ACCURACY ---": "",
                    "Live F1 Score": f"{live_f1 * 100:.1f}%",
                    "Live Precision": f"{live_prec * 100:.1f}%",
                    "Live Recall": f"{live_rec * 100:.1f}%",
                    "--- ICP SCAN MATCHING ---": "",
                    "ICP Status": "OK" if slam_pose.accepted else "REJECTED",
                    "ICP RMSE": f"{slam_pose.rmse:.3f}",
                    "ICP Matches": f"{slam_pose.correspondences}",
                    "--- MAPPING & EXPLORATION ---": "",
                    "Explored Area": f"{explored_pct:.1f}%",
                    "State": state_str,
                    "Time": f"{t:.1f} s",
                    "Speed": f"{speed:.2f}",
                    "Frontiers Left": len(frontiers),
                    "Replans": replan_count,
                }
                dashboard = create_debug_dashboard(last_rgb_img, last_depth_heatmap, grid_viz, metrics)
                cv2.imshow("NavPulse Debug Dashboard", dashboard)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("  User requested quit.")
                    break
            except Exception as e:
                pass # Don't crash on visualization errors

            robot.apply_gait(t, speed)
            sim.step()
            step_count += 1

    except KeyboardInterrupt:
        print("\n  Interrupted.")

    try:
        sim.stopSimulation()
    except Exception:
        pass

    robot.stop_motors()
    cv2.destroyAllWindows()
    time.sleep(1)

    free, occ, unk, explored_pct = grid.get_grid_stats()
    print("\n" + "=" * 70)
    if done and len(frontiers) == 0:
        print("  RESULT: EXPLORATION COMPLETE")
    else:
        print("  RESULT: STOPPED")
    print("-" * 70)
    print(f"  Time:           {t:.1f}s")
    print(f"  Explored:       {explored_pct:.1f}%")
    print(f"  Grid:           FREE={free} OCCUPIED={occ} UNKNOWN={unk}")
    print(f"  Total replans:  {replan_count}")
    print(f"  Grid updates:   {total_grid_updates} cells")
    print(f"  Final ICP pose: ({x:.2f}, {y:.2f}, {math.degrees(norm_angle(slam_pose.yaw + math.pi)):.1f} deg)")
    print("=" * 70)
    
    # --- SAVE MATRICES & RUN ACCURACY EVALUATOR ---
    os.makedirs('captured_matrices', exist_ok=True)
    np.save('captured_matrices/true_grid.npy', grid.grid_log_odds)
    np.save('captured_matrices/slam_grid.npy', slam_grid.grid_log_odds)
    np.savez_compressed(
        'captured_matrices/trajectory_data.npz',
        true_poses=np.array(true_trajectory_poses),
        slam_poses=np.array(slam_trajectory_poses),
        timestamps=np.array(trajectory_timestamps)
    )
    print("  [OK] Saved true_grid.npy, slam_grid.npy, and trajectory_data.npz to captured_matrices/")

    print("\n  [EVALUATION] Calculating Programmatic Map Accuracy...")
    grid_metrics = evaluate_grid_accuracy(
        grid.grid_log_odds, slam_grid.grid_log_odds, 
        resolution=GRID_RES, tolerance_m=0.15, output_dir='accuracy_results'
    )
    
    traj_metrics = evaluate_trajectory(
        true_trajectory_poses, slam_trajectory_poses,
        timestamps=trajectory_timestamps, output_dir='accuracy_results'
    )
    
    print("\n  ========================================================")
    print("                 FINAL EVALUATION SUMMARY                 ")
    print("  ========================================================")
    print(f"  * Obstacle F1 Accuracy Score:      {grid_metrics['f1_score'] * 100:6.2f} %")
    print(f"  * Precision (SLAM Wall Accuracy):   {grid_metrics['precision'] * 100:6.2f} %")
    print(f"  * Recall (True Wall Coverage):     {grid_metrics['recall'] * 100:6.2f} %")
    print(f"  * Obstacle IoU:                    {grid_metrics['obstacle_iou'] * 100:6.2f} %")
    print(f"  * Mean Chamfer Wall Error:          {grid_metrics['chamfer_distance_meters'] * 100:6.2f} cm")
    print(f"  * 95th-Percentile Hausdorff Error:  {grid_metrics['hausdorff_95_meters'] * 100:6.2f} cm")
    print(f"  * Absolute Trajectory Error (RMSE): {traj_metrics['ate_rmse_meters'] * 100:6.2f} cm")
    print(f"  * Mean Heading Error:              {traj_metrics['mean_heading_error_deg']:6.2f} deg")
    print(f"  * Total Traveled Distance:         {traj_metrics['total_distance_traveled_m']:6.2f} m")
    print("  ========================================================\n")
    
    return done

if __name__ == '__main__':
    main()
