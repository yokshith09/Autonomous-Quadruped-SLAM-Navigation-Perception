#!/usr/bin/env python3
"""Autonomous Quadruped Multi-Agent Exploration & Navigation Orchestrator.

Deconstructs monolithic simulation loops into specialized autonomous agents:
- PerceptionAgent: RGB-D streaming, pinhole 3D back-projection, height & radial self-masking
- SLAMAgent: IMU-stabilized SVD-ICP scan matching & pose estimation
- MappingAgent: Bayesian log-odds occupancy grid & Watershed topological room segmentation
- FrontierAgent: Hierarchical room-by-room frontier selection & doorway transitions
- PlanningAgent: Vector-dilated A* search & line-of-sight path smoothing
- LocomotionAgent: Sinusoidal quadruped gait generator & heading steering
- SupervisorAgent: Spatial Knowledge Graph custodian & Episodic Memory timeline manager
"""

import os
import sys
import math
import time
import json
import argparse
import platform
import shutil
import numpy as np
import cv2

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    sys.path.append(r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python')
    from zmqRemoteApi import RemoteAPIClient

from agents import (
    MessageBus, PoseMsg, PerceptionAgent, SLAMAgent, MappingAgent, FrontierAgent, PlanningAgent,
    LocomotionAgent, SupervisorAgent
)
from robot import LizardRobot, patch_coppelasim_scripts, norm_angle
from sensors import initialize_sensors, get_depth_heatmap
from grid import BayesianOccupancyGrid
from viz import create_debug_dashboard
from accuracy_evaluator import evaluate_grid_accuracy, evaluate_trajectory
import report
from run_telemetry import RunTelemetry

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

MAX_TIME = 3600.0  # safety cap on simulated seconds; a run is meant to end on MISSION_COMPLETE
MAP_X_MIN, MAP_X_MAX = -5.5, 5.5
MAP_Y_MIN, MAP_Y_MAX = -5.5, 5.5
GRID_RES = 0.05

path_drawing_handle = -1


def cpu_name():
    if sys.platform == "win32":
        try:
            import winreg
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0")
            return winreg.QueryValueEx(key, "ProcessorNameString")[0].strip()
        except OSError:
            pass
    return platform.processor() or platform.machine()


def cleanup_sim_drawings(sim):
    global path_drawing_handle
    for key in ['quadruped_trail_handle', 'quadruped_path_handle']:
        try:
            val = sim.getStringNamedParam(key)
            if val:
                h = int(val)
                sim.addDrawingObjectItem(h, None)
                sim.removeDrawingObject(h)
                sim.setStringNamedParam(key, '')
        except Exception:
            pass
    if path_drawing_handle != -1:
        try:
            sim.addDrawingObjectItem(path_drawing_handle, None)
            sim.removeDrawingObject(path_drawing_handle)
        except Exception:
            pass
    # Sweep CoppeliaSim drawing handles from 0 to 128 to remove any leftover ghost lines from prior runs
    for h_cand in range(128):
        try:
            sim.removeDrawingObject(h_cand)
        except Exception:
            pass
    path_drawing_handle = -1


def init_sim_drawings(sim):
    global path_drawing_handle
    cleanup_sim_drawings(sim)
    # The trajectory is rendered in real time on the 2D HUD dashboard with zero clutter.
    # We only draw the active, dynamically updating planned path waypoints in the 3D scene.
    try:
        path_drawing_handle = sim.addDrawingObject(
            sim.drawing_lines, 3, 0, -1, 9999, [0.1, 0.65, 1.0]  # Crisp cyan path
        )
        sim.setStringNamedParam('quadruped_path_handle', str(path_drawing_handle))
    except Exception as e:
        print(f"  [WARN] Failed to create path drawing object: {e}")


def draw_planned_path_in_sim(sim, waypoints):
    if path_drawing_handle != -1:
        try:
            sim.addDrawingObjectItem(path_drawing_handle, None)  # Clear previous
            if waypoints and len(waypoints) > 1:
                for i in range(len(waypoints) - 1):
                    p1, p2 = waypoints[i], waypoints[i + 1]
                    sim.addDrawingObjectItem(path_drawing_handle, [p1[0], p1[1], 0.08, p2[0], p2[1], 0.08])
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="Autonomous Quadruped Multi-Agent Exploration")
    parser.add_argument("--max-steps", type=int, default=None, help="Maximum simulation steps")
    parser.add_argument("--max-time", type=float, default=MAX_TIME, help="Maximum mission time in seconds")
    parser.add_argument("--headless", action="store_true", help="Run without OpenCV GUI window")
    parser.add_argument("--closed-loop-slam", action="store_true", default=True, help="Navigate strictly using closed-loop SLAM pose (default)")
    parser.add_argument("--ground-truth-nav", action="store_true", help="Bypass SLAM drift with ground-truth navigation (for ablation only)")
    parser.add_argument("--record", default=None, help="SLAM input recording path (.npz); default runs/<run_id>/recording.npz")
    parser.add_argument("--no-record", action="store_true", help="Do not save the SLAM input recording (disables offline ablation)")
    args, _ = parser.parse_known_args()

    print("\n" + "=" * 75)
    print("  AUTONOMOUS QUADRUPED MULTI-AGENT STACK: ROOM-BY-ROOM EXPLORATION")
    print("=" * 75)

    print("  Connecting to CoppeliaSim ZMQ Remote API...")
    try:
        import socket
        with socket.create_connection(("127.0.0.1", 23000), timeout=2.5):
            pass
        client = RemoteAPIClient()
        sim = client.require('sim')
    except Exception as e:
        print(f"  [ERROR] Could not connect to CoppeliaSim (port 23000 not listening): {e}")
        return False

    try:
        sim.stopSimulation()
        time.sleep(0.5)
    except Exception:
        pass

    patch_coppelasim_scripts(sim)

    try:
        base = sim.getObject('/Base')
    except Exception:
        base = sim.getObject('/Back_joint')

    depth_cam, rgb_cam, camera_matrix = initialize_sensors(sim, base)
    if depth_cam is None:
        print("  [ERROR] Sensors not found. Scene may not be running.")
        return False

    robot = LizardRobot(sim)
    init_x, init_y, init_heading, _ = robot.get_pose()

    # 1. Instantiate Message Bus
    bus = MessageBus()

    # 2. Instantiate Autonomous Subsystem Agents
    perception_agent = PerceptionAgent(bus, sim, depth_cam, rgb_cam, camera_matrix, update_rate_hz=10.0)
    slam_agent = SLAMAgent(
        bus, initial_x=init_x, initial_y=init_y, initial_yaw=init_heading,
        use_ground_truth_nav=args.ground_truth_nav, update_rate_hz=10.0
    )
    mapping_agent = MappingAgent(bus, MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX, resolution=GRID_RES, update_rate_hz=10.0)
    frontier_agent = FrontierAgent(bus, grid_ref=mapping_agent.grid, update_rate_hz=1.0)
    planning_agent = PlanningAgent(bus, grid_ref=mapping_agent.grid, update_rate_hz=5.0)
    locomotion_agent = LocomotionAgent(bus, robot_driver=robot, update_rate_hz=20.0)
    supervisor_agent = SupervisorAgent(bus, update_rate_hz=2.0)

    # Reference Ground-Truth Grid for rigorous numerical benchmarking (decoupled from SLAM grid)
    true_grid = BayesianOccupancyGrid(MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX, resolution=GRID_RES)

    # Initialize all agents
    agents = [
        perception_agent, slam_agent, mapping_agent,
        frontier_agent, planning_agent, locomotion_agent, supervisor_agent
    ]
    for agent in agents:
        agent.initialize()
    tel = RunTelemetry()
    tel.attach(bus, agents)
    if not args.no_record and not args.record:
        args.record = os.path.join(tel.dir, "recording.npz")
    print(f"  Run {tel.run_id}: results -> {tel.dir}")

    # Pre-seed ground in robot vicinity on both SLAM grid and True reference grid
    rgx, rgy = mapping_agent.grid.world_to_grid(init_x, init_y)
    for dy in range(-8, 9):
        for dx in range(-8, 9):
            gx, gy = rgx + dx, rgy + dy
            if 0 <= gx < mapping_agent.grid.width and 0 <= gy < mapping_agent.grid.height:
                mapping_agent.grid.grid_log_odds[gy, gx] = mapping_agent.grid.l_min
                true_grid.grid_log_odds[gy, gx] = true_grid.l_min

    sim.setStepping(True)
    sim.startSimulation()
    t0 = sim.getSimulationTime()
    init_sim_drawings(sim)

    # Telemetry tracking
    true_trajectory_poses = []
    slam_trajectory_poses = []
    trajectory_timestamps = []
    live_grid_metrics = {"f1_score": 1.0, "precision": 1.0, "recall": 1.0, "chamfer_distance_meters": 0.0}
    live_drift_m = 0.0
    live_ate_rmse = 0.0
    live_heading_err_deg = 0.0
    step_count = 0
    done = False
    end_reason = "interrupted"
    t = 0.0
    complete_streak = 0  # frontier evaluations in a row reporting MISSION_COMPLETE
    rec = {"t": [], "gt": [], "scans": [], "act": []}

    print("  [OK] Multi-Agent System operational. Commencing systematic exploration.\n")

    try:
        while not done:
            if sim.getSimulationState() == 0:
                print("  Simulation stopped externally.")
                end_reason = "simulation stopped externally"
                break

            if args.max_steps is not None and step_count >= args.max_steps:
                print(f"  [COMPLETED] Reached max steps limit ({args.max_steps}).")
                end_reason = f"max steps ({args.max_steps})"
                break

            t = sim.getSimulationTime() - t0
            if t > args.max_time:
                print(f"  [TIMEOUT] Reached max mission duration ({args.max_time}s).")
                end_reason = f"timeout ({args.max_time:.0f}s) before all frontiers were cleared"
                break

            loop_t0 = time.perf_counter()
            # A. Sample Ground Truth Pose
            true_x, true_y, true_heading, _ = robot.get_pose()
            bus.publish("pose/ground_truth", PoseMsg(timestamp=t, x=true_x, y=true_y, yaw=true_heading, is_ground_truth=True))

            true_trajectory_poses.append((true_x, true_y, true_heading))
            trajectory_timestamps.append(t)

            # B. Step Multi-Rate Agents
            # Step 1: Perception & SLAM (every step / 10Hz)
            tel.step(perception_agent, t)
            if args.record:
                rec_scan = bus.get_latest("sensor/scan")
                # float64, matching what SLAM actually consumes: float32 here would truncate points enough to
                # flip borderline ICP/loop-closure accept decisions, making the replay silently diverge from the
                # live run it is supposed to reproduce exactly.
                rec["scans"].append(np.asarray(rec_scan.points_2d, dtype=np.float64) if rec_scan else np.empty((0, 2), np.float64))
                rec["t"].append(t)
                rec["gt"].append((true_x, true_y, true_heading))
            n_track = len(slam_agent.track_times_ms)
            tel.step(slam_agent, t)
            tel.slam_tracked(slam_agent, n_track, t)
            # One SLAM pose per ground-truth sample, so index i of both trajectories is the same instant.
            slam_pose = bus.get_latest("pose/slam")
            slam_trajectory_poses.append((slam_pose.x, slam_pose.y, slam_pose.yaw) if slam_pose
                                         else (init_x, init_y, init_heading))

            # Step 2: Mapping Agent (Bayesian raycasting & periodic room segmentation)
            tel.step(mapping_agent, t)

            # Update Reference True Grid with true pose (for rigorous, unbiased benchmark comparison)
            scan = bus.get_latest("sensor/scan")
            if scan is not None and getattr(scan, 'base_points_3d', None) is not None and step_count % 2 == 0:
                pts_b = scan.base_points_3d
                if len(pts_b) > 0:
                    cos_gt, sin_gt = math.cos(true_heading), math.sin(true_heading)
                    wx_gt = cos_gt * pts_b[:, 0] - sin_gt * pts_b[:, 1] + true_x
                    wy_gt = sin_gt * pts_b[:, 0] + cos_gt * pts_b[:, 1] + true_y
                    wz_gt = pts_b[:, 2]
                    pts_w_gt = np.column_stack([wx_gt, wy_gt, wz_gt])
                    true_grid.update_from_depth_points(pts_w_gt, (true_x, true_y))

            # Step 3: Frontier Agent (Hierarchical Room-by-Room selection)
            if step_count % 10 == 0:
                tel.step(frontier_agent, t)
                complete_streak = complete_streak + 1 if frontier_agent.state == "MISSION_COMPLETE" else 0
                if complete_streak >= 3:
                    print(f"  [COMPLETED] No reachable frontiers left at t={t:.1f}s. All rooms explored.")
                    end_reason = "all reachable frontiers explored"
                    break

            # Step 4: Planning Agent (A* search towards room frontier / doorway)
            if step_count % 5 == 0:
                tel.step(planning_agent, t)
                latest_path = bus.get_latest("plan/path")
                if latest_path and latest_path.waypoints:
                    draw_planned_path_in_sim(sim, latest_path.waypoints)

            # Step 5: Locomotion Agent (Gait cycle & heading steering)
            tel.step(locomotion_agent, t)
            if args.record:
                act = bus.get_latest("control/actuation")
                fresh = act is not None and act.timestamp == t
                rec["act"].append((act.odom_forward, act.odom_lateral, act.gyro_dyaw, act.speed, 1.0) if fresh
                                  else (0.0, 0.0, 0.0, 0.0, 0.0))

            # Step 6: Supervisor Agent (Knowledge graph & memory events)
            if step_count % 10 == 0:
                tel.step(supervisor_agent, t)

            # C. Real-Time Diagnostics, Telemetry & Dashboard
            actuation_msg = bus.get_latest("control/actuation")
            active_room_str = f"Room {frontier_agent.active_room_id}" if frontier_agent.active_room_id > 0 else "Exploring"
            all_frontiers = mapping_agent.grid.detect_frontiers()

            if step_count % 5 == 0:
                grid_msg = bus.get_latest("map/grid")
                rgbd_msg = bus.get_latest("sensor/rgbd")
                path_msg = bus.get_latest("plan/path")
                target_msg = bus.get_latest("frontier/target")

                last_rgb = rgbd_msg.rgb_image if rgbd_msg and rgbd_msg.rgb_image is not None else np.zeros((128, 128, 3), dtype=np.uint8)
                last_depth = rgbd_msg.depth_matrix if rgbd_msg and rgbd_msg.depth_matrix is not None else np.zeros((128, 128), dtype=np.float32)
                heatmap = get_depth_heatmap(last_depth)

                w_pts = path_msg.waypoints if path_msg else []
                active_pose = bus.get_latest("pose/state")
                hud_x = active_pose.x if active_pose else true_x
                hud_y = active_pose.y if active_pose else true_y
                hud_yaw = active_pose.yaw if active_pose else true_heading
                grid_img = mapping_agent.grid.to_debug_image(
                    slam_pos=(hud_x, hud_y),
                    slam_heading=hud_yaw,
                    frontiers=all_frontiers,
                    path=w_pts,
                    trajectory=[(p[0], p[1]) for p in slam_trajectory_poses[::5]] or [(true_x, true_y)],
                    scale=2
                )

                # Overlay Room Segmentation, Doorways & Semantic Objects
                if grid_msg and grid_msg.doorways:
                    for dw in grid_msg.doorways:
                        dgx, dgy = mapping_agent.grid.world_to_grid(dw[0], dw[1])
                        cv2.circle(grid_img, (dgx * 2, dgy * 2), 4, (0, 255, 255), -1)
                for ent in supervisor_agent.knowledge_graph.entities.values():
                    if ent.class_name == "chair":
                        egx, egy = mapping_agent.grid.world_to_grid(ent.world_pos[0], ent.world_pos[1])
                        cv2.circle(grid_img, (egx * 2, egy * 2), 4, (255, 0, 255), -1)

                state_label = f"{frontier_agent.state} [{active_room_str}]"
                explored_pct = grid_msg.explored_pct if grid_msg else 0.0
                target_desc = f"({target_msg.target_wx:.2f}, {target_msg.target_wy:.2f}) [{'DW' if target_msg.is_doorway else 'R' + str(target_msg.room_id)}]" if target_msg else "Searching"
                live_f1 = live_grid_metrics.get("f1_score", 1.0)
                live_chamfer = live_grid_metrics.get("chamfer_distance_meters", 0.0)

                metrics = {
                    "Sim Time": f"{t:.1f}s",
                    "Exploration State": state_label,
                    "Active Room": active_room_str,
                    "Target": target_desc,
                    "Rooms Cleared": f"{len(frontier_agent.explored_rooms)}",
                    "Frontiers Total": f"{len(all_frontiers)}",
                    "Explored Grid": f"{explored_pct:.1f}%",
                    "Live Map F1": f"{live_f1 * 100:.1f}%",
                    "Live Wall Chamfer": f"{live_chamfer * 100:.1f}cm",
                    "Drift / Traj ATE": f"{live_drift_m * 100:.1f}cm / {live_ate_rmse * 100:.1f}cm",
                    "Gait Status": actuation_msg.state_str if actuation_msg else "IDLE"
                }

                dashboard = create_debug_dashboard(last_rgb, heatmap, grid_img, metrics)

                # Persist live dashboard image so output can be inspected visually at any time
                os.makedirs("accuracy_results", exist_ok=True)
                cv2.imwrite("accuracy_results/live_hud_latest.png", dashboard)

                if not args.headless:
                    try:
                        cv2.imshow("Quadruped Multi-Agent HUD", dashboard)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q'):
                            print("  User requested termination ('q' pressed).")
                            end_reason = "user pressed q"
                            done = True
                    except Exception:
                        pass

            # Real-time console telemetry & live accuracy evaluation
            if step_count % 200 == 0:
                slam_pose = bus.get_latest("pose/slam")
                icp_rmse = getattr(slam_pose, 'rmse', 0.0) if slam_pose else 0.0
                live_drift_m = math.hypot(slam_pose.x - true_x, slam_pose.y - true_y) if slam_pose else 0.0
                live_heading_err_deg = math.degrees(abs(norm_angle(slam_pose.yaw - true_heading))) if slam_pose else 0.0

                min_len = min(len(true_trajectory_poses), len(slam_trajectory_poses))
                if min_len > 0:
                    live_ate_rmse = math.sqrt(sum((true_trajectory_poses[i][0] - slam_trajectory_poses[i][0])**2 +
                                                  (true_trajectory_poses[i][1] - slam_trajectory_poses[i][1])**2
                                                  for i in range(min_len)) / min_len)
                else:
                    live_ate_rmse = 0.0

                # Compute live map accuracy metrics (true reference grid vs SLAM grid)
                try:
                    live_grid_metrics = evaluate_grid_accuracy(
                        true_grid.grid_log_odds, mapping_agent.grid.grid_log_odds,
                        resolution=GRID_RES, tolerance_m=0.15, save_plots=False
                    )
                except Exception:
                    pass

                live_f1 = live_grid_metrics.get("f1_score", 1.0)
                live_chamfer = live_grid_metrics.get("chamfer_distance_meters", 0.0)
                target_msg = bus.get_latest("frontier/target")
                target_desc = f"({target_msg.target_wx:.2f}, {target_msg.target_wy:.2f}) [{'Doorway' if target_msg.is_doorway else 'Room ' + str(target_msg.room_id)}]" if target_msg else "Searching"

                grid_msg = bus.get_latest("map/grid")
                exp_pct = grid_msg.explored_pct if grid_msg else 0.0
                gait_str = actuation_msg.state_str if actuation_msg else "IDLE"

                if step_count % 600 == 0:  # concise progress line every 30 s of sim time
                    print(f"  [t={t:6.0f}s] state {frontier_agent.state} | rooms cleared {len(frontier_agent.explored_rooms)}"
                          f" | explored {exp_pct:.1f}% | ATE {live_ate_rmse * 100:.0f} cm | err now {live_drift_m * 100:.0f} cm"
                          f" | yaw err {live_heading_err_deg:.1f} deg | F1 {live_f1 * 100:.0f}% | {gait_str}", flush=True)

            tel.loop_time((time.perf_counter() - loop_t0) * 1000.0)
            step_count += 1
            sim.step()

    finally:
        print("\n  Shutting down Multi-Agent System...")
        for agent in agents:
            agent.shutdown()

        cleanup_sim_drawings(sim)

        try:
            sim.stopSimulation()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        os.makedirs("accuracy_results", exist_ok=True)
        os.makedirs("captured_matrices", exist_ok=True)

        if args.record and rec["act"]:
            n_rec = len(rec["act"])
            scans = rec["scans"][:n_rec]
            np.savez_compressed(
                args.record, t=np.array(rec["t"][:n_rec]), gt=np.array(rec["gt"][:n_rec]), act=np.array(rec["act"]),
                scan_points=np.concatenate(scans) if scans else np.empty((0, 2), np.float64),
                scan_offsets=np.cumsum([0] + [len(sc) for sc in scans]),
                initial_pose=np.array([init_x, init_y, init_heading]))
            print(f"  [SAVED] SLAM input recording ({n_rec} steps) -> {args.record}")

        # Export Knowledge Graph & Episodic Memory
        report = supervisor_agent.export_mission_report()
        with open("accuracy_results/knowledge_graph.json", "w", encoding="utf-8") as f:
            f.write(report["knowledge_graph"])
        with open("accuracy_results/mission_timeline.md", "w", encoding="utf-8") as f:
            f.write(report["timeline_markdown"])
        print("  [SAVED] Spatial Knowledge Graph -> accuracy_results/knowledge_graph.json")
        print("  [SAVED] Episodic Timeline -> accuracy_results/mission_timeline.md")

        grid_metrics = None
        # Save trajectory poses for evaluation
        if len(true_trajectory_poses) > 5:
            np.savez_compressed(
                "captured_matrices/trajectory_data.npz",
                true_poses=np.array(true_trajectory_poses),
                slam_poses=np.array(slam_trajectory_poses[:len(true_trajectory_poses)]),
                timestamps=np.array(trajectory_timestamps)
            )
            np.save("captured_matrices/true_grid.npy", true_grid.grid_log_odds)
            np.save("captured_matrices/slam_grid.npy", mapping_agent.grid.grid_log_odds)
            print("  [SAVED] Trajectories and Grids for quantitative benchmarking.")

            # Trigger automated accuracy evaluation (true reference grid vs active SLAM grid)
            try:
                grid_metrics = evaluate_grid_accuracy(
                    true_grid.grid_log_odds, mapping_agent.grid.grid_log_odds,
                    resolution=GRID_RES, tolerance_m=0.15, output_dir="accuracy_results"
                )
                min_len = min(len(true_trajectory_poses), len(slam_trajectory_poses))
                if min_len > 5:
                    traj_metrics = evaluate_trajectory(
                        true_trajectory_poses[:min_len], slam_trajectory_poses[:min_len],
                        timestamps=trajectory_timestamps[:min_len], output_dir="accuracy_results"
                    )
                    _, _, _, final_explored_pct = mapping_agent.grid.get_grid_stats()

                    def mean_ms(samples):
                        return float(np.mean(samples)) if samples else None

                    mem_bytes = mapping_agent.grid.grid_log_odds.nbytes
                    if mapping_agent.room_labels is not None:
                        mem_bytes += mapping_agent.room_labels.nbytes
                    with open("accuracy_results/latest_metrics.json", "w", encoding="utf-8") as f:
                        json.dump({
                            **grid_metrics,
                            **traj_metrics,
                            "explored_pct": float(final_explored_pct),
                            "cpu_track_ms": mean_ms(slam_agent.track_times_ms),
                            "cpu_map_ms": mean_ms(getattr(mapping_agent, "map_times_ms", [])),
                            "cpu_plan_ms": mean_ms(getattr(planning_agent, "plan_times_ms", [])),
                            "map_memory_mb": mem_bytes / (1024.0 * 1024.0),
                            "rooms_cleared": len(frontier_agent.explored_rooms),
                            "frontiers_remaining": len(mapping_agent.grid.detect_frontiers()),
                            "loop_closures": len(slam_agent.pose_estimator.loop_manager.loop_closures),
                            "odom_distance_m": slam_agent.odom_distance_m,
                            "mission_time_s": float(t),
                            "end_reason": end_reason,
                            "closed_loop_slam": not bool(args.ground_truth_nav),
                            "odometry_model": "commanded gait kinematics + noisy simulated gyro",
                            "cpu_name": cpu_name(),
                        }, f, indent=2)
            except Exception as e:
                print(f"  [WARN] Benchmark evaluation skipped: {e}", flush=True)

        free_cells, _, _, explored_pct = mapping_agent.grid.get_grid_stats()
        n_rooms = len(supervisor_agent.knowledge_graph.rooms)
        tel.finalize(
            agents=agents, gt=true_trajectory_poses, est=slam_trajectory_poses, times=trajectory_timestamps,
            grid_metrics=grid_metrics,
            ctx={
                "end_reason": end_reason, "closed_loop_slam": not bool(args.ground_truth_nav),
                "rooms_discovered": n_rooms, "doorways_detected": len(supervisor_agent.knowledge_graph.doorways), "rooms_completed": len(frontier_agent.explored_rooms),
                "free_area_m2": float(free_cells) * GRID_RES ** 2, "explored_pct": float(explored_pct),
                "frontiers_remaining": len(mapping_agent.grid.detect_frontiers()),
                "map_memory_mb": mapping_agent.grid.grid_log_odds.nbytes / 2 ** 20,
                "odom_distance_m": slam_agent.odom_distance_m, "cpu_name": cpu_name(),
                "recording": args.record if args.record and rec["act"] else None,
                "config": {"max_time_s": args.max_time, "max_steps": args.max_steps, "grid_res_m": GRID_RES,
                           "map_bounds_m": [MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX],
                           "ground_truth_nav": bool(args.ground_truth_nav), "headless": bool(args.headless),
                           "agent_rates_hz": {a.name: a.update_rate_hz for a in agents}},
                "agent_task": {
                    "PerceptionAgent": f"median {int(np.median(tel.scan_pts)) if tel.scan_pts else 0} scan pts/update",
                    "SLAMAgent": f"{100.0 * np.mean(tel.track_ok) if tel.track_ok else 0:.1f}% scans matched, "
                                 f"{len(slam_agent.pose_estimator.loop_manager.loop_closures)} loop closures",
                    "MappingAgent": f"{explored_pct:.1f}% of grid observed",
                    "FrontierAgent": f"{len(frontier_agent.explored_rooms)} rooms completed",
                    "PlanningAgent": f"{planning_agent.replan_count} paths, {tel.events.get('PATH_UNREACHABLE', 0)} unreachable",
                    "LocomotionAgent": f"{tel.events.get('ROBOT_STUCK', 0)} stuck/watchdog events",
                    "SupervisorAgent": f"{sum(tel.events.values())} events logged, {n_rooms} rooms in graph",
                },
            })

        try:
            art = os.path.join(tel.dir, "artifacts")
            os.makedirs(art, exist_ok=True)
            for src in ("captured_matrices/slam_grid.npy", "captured_matrices/true_grid.npy",
                        "accuracy_results/knowledge_graph.json", "accuracy_results/mission_timeline.md"):
                if os.path.exists(src):
                    shutil.copy(src, art)
            report.build(tel.dir)
            print(f"  Next: python ablate.py {tel.dir}   then   python report.py {tel.dir}", flush=True)
        except Exception as e:
            print(f"  [WARN] Report generation failed: {e}", flush=True)

    print("=" * 75)
    print("  MULTI-AGENT EXPLORATION MISSION COMPLETE")
    print("=" * 75, flush=True)
    return True


if __name__ == "__main__":
    main()
