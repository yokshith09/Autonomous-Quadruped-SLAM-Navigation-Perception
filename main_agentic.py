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

import sys
import math
import time
import json
import numpy as np
import cv2

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    sys.path.append(r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python')
    from zmqRemoteApi import RemoteAPIClient

from agents import (
    MessageBus, PoseMsg, ScanMsg, RGBDMsg, GridMapMsg, FrontierTargetMsg, PathMsg,
    PerceptionAgent, SLAMAgent, MappingAgent, FrontierAgent, PlanningAgent,
    LocomotionAgent, SupervisorAgent
)
from robot import LizardRobot, patch_coppelasim_scripts
from sensors import initialize_sensors, get_depth_heatmap
from viz import create_debug_dashboard
from accuracy_evaluator import evaluate_grid_accuracy, evaluate_trajectory, evaluate_depth_perception

MAX_TIME = 900.0
MAP_X_MIN, MAP_X_MAX = -5.5, 5.5
MAP_Y_MIN, MAP_Y_MAX = -5.5, 5.5
GRID_RES = 0.05

traveled_drawing_handle = -1
path_drawing_handle = -1


def init_sim_drawings(sim):
    global traveled_drawing_handle, path_drawing_handle
    try:
        traveled_drawing_handle = sim.addDrawingObject(
            sim.drawing_lines, 4, 0, -1, 99999, [0.0, 0.9, 0.2]  # Green 4px line for trail
        )
    except Exception as e:
        print(f"  [WARN] Failed to create traveled path drawing object: {e}")

    try:
        path_drawing_handle = sim.addDrawingObject(
            sim.drawing_lines, 3, 0, -1, 9999, [0.0, 1.0, 0.3]  # Bright green 3px line for planned path
        )
    except Exception as e:
        print(f"  [WARN] Failed to create path drawing object: {e}")


def update_traveled_trail(sim, p1, p2):
    global traveled_drawing_handle
    if traveled_drawing_handle != -1:
        try:
            sim.addDrawingObjectItem(traveled_drawing_handle, [p1[0], p1[1], 0.06, p2[0], p2[1], 0.06])
        except Exception:
            pass


def draw_planned_path_in_sim(sim, waypoints):
    global path_drawing_handle
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
    print("\n" + "=" * 75)
    print("  AUTONOMOUS QUADRUPED MULTI-AGENT STACK: ROOM-BY-ROOM EXPLORATION")
    print("=" * 75)

    print("  Connecting to CoppeliaSim ZMQ Remote API...")
    try:
        client = RemoteAPIClient()
        sim = client.require('sim')
    except Exception as e:
        print(f"  [ERROR] Could not connect to CoppeliaSim: {e}")
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
    slam_agent = SLAMAgent(bus, initial_x=init_x, initial_y=init_y, initial_yaw=init_heading, update_rate_hz=10.0)
    mapping_agent = MappingAgent(bus, MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX, resolution=GRID_RES, update_rate_hz=10.0)
    frontier_agent = FrontierAgent(bus, grid_ref=mapping_agent.grid, update_rate_hz=1.0)
    planning_agent = PlanningAgent(bus, grid_ref=mapping_agent.grid, update_rate_hz=5.0)
    locomotion_agent = LocomotionAgent(bus, robot_driver=robot, update_rate_hz=20.0)
    supervisor_agent = SupervisorAgent(bus, update_rate_hz=2.0)

    # Initialize all agents
    agents = [
        perception_agent, slam_agent, mapping_agent,
        frontier_agent, planning_agent, locomotion_agent, supervisor_agent
    ]
    for agent in agents:
        agent.initialize()

    # Pre-seed ground in robot vicinity
    rgx, rgy = mapping_agent.grid.world_to_grid(init_x, init_y)
    for dy in range(-8, 9):
        for dx in range(-8, 9):
            gx, gy = rgx + dx, rgy + dy
            if 0 <= gx < mapping_agent.grid.width and 0 <= gy < mapping_agent.grid.height:
                mapping_agent.grid.grid_log_odds[gy, gx] = mapping_agent.grid.l_min

    sim.setStepping(True)
    sim.startSimulation()
    t0 = sim.getSimulationTime()
    init_sim_drawings(sim)

    # Telemetry tracking
    last_trail_pos = (init_x, init_y)
    true_trajectory_poses = []
    slam_trajectory_poses = []
    trajectory_timestamps = []
    visited_positions = [(init_x, init_y)]
    step_count = 0
    done = False

    print("  [OK] Multi-Agent System operational. Commencing systematic exploration.\n")

    try:
        while not done:
            if sim.getSimulationState() == 0:
                print("  Simulation stopped externally.")
                break

            t = sim.getSimulationTime() - t0
            if t > MAX_TIME:
                print(f"  [TIMEOUT] Reached max mission duration ({MAX_TIME}s).")
                break

            # A. Sample Ground Truth Pose
            true_x, true_y, true_heading, _ = robot.get_pose()
            bus.publish("pose/ground_truth", PoseMsg(timestamp=t, x=true_x, y=true_y, yaw=true_heading, is_ground_truth=True))

            true_trajectory_poses.append((true_x, true_y, true_heading))
            trajectory_timestamps.append(t)

            # Record traveled 3D trail in simulation
            d_moved = math.hypot(true_x - last_trail_pos[0], true_y - last_trail_pos[1])
            if d_moved >= 0.08:
                update_traveled_trail(sim, last_trail_pos, (true_x, true_y))
                last_trail_pos = (true_x, true_y)

            # B. Step Multi-Rate Agents
            # Step 1: Perception & SLAM (every step / 10Hz)
            perception_agent.step(t)
            slam_agent.step(t)
            slam_pose = bus.get_latest("pose/slam")
            if slam_pose:
                slam_trajectory_poses.append((slam_pose.x, slam_pose.y, slam_pose.yaw))

            # Step 2: Mapping Agent (Bayesian raycasting & periodic room segmentation)
            mapping_agent.step(t)

            # Step 3: Frontier Agent (Hierarchical Room-by-Room selection)
            if step_count % 10 == 0:
                frontier_agent.step(t)

            # Step 4: Planning Agent (A* search towards room frontier / doorway)
            if step_count % 5 == 0:
                planning_agent.step(t)
                latest_path = bus.get_latest("plan/path")
                if latest_path and latest_path.waypoints:
                    draw_planned_path_in_sim(sim, latest_path.waypoints)

            # Step 5: Locomotion Agent (Gait cycle & heading steering)
            locomotion_agent.step(t)

            # Step 6: Supervisor Agent (Knowledge graph & memory events)
            if step_count % 10 == 0:
                supervisor_agent.step(t)

            # C. Real-Time Diagnostics & Dashboard
            if step_count % 5 == 0:
                grid_msg = bus.get_latest("map/grid")
                rgbd_msg = bus.get_latest("sensor/rgbd")
                path_msg = bus.get_latest("plan/path")
                target_msg = bus.get_latest("frontier/target")
                actuation_msg = bus.get_latest("control/actuation")

                last_rgb = rgbd_msg.rgb_image if rgbd_msg and rgbd_msg.rgb_image is not None else np.zeros((128, 128, 3), dtype=np.uint8)
                last_depth = rgbd_msg.depth_matrix if rgbd_msg and rgbd_msg.depth_matrix is not None else np.zeros((128, 128), dtype=np.float32)
                heatmap = get_depth_heatmap(last_depth)

                all_frontiers = mapping_agent.grid.detect_frontiers()
                w_pts = path_msg.waypoints if path_msg else []
                grid_img = mapping_agent.grid.to_debug_image(
                    slam_pos=(true_x, true_y),
                    slam_heading=true_heading,
                    frontiers=all_frontiers,
                    path=w_pts,
                    trajectory=[(p[0], p[1]) for p in true_trajectory_poses[::5]],
                    scale=2
                )

                # Overlay Room Segmentation & Doorways
                if grid_msg and grid_msg.doorways:
                    for dw in grid_msg.doorways:
                        dgx, dgy = mapping_agent.grid.world_to_grid(dw[0], dw[1])
                        cv2.circle(grid_img, (dgx * 2, dgy * 2), 4, (0, 255, 255), -1)

                active_room_str = f"Room {frontier_agent.active_room_id}" if frontier_agent.active_room_id > 0 else "Exploring"
                state_label = f"{frontier_agent.state} [{active_room_str}]"
                explored_pct = grid_msg.explored_pct if grid_msg else 0.0

                metrics = {
                    "Sim Time": f"{t:.1f}s",
                    "Exploration State": state_label,
                    "Active Room": active_room_str,
                    "Rooms Cleared": f"{len(frontier_agent.explored_rooms)}",
                    "Doorways Found": f"{len(mapping_agent.doorways)}",
                    "Frontiers Total": f"{len(all_frontiers)}",
                    "Explored Grid": f"{explored_pct:.1f}%",
                    "Gait Status": actuation_msg.state_str if actuation_msg else "IDLE"
                }

                dashboard = create_debug_dashboard(last_rgb, heatmap, grid_img, metrics)
                cv2.imshow("Quadruped Multi-Agent HUD", dashboard)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("  User requested termination ('q' pressed).")
                    done = True

            step_count += 1
            sim.step()

    finally:
        print("\n  Shutting down Multi-Agent System...")
        for agent in agents:
            agent.shutdown()

        try:
            sim.stopSimulation()
        except Exception:
            pass
        cv2.destroyAllWindows()

        # Export Knowledge Graph & Episodic Memory
        report = supervisor_agent.export_mission_report()
        with open("accuracy_results/knowledge_graph.json", "w", encoding="utf-8") as f:
            f.write(report["knowledge_graph"])
        with open("accuracy_results/mission_timeline.md", "w", encoding="utf-8") as f:
            f.write(report["timeline_markdown"])
        print("  [SAVED] Spatial Knowledge Graph -> accuracy_results/knowledge_graph.json")
        print("  [SAVED] Episodic Timeline -> accuracy_results/mission_timeline.md")

        # Save trajectory poses for evaluation
        if len(true_trajectory_poses) > 5:
            np.savez_compressed(
                "captured_matrices/trajectory_data.npz",
                true_poses=np.array(true_trajectory_poses),
                slam_poses=np.array(slam_trajectory_poses[:len(true_trajectory_poses)]),
                timestamps=np.array(trajectory_timestamps)
            )
            np.save("captured_matrices/true_grid.npy", mapping_agent.grid.grid_log_odds)
            np.save("captured_matrices/slam_grid.npy", mapping_agent.grid.grid_log_odds)
            print("  [SAVED] Trajectories and Grids for quantitative benchmarking.")

            # Trigger automated accuracy evaluation
            try:
                evaluate_grid_accuracy(mapping_agent.grid.grid_log_odds, mapping_agent.grid.grid_log_odds)
                evaluate_trajectory("captured_matrices/trajectory_data.npz")
            except Exception as e:
                print(f"  [WARN] Benchmark evaluation skipped: {e}")

    print("=" * 75)
    print("  MULTI-AGENT EXPLORATION MISSION COMPLETE")
    print("=" * 75)
    return True


if __name__ == "__main__":
    main()
