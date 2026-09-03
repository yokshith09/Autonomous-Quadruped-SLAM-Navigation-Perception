#!/usr/bin/env python3
"""
One-shot diagnostic for the two reported failures. Run this BEFORE touching
main.py again. It does not plan or explore -- it just asks the sim four hard
factual questions and prints the answers.

  1. Is the Kinect rig actually parented under /Base? (if not, it never moves
     with the robot, so it can only ever see whatever is near the robot's
     START position -- this alone would explain "not detecting walls")
  2. Are the leg joints actually motor/ctrl enabled, or is dynamics disabled?
  3. Does the base move at all under pure physics (gait only, zero teleport)?
  4. Does the depth camera return real, in-range 3D points, and do any of
     them land in the [0.04m, 0.6m] obstacle height band the grid expects?

Usage:
    python diagnose_sim.py
"""
import sys
import time
import math
import numpy as np

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    sys.path.append(r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python')
    from zmqRemoteApi import RemoteAPIClient

from robot import LizardRobot
from sensors import initialize_sensors, grab_depth_frame, depth_to_world_points


def main():
    print("\nConnecting to CoppeliaSim ZMQ API...")
    client = RemoteAPIClient()
    sim = client.require('sim')

    try:
        sim.stopSimulation()
        time.sleep(0.5)
    except Exception:
        pass

    try:
        base = sim.getObject('/Base')
    except Exception:
        base = sim.getObject('/Back_joint')

    robot = LizardRobot(sim)
    depth_cam, rgb_cam, cam_matrix = initialize_sensors(sim, base)
    if depth_cam is None:
        print("[FATAL] Could not get a depth camera handle at all -- fix this first.")
        return

    # ---- Check 1: camera parenting ----
    print("\n=== CHECK 1: is the Kinect rig parented under /Base? ===")
    try:
        descendants = set(sim.getObjectsInTree(base, sim.handle_all, 0))
        is_child = depth_cam in descendants
        print(f"  depth_cam is a descendant of /Base: {is_child}")
        if not is_child:
            print("  [LIKELY ROOT CAUSE] It is NOT parented under /Base.")
            print("  It will sit still in world space while the robot moves,")
            print("  so it stops seeing anything near the robot after the")
            print("  first few steps. path_planning_nav.py used the path")
            print("  '/Base/Kinect_camera_support/...' (nested under Base);")
            print("  this scene/codebase uses '/Kinect_camera_support/...'")
            print("  (not nested). Re-parent it in the CoppeliaSim scene tree")
            print("  (drag Kinect_camera_support onto Base) and re-save.")
    except Exception as e:
        print(f"  Could not check parenting: {e}")

    # ---- Check 2: joint dynamics ----
    print("\n=== CHECK 2: are the leg/hip joints motor-enabled? ===")
    joints = {'fh': robot.fh, 'bh': robot.bh, 'fl': robot.fl,
              'fr': robot.fr, 'bl': robot.bl, 'br': robot.br}
    for name, handle in joints.items():
        try:
            motor_enabled = sim.getObjectInt32Param(handle, sim.jointintparam_motor_enabled)
            ctrl_enabled = sim.getObjectInt32Param(handle, sim.jointintparam_ctrl_enabled)
            print(f"  {name:3s}: motor_enabled={motor_enabled}  ctrl_enabled={ctrl_enabled}")
        except Exception as e:
            print(f"  {name:3s}: could not read joint params ({e})")
    print("  If either flag is 0 for any joint, that joint cannot actually")
    print("  move the body under physics no matter what target you set.")

    # ---- Check 3: physics-only walk test ----
    print("\n=== CHECK 3: does the base move under gait alone (no teleport)? ===")
    sim.setStepping(True)
    sim.startSimulation()
    t0 = sim.getSimulationTime()
    p0 = sim.getObjectPosition(base, sim.handle_world)
    steps = 0
    while sim.getSimulationTime() - t0 < 5.0:
        t = sim.getSimulationTime() - t0
        robot.apply_gait(t, 1.0)  # full-speed forward, gait only, no advance_kinematic
        sim.step()
        steps += 1
    p1 = sim.getObjectPosition(base, sim.handle_world)
    dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    print(f"  Distance moved by gait/physics alone in 5s ({steps} steps): {dist:.3f} m")
    if dist < 0.05:
        print("  [ROOT CAUSE CONFIRMED] Physics is not propelling the robot.")
        print("  Joint targets are being set but the base doesn't move --")
        print("  dynamics are frozen/disabled in this scene. That's why a")
        print("  teleport fallback exists, and why the walk looks different")
        print("  from the old scene: translation is no longer coupled to")
        print("  the leg cycle at all.")
    else:
        print("  Physics IS moving the robot. KINEMATIC_DRIVE_FALLBACK in")
        print("  main.py may no longer be needed -- try setting it to False.")

    # ---- Check 4: depth sensor sanity ----
    print("\n=== CHECK 4: is the depth sensor returning usable obstacle points? ===")
    raw_matrix = sim.getObjectMatrix(depth_cam, sim.handle_world)
    depth_img = grab_depth_frame(sim, depth_cam, cam_matrix)
    if depth_img is None:
        print("  [BUG] grab_depth_frame() returned None -- no depth data at all.")
    else:
        valid = depth_img[depth_img > 0]
        print(f"  Depth frame shape: {depth_img.shape} | valid px: {len(valid)}")
        if len(valid) == 0:
            print("  [BUG] Depth frame is entirely zero. Check the sensor's")
            print("  near/far clipping planes and that it isn't pointed at")
            print("  its own mounting geometry.")
        else:
            pts = depth_to_world_points(depth_img, cam_matrix, raw_matrix)
            if len(pts) == 0:
                print("  [BUG] 0 world points reconstructed from a non-empty depth frame.")
            else:
                z = pts[:, 2]
                in_band = np.sum((z >= 0.04) & (z <= 0.6))
                print(f"  World points: {len(pts)}")
                print(f"  Height (z) range: min={z.min():.3f} max={z.max():.3f} mean={z.mean():.3f}")
                print(f"  Points in obstacle band [0.04, 0.6]m: {in_band} "
                      f"({100.0 * in_band / len(pts):.1f}%)")
                if in_band == 0:
                    print("  [BUG] Nothing falls in the obstacle height band. Either the")
                    print("  camera is pitched wrong (check raw_matrix rotation) or the")
                    print("  wall geometry in this scene sits outside 4cm-60cm off the floor.")

    sim.stopSimulation()
    robot.stop_motors()
    print("\nDone. Fix whichever CHECK above printed a [BUG]/[ROOT CAUSE] line first --")
    print("that's the one actually blocking wall detection, not the planner or SLAM math.")


if __name__ == '__main__':
    main()
