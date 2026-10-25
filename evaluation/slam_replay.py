#!/usr/bin/env python3
"""Offline SLAM benchmark: replay a recording made with `main_agentic.py --record FILE` through SLAMAgent.

The recording holds exactly what SLAM received each step (body-frame scan, gait odometry, gyro increment,
gait speed) plus ground truth for scoring, so SLAM variants can be compared on identical data in seconds.

  python -m evaluation.slam_replay run.npz
  python -m evaluation.slam_replay run.npz --set icp_yaw_weight=0.2 --set degeneracy_filter=False
"""

import argparse
import ast
import time
import numpy as np

from agents import MessageBus, SLAMAgent
from agents.bus import ActuationMsg, ScanMsg
from evaluation.accuracy_evaluator import evaluate_trajectory
from core.robot import gait_body_speed


def replay(path, overrides=None, verbose=False):
    rec = np.load(path)
    t, gt, act = rec["t"], rec["gt"], rec["act"]
    pts, offs = rec["scan_points"], rec["scan_offsets"]
    x0, y0, yaw0 = rec["initial_pose"]

    bus = MessageBus()
    slam = SLAMAgent(bus, initial_x=x0, initial_y=y0, initial_yaw=yaw0)
    for key, value in (overrides or {}).items():
        target = next((o for o in (slam.pose_estimator, slam.pose_estimator.loop_manager) if hasattr(o, key)), None)
        assert target is not None, f"unknown SLAM parameter {key}"
        setattr(target, key, value)
    slam.initialize()

    est = []
    last_cmd_t = None
    start = time.perf_counter()
    for k in range(len(t)):
        bus.publish("sensor/scan", ScanMsg(timestamp=float(t[k]), points_2d=pts[offs[k]:offs[k + 1]].astype(np.float64)))
        slam.step(float(t[k]))
        p = bus.get_latest("pose/slam")
        est.append((p.x, p.y, p.yaw) if p else (x0, y0, yaw0))
        _, lat, dyaw, speed, fresh = act[k]
        if fresh:
            # Forward odometry is recomputed with the current gait calibration, as LocomotionAgent would.
            dt = max(0.01, min(float(t[k]) - last_cmd_t, 0.25)) if last_cmd_t is not None else 0.0
            last_cmd_t = float(t[k])
            fwd = gait_body_speed(float(speed)) * dt
            bus.publish("control/actuation", ActuationMsg(timestamp=float(t[k]), joint_targets={}, heading_error=0.0,
                                                          speed=float(speed), state_str="", odom_forward=float(fwd),
                                                          odom_lateral=float(lat), gyro_dyaw=float(dyaw)))
    wall = time.perf_counter() - start
    m = evaluate_trajectory(gt, np.array(est), timestamps=t, save_plots=False)
    m["loop_closures"] = len(slam.pose_estimator.loop_manager.loop_closures)
    m["replay_wall_s"] = wall
    if verbose:
        print(f"  steps {len(t)} | path {m['total_distance_traveled_m']:.2f} m | ATE RMSE "
              f"{m['ate_rmse_meters'] * 100:.1f} cm | max {m['ate_max_meters'] * 100:.1f} cm | drift "
              f"{m['drift_rate_percent']:.2f} % | heading {m['mean_heading_error_deg']:.2f} deg | loops "
              f"{m['loop_closures']} | {wall:.1f} s")
    return m, np.array(est)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="override an ICPPoseEstimator / KeyframeLoopManager attribute")
    args = ap.parse_args()
    overrides = {k: ast.literal_eval(v) for k, v in (kv.split("=", 1) for kv in args.set)}
    replay(args.recording, overrides, verbose=True)


if __name__ == "__main__":
    main()
