#!/usr/bin/env python3
"""SLAM baselines on the same recordings and the same scoring as evaluation/slam_replay.py.

  odom       dead reckoning: leg odometry (forward + lateral) and gyro, gyro bias from the stand-still calibration
  breezy     BreezySLAM RMHC_SLAM (CoreSLAM/tinySLAM, Steux & El Hamzaoui 2010), github.com/simondlevy/BreezySLAM,
             default parameters; scan + forward odometry + gyro (it takes no lateral odometry)
  breezy0    BreezySLAM Deterministic_SLAM: same input, no scan matching (checks the frame conversion = odom)

  python -m evaluation.baselines runs/<run>/recording.npz --calibrate-s 62 --method odom breezy
"""

import argparse
import math
import time
import numpy as np

from evaluation.accuracy_evaluator import evaluate_trajectory

N_BINS = 360            # range bins across the camera field of view
MAP_PIXELS, MAP_METERS = 800, 20.0   # 2.5 cm cells, covers both scenes from any start


def _odometry(rec, calibrate_s):
    """Per-step odometry increments (fwd m, lat m, dyaw rad) applied after the scan of that step, bias-corrected."""
    t, act = rec["t"], rec["act"]
    fresh = act[:, 4] > 0
    fwd, lat, dyaw = act[:, 0] * fresh, act[:, 1] * fresh, act[:, 2] * fresh
    bias = 0.0
    if calibrate_s > 0:   # robot stands still: mean gyro rate is bias (what our SLAM's calibration estimates)
        m = t < calibrate_s
        bias = dyaw[m].sum() / max(t[m][-1] - t[0], 1e-6)
    dt = np.diff(t, prepend=t[0])
    return fwd, lat, dyaw - bias * dt * fresh, bias


def run_odom(rec, calibrate_s):
    x, y, yaw = rec["initial_pose"]
    fwd, lat, dyaw, _ = _odometry(rec, calibrate_s)
    est = np.empty((len(rec["t"]), 3))
    for k in range(len(est)):
        est[k] = x, y, yaw
        yaw += dyaw[k]
        x += fwd[k] * math.cos(yaw) - lat[k] * math.sin(yaw)
        y += fwd[k] * math.sin(yaw) + lat[k] * math.cos(yaw)
    return est


def _fov_deg(rec):
    p = rec["scan_points"][: 200000]
    a = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
    return 2 * float(np.percentile(np.abs(a), 99.9)), float(np.percentile(np.hypot(p[:, 0], p[:, 1]), 99.9))


def run_breezy(rec, calibrate_s, deterministic=False, sigma_xy_mm=100.0, sigma_theta_deg=20.0, hole_mm=600.0, every=1):
    from breezyslam.algorithms import RMHC_SLAM, Deterministic_SLAM
    from breezyslam.sensors import Laser
    t, pts, offs = rec["t"], rec["scan_points"], rec["scan_offsets"]
    x0, y0, yaw0 = rec["initial_pose"]
    fov, rmax = _fov_deg(rec)
    rate = 1.0 / float(np.median(np.diff(t)))
    laser = Laser(N_BINS, rate / every, fov, int(rmax * 1000))
    slam = (Deterministic_SLAM(laser, MAP_PIXELS, MAP_METERS, hole_width_mm=hole_mm) if deterministic else
            RMHC_SLAM(laser, MAP_PIXELS, MAP_METERS, hole_width_mm=hole_mm, sigma_xy_mm=sigma_xy_mm,
                      sigma_theta_degrees=sigma_theta_deg))
    fwd, _, dyaw, _ = _odometry(rec, calibrate_s)
    edges = np.linspace(-fov / 2, fov / 2, N_BINS + 1)
    bx, by, _ = slam.getpos()
    c, s = math.cos(yaw0), math.sin(yaw0)
    est = np.empty((len(t), 3))
    acc_d, acc_th, last_t = 0.0, 0.0, float(t[0])
    for k in range(len(t)):
        if k % every:   # skipped scan: keep accumulating odometry, hold the last pose
            est[k] = est[k - 1]
            acc_d += fwd[k]
            acc_th += dyaw[k]
            continue
        q = pts[offs[k]:offs[k + 1]]
        scan = [0.0] * N_BINS   # 0 = no return in that bin
        if len(q):
            a = np.degrees(np.arctan2(q[:, 1], q[:, 0]))
            r = np.hypot(q[:, 0], q[:, 1]) * 1000.0
            i = np.clip(np.searchsorted(edges, a) - 1, 0, N_BINS - 1)
            best = np.full(N_BINS, np.inf)
            np.minimum.at(best, i, r)
            scan = np.where(np.isfinite(best), best, 0.0).tolist()
        dt = float(t[k]) - last_t
        slam.update(scan, (acc_d * 1000.0, math.degrees(acc_th), dt))
        acc_d, acc_th, last_t = 0.0, 0.0, float(t[k])
        px, py, pth = slam.getpos()
        dx, dy = (px - bx) / 1000.0, (py - by) / 1000.0     # BreezySLAM frame: start at map centre, heading 0
        est[k] = x0 + c * dx - s * dy, y0 + s * dx + c * dy, yaw0 + math.radians(pth)
        acc_d += fwd[k]
        acc_th += dyaw[k]
    return est


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--calibrate-s", type=float, default=0.0)
    ap.add_argument("--method", nargs="+", default=["odom", "breezy"], choices=["odom", "breezy", "breezy0"])
    ap.add_argument("--sigma-xy-mm", type=float, default=100.0, help="RMHC search spread (BreezySLAM default 100)")
    ap.add_argument("--sigma-theta-deg", type=float, default=20.0, help="RMHC search spread (BreezySLAM default 20)")
    ap.add_argument("--every", type=int, default=1, help="use every n-th scan (odometry accumulated in between)")
    ap.add_argument("--hole-mm", type=float, default=600.0, help="BreezySLAM hole width (default 600)")
    args = ap.parse_args()
    rec = np.load(args.recording)
    for m in args.method:
        t0 = time.perf_counter()
        est = {"odom": lambda: run_odom(rec, args.calibrate_s),
               "breezy": lambda: run_breezy(rec, args.calibrate_s, False, args.sigma_xy_mm, args.sigma_theta_deg, args.hole_mm, args.every),
               "breezy0": lambda: run_breezy(rec, args.calibrate_s, True, hole_mm=args.hole_mm)}[m]()
        r = evaluate_trajectory(rec["gt"], est, timestamps=rec["t"], save_plots=False)
        print(f"  {m:8s} | path {r['total_distance_traveled_m']:.2f} m | ATE RMSE {r['ate_rmse_meters'] * 100:.1f} cm | "
              f"max {r['ate_max_meters'] * 100:.1f} cm | heading {r['mean_heading_error_deg']:.2f} deg | "
              f"{time.perf_counter() - t0:.1f} s", flush=True)


if __name__ == "__main__":
    main()
