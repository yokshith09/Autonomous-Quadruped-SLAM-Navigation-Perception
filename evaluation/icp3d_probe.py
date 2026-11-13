#!/usr/bin/env python3
"""Does matching the full 3D depth view track better than the 2D slice? (offline study, scoring only)

Needs a recording made with 3D clouds (cloud_points / cloud_offsets in recording.npz). Compares three front ends with
loop closures off, so only the per-step tracking is compared:
  * odometry   - leg odometry forward/lateral + gyro yaw
  * 2D         - the system's own scan matcher (slam_replay with loop closures disabled)
  * 3D         - 6-DoF point-to-plane ICP of each 3D view against the last few 3D keyframes, seeded by odometry
Reports ATE against ground truth and how often each 3D step lands closer to the truth than odometry.

  python -m evaluation.icp3d_probe runs/<run_id>/recording.npz [--seconds 600]
"""
import argparse
import math

import numpy as np
from scipy.spatial import cKDTree

from evaluation.slam_replay import replay

KF_DIST, KF_ROT = 0.30, math.radians(15)   # new 3D keyframe after this much motion
SUBMAP_KF = 6                               # keyframes in the local 3D map
MAX_CORR = 0.25                             # correspondence distance (m)
ITERS = 12
FIX_YAW = False      # keep heading from the gyro (as the 2D front end does)
DROP_FLOOR = False   # match only points above the floor (floor carries no x/y information)


def rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def normals(points, tree, k=10):
    _, idx = tree.query(points, k=min(k, len(points)))
    nb = points[idx] - points[idx].mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb, nb)
    w, v = np.linalg.eigh(cov)
    flat = w[:, 0] / np.maximum(w.sum(axis=1), 1e-12)           # 0 = perfectly planar
    return v[:, :, 0], flat


def icp_point_to_plane(src, tgt, tgt_n, tree, R, t):
    """Gauss-Newton point-to-plane ICP on a 6-DoF pose (R, t); returns R, t, rmse, inliers."""
    rmse, n_in = float("inf"), 0
    for _ in range(ITERS):
        p = src @ R.T + t
        d, j = tree.query(p, distance_upper_bound=MAX_CORR)
        ok = np.isfinite(d)
        n_in = int(ok.sum())
        if n_in < 50:
            return R, t, float("inf"), n_in
        p, q, n = p[ok], tgt[j[ok]], tgt_n[j[ok]]
        r = np.einsum("ij,ij->i", p - q, n)
        rmse = float(np.sqrt(np.mean(r ** 2)))
        A = np.hstack([np.cross(p, n), n])                      # d(residual)/d(rotation, translation)
        if FIX_YAW:
            A[:, 2] = 0.0
        x = np.linalg.lstsq(A, -r, rcond=None)[0]
        if FIX_YAW:
            x[2] = 0.0
        dR = rot(x[0], x[1], x[2])
        R, t = dR @ R, dR @ t + x[3:]
        if np.linalg.norm(x) < 1e-5:
            break
    return R, t, rmse, n_in


def odometry_chain(t, act, x0, y0, yaw0):
    xs, x, y, yaw = [], x0, y0, yaw0
    for k in range(len(t)):
        fwd, lat, dyaw, _speed, fresh = act[k]
        if fresh:
            mid = yaw + 0.5 * dyaw
            x += fwd * math.cos(mid) - lat * math.sin(mid)
            y += fwd * math.sin(mid) + lat * math.cos(mid)
            yaw += dyaw
        xs.append((x, y, yaw))
    return np.array(xs)


def track_3d(t, act, clouds, offs, x0, y0, yaw0):
    R, tr = rot(0, 0, yaw0), np.array([x0, y0, 0.0])
    kfs, last_kf = [], None
    tree = tgt = tgt_n = None
    out, closer, judged = [], 0, 0
    for k in range(len(t)):
        fwd, lat, dyaw, _speed, fresh = act[k]
        # prediction from odometry (in the body frame, yaw only)
        if fresh:
            step = np.array([fwd, lat, 0.0])
            R_pred, t_pred = R @ rot(0, 0, dyaw), tr + R @ step
        else:
            R_pred, t_pred = R, tr
        src = clouds[offs[k]:offs[k + 1]].astype(np.float64)
        if DROP_FLOOR:
            src = src[src[:, 2] > 0.0]
        if tree is not None and len(src) >= 100:
            R_new, t_new, rmse, n_in = icp_point_to_plane(src, tgt, tgt_n, tree, R_pred.copy(), t_pred.copy())
            if np.isfinite(rmse) and rmse < 0.05 and n_in >= 0.4 * len(src) and np.linalg.norm(t_new - t_pred) < 0.10:
                R, tr = R_new, t_new
            else:
                R, tr = R_pred, t_pred
        else:
            R, tr = R_pred, t_pred
        yaw = math.atan2(R[1, 0], R[0, 0])
        out.append((tr[0], tr[1], yaw))
        if len(src) >= 100 and (last_kf is None or math.hypot(tr[0] - last_kf[0], tr[1] - last_kf[1]) >= KF_DIST
                                or abs((yaw - last_kf[2] + math.pi) % (2 * math.pi) - math.pi) >= KF_ROT):
            kfs.append(src @ R.T + tr)
            kfs = kfs[-SUBMAP_KF:]
            last_kf = (tr[0], tr[1], yaw)
            tgt = np.vstack(kfs)
            tree = cKDTree(tgt)
            tgt_n, _ = normals(tgt, tree)
    return np.array(out)


def ate(est, gt):
    return float(np.sqrt(np.mean((est[:, 0] - gt[:, 0]) ** 2 + (est[:, 1] - gt[:, 1]) ** 2)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    args = ap.parse_args()
    rec = np.load(args.recording)
    if "cloud_points" not in rec.files:
        raise SystemExit("this recording has no 3D clouds; record a run with the current main_agentic.py")
    t, gt, act = rec["t"], rec["gt"], rec["act"]
    x0, y0, yaw0 = rec["initial_pose"]
    odo = odometry_chain(t, act, x0, y0, yaw0)
    m2d, est2d = replay(args.recording, {"max_loop_correction_cap": 0.0, "max_loop_correction": 0.0})
    m2c, _ = replay(args.recording, {})
    est3d = track_3d(t, act, rec["cloud_points"], rec["cloud_offsets"], x0, y0, yaw0)
    print(f"  steps {len(t)} | mean 3D points per view {np.diff(rec['cloud_offsets']).mean():.0f}")
    print(f"  ATE odometry only           {ate(odo, gt) * 100:6.1f} cm")
    print(f"  ATE 2D front end, no closures {ate(np.array(est2d), gt) * 100:6.1f} cm")
    print(f"  ATE 3D front end, no closures {ate(est3d, gt) * 100:6.1f} cm")
    print(f"  (2D system with its loop closures: {m2c['ate_rmse_meters'] * 100:.1f} cm)")


if __name__ == "__main__":
    main()
