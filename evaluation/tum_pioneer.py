#!/usr/bin/env python3
"""Run our SLAM on the TUM RGB-D Pioneer sequences (Sturm et al., IROS 2012).

  python -m evaluation.tum_pioneer E:/project_phase1/datasets/tum [--set key=value ...]

Each ROS bag (Kinect on a Pioneer 3, planar motion, motion-capture ground truth) is converted once into our recording
format (cached next to the bag as <name>.rec.npz): every depth frame becomes a 2D scan of the obstacle band, wheel
odometry (/pose) becomes the forward/lateral/turn increments (the Pioneer has no gyro, so the turn comes from the
wheels), and the ground truth is the Kinect pose projected onto the floor plane. The recording is then replayed through
SLAMAgent exactly like a simulator run. Scored like the TUM benchmark tool: ATE RMSE after a rigid alignment of the
estimated to the true trajectory (here in the plane, x-y), plus the unaligned error from the true start pose.
"""

import argparse
import ast
import glob
import math
import os

import numpy as np

SEQS = ["pioneer_360", "pioneer_slam", "pioneer_slam2", "pioneer_slam3"]
FX = FY = 525.0
CX, CY = 319.5, 239.5           # camera_info in the bags
BAND_M = (0.15, 0.80)           # obstacle band above the floor
MAX_RANGE_M = 5.0
COL_STEP = 2                    # one scan point per 2 image columns (320 bins)


def _yaw(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def _cam_heading(qx, qy, qz, qw):
    # optical axis (camera z) in the world frame, projected on the floor
    r02 = 2 * (qx * qz + qw * qy)
    r12 = 2 * (qy * qz - qw * qx)
    return math.atan2(r12, r02)


def convert(bag_path, gt_path, out_path):
    from pathlib import Path
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS1_NOETIC)

    gt = np.loadtxt(gt_path)                       # t tx ty tz qx qy qz qw
    gtt, gx, gy, gz = gt[:, 0], gt[:, 1], gt[:, 2], gt[:, 3]
    gyaw = np.unwrap([_cam_heading(*q) for q in gt[:, 4:8]])
    odo, frames, tilt = [], [], -7.5
    v, u = np.mgrid[0:480, 0:640]
    with Reader(Path(bag_path)) as r:
        for c, stamp, raw in r.messages():
            if c.topic == "/pose":
                m = ts.deserialize_ros1(raw, c.msgtype)
                o = m.pose.pose.orientation
                odo.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x,
                            m.pose.pose.position.y, _yaw(o.x, o.y, o.z, o.w)))
            elif c.topic == "/cur_tilt_angle":
                tilt = ts.deserialize_ros1(raw, c.msgtype).data
            elif c.topic == "/camera/depth/image":
                m = ts.deserialize_ros1(raw, c.msgtype)
                t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                if t < gtt[0] or t > gtt[-1]:
                    continue
                z = np.frombuffer(m.data, np.float32).reshape(480, 640)
                ok = np.isfinite(z) & (z > 0.3) & (z < MAX_RANGE_M)
                zc = z[ok]
                xc, yc = (u[ok] - CX) * zc / FX, (v[ok] - CY) * zc / FY   # camera: x right, y down, z forward
                th = math.radians(-tilt)                                   # tilt < 0: looking down
                fwd = zc * math.cos(th) - yc * math.sin(th)
                up = -yc * math.cos(th) - zc * math.sin(th)
                cam_h = float(np.interp(t, gtt, gz))
                h = up + cam_h                                             # height above the floor
                sel = (h > BAND_M[0]) & (h < BAND_M[1])
                cols = (u[ok][sel] // COL_STEP)
                left = -xc[sel]
                rng = np.hypot(fwd[sel], left)
                best = np.full(640 // COL_STEP, np.inf)
                idx = np.full(640 // COL_STEP, -1)
                order = np.argsort(rng)[::-1]                               # nearest written last wins
                best[cols[order]] = rng[order]
                idx[cols[order]] = order
                keep = idx >= 0
                pts = np.stack([fwd[sel][idx[keep]], left[idx[keep]]], 1)
                frames.append((t, pts.astype(np.float32)))
    odo = np.array(odo)
    ot, ox, oy, oyaw = odo[:, 0], odo[:, 1], odo[:, 2], np.unwrap(odo[:, 3])
    t = np.array([f[0] for f in frames])
    px, py, pyaw = np.interp(t, ot, ox), np.interp(t, ot, oy), np.interp(t, ot, oyaw)
    act = np.zeros((len(t), 5))
    # increments applied after the scan of step k (as in slam_replay): motion from k to k+1, in the body frame at k
    dx, dy = np.diff(px, append=px[-1]), np.diff(py, append=py[-1])
    act[:, 0] = dx * np.cos(pyaw) + dy * np.sin(pyaw)
    act[:, 1] = -dx * np.sin(pyaw) + dy * np.cos(pyaw)
    act[:, 2] = np.diff(pyaw, append=pyaw[-1])
    act[:, 4] = 1
    gtr = np.stack([np.interp(t, gtt, gx), np.interp(t, gtt, gy), np.interp(t, gtt, gyaw)], 1)
    offs = np.concatenate([[0], np.cumsum([len(f[1]) for f in frames])])
    np.savez_compressed(out_path, t=t - t[0], gt=gtr, act=act, scan_points=np.concatenate([f[1] for f in frames]),
                        scan_offsets=offs, initial_pose=gtr[0])
    return out_path


def aligned_ate(gt, est):
    """RMSE after the best rigid 2D alignment (Horn/Umeyama without scale), as the TUM tool does in 3D."""
    a, b = est[:, :2], gt[:, :2]
    ma, mb = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((b - mb).T @ (a - ma))
    d = np.diag([1, np.sign(np.linalg.det(u @ vt))])
    rot = u @ d @ vt
    res = (rot @ (a - ma).T).T + mb - b
    return float(np.sqrt((res ** 2).sum(1).mean()))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("data_dir")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")
    ap.add_argument("--seq", nargs="+", default=SEQS)
    a = ap.parse_args()
    from evaluation.slam_replay import replay
    from evaluation.baselines import run_odom
    over = {k: ast.literal_eval(v) for k, v in (s.split("=", 1) for s in a.set)}
    for s in a.seq:
        base = os.path.join(a.data_dir, f"rgbd_dataset_freiburg2_{s}")
        rec_path = base + ".rec.npz"
        if not os.path.exists(rec_path):
            convert(base + ".bag", base + "-groundtruth.txt", rec_path)
        rec = np.load(rec_path)
        m, est = replay(rec_path, over)
        odo = run_odom(rec, 0.0)
        path = float(np.sum(np.hypot(*np.diff(rec["gt"][:, :2], axis=0).T)))
        print(f"  {s:14s} frames {len(rec['t']):5d} | path {path:5.1f} m | ours ATE aligned "
              f"{aligned_ate(rec['gt'], est) * 100:6.1f} cm (from start {m['ate_rmse_meters'] * 100:6.1f}) | "
              f"wheel odometry aligned {aligned_ate(rec['gt'], odo) * 100:6.1f} cm | loops {m['loop_closures']}",
              flush=True)


if __name__ == "__main__":
    main()
