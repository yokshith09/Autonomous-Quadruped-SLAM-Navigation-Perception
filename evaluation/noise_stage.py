#!/usr/bin/env python3
"""Robustness (sim-to-real) stage: write a recording copy with seeded sensor faults, then replay it.

  python -m evaluation.noise_stage runs/<run>/recording.npz out.npz --seed 1 --depth-sigma 0.02 --dropout 0.2 \
      --gyro-bias-dps 0.01 --gyro-walk-dps 0.002 --odom-noise 0.05 --odom-bias -0.03
  python -m evaluation.slam_replay out.npz ...

Faults (all off by default):
  depth-sigma    range noise on every scan point, metres (Gaussian, along the ray)
  dropout        fraction of scan points removed at random
  gyro-bias-dps  constant gyro bias added after calibration (deg/s): the part a stand-still calibration misses
  gyro-walk-dps  gyro bias random walk, deg/s per sqrt(s)
  odom-noise     per-command multiplicative noise on leg odometry (fraction, Gaussian)
  odom-bias      systematic leg-odometry scale error (fraction): slip or a wrong floor calibration
"""

import argparse
import math
import numpy as np


def perturb(rec, seed, depth_sigma=0.0, dropout=0.0, gyro_bias_dps=0.0, gyro_walk_dps=0.0, odom_noise=0.0,
            odom_bias=0.0, calibrate_s=0.0):
    rng = np.random.default_rng(seed)
    out = {k: rec[k] for k in rec.files}
    pts, offs = rec["scan_points"].astype(np.float64), rec["scan_offsets"]
    if depth_sigma > 0:
        r = np.hypot(pts[:, 0], pts[:, 1])
        scale = 1.0 + rng.normal(0.0, depth_sigma, len(r)) / np.maximum(r, 1e-6)
        pts = pts * scale[:, None]
    if dropout > 0:
        keep = rng.random(len(pts)) >= dropout
        counts = np.add.reduceat(keep.astype(np.int64), offs[:-1]) if len(pts) else np.zeros(len(offs) - 1, np.int64)
        counts[np.diff(offs) == 0] = 0   # reduceat returns the next element for empty scans
        pts = pts[keep]
        offs = np.concatenate([[0], np.cumsum(counts)])
    out["scan_points"], out["scan_offsets"] = pts.astype(rec["scan_points"].dtype), offs.astype(rec["scan_offsets"].dtype)

    t, act = rec["t"], rec["act"].astype(np.float64).copy()
    fresh = act[:, 4] > 0
    dt = np.diff(t, prepend=t[0])
    moving = t >= calibrate_s
    n = int(fresh.sum())
    act[fresh, 0] *= (1.0 + odom_bias) * (1.0 + rng.normal(0.0, odom_noise, n))
    act[fresh, 1] *= (1.0 + odom_bias) * (1.0 + rng.normal(0.0, odom_noise, n))
    walk = np.cumsum(rng.normal(0.0, math.radians(gyro_walk_dps), len(t)) * np.sqrt(dt))
    # gyro increments are delivered on fresh commands; add the bias accumulated since the previous one
    bias_rate = math.radians(gyro_bias_dps) * moving + walk
    acc = np.cumsum(bias_rate * dt)
    idx = np.flatnonzero(fresh)
    add = np.diff(acc[idx], prepend=0.0)
    act[idx, 2] += add
    out["act"] = act.astype(rec["act"].dtype)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("out")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--calibrate-s", type=float, default=0.0)
    for k in ("depth-sigma", "dropout", "gyro-bias-dps", "gyro-walk-dps", "odom-noise", "odom-bias"):
        ap.add_argument(f"--{k}", type=float, default=0.0)
    a = ap.parse_args()
    rec = np.load(a.recording)
    out = perturb(rec, a.seed, a.depth_sigma, a.dropout, a.gyro_bias_dps, a.gyro_walk_dps, a.odom_noise, a.odom_bias,
                  a.calibrate_s)
    np.savez(a.out, **out)


if __name__ == "__main__":
    main()
