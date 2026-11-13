#!/usr/bin/env python3
"""Floor calibration: how far the legs really carry the body on this floor, from a recorded run (scoring/setup only).

Walking speed depends on the floor: at the same gait command the body covers ~1.5x more ground in scene 1 than on the
rescue floor. This compares true distance (ground truth) with leg odometry over the walking steps of a recording and
prints the odom_scale for config/scenes/<scene>.json. On a real robot the same number comes from walking a measured
distance.

  python -m evaluation.floor_calibration runs/<run_id>/recording.npz [--current-scale 1.0]
"""
import argparse

import numpy as np


def ratio(recording, cmd=0.75):
    z = np.load(recording)
    t, a, g = z["t"], z["act"], z["gt"]
    d = np.hypot(np.diff(g[:, 0]), np.diff(g[:, 1]))
    walk = (np.abs(a[1:, 3] - cmd) < 1e-3) & (a[1:, 4] > 0)
    if walk.sum() < 200:
        raise SystemExit(f"only {walk.sum()} walking steps at command {cmd}; record a longer run")
    return float(d[walk].sum() / a[1:][walk, 0].sum()), int(walk.sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--current-scale", type=float, default=1.0, help="odom_scale the recording was made with")
    args = ap.parse_args()
    r, n = ratio(args.recording)
    print(f"  true / odometry distance over {n} walking steps: {r:.3f}")
    print(f"  odom_scale for this floor: {args.current_scale * r:.3f}")


if __name__ == "__main__":
    main()
