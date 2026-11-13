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


WINDOW = 100   # steps (5 s at 20 Hz): long enough for the sideways sway of each step to cancel


def ratio(recording, cmd=0.75):
    """True net progress / odometry over windows of steady walking. Net progress, not path length: the body sways
    sideways with every step, so its path is longer than the ground it gains (rescue floor: path 0.995, net 0.931),
    and SLAM integrates odometry as progress along the heading."""
    z = np.load(recording)
    a, g = z["act"], z["gt"]
    walk = (np.abs(a[:, 3] - cmd) < 1e-3) & (a[:, 4] > 0)
    true_sum = odo_sum = 0.0
    n = 0
    for k in range(0, len(g) - WINDOW, WINDOW):
        if not walk[k + 1:k + WINDOW + 1].all():
            continue
        h = g[k:k + WINDOW, 2]                       # odometry laid along the true heading: isolates distance error
        f = a[k + 1:k + WINDOW + 1, 0]
        odo_sum += np.hypot((f * np.cos(h)).sum(), (f * np.sin(h)).sum())
        true_sum += np.hypot(*(g[k + WINDOW, :2] - g[k, :2]))
        n += WINDOW
    if n < 2000:
        raise SystemExit(f"only {n} steps of steady walking at command {cmd}; record a longer run")
    return float(true_sum / odo_sum), n


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--current-scale", type=float, default=1.0, help="odom_scale the recording was made with")
    args = ap.parse_args()
    r, n = ratio(args.recording)
    print(f"  true net progress / odometry over {n} walking steps: {r:.3f}")
    print(f"  odom_scale for this floor: {args.current_scale * r:.3f}")


if __name__ == "__main__":
    main()
