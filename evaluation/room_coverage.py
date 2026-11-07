#!/usr/bin/env python3
"""Coverage per real room, from the scene's wall geometry (scoring only; nothing here reaches the agents).

Rooms are the free regions left between the true walls once door-sized gaps are closed. For each room: share of its
floor the camera actually observed (the reference grid built with the true pose, so independent of SLAM error) and
whether the true path entered it. This scores "all rooms explored" against the building, not against the robot's
own room labels.

  python -m evaluation.room_coverage runs/<run_id> [runs/<run_id> ...]
"""
import json
import os
import sys

import cv2
import numpy as np

from evaluation.truth_eval import RES, X0, ARENA_HALF, _centres, load_truth

DOOR_M = 1.2          # gaps up to this wide in a wall line count as doorways between rooms
MIN_ROOM_M2 = 1.5     # smaller free pockets are not rooms
SEEN_PCT = 80.0       # a room counts as explored when this share of its floor was observed


def real_rooms(scene_json, ref):
    walls, objects = load_truth(scene_json, ref)
    N = ref.shape[0]
    X, Y = _centres(N)
    inside = (np.abs(X) < ARENA_HALF) & (np.abs(Y) < ARENA_HALF)
    k = int(round(DOOR_M / RES)) | 1
    # Close doorways along each axis separately, so a gap is bridged only along its own wall line.
    closed = walls.astype(np.uint8)
    closed = cv2.morphologyEx(closed, cv2.MORPH_CLOSE, np.ones((1, k), np.uint8))
    closed = cv2.morphologyEx(closed, cv2.MORPH_CLOSE, np.ones((k, 1), np.uint8)) > 0
    free = inside & ~closed
    n, lab, stats, cents = cv2.connectedComponentsWithStats(free.astype(np.uint8), connectivity=4)
    rooms = []
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA] * RES ** 2
        if area >= MIN_ROOM_M2:
            cx, cy = X0 + (cents[i][0] + 0.5) * RES, X0 + (cents[i][1] + 0.5) * RES
            rooms.append({"mask": lab == i, "area_m2": float(area), "centre": (float(cx), float(cy))})
    occ = walls.copy()
    for m in objects.values():
        occ |= m
    return rooms, occ


def score(run_dir, scene_json="evaluation/data/scene_truth.json"):
    art = os.path.join(run_dir, "artifacts")
    ref = np.load(os.path.join(art, "true_grid.npy"))
    rooms, occ = real_rooms(scene_json, ref)
    seen = (ref <= -0.5) | (ref >= 1.0)
    traj = np.genfromtxt(os.path.join(run_dir, "trajectory.csv"), delimiter=",", names=True)
    gx = np.clip(((traj["gt_x_m"] - X0) / RES).astype(int), 0, ref.shape[1] - 1)
    gy = np.clip(((traj["gt_y_m"] - X0) / RES).astype(int), 0, ref.shape[0] - 1)
    out = []
    for r in sorted(rooms, key=lambda r: (-r["centre"][1], r["centre"][0])):
        floor = r["mask"] & ~occ
        pct = 100.0 * (seen & floor).sum() / max(1, floor.sum())
        entered = bool(r["mask"][gy, gx].any())
        out.append({"centre": r["centre"], "area_m2": r["area_m2"], "floor_seen_pct": float(pct), "entered": entered,
                    "explored": bool(pct >= SEEN_PCT)})
    return out


def main(*run_dirs):
    for rd in run_dirs:
        res = score(rd)
        n_exp = sum(r["explored"] for r in res)
        print(f"{rd}: {len(res)} real rooms | explored (>= {SEEN_PCT:.0f} % floor seen) {n_exp} | entered "
              f"{sum(r['entered'] for r in res)}")
        for k, r in enumerate(res, 1):
            print(f"  room {k}: centre ({r['centre'][0]:+.1f}, {r['centre'][1]:+.1f}) | {r['area_m2']:5.1f} m2 | floor seen "
                  f"{r['floor_seen_pct']:5.1f} % | {'entered' if r['entered'] else 'NOT entered'}")
        json.dump(res, open(os.path.join(rd, "room_coverage.json"), "w"), indent=1)


if __name__ == "__main__":
    main(*sys.argv[1:])
