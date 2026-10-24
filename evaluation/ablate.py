#!/usr/bin/env python3
"""Offline component ablation on one run's recording: the full SLAM stack vs. one component removed, all
replayed through the same recorded scans/odometry and scored against the same ground truth.

  python -m evaluation.ablate runs/<run_id>      # writes runs/<run_id>/ablation.csv and ablation.json

The live robot is never modified. Two reproducibility checks are included:
  * determinism: the full system is replayed twice; the trajectories must be identical;
  * replay vs live: the full replay is compared with the trajectory the live run produced.
"""
import contextlib
import csv
import io
import json
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

from evaluation.run_telemetry import rpe_distance
from evaluation.slam_replay import replay

VARIANTS = [
    ("full system", {}),
    ("without scan matching (odometry + gyro only)", {"max_rmse": -1.0}),
    ("without loop closure", {"max_loop_correction": 0.0}),
    ("without degeneracy filter", {"degeneracy_filter": False}),
    ("without Manhattan heading prior", {"manhattan_gain": 0.0}),
    ("full system (repeat, determinism check)", {}),
]
LIVE_TOLERANCE_M = 0.05


def _run(job):
    rec_path, name, overrides = job
    with contextlib.redirect_stdout(io.StringIO()):
        m, est = replay(rec_path, overrides)
    return name, m, est


def main(target, workers=4):
    rec_path = os.path.join(target, "recording.npz")
    gt = np.load(rec_path)["gt"]
    live_csv = os.path.join(target, "trajectory.csv")
    live = np.loadtxt(live_csv, delimiter=",", skiprows=1)[:, 4:7] if os.path.exists(live_csv) else None

    with ProcessPoolExecutor(max_workers=workers) as pool:
        results = list(pool.map(_run, [(rec_path, n, o) for n, o in VARIANTS]))

    rows, est_full = [], results[0][2]
    for name, m, est in results:
        t_rmse, t_pct, r_rmse, _ = rpe_distance(gt, est)
        rows.append({"variant": name, "ate_rmse_cm": m["ate_rmse_meters"] * 100, "ate_max_cm": m["ate_max_meters"] * 100,
                     "final_err_cm": float(np.hypot(*(est[-1, :2] - gt[-1, :2]))) * 100,
                     "rpe_trans_pct": t_pct, "rpe_rot_deg_per_m": r_rmse,
                     "heading_err_deg": m["mean_heading_error_deg"], "loop_closures": m["loop_closures"],
                     "replay_wall_s": m["replay_wall_s"]})
    full = rows[0]
    full["determinism_max_diff_m"] = float(np.abs(results[-1][2] - est_full).max())
    if live is not None:
        n = min(len(live), len(est_full))
        d = np.hypot(*(live[:n, :2] - est_full[:n, :2]).T)
        over = np.nonzero(d > LIVE_TOLERANCE_M)[0]
        full["replay_vs_live_max_cm"] = float(d.max()) * 100
        full["replay_vs_live_first_step_over_5cm"] = int(over[0]) if len(over) else None
        full["replay_steps"] = n
    for r in rows[1:]:
        r["delta_ate_cm_vs_full"] = r["ate_rmse_cm"] - full["ate_rmse_cm"]

    for r in rows:
        print(f"  {r['variant']:46s} ATE {r['ate_rmse_cm']:6.1f} cm | final {r['final_err_cm']:6.1f} cm | "
              f"RPE {r['rpe_trans_pct']:.1f} % {r['rpe_rot_deg_per_m']:.2f} deg/m | heading {r['heading_err_deg']:.2f} deg | "
              f"loops {r['loop_closures']}")
    print(f"  Determinism: max |difference| between two identical replays = {full['determinism_max_diff_m']:.3g} m")
    if "replay_vs_live_max_cm" in full:
        print(f"  Replay vs live trajectory: max gap {full['replay_vs_live_max_cm']:.2f} cm over {full['replay_steps']} steps")

    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(os.path.join(target, "ablation.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(target, "ablation.json"), "w", encoding="utf-8") as f:
        json.dump({"recording": rec_path, "n_runs": 1, "rows": rows}, f, indent=2)
    print(f"  [SAVED] {target}/ablation.csv (one recording: differences are not statistically tested)")


if __name__ == "__main__":
    main(sys.argv[1])
