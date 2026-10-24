#!/usr/bin/env python3
"""Offline component ablation on one run's recording: the full SLAM stack vs. one component removed, all
replayed through the same recorded scans/odometry, scored against the same ground truth.

  python ablate.py runs/<run_id>            # writes runs/<run_id>/ablation.csv and ablation.json
  python ablate.py path/to/recording.npz

The live robot is never modified. The FULL replay is also checked against the live run's own trajectory
(replay consistency): a large gap means the replay is not reproducing the run and the ablation is unreliable.
"""
import contextlib
import csv
import io
import json
import math
import os
import sys

import numpy as np

from run_telemetry import rpe_distance
from slam_replay import replay

VARIANTS = [
    ("full system", {}),
    ("without scan matching (odometry + gyro only)", {"max_rmse": -1.0}),
    ("without loop closure", {"max_loop_correction": 0.0}),
    ("without degeneracy filter", {"degeneracy_filter": False}),
    ("without Manhattan heading prior", {"manhattan_gain": 0.0}),
]


def main(target):
    run_dir = target if os.path.isdir(target) else None
    rec_path = os.path.join(target, "recording.npz") if run_dir else target
    out_dir = run_dir or os.path.dirname(os.path.abspath(rec_path))
    gt = np.load(rec_path)["gt"]

    live = None
    if run_dir and os.path.exists(os.path.join(run_dir, "trajectory.csv")):
        live = np.loadtxt(os.path.join(run_dir, "trajectory.csv"), delimiter=",", skiprows=1)[:, 4:7]

    rows = []
    for name, overrides in VARIANTS:
        with contextlib.redirect_stdout(io.StringIO()):
            m, est = replay(rec_path, overrides)
        t_rmse, t_pct, r_rmse, _ = rpe_distance(gt, est)
        row = {"variant": name, "ate_rmse_cm": m["ate_rmse_meters"] * 100, "ate_max_cm": m["ate_max_meters"] * 100,
               "final_err_cm": float(np.hypot(*(est[-1, :2] - gt[-1, :2]))) * 100,
               "rpe_trans_pct": t_pct, "rpe_rot_deg_per_m": r_rmse,
               "heading_err_deg": m["mean_heading_error_deg"], "loop_closures": m["loop_closures"]}
        if name == "full system" and live is not None:
            n = min(len(live), len(est))
            d = np.hypot(*(live[:n, :2] - est[:n, :2]).T)
            div = np.nonzero(d > 0.05)[0]
            row["replay_vs_live_max_cm"] = float(d.max()) * 100
            row["replay_matches_live_until_step"] = int(div[0]) if len(div) else n
            row["replay_steps"] = n
        rows.append(row)
        print(f"  {name:46s} ATE {row['ate_rmse_cm']:6.1f} cm | final {row['final_err_cm']:6.1f} cm | "
              f"RPE {row['rpe_trans_pct']:.1f} % {row['rpe_rot_deg_per_m']:.2f} deg/m | heading {row['heading_err_deg']:.2f} deg",
              flush=True)

    full = rows[0]
    if "replay_vs_live_max_cm" in full:
        ok = full["replay_vs_live_max_cm"] <= 5.0
        print(f"  Replay consistency: full-system replay vs live trajectory max gap {full['replay_vs_live_max_cm']:.1f} cm "
              f"({'OK' if ok else 'DIVERGES at step ' + str(full['replay_matches_live_until_step']) + ' - treat ablation as indicative only'})")
    for r in rows[1:]:
        r["delta_ate_cm_vs_full"] = r["ate_rmse_cm"] - full["ate_rmse_cm"]
    keys = sorted({k for r in rows for k in r}, key=lambda k: list(rows[0]).index(k) if k in rows[0] else 99)
    with open(os.path.join(out_dir, "ablation.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(out_dir, "ablation.json"), "w", encoding="utf-8") as f:
        json.dump({"recording": rec_path, "single_run": True, "rows": rows}, f, indent=2,
                  default=lambda v: None if isinstance(v, float) and not math.isfinite(v) else v)
    print(f"  [SAVED] {out_dir}/ablation.csv  (single recording: differences are not statistically tested)")


if __name__ == "__main__":
    main(sys.argv[1])
