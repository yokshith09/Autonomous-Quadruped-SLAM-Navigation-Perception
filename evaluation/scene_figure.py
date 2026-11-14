"""Two-panel scene figure: (a) scene layout from the scene model, (b) explored map with true and estimated paths.

  python -m evaluation.scene_figure runs/<run> <scene_truth.json> out.png "title"
"""
import csv
import json
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from evaluation import truth_eval as T  # noqa: E402


def truth_grid(run_dir, scene_json):
    ref = np.load(f"{run_dir}/artifacts/true_grid.npy")
    spec = json.load(open(scene_json, encoding="utf-8"))
    n = ref.shape[0]
    x0 = spec.get("x0", T.X0)
    X, Y = T._centres(n, x0)
    if "grid" in spec:
        occ = np.load(spec["grid"]).astype(bool)
        a = spec["area_xyxy"]
        inside = (X > a[0]) & (X < a[1]) & (Y > a[2]) & (Y < a[3])
    else:
        walls, objects = T.load_truth(scene_json, ref)
        occ = walls.copy()
        for m in objects.values():
            occ |= m
        inside = (np.abs(X) < T.ARENA_HALF) & (np.abs(Y) < T.ARENA_HALF)
    return occ, inside, [x0, x0 + n * T.RES, x0, x0 + n * T.RES]


def figure(run_dir, scene_json, out, title):
    occ, inside, ext = truth_grid(run_dir, scene_json)
    slam = np.load(f"{run_dir}/artifacts/slam_grid.npy")
    r = list(csv.DictReader(open(f"{run_dir}/trajectory.csv")))
    g = np.array([[float(a["gt_x_m"]), float(a["gt_y_m"])] for a in r])
    e = np.array([[float(a["est_x_m"]), float(a["est_y_m"])] for a in r])
    fig, ax = plt.subplots(1, 2, figsize=(13, 6.6))
    lay = np.full(occ.shape + (3,), 0.55)
    lay[inside] = (1, 1, 1)
    lay[occ] = (0.1, 0.1, 0.1)
    ax[0].imshow(lay, origin="lower", extent=ext)
    ax[0].plot(g[0, 0], g[0, 1], "r*", ms=13, label="start")
    ax[0].set_title("(a) Scene layout (scene model)")
    ax[0].legend(loc="lower right")
    m = np.full(slam.shape + (3,), 0.82)
    m[slam < -0.5] = (1, 1, 1)
    m[slam > 0.5] = (0.1, 0.1, 0.1)
    ax[1].imshow(m, origin="lower", extent=ext)
    ax[1].plot(g[:, 0], g[:, 1], "b-", lw=0.8, label="true path")
    ax[1].plot(e[:, 0], e[:, 1], "-", c="orange", lw=0.8, label="estimated path")
    ax[1].plot(g[0, 0], g[0, 1], "r*", ms=13, label="start")
    ax[1].set_title("(b) Map built by the robot, with paths")
    ax[1].legend(loc="lower right")
    for a in ax:
        a.set_xlabel("x (m)"); a.set_ylabel("y (m)")
    fig.suptitle(title, fontsize=12)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    figure(*sys.argv[1:5])
