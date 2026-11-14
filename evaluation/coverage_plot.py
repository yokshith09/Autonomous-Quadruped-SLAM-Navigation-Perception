# Full-scene coverage map of a rescue run: python evaluation/coverage_plot.py runs/<run> out.png
import sys, numpy as np, csv
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
run, out = sys.argv[1], sys.argv[2]
slam = np.load(f"{run}/artifacts/slam_grid.npy"); truth = np.load("evaluation/data/scene_truth_searchrescuescene_grid.npy")
vis = np.load("evaluation/data/scene_truth_searchrescuescene_grid_observable.npy")
x0, res, n = -8.25, 0.05, slam.shape[0]; ext = [x0, x0 + n*res, x0, x0 + n*res]
img = np.full((n, n, 3), 0.85)
img[slam < -0.5] = (1, 1, 1)                        # mapped free: white
occ = slam > 0.5
img[occ & truth] = (0.1, 0.5, 0.1)                  # correct obstacle: green
img[occ & ~truth] = (0.9, 0.5, 0)                   # obstacle not in scene: orange
img[truth & ~occ & ~vis] = (0.35, 0.35, 0.35)
img[truth & ~occ & vis] = (0.85, 0.1, 0.1)
r = list(csv.DictReader(open(f"{run}/trajectory.csv")))
gx, gy = [float(a["gt_x_m"]) for a in r], [float(a["gt_y_m"]) for a in r]
ex, ey = [float(a["est_x_m"]) for a in r], [float(a["est_y_m"]) for a in r]
plt.figure(figsize=(9, 9)); plt.imshow(img, origin="lower", extent=ext)
plt.plot(gx, gy, "b-", lw=1, label="true path"); plt.plot(ex, ey, "-", c="orange", lw=1, label="estimated path")
plt.plot(gx[0], gy[0], "k*", ms=12, label="start")
plt.legend(loc="lower right"); plt.title("Rescue run 19 (12 min): white = mapped free, green = correct obstacle,\norange = false obstacle, red = visible obstacle not yet mapped,\ndark grey = obstacle side not visible from floor, light grey = not explored")
plt.savefig(out, dpi=110, bbox_inches="tight")
