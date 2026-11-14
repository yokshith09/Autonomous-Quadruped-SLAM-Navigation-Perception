"""Turn an operator's route drawn over a map image into ordered route waypoints for a scene profile.

  python -m evaluation.route_from_drawing drawing.png out_plan.png [--spacing 1.4]

The drawing is a cyan line over the full-scene map made by evaluation/coverage_plot.py (axes frame located from the
image: left/top spines; data extent = scene map window). Cyan pixels are sampled every `spacing` metres, each sample is
moved to the nearest floor cell with enough clearance from the scene model's obstacles and from the floor edge, and the
samples are ordered into one short tour from the start pose (greedy nearest neighbour + 2-opt). Prints the JSON list.
"""
import argparse
import json

import cv2
import numpy as np
from PIL import Image

SCENE = "evaluation/data/scene_truth_searchrescuescene.json"
X0, RES = -8.25, 0.05


def frame(dark):
    left = int(np.argmax(dark.sum(0)))
    top = int(np.argmax(dark.sum(1)))
    rows = np.where(dark[:, left])[0]
    cols = np.where(dark[top])[0]
    return left, int(cols.max()), top, int(rows.max())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("drawing")
    ap.add_argument("plan_png")
    ap.add_argument("--spacing", type=float, default=1.4)
    ap.add_argument("--clear-m", type=float, default=0.35)
    ap.add_argument("--edge-m", type=float, default=0.7)
    ap.add_argument("--start", type=float, nargs=2, default=(0.0, -1.8))
    ap.add_argument("--prefix", action="append", default=[], help="x,y waypoints visited first, in this order")
    ap.add_argument("--extra", action="append", default=[], help="x,y waypoints added to the drawn ones")
    a = ap.parse_args()

    img = np.asarray(Image.open(a.drawing).convert("RGB")).astype(int)
    l, r, t, b = frame(img.sum(2) < 250)
    spec = json.load(open(SCENE))
    occ = np.load(spec["grid"]).astype(bool)
    n = occ.shape[0]
    x1 = X0 + n * RES
    sx, sy = (r - l) / (x1 - X0), (b - t) / (x1 - X0)
    cyan = (img[:, :, 2] > 200) & (img[:, :, 1] > 150) & (img[:, :, 0] < 150)
    ys, xs = np.where(cyan)
    wx, wy = X0 + (xs - l) / sx, x1 - (ys - t) / sy

    # sample: one point per spacing x spacing cell (centroid of the drawn pixels in it)
    key = np.stack([np.floor(wx / a.spacing), np.floor(wy / a.spacing)], 1).astype(int)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    pts = np.array([[wx[inv == k].mean(), wy[inv == k].mean()] for k in range(inv.max() + 1)])

    # safe floor: inside the area, clear of obstacles and of the floor edge
    ax0, ax1, ay0, ay1 = spec["area_xyxy"]
    c = X0 + (np.arange(n) + 0.5) * RES
    X, Y = np.meshgrid(c, c)
    inside = (X > ax0) & (X < ax1) & (Y > ay0) & (Y < ay1)
    d_obs = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 5) * RES
    d_edge = cv2.distanceTransform(inside.astype(np.uint8), cv2.DIST_L2, 5) * RES
    safe = inside & (d_obs >= a.clear_m) & (d_edge >= a.edge_m)
    sy_, sx_ = np.where(safe)
    snapped = []
    for p in pts:
        gx, gy = (p - X0) / RES
        k = np.argmin((sx_ - gx) ** 2 + (sy_ - gy) ** 2)
        if np.hypot(sx_[k] - gx, sy_[k] - gy) * RES <= 0.8:
            snapped.append((round(X0 + (sx_[k] + 0.5) * RES, 2), round(X0 + (sy_[k] + 0.5) * RES, 2)))
    pts = np.unique(np.array(snapped), axis=0)
    prefix = [np.array(list(map(float, q.split(",")))) for q in a.prefix]
    extra = [list(map(float, q.split(","))) for q in a.extra]
    if extra:
        pts = np.vstack([pts, extra])
    if prefix:   # drawn points already covered by the prefix are dropped
        pts = np.array([q for q in pts if min(np.hypot(*(q - p)) for p in prefix) > 1.0])

    # order: greedy nearest neighbour from the start, then 2-opt
    order, left_ = [], list(range(len(pts)))
    cur = prefix[-1] if prefix else np.array(a.start)
    while left_:
        k = min(left_, key=lambda i: np.hypot(*(pts[i] - cur)))
        order.append(k); left_.remove(k); cur = pts[k]
    route = [cur] + [pts[i] for i in order]
    improved = True
    while improved:
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                d0 = np.hypot(*(route[i - 1] - route[i])) + np.hypot(*(route[j] - route[j + 1]))
                d1 = np.hypot(*(route[i - 1] - route[j])) + np.hypot(*(route[i] - route[j + 1]))
                if d1 < d0 - 1e-6:
                    route[i:j + 1] = route[i:j + 1][::-1]
                    improved = True
    route = [[float(p[0]), float(p[1])] for p in prefix + route[1:]]

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    lay = np.full(occ.shape + (3,), 0.6)
    lay[inside] = (1, 1, 1)
    lay[occ] = (0.1, 0.1, 0.1)
    fig, axp = plt.subplots(figsize=(8, 8))
    axp.imshow(lay, origin="lower", extent=[X0, x1, X0, x1])
    axp.plot(wx[::7], wy[::7], ".", c="#4fc3f7", ms=1, label="drawn route")
    rr = np.array([a.start] + route)
    axp.plot(rr[:, 0], rr[:, 1], "-o", c="#d81b60", ms=3, lw=1, label=f"waypoints ({len(route)}), in order")
    axp.plot(*a.start, "k*", ms=12, label="start")
    axp.legend(loc="lower right")
    axp.set_title("Operator route from the drawing, snapped to safe floor")
    fig.savefig(a.plan_png, dpi=110, bbox_inches="tight")
    length = float(np.sum(np.hypot(*np.diff(rr, axis=0).T)))
    print(json.dumps(route))
    print(f"# {len(route)} waypoints, tour {length:.1f} m", flush=True)


if __name__ == "__main__":
    main()
