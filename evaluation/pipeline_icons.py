"""Small pictures for the pipeline figure, made from the final scene 1 run (20261115_001709).

icon_scan.png   one 2D obstacle scan in the robot frame (robot at the left, looking right)
icon_grid.png   the occupancy grid the robot built
icon_path.png   an A* path on that grid (inflated obstacles), start green, goal red
"""
import heapq
import os

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

RUN = os.path.join("runs", "20261115_001709")
OUT = os.path.join("docs", "report_phase2", "figures")
EXT = (-5.5, 5.5, -5.5, 5.5)
RES = 0.05


def _save(fig, name):
    fig.savefig(os.path.join(OUT, name), dpi=100, bbox_inches="tight", pad_inches=0.02, facecolor="white")
    plt.close(fig)


def _grid_axes():
    fig = plt.figure(figsize=(2.4, 2.4))
    ax = fig.add_axes([0, 0, 1, 1]); ax.axis("off")
    return fig, ax


def _astar(free, s, g):
    """8-connected A* on a boolean free mask; returns a list of (row, col)."""
    H, W = free.shape
    dist, prev, pq = {s: 0.0}, {}, [(0.0, s)]
    while pq:
        _, u = heapq.heappop(pq)
        if u == g:
            break
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                v = (u[0] + dr, u[1] + dc)
                if (dr or dc) and 0 <= v[0] < H and 0 <= v[1] < W and free[v]:
                    nd = dist[u] + (1.414 if dr and dc else 1.0)
                    if nd < dist.get(v, 1e9):
                        dist[v], prev[v] = nd, u
                        heapq.heappush(pq, (nd + np.hypot(v[0] - g[0], v[1] - g[1]), v))
    path, u = [g], g
    while u != s:
        u = prev[u]; path.append(u)
    return path[::-1]


def icons():
    d = np.load(os.path.join(RUN, "recording.npz"))
    grid = np.load(os.path.join(RUN, "artifacts", "slam_grid.npy"))   # [row = y, col = x], log odds

    k = 20000
    o = d["scan_offsets"]
    p = d["scan_points"][o[k]:o[k + 1]]
    fig = plt.figure(figsize=(2.6, 1.8))
    ax = fig.add_axes([0, 0, 1, 1]); ax.axis("off"); ax.set_aspect("equal")
    for a in (-30, 30):   # 60 degree field of view
        ax.plot([0, 6 * np.cos(np.radians(a))], [0, 6 * np.sin(np.radians(a))], color="#999999", lw=0.8, ls="--")
    ax.scatter(p[:, 0], p[:, 1], s=1.2, c="#a3154f")
    ax.plot(0, 0, marker=">", ms=9, color="#1f4e9c")
    ax.set_xlim(-0.4, 6.2); ax.set_ylim(-3.6, 3.6)
    _save(fig, "icon_scan.png")

    fig, ax = _grid_axes()
    ax.imshow(grid, cmap="gray_r", origin="lower", extent=EXT)
    _save(fig, "icon_grid.png")

    occ = (grid > 0.5).astype(np.uint8)
    free = (cv2.dilate(occ, np.ones((7, 7), np.uint8)) == 0) & (grid < -0.5)
    to_cell = lambda x, y: (int((y + 5.5) / RES), int((x + 5.5) / RES))
    s, g = to_cell(-3.5, -4.0), to_cell(3.0, 3.0)
    assert free[s] and free[g], "start or goal not on free floor"
    path = np.array(_astar(free, s, g)) * RES - 5.5    # (y, x) in metres
    fig, ax = _grid_axes()
    ax.imshow(grid, cmap="gray_r", origin="lower", extent=EXT, alpha=0.6)
    ax.plot(path[:, 1], path[:, 0], color="#a3154f", lw=2.2)
    ax.plot(path[0, 1], path[0, 0], "o", color="#2e8b3d", ms=8)
    ax.plot(path[-1, 1], path[-1, 0], "o", color="#c0392b", ms=8)
    _save(fig, "icon_path.png")


if __name__ == "__main__":
    icons()
    print("ok")
