#!/usr/bin/env python3
"""Ground truth for any scene from the shapes' triangle meshes (scoring only; read with the simulation stopped).

The bounding-box truth (scene_truth.py) works for scene 1, where every wall and object is its own box. Scenes built from
compound shapes (the rescue scene keeps all scattered concrete blocks in one shape with a 13.8 x 12.2 m box) need the real
geometry: every triangle is sampled densely and a map cell is an obstacle when real surface passes through the obstacle
band (0.04-0.25 m above the floor, the band the robot treats as obstacles). The robot's own parts and the floor are
skipped. The scored area is the floor's footprint clipped to the map window.

  python -m evaluation.scene_truth_mesh <scene_key>   # writes evaluation/data/scene_truth_<key>.json + _grid.npy
"""
import json
import os
import sys

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

RES = 0.05                            # the robot's map grid (main_agentic GRID_RES)
BAND = (0.04, 0.25)                   # metres above the floor top
SAMPLE = 0.01                         # surface sampling step (m)


def _triangles_world(sim, h):
    v, idx, _ = sim.getShapeMesh(h)
    v = np.asarray(v, dtype=np.float64).reshape(-1, 3)
    m = np.asarray(sim.getObjectMatrix(h, sim.handle_world), dtype=np.float64).reshape(3, 4)
    vw = v @ m[:, :3].T + m[:, 3]
    return vw[np.asarray(idx, dtype=np.int64).reshape(-1, 3)]


def _sample(tris):
    out = []
    for a, b, c in tris:
        n = int(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / SAMPLE) + 1
        u, w = np.meshgrid(np.linspace(0, 1, n + 1), np.linspace(0, 1, n + 1))
        keep = (u + w) <= 1.0
        u, w = u[keep], w[keep]
        out.append(a + np.outer(u, b - a) + np.outer(w, c - a))
    return np.vstack(out) if out else np.empty((0, 3))


def build(key, half=None):
    """half: map window half-size (m); default the scene profile's map_half_m, else main_agentic's 5.5."""
    if half is None:
        prof = f"config/scenes/{key}.json"
        half = json.load(open(prof, encoding="utf-8")).get("map_half_m", 5.5) if os.path.exists(prof) else 5.5
    X0 = -float(half)
    N = int(round(2 * half / RES)) + 1    # BayesianOccupancyGrid cell count for that window
    sim = RemoteAPIClient().require("sim")
    shapes = [h for h in sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)]
    robot = None
    for alias in ("/Base", "/guard15/Base"):
        try:
            robot = sim.getObject(alias)
            break
        except Exception:
            pass
    robot_parts = set(sim.getObjectsInTree(robot, sim.handle_all, 0)) if robot is not None else set()
    floor_top, floor_box, occ = 0.0, None, np.zeros((N, N), bool)
    for h in shapes:
        alias = sim.getObjectAlias(h, 2)
        if "floor" in alias.lower():
            tris = _triangles_world(sim, h)
            pts = tris.reshape(-1, 3)
            floor_top = max(floor_top, float(pts[:, 2].max()))
            box = (pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(), pts[:, 1].max())
            floor_box = box if floor_box is None else (min(floor_box[0], box[0]), max(floor_box[1], box[1]),
                                                        min(floor_box[2], box[2]), max(floor_box[3], box[3]))
    for h in shapes:
        alias = sim.getObjectAlias(h, 2)
        if h in robot_parts or "floor" in alias.lower():
            continue
        try:
            p = _sample(_triangles_world(sim, h))
        except Exception:
            continue
        z = p[:, 2] - floor_top
        p = p[(z >= BAND[0]) & (z <= BAND[1])]
        gx = np.floor((p[:, 0] - X0) / RES).astype(int)
        gy = np.floor((p[:, 1] - X0) / RES).astype(int)
        ok = (gx >= 0) & (gx < N) & (gy >= 0) & (gy < N)
        occ[gy[ok], gx[ok]] = True
    os.makedirs("evaluation/data", exist_ok=True)
    grid_path = f"evaluation/data/scene_truth_{key}_grid.npy"
    np.save(grid_path, occ)
    lo, hi = X0, X0 + N * RES
    fb = floor_box or (lo, hi, lo, hi)
    area = [max(lo, fb[0]), min(hi, fb[1]), max(lo, fb[2]), min(hi, fb[3])]
    out = f"evaluation/data/scene_truth_{key}.json"
    json.dump({"grid": grid_path, "x0": X0, "area_xyxy": area, "band_above_floor_m": BAND, "floor_top_z": floor_top},
              open(out, "w", encoding="utf-8"), indent=1)
    print(f"  obstacle cells {int(occ.sum())} ({occ.sum() * RES * RES:.1f} m2) | scored area x {area[0]:.2f}..{area[1]:.2f} "
          f"y {area[2]:.2f}..{area[3]:.2f} | floor top z {floor_top:.3f} -> {out}")
    return out


if __name__ == "__main__":
    build(sys.argv[1] if len(sys.argv) > 1 else "scene")
