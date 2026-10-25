#!/usr/bin/env python3
"""Score a run against the scene's real geometry (evaluation/data/scene_truth.json, read from CoppeliaSim by scene_truth.py).

  python -m evaluation.truth_eval runs/<run_id>      # writes runs/<run_id>/truth_eval.json

Adds what the sensor-built reference grid cannot give:
  * map accuracy against the real walls and objects (independent of the robot's own sensor),
  * true floor coverage and free space mapped outside the arena,
  * object detection: how many real objects were detected and how many detections are real.
The scene file stores sizes but not orientations, so each footprint's orientation is taken as the one that overlaps the
run's reference grid best (a near-square footprint is unaffected). Used for scoring only; the robot never sees it.
"""
import json
import os
import sys

import cv2
import numpy as np

from evaluation.accuracy_evaluator import evaluate_grid_accuracy

RES, X0 = 0.05, -5.5
ARENA_HALF = 4.95            # inner face of the 10 x 10 m outer wall (wall centres at +/-5 m, 0.10 m thick)
BAND = (0.04, 0.60)          # obstacle height band the perception agent keeps (m)
MATCH_M = 0.5                # a detection within this distance of a real object counts as that object
ROBOT_PREFIXES = ("/Base", "/Kinect", "/kinect")


def _centres(n):
    c = X0 + (np.arange(n) + 0.5) * RES
    return np.meshgrid(c, c)  # X[gy, gx], Y[gy, gx]


def _rect(X, Y, cx, cy, sx, sy):
    return (np.abs(X - cx) <= sx / 2 + RES / 2) & (np.abs(Y - cy) <= sy / 2 + RES / 2)


def load_truth(scene_json, ref_logodds):
    N = ref_logodds.shape[0]
    X, Y = _centres(N)
    ref_occ = cv2.dilate((ref_logodds >= 1.0).astype(np.uint8), np.ones((5, 5), np.uint8)).astype(bool)  # tolerate 10 cm offsets
    objs = json.load(open(scene_json, encoding="utf-8"))["objects"]
    walls, objects = np.zeros((N, N), bool), {}
    for o in objs:
        a, size = o["alias"], o.get("bbox_size")
        if size is None or a.startswith(ROBOT_PREFIXES) or "ResizableFloor" in a:
            continue
        z0, z1 = o["pos"][2] - size[2] / 2, o["pos"][2] + size[2] / 2
        if z1 < BAND[0] or z0 > BAND[1]:
            continue
        cx, cy = o["pos"][:2]
        cand = [_rect(X, Y, cx, cy, size[0], size[1]), _rect(X, Y, cx, cy, size[1], size[0])]
        m = max(cand, key=lambda c: ref_occ[c].mean() if c.any() else 0.0)
        top = a.split("/")[1]
        if top.startswith(("wall_", "guard")):
            walls |= m
        else:
            objects[top] = objects.get(top, np.zeros((N, N), bool)) | m
    return walls, objects


def evaluate(run_dir, scene_json="evaluation/data/scene_truth.json"):
    art = os.path.join(run_dir, "artifacts")
    slam, ref = np.load(os.path.join(art, "slam_grid.npy")), np.load(os.path.join(art, "true_grid.npy"))
    walls, objects = load_truth(scene_json, ref)
    occ = walls.copy()
    for m in objects.values():
        occ |= m
    N = ref.shape[0]
    X, Y = _centres(N)
    inside = (np.abs(X) < ARENA_HALF) & (np.abs(Y) < ARENA_HALF)
    truth = np.zeros((N, N), np.float32)
    truth[inside] = -5.0
    truth[occ] = 5.0

    out = {"scene": scene_json, "objects_in_band": sorted(objects), "match_radius_m": MATCH_M}
    out["map_vs_scene"] = evaluate_grid_accuracy(truth, slam, resolution=RES, tolerance_m=0.15, save_plots=False)
    out["reference_vs_scene"] = evaluate_grid_accuracy(truth, ref, resolution=RES, tolerance_m=0.15, save_plots=False)

    free_truth = inside & ~occ
    slam_free, slam_known = slam <= -0.5, (slam <= -0.5) | (slam >= 1.0)
    out["true_floor_area_m2"] = float(free_truth.sum() * RES ** 2)
    out["floor_mapped_as_free_pct"] = float(100.0 * (slam_free & free_truth).sum() / free_truth.sum())
    out["arena_observed_pct"] = float(100.0 * (slam_known & inside).sum() / inside.sum())
    outside = (np.abs(X) > 5.05) | (np.abs(Y) > 5.05)
    out["free_mapped_outside_arena_m2"] = float((slam_free & outside).sum() * RES ** 2)

    kg = json.load(open(os.path.join(art, "knowledge_graph.json"), encoding="utf-8"))
    ents = [e for e in kg["entities"].values() if e["class_name"] != "wall"]
    pts = np.array([e["world_pos"][:2] for e in ents]) if ents else np.empty((0, 2))
    names = sorted(objects)
    dist = np.full((len(pts), len(names)), np.inf)
    for j, nm in enumerate(names):
        cells = np.column_stack([X[objects[nm]], Y[objects[nm]]])
        for i, p in enumerate(pts):
            dist[i, j] = float(np.min(np.hypot(*(cells - p).T)))
    found = [nm for j, nm in enumerate(names) if len(pts) and dist[:, j].min() <= MATCH_M]
    real_det = int((dist.min(axis=1) <= MATCH_M).sum()) if len(pts) else 0
    chair_det = [i for i, e in enumerate(ents) if e["class_name"] == "chair"]
    real_chairs = [j for j, nm in enumerate(names) if "chair" in nm.lower()]
    out["detection"] = {
        "real_objects": len(names), "objects_found": len(found), "objects_found_names": found,
        "object_recall_pct": 100.0 * len(found) / max(1, len(names)),
        "detections": len(pts), "detections_on_a_real_object": real_det,
        "detection_precision_pct": 100.0 * real_det / max(1, len(pts)),
        "labelled_chair": len(chair_det), "real_chairs": len(real_chairs),
        "chair_labels_on_the_real_chair": int(sum(1 for i in chair_det for j in real_chairs if dist[i, j] <= MATCH_M)),
        "wall_entities": len(kg["entities"]) - len(ents),
    }
    out["knowledge_graph"] = {"rooms": len(kg["rooms"]), "doorways": len(kg["doorways"]),
                              "rooms_with_centroid_set": sum(1 for r in kg["rooms"].values() if tuple(r["centroid"]) != (0.0, 0.0)),
                              "doorways_connecting_rooms_other_than_1_2": sum(1 for d in kg["doorways"].values() if d["connects_rooms"] != [1, 2])}
    json.dump(out, open(os.path.join(run_dir, "truth_eval.json"), "w", encoding="utf-8"), indent=1)
    print(f"  [SAVED] {os.path.join(run_dir, 'truth_eval.json')}")
    return out


if __name__ == "__main__":
    r = evaluate(sys.argv[1])
    g, f = r["map_vs_scene"], r["reference_vs_scene"]
    print(f"  map vs real scene: accuracy {g['accuracy_meters'] * 100:.1f} cm, completion {g['completion_meters'] * 100:.1f} cm, "
          f"precision {g['precision'] * 100:.1f} %, recall {g['recall'] * 100:.1f} %, F1 {g['f1_score'] * 100:.1f} %")
    print(f"  sensor reference vs real scene: accuracy {f['accuracy_meters'] * 100:.1f} cm, recall {f['recall'] * 100:.1f} %")
    print(f"  floor mapped as free {r['floor_mapped_as_free_pct']:.1f} % of {r['true_floor_area_m2']:.1f} m2 | free outside arena "
          f"{r['free_mapped_outside_arena_m2']:.2f} m2")
    print("  detection:", r["detection"])
    print("  knowledge graph:", r["knowledge_graph"])
