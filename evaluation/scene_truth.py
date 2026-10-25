#!/usr/bin/env python3
"""Read the scene's ground-truth layout from CoppeliaSim (read-only, simulation stopped) for scoring only:
every shape's alias, position and bounding box. Nothing here is ever given to the robot's agents.

  python -m evaluation.scene_truth            # writes evaluation/data/scene_truth.json
"""
import json
import os
import sys

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def main(out="evaluation/data/scene_truth.json"):
    sim = RemoteAPIClient().require("sim")
    objs = []
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        t = sim.getObjectType(h)
        rec = {"alias": sim.getObjectAlias(h, 2), "type": t, "pos": sim.getObjectPosition(h, sim.handle_world)}
        if t == sim.object_shape_type:
            try:
                rec["bbox_size"] = list(sim.getShapeBB(h)[0])  # x, y, z extent of the shape's bounding box (m)
            except Exception:
                pass
        objs.append(rec)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump({"objects": objs}, open(out, "w", encoding="utf-8"), indent=1)
    print(f"  {len(objs)} objects -> {out}")


if __name__ == "__main__":
    main(*sys.argv[1:])
