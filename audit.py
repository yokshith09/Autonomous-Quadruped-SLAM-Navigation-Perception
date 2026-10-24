"""Measurement-integrity checks for a run: did the SLAM estimate ever see simulator ground truth, and are the
saved trajectories/grids genuinely independent of it?  `audit(run_dir)` -> [(check, passed, detail)]."""
import ast
import json
import os

import numpy as np

GT_POSE_APIS = {"getObjectPosition", "getObjectOrientation", "getObjectMatrix", "getObjectPose", "getObjectVelocity"}
NAV_FILES = ["slam.py"] + [os.path.join("agents", f) for f in (
    "slam_agent.py", "mapping_agent.py", "perception_agent.py", "frontier_agent.py", "planning_agent.py")]


def _gt_calls(tree, allowed=frozenset()):
    return [f"line {n.lineno}: {n.attr}" for n in ast.walk(tree)
            if isinstance(n, ast.Attribute) and n.attr in GT_POSE_APIS - allowed]


def audit(run_dir):
    checks = []
    leaks = []
    for fname in NAV_FILES:
        if os.path.exists(fname):
            with open(fname, encoding="utf-8") as f:
                leaks += [f"{fname} {c}" for c in _gt_calls(ast.parse(f.read()))]
    with open("robot.py", encoding="utf-8") as f:
        tree = ast.parse(f.read())
    odom = next((n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "get_odometry_imu_delta"), None)
    if odom is None:
        leaks.append("robot.py: get_odometry_imu_delta not found")
    else:  # the simulated gyro may read orientation (a per-step delta), never position
        leaks += [f"robot.py get_odometry_imu_delta {c}" for c in _gt_calls(odom, {"getObjectOrientation"})]
    checks.append(("No ground-truth pose API in SLAM, mapping, perception, planning or odometry (static source scan)",
                   not leaks, "; ".join(leaks) if leaks else f"{len(NAV_FILES)} files + odometry clean; gyro reads orientation delta only"))

    from agents import MessageBus, SLAMAgent
    bus = MessageBus()
    SLAMAgent(bus, use_ground_truth_nav=False).initialize()
    checks.append(("SLAMAgent does not subscribe to pose/ground_truth in closed-loop mode",
                   "pose/ground_truth" not in bus._subscribers, "subscribes to: " + ", ".join(sorted(bus._subscribers))))

    traj = os.path.join(run_dir, "trajectory.csv")
    ok, detail = False, "trajectory.csv missing"
    if os.path.exists(traj):
        d = np.loadtxt(traj, delimiter=",", skiprows=1)
        err = np.hypot(d[:, 4] - d[:, 1], d[:, 5] - d[:, 2])
        ok, detail = float(err.max()) > 0.001, f"estimate differs from ground truth (max {err.max() * 100:.1f} cm over {len(d)} steps)"
    checks.append(("Estimated trajectory is not a copy of ground truth", ok, detail))

    man = os.path.join(run_dir, "manifest.json")
    ok, detail = False, "manifest.json missing"
    if os.path.exists(man):
        gtnav = json.load(open(man, encoding="utf-8")).get("config", {}).get("ground_truth_nav")
        ok, detail = gtnav is False, f"config.ground_truth_nav = {gtnav}"
    checks.append(("Run navigated on the SLAM pose (not ground truth)", ok, detail))
    return checks
