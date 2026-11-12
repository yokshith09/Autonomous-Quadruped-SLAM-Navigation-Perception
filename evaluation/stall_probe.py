#!/usr/bin/env python3
"""Measure how the front-hip load separates free walking from pushing on a wall (scoring only; ground truth is used
here to know when the body is blocked, never by the agents).

The robot walks toward a heading until it meets a wall. Per second it prints the true speed and the mean |hip force|;
the summary line gives the load range while moving freely and while blocked. Choose LocomotionAgent's
STALL_LOAD_RATIO between (free max / command) and (blocked min / command). Open scene/robot_working.ttt in
CoppeliaSim (not running) first. Re-run after any change to the gait constants in core/robot.py.

  python -m evaluation.stall_probe --heading-deg 90 --cmd 0.75 --seconds 24
"""
import argparse
import math
import time

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from core.robot import LizardRobot


def probe(sim, heading, cmd, seconds):
    sim.setStepping(True)
    sim.startSimulation()
    robot = LizardRobot(sim)
    t0 = sim.getSimulationTime()
    rows = []
    while sim.getSimulationTime() - t0 < seconds:
        t = sim.getSimulationTime() - t0
        x, y, h, _ = robot.get_pose()
        robot.steer_heading(sim.getObjectOrientation(robot.base, sim.handle_world), h, heading)
        robot.apply_gait(t, cmd)
        sim.step()
        rows.append((t, x, y, robot.leg_load()))
    sim.stopSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        time.sleep(0.05)
    return np.array(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--heading-deg", type=float, default=90.0)
    ap.add_argument("--cmd", type=float, default=0.75)
    ap.add_argument("--seconds", type=float, default=24.0)
    args = ap.parse_args()
    a = probe(RemoteAPIClient().require("sim"), math.radians(args.heading_deg), args.cmd, args.seconds)
    t, x, y, load = a.T
    w = 20  # 1 s windows at 20 Hz
    free, blocked = [], []
    for i in range(0, len(t) - w, w):
        speed = math.hypot(x[i + w] - x[i], y[i + w] - y[i])
        mean = float(np.mean(load[i:i + w]))
        print(f"  t {t[i]:4.0f} s | true speed {speed * 100:5.1f} cm/s | mean |hip force| {mean:.4f}")
        if i >= 2 * w and speed > 0.05:
            free.append(mean)
        elif i >= 2 * w and speed < 0.015:
            blocked.append(mean)
    rng = lambda v: f"{min(v):.4f}..{max(v):.4f}" if v else "none"
    print(f"cmd {args.cmd:.2f}: free {rng(free)} | blocked {rng(blocked)} | "
          f"normalised by cmd: free max {max(free) / args.cmd:.4f}" + (f", blocked min {min(blocked) / args.cmd:.4f}" if blocked else "")
          if free else "no free-walking window")


if __name__ == "__main__":
    main()
