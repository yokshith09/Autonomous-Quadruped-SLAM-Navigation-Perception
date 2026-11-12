#!/usr/bin/env python3
"""Gait benchmark: walk the robot straight in the simulator and measure how smoothly it moves (scoring only).

For each leg-lift setting it walks at full gait speed for a fixed time, steering to its start heading, and reports
from the ground-truth pose: mean forward speed, share of 50 ms samples moving backward (stride surge), forward-speed
spread, lateral sway and yaw wobble. Open scene/robot_working.ttt in CoppeliaSim (not running) first.

  python -m evaluation.gait_bench --lift 0.025 0.06 0.10 --seconds 20
"""
import argparse
import math
import time

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import core.robot as cr
from core.robot import LizardRobot, norm_angle


def walk(sim, lift, plant, seconds, speed=1.0, hip=None, freq=None):
    cr.LEG_LIFT, cr.LEG_PLANT = lift, plant
    if hip is not None:
        cr.HIP_AMP = hip
    if freq is not None:
        cr.OMEGA = 2 * math.pi * freq
    sim.setStepping(True)
    sim.startSimulation()
    robot = LizardRobot(sim)
    t0 = sim.getSimulationTime()
    x0, y0, h0, _ = robot.get_pose()
    poses = []
    while sim.getSimulationTime() - t0 < seconds:
        t = sim.getSimulationTime() - t0
        x, y, h, _ = robot.get_pose()
        poses.append((t, x, y, h))
        orient = sim.getObjectOrientation(robot.base, sim.handle_world)
        robot.steer_heading(orient, h, h0)
        robot.apply_gait(t, speed)
        sim.step()
    sim.stopSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        time.sleep(0.05)
    p = np.array(poses)
    t, x, y, h = p.T
    skip = t > 2.0  # ignore the first 2 s while the gait starts
    dt, dx, dy = np.diff(t), np.diff(x), np.diff(y)
    v_fwd = (dx * np.cos(h0) + dy * np.sin(h0)) / dt
    v_lat = (-dx * np.sin(h0) + dy * np.cos(h0)) / dt
    yaw_rate = np.diff(np.unwrap(h)) / dt
    s = skip[1:]
    dist = math.hypot(x[-1] - x[skip][0], y[-1] - y[skip][0])
    return {
        "lift": lift, "hip": cr.HIP_AMP, "freq": cr.OMEGA / (2 * math.pi), "mean_speed": dist / (t[-1] - t[skip][0]),
        "backward_pct": 100.0 * np.mean(v_fwd[s] < -0.005),
        "speed_spread": float(np.std(v_fwd[s])),
        "lateral_sway_cm": 100.0 * float(np.std((-(x - x0) * math.sin(h0) + (y - y0) * math.cos(h0))[skip])),
        "yaw_wobble_deg_s": math.degrees(float(np.std(yaw_rate[s]))),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lift", type=float, nargs="+", default=[cr.LEG_LIFT])
    ap.add_argument("--hip", type=float, nargs="+", default=[cr.HIP_AMP])
    ap.add_argument("--freq", type=float, nargs="+", default=[cr.FREQ])
    ap.add_argument("--speed", type=float, nargs="+", default=[1.0])
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()
    sim = RemoteAPIClient().require("sim")
    plant = cr.LEG_PLANT
    for lift in args.lift:
      for hip in args.hip:
        for freq in args.freq:
         for spd in args.speed:
          r = walk(sim, lift, plant, args.seconds, speed=spd, hip=hip, freq=freq)
          print(f"  lift {r['lift']:.3f} hip {r['hip']:.2f} freq {r['freq']:.1f} cmd {spd:.2f} | speed {r['mean_speed']:.4f} m/s | backward {r['backward_pct']:4.1f} % of samples | "
              f"speed spread {r['speed_spread']:.3f} m/s | lateral sway {r['lateral_sway_cm']:.2f} cm | "
              f"yaw wobble {r['yaw_wobble_deg_s']:.1f} deg/s", flush=True)


if __name__ == "__main__":
    main()
