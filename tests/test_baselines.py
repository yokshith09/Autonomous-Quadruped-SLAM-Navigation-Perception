import math
import numpy as np

from evaluation.baselines import run_odom


def _rec(steps, fwd, dyaw, yaw0=0.0, gyro_bias=0.0, calib_steps=0):
    t = np.arange(steps) * 0.05
    act = np.zeros((steps, 5))
    act[:, 4] = 1
    act[calib_steps:, 0] = fwd
    act[calib_steps:, 2] = dyaw
    act[:, 2] += gyro_bias * 0.05
    return {"t": t, "act": act, "initial_pose": np.array([1.0, 2.0, yaw0])}


def test_odometry_baseline_dead_reckons_from_the_start_pose():
    est = run_odom(_rec(101, 0.01, 0.0, yaw0=math.pi / 2), 0.0)
    assert np.allclose(est[0], [1.0, 2.0, math.pi / 2])
    assert np.allclose(est[-1][:2], [1.0, 3.0], atol=1e-9)   # 100 steps x 1 cm along +y


def test_odometry_baseline_removes_gyro_bias_seen_while_standing():
    est = run_odom(_rec(400, 0.01, 0.0, gyro_bias=0.01, calib_steps=200), calibrate_s=200 * 0.05)
    assert abs(est[-1][2]) < 0.002          # bias estimated during the stand-still is removed
    assert abs(est[-1][1] - 2.0) < 0.01     # so the straight walk stays straight
