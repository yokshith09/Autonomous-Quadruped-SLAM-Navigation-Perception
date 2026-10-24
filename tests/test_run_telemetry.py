"""Formula checks for run_telemetry on synthetic trajectories (test fixtures only, never reported as results)."""
import json
import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from run_telemetry import RunTelemetry, extra_warnings, loss_episodes, overhead_per_step_ms, path_length, rpe_distance  # noqa: E402


def straight(n=201, length=10.0):
    x = np.linspace(0, length, n)
    return np.column_stack([x, np.zeros(n), np.zeros(n)])


def test_path_length():
    assert abs(path_length(straight()[:, :2]) - 10.0) < 1e-9
    assert path_length(np.zeros((1, 2))) == 0.0


def test_rpe_perfect_and_constant_offset():
    gt = straight()
    t, pct, r, n = rpe_distance(gt, gt)
    assert t == 0 and pct == 0 and r == 0 and n > 0
    # A constant global offset is not relative error.
    off = gt.copy()
    off[:, 1] += 3.0
    assert rpe_distance(gt, off)[0] < 1e-12


def test_rpe_scale_error():
    gt = straight()
    est = gt.copy()
    est[:, 0] *= 1.05          # 5 % odometry over-estimate -> 5 % translational RPE
    t, pct, r, _ = rpe_distance(gt, est)
    assert abs(pct - 5.0) < 1e-6 and 0.05 - 1e-9 <= t <= 0.0525 + 1e-9 and r == 0  # segments are >= 1 m (grid spacing 5 cm)


def test_rpe_heading_drift():
    gt = straight()
    est = gt.copy()
    est[:, 2] = np.radians(1.0) * gt[:, 0]      # 1 deg of yaw error per metre
    _, _, r, _ = rpe_distance(gt, est)
    assert abs(r - 1.0) < 1e-6


def test_loss_episodes():
    acc = [True] * 5 + [False] * 12 + [True] * 3 + [False] * 4 + [False] * 10
    times = np.arange(len(acc)) * 0.1
    eps = loss_episodes(acc, times, min_len=10)
    assert [e[2] for e in eps] == [12, 14]
    assert loss_episodes([True] * 50, np.arange(50)) == []


class _Agent:
    def __init__(self, name):
        self.name, self.update_rate_hz, self.error_count = name, 10.0, 0

    def step(self, t):
        if t == 0.5:
            self.error_count += 1


def test_finalize_end_to_end():
    with tempfile.TemporaryDirectory() as d:
        tel = RunTelemetry(out_root=d, argv=["test"])

        class Bus:
            def subscribe(self, *a):
                pass

        a = _Agent("SLAMAgent")
        tel.attach(Bus(), [a])
        gt = straight(n=1001, length=10.0)
        est = gt.copy()
        est[:, 1] += 0.1                     # constant 10 cm error
        times = np.arange(len(gt)) * 0.05
        for t in times[:20]:
            tel.step(a, float(t))
        m = tel.finalize(agents=[a], gt=gt, est=est, times=times, grid_metrics=None,
                         ctx={"end_reason": "test", "odom_distance_m": 11.0})
        loc = m["localization"]
        assert abs(loc["ate_rmse_m"] - 0.1) < 1e-9 and abs(loc["final_drift_pct_of_distance"] - 1.0) < 1e-9
        assert abs(loc["odometry_scale_error_pct"] - 10.0) < 1e-6
        assert m["agents"][0]["calls"] == 20 and m["agents"][0]["errors"] == 1
        assert m["mapping"]["wall_accuracy_m"]["value"] is None          # missing -> explicit n/a, not 0
        assert m["overall"]["task_success"] is False
        files = set(os.listdir(tel.dir))
        assert {"metrics.json", "agents.csv", "trajectory.csv", "manifest.json"} <= files
        json.load(open(os.path.join(tel.dir, "manifest.json"), encoding="utf-8"))


def test_extra_warnings():
    cap = extra_warnings({"end_reason": "timeout (10s) before all frontiers were cleared", "task_success": False}, {"icp_acceptance_pct": 20.0})
    assert len(cap) == 2 and "not by mission completion" in cap[0]
    assert extra_warnings({"end_reason": "all reachable frontiers explored", "task_success": True}, {"icp_acceptance_pct": 90.0}) == []


def test_overhead_estimate_is_small_and_scales():
    one = overhead_per_step_ms(1000, 1000, n=5000)
    two = overhead_per_step_ms(2000, 1000, n=5000)
    assert 0 <= one < 0.05 and two > one * 1.2


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(name, "ok")
