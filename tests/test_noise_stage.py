import math
import numpy as np

from evaluation.noise_stage import perturb


def _rec(tmp_path):
    n = 200
    offs = np.arange(n + 1) * 10
    offs[50:] -= 10                      # step 49 has an empty scan
    pts = np.random.default_rng(0).uniform(0.5, 3.0, (offs[-1], 2)).astype(np.float32)
    act = np.zeros((n, 5), np.float32)
    act[:, 0], act[:, 4] = 0.01, 1
    np.savez(tmp_path / "r.npz", t=np.arange(n) * 0.05, act=act, scan_points=pts, scan_offsets=offs,
             gt=np.zeros((n, 3)), initial_pose=np.zeros(3))
    return np.load(tmp_path / "r.npz")


def test_no_faults_leaves_the_recording_unchanged(tmp_path):
    r = _rec(tmp_path)
    o = perturb(r, 1)
    assert np.array_equal(o["scan_points"], r["scan_points"]) and np.allclose(o["act"], r["act"])


def test_dropout_keeps_scan_offsets_consistent(tmp_path):
    r = _rec(tmp_path)
    o = perturb(r, 1, dropout=0.5)
    offs = o["scan_offsets"]
    assert offs[-1] == len(o["scan_points"]) and np.all(np.diff(offs) >= 0) and offs[50] == offs[49]


def test_gyro_bias_is_added_only_after_calibration(tmp_path):
    r = _rec(tmp_path)
    o = perturb(r, 1, gyro_bias_dps=1.0, calibrate_s=5.0)
    added = np.degrees(np.cumsum(o["act"][:, 2] - r["act"][:, 2]))
    assert abs(added[99]) < 1e-3                       # t < 5 s: calibration, untouched
    assert math.isclose(added[-1], 1.0 * (r["t"][-1] - 5.0), rel_tol=0.02)
