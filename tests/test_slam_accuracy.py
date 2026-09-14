"""Unit tests for Point-to-Line Metric SLAM and Loop Closure Engine."""
import numpy as np
import math
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from slam import (
    estimate_normals_2d, icp_point_to_line, icp_match,
    KeyframeLoopManager, ICPPoseEstimator, wrap_angle, world_to_base_points
)


def generate_simulated_wall_scan(length=4.0, num_points=80, noise_std=0.002):
    xs = np.linspace(-length / 2.0, length / 2.0, num_points)
    ys = np.random.normal(0.0, noise_std, num_points)
    return np.column_stack([xs, ys])


def test_normal_estimation():
    print("Running test_normal_estimation...", end=" ")
    wall_points = generate_simulated_wall_scan(length=3.0, num_points=60, noise_std=0.0)
    normals = estimate_normals_2d(wall_points, k=5)

    assert normals.shape == wall_points.shape
    mean_abs_nx = np.mean(np.abs(normals[:, 0]))
    mean_abs_ny = np.mean(np.abs(normals[:, 1]))

    assert mean_abs_nx < 0.05, f"Expected near 0 X-normal, got {mean_abs_nx}"
    assert mean_abs_ny > 0.95, f"Expected near 1.0 Y-normal, got {mean_abs_ny}"
    print("PASSED")


def test_point_to_line_icp_convergence():
    print("Running test_point_to_line_icp_convergence...", end=" ")
    wall1 = np.column_stack([np.linspace(0, 3, 50), np.zeros(50)])
    wall2 = np.column_stack([np.zeros(50), np.linspace(0, 3, 50)])
    target_scan = np.vstack([wall1, wall2])

    true_dx = 0.12
    true_dy = -0.08
    true_theta = math.radians(5.0)

    c, s = math.cos(true_theta), math.sin(true_theta)
    R = np.array([[c, -s], [s, c]])

    source_scan = (R.T @ (target_scan - [true_dx, true_dy]).T).T
    source_scan += np.random.normal(0, 0.001, source_scan.shape)

    transform, rmse, matches = icp_point_to_line(source_scan, target_scan)

    recovered_dx = transform[0, 2]
    recovered_dy = transform[1, 2]
    recovered_theta = math.atan2(transform[1, 0], transform[0, 0])

    err_trans = math.hypot(recovered_dx - true_dx, recovered_dy - true_dy)
    err_rot = abs(wrap_angle(recovered_theta - true_theta))

    assert err_trans < 0.008, f"Translation error too high: {err_trans*1000:.2f} mm"
    assert err_rot < math.radians(0.5), f"Rotation error too high: {math.degrees(err_rot):.2f} deg"
    assert rmse < 0.01
    print("PASSED")


def test_loop_closure_manager():
    print("Running test_loop_closure_manager...", end=" ")
    klm = KeyframeLoopManager(keyframe_dist=1.0, min_loop_time_diff=5.0, loop_search_radius=1.5)

    wall_scan = generate_simulated_wall_scan()

    klm.add_keyframe(timestamp=1.0, x=0.0, y=0.0, yaw=0.0, scan=wall_scan)
    klm.add_keyframe(timestamp=5.0, x=2.0, y=0.0, yaw=0.2, scan=wall_scan)
    klm.add_keyframe(timestamp=10.0, x=3.5, y=1.0, yaw=0.5, scan=wall_scan)
    klm.add_keyframe(timestamp=15.0, x=4.0, y=3.0, yaw=1.0, scan=wall_scan)

    closure = klm.detect_loop_closure(
        current_time=25.0,
        current_x=0.15,
        current_y=0.05,
        current_yaw=0.02,
        current_scan=wall_scan
    )

    assert closure is not None, "Loop closure should be detected"
    matched_kf, err_dx, err_dy, err_dyaw = closure
    assert matched_kf.keyframe_id == 1, f"Should match Keyframe 1, matched {matched_kf.keyframe_id}"
    assert math.hypot(err_dx, err_dy) < 0.25
    print("PASSED")


def test_world_to_base_projection():
    print("Running test_world_to_base_projection...", end=" ")
    pts = np.array([[2.0, 3.0], [4.0, 5.0]])
    base_pts = world_to_base_points(pts, 1.0, 1.0, 0.0)
    assert np.allclose(base_pts, [[1.0, 2.0], [3.0, 4.0]])
    print("PASSED")


if __name__ == "__main__":
    print("=== RUNNING SLAM ACCURACY & LOOP CLOSURE UNIT TESTS ===")
    test_normal_estimation()
    test_point_to_line_icp_convergence()
    test_loop_closure_manager()
    test_world_to_base_projection()
    print("=== ALL SLAM ACCURACY TESTS PASSED SUCCESSFULLY! ===")
