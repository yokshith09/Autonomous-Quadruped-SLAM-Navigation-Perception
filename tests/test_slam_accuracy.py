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


def test_depth_to_base_projection():
    print("Running test_depth_to_base_projection...", end=" ")
    from sensors import depth_to_base_points
    cam_matrix = {'fx': 100.0, 'fy': 100.0, 'cx': 50.0, 'cy': 50.0}
    # Synthetic depth: 100x100 array with 2.0 meters at center
    depth_img = np.zeros((100, 100), dtype=np.float32)
    depth_img[50, 50] = 2.0  # Exactly along optical axis

    # Identity extrinsics
    pts_base = depth_to_base_points(depth_img, cam_matrix)
    assert len(pts_base) == 1
    # Optical axis point should have x_cam=0, y_cam=0, z_cam=2.0
    assert abs(pts_base[0, 0]) < 1e-4
    assert abs(pts_base[0, 1]) < 1e-4
    assert abs(pts_base[0, 2] - 2.0) < 1e-4

    # With mounting translation: e.g. camera mounted at x=0.3m ahead, z=0.2m above base
    # In CoppeliaSim standard 12-element matrix: [r11, r12, r13, tx, r21, r22, r23, ty, r31, r32, r33, tz]
    T_mount = [1, 0, 0, 0.3, 0, 1, 0, 0.0, 0, 0, 1, 0.2]
    pts_mounted = depth_to_base_points(depth_img, cam_matrix, T_base_cam=T_mount)
    assert abs(pts_mounted[0, 0] - 0.3) < 1e-4
    assert abs(pts_mounted[0, 1] - 0.0) < 1e-4
    assert abs(pts_mounted[0, 2] - 2.2) < 1e-4
    print("PASSED")


def test_icp_odometry_prior_and_recovery():
    print("Running test_icp_odometry_prior_and_recovery...", end=" ")
    wall_x = np.linspace(0.5, 3.0, 60)
    wall_1 = np.column_stack([wall_x, np.zeros_like(wall_x)])
    wall_y = np.linspace(0.0, 2.5, 60)
    wall_2 = np.column_stack([np.full_like(wall_y, 3.0), wall_y])
    base_scan = np.vstack([wall_1, wall_2])

    est = ICPPoseEstimator(0.0, 0.0, 0.0)
    est.update(base_scan, timestamp=1.0)

    # Simulate a sharp 35-deg turn where ICP fails/rejects on an outlier frame,
    # followed by a normal frame at the new heading.
    rot_35 = math.radians(35.0)
    R_inv = np.array([[math.cos(-rot_35), -math.sin(-rot_35)],
                      [math.sin(-rot_35),  math.cos(-rot_35)]])
    turned_scan = base_scan @ R_inv.T

    # Force a rejected ICP frame with huge outlier offset, bridged by gyro odom_dyaw
    corrupted_scan = turned_scan + np.array([5.0, 5.0])
    p2 = est.update(corrupted_scan, odom_dx=0.0, odom_dy=0.0, odom_dyaw=rot_35, timestamp=2.0)
    assert not p2.accepted
    assert abs(wrap_angle(p2.yaw - rot_35)) < 1e-4

    # Now feed a valid scan at the turned orientation (shifted slightly by 2cm from corrupted_scan's true frame)
    # Wait: if previous_scan was updated to turned_scan on fallback, let's test consecutive valid scans after rejection:
    p2b = est.update(turned_scan, odom_dx=0.0, odom_dy=0.0, odom_dyaw=0.0, timestamp=2.5)
    next_scan = turned_scan - np.array([0.05, 0.0])
    p3 = est.update(next_scan, odom_dx=0.05 * math.cos(rot_35), odom_dy=0.05 * math.sin(rot_35), odom_dyaw=0.0, timestamp=3.0)
    assert p3.accepted, "ICP failed to recover after a rejected scan frame!"
    print("PASSED")


def test_pose_graph_optimization():
    print("Running test_pose_graph_optimization...", end=" ")
    mgr = KeyframeLoopManager(keyframe_dist=0.5, min_loop_time_diff=5.0)
    dummy_scan = np.column_stack([np.linspace(0, 2, 40), np.zeros(40)])
    kf1 = mgr.add_keyframe(0.0, 0.0, 0.0, 0.0, dummy_scan)
    kf2 = mgr.add_keyframe(2.0, 1.0, 0.0, 0.0, dummy_scan)
    kf3 = mgr.add_keyframe(4.0, 2.0, 0.0, 0.0, dummy_scan)
    kf4 = mgr.add_keyframe(6.0, 1.0, 1.0, 0.0, dummy_scan)

    # Optimize loop between kf1 (idx 0) and current end of chain with error (-0.3, -0.6, -0.09)
    n_opt = mgr.optimize_pose_graph(kf1, err_dx=-0.3, err_dy=-0.6, err_dyaw=-0.09)
    assert n_opt == 3
    # Anchor kf1 must not move
    assert abs(kf1.x - 0.0) < 1e-6
    # Intermediate keyframes should receive proportional harmonic corrections (1/3, 2/3, 3/3)
    assert abs(kf2.x - (1.0 - 0.1)) < 1e-5
    assert abs(kf3.x - (2.0 - 0.2)) < 1e-5
    assert abs(kf4.x - (1.0 - 0.3)) < 1e-5
    assert abs(kf4.y - (1.0 - 0.6)) < 1e-5
    print("PASSED")


def load_tests(loader, standard_tests, pattern):
    import unittest
    suite = unittest.TestSuite()
    for name, obj in list(globals().items()):
        if name.startswith("test_") and callable(obj):
            suite.addTest(unittest.FunctionTestCase(obj))
    return suite


if __name__ == "__main__":
    print("=== RUNNING SLAM ACCURACY & LOOP CLOSURE UNIT TESTS ===")
    test_normal_estimation()
    test_point_to_line_icp_convergence()
    test_loop_closure_manager()
    test_world_to_base_projection()
    test_depth_to_base_projection()
    test_icp_odometry_prior_and_recovery()
    test_pose_graph_optimization()
    print("=== ALL SLAM ACCURACY TESTS PASSED SUCCESSFULLY! ===")


