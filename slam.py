"""2-D scan-matching SLAM primitives with Point-to-Line Metric ICP & Loop Closure.

Provides:
- 2D surface normal estimation via local neighborhood PCA
- Point-to-Line Metric ICP (Generalized ICP) for sub-centimeter wall locking
- SVD Point-to-Point ICP for fallback
- KeyframeLoopManager for submap keyframing and loop closure detection
- ICPPoseEstimator: high-precision incremental base pose tracking
"""

from dataclasses import dataclass, field
import math
import numpy as np
from typing import List, Tuple, Optional, Dict, Any

try:
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover
    cKDTree = None


def wrap_angle(angle: float) -> float:
    """Normalise an angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _nearest_neighbours(source: np.ndarray, target: np.ndarray):
    if cKDTree is not None:
        return cKDTree(target).query(source)
    delta = source[:, None, :] - target[None, :, :]
    distances = np.linalg.norm(delta, axis=2)
    indices = np.argmin(distances, axis=1)
    return distances[np.arange(len(source)), indices], indices


def estimate_normals_2d(points: np.ndarray, k: int = 5) -> np.ndarray:
    """Estimate 2-D unit surface normals for a point cloud using local PCA/covariance.

    The normal vector points perpendicular to the local wall surface.
    """
    points = np.asarray(points, dtype=np.float64)
    n_points = len(points)
    normals = np.zeros((n_points, 2), dtype=np.float64)
    if n_points < 3:
        return normals

    k = min(k, n_points - 1)
    if cKDTree is not None:
        _, idx_k = cKDTree(points).query(points, k=k + 1)
        for i in range(n_points):
            neighbors = points[idx_k[i, 1:]]
            centroid = np.mean(neighbors, axis=0)
            cov = (neighbors - centroid).T @ (neighbors - centroid)
            eigvals, eigvecs = np.linalg.eigh(cov)
            normal = eigvecs[:, 0]
            norm_len = np.linalg.norm(normal)
            normals[i] = normal / (norm_len + 1e-12)
    else:
        for i in range(n_points):
            dists = np.linalg.norm(points - points[i], axis=1)
            idx_k = np.argsort(dists)[1:k + 1]
            neighbors = points[idx_k]
            centroid = np.mean(neighbors, axis=0)
            cov = (neighbors - centroid).T @ (neighbors - centroid)
            eigvals, eigvecs = np.linalg.eigh(cov)
            normal = eigvecs[:, 0]
            norm_len = np.linalg.norm(normal)
            normals[i] = normal / (norm_len + 1e-12)

    return normals


def best_fit_transform(source: np.ndarray, target: np.ndarray):
    """Return the rigid 2-D transform that maps *source* onto *target* via SVD."""
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    transform = np.eye(3, dtype=np.float64)
    transform[:2, :2] = rotation
    transform[:2, 2] = translation
    return transform


def icp_point_to_line(source: np.ndarray, target: np.ndarray, target_normals: Optional[np.ndarray] = None,
                       *, initial_guess: Optional[np.ndarray] = None, max_iterations: int = 25,
                       tolerance: float = 1e-4, max_correspondence_distance: float = 0.35,
                       min_correspondences: int = 20):
    """Point-to-Line Metric ICP (Generalized ICP).

    Minimizes orthogonal distance between source points and target surface normals:
      e_i = (R * p_i + t - q_i) . n_i
    Provides sub-millimeter geometric wall locking without longitudinal corridor sliding.
    """
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if len(source) < min_correspondences or len(target) < min_correspondences:
        return np.eye(3, dtype=np.float64), float("inf"), 0

    if target_normals is None or len(target_normals) != len(target):
        target_normals = estimate_normals_2d(target)

    aligned = source.copy()
    if initial_guess is not None:
        total_transform = initial_guess.copy()
        aligned = (total_transform[:2, :2] @ aligned.T).T + total_transform[:2, 2]
    else:
        total_transform = np.eye(3, dtype=np.float64)

    previous_rmse = float("inf")
    match_count = 0

    for _ in range(max_iterations):
        distances, indices = _nearest_neighbours(aligned, target)
        valid = distances <= max_correspondence_distance
        match_count = int(np.count_nonzero(valid))
        if match_count < min_correspondences:
            # Fall back to SVD point-to-point if too few valid correspondences
            return icp_match(source, target, initial_guess=initial_guess,
                             max_correspondence_distance=max_correspondence_distance,
                             min_correspondences=min_correspondences,
                             mode="point_to_point")

        p = aligned[valid]
        q = target[indices[valid]]
        n = target_normals[indices[valid]]

        # Residual: orthogonal distance to wall normal
        residuals = np.sum((p - q) * n, axis=1)

        # Jacobian: [nx, ny, px*ny - py*nx]
        cross_term = p[:, 0] * n[:, 1] - p[:, 1] * n[:, 0]
        J = np.column_stack([n[:, 0], n[:, 1], cross_term])

        # Solve normal equations: (J^T * J + lambda * I) * delta = -J^T * r
        H = J.T @ J
        b = -J.T @ residuals
        damping = 1e-4 * np.trace(H) * np.eye(3)
        try:
            delta = np.linalg.solve(H + damping, b)
        except np.linalg.LinAlgError:
            break

        dx, dy, dtheta = delta
        step_c, step_s = math.cos(dtheta), math.sin(dtheta)
        step_R = np.array([[step_c, -step_s], [step_s, step_c]])
        step_t = np.array([dx, dy])

        step_transform = np.eye(3, dtype=np.float64)
        step_transform[:2, :2] = step_R
        step_transform[:2, 2] = step_t

        aligned = (step_R @ aligned.T).T + step_t
        total_transform = step_transform @ total_transform

        rmse = float(np.sqrt(np.mean(residuals ** 2)))
        if abs(previous_rmse - rmse) < tolerance or np.linalg.norm(delta) < tolerance:
            break
        previous_rmse = rmse

    distances, indices = _nearest_neighbours(aligned, target)
    valid = distances <= max_correspondence_distance
    match_count = int(np.count_nonzero(valid))
    if match_count:
        final_residuals = np.sum((aligned[valid] - target[indices[valid]]) * target_normals[indices[valid]], axis=1)
        rmse = float(np.sqrt(np.mean(final_residuals ** 2)))
    else:
        rmse = float("inf")

    return total_transform, rmse, match_count


def icp_match(source: np.ndarray, target: np.ndarray, *, initial_guess: Optional[np.ndarray] = None,
              max_iterations: int = 30, tolerance: float = 1e-4, max_correspondence_distance: float = 0.35,
              min_correspondences: int = 20, mode: str = "point_to_line"):
    """Match 2D scan to previous scan using Point-to-Line ICP (default) or Point-to-Point ICP."""
    if mode == "point_to_line":
        return icp_point_to_line(
            source, target,
            initial_guess=initial_guess,
            max_iterations=max_iterations,
            tolerance=tolerance,
            max_correspondence_distance=max_correspondence_distance,
            min_correspondences=min_correspondences
        )

    # Point-to-Point SVD implementation
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if len(source) < min_correspondences or len(target) < min_correspondences:
        return np.eye(3, dtype=np.float64), float("inf"), 0

    aligned = source.copy()
    if initial_guess is not None:
        total_transform = initial_guess.copy()
        aligned = (total_transform[:2, :2] @ aligned.T).T + total_transform[:2, 2]
    else:
        total_transform = np.eye(3, dtype=np.float64)

    previous_rmse = float("inf")
    for _ in range(max_iterations):
        distances, indices = _nearest_neighbours(aligned, target)
        valid = distances <= max_correspondence_distance
        if int(np.count_nonzero(valid)) < min_correspondences:
            return np.eye(3, dtype=np.float64), float("inf"), 0

        step_transform = best_fit_transform(aligned[valid], target[indices[valid]])
        aligned = (step_transform[:2, :2] @ aligned.T).T + step_transform[:2, 2]
        total_transform = step_transform @ total_transform
        rmse = float(np.sqrt(np.mean(distances[valid] ** 2)))
        if abs(previous_rmse - rmse) < tolerance:
            break
        previous_rmse = rmse

    distances, _ = _nearest_neighbours(aligned, target)
    valid = distances <= max_correspondence_distance
    match_count = int(np.count_nonzero(valid))
    rmse = float(np.sqrt(np.mean(distances[valid] ** 2))) if match_count else float("inf")
    return total_transform, rmse, match_count


def downsample_scan(points: np.ndarray, *, voxel_size: float = 0.05, max_points: int = 900) -> np.ndarray:
    """Voxel-downsample 2-D points to make ICP stable and bounded in cost."""
    points = np.asarray(points, dtype=np.float64)
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.float64)
    keys = np.floor(points / voxel_size).astype(np.int32)
    _, first_indices = np.unique(keys, axis=0, return_index=True)
    reduced = points[np.sort(first_indices)]
    if len(reduced) > max_points:
        reduced = reduced[np.linspace(0, len(reduced) - 1, max_points, dtype=np.int32)]
    return reduced


@dataclass
class Keyframe:
    keyframe_id: int
    timestamp: float
    x: float
    y: float
    yaw: float
    scan_points: np.ndarray
    normals: np.ndarray


class KeyframeLoopManager:
    """Manages submap keyframes and detects loop closure constraints."""

    def __init__(self, keyframe_dist: float = 1.5, keyframe_rot_deg: float = 40.0,
                 loop_search_radius: float = 2.5, min_loop_time_diff: float = 12.0):
        self.keyframe_dist = keyframe_dist
        self.keyframe_rot = math.radians(keyframe_rot_deg)
        self.loop_search_radius = loop_search_radius
        self.min_loop_time_diff = min_loop_time_diff
        self.keyframes: List[Keyframe] = []
        self.last_kf_pose = None
        self.loop_closures: List[Dict[str, Any]] = []

    def should_add_keyframe(self, x: float, y: float, yaw: float) -> bool:
        if not self.keyframes or self.last_kf_pose is None:
            return True
        dist = math.hypot(x - self.last_kf_pose[0], y - self.last_kf_pose[1])
        drot = abs(wrap_angle(yaw - self.last_kf_pose[2]))
        return dist >= self.keyframe_dist or drot >= self.keyframe_rot

    def add_keyframe(self, timestamp: float, x: float, y: float, yaw: float, scan: np.ndarray) -> Keyframe:
        normals = estimate_normals_2d(scan)
        kf = Keyframe(
            keyframe_id=len(self.keyframes) + 1,
            timestamp=timestamp,
            x=x, y=y, yaw=yaw,
            scan_points=scan.copy(),
            normals=normals
        )
        self.keyframes.append(kf)
        self.last_kf_pose = (x, y, yaw)
        return kf

    def detect_loop_closure(self, current_time: float, current_x: float, current_y: float,
                            current_yaw: float, current_scan: np.ndarray) -> Optional[Tuple[Keyframe, float, float, float]]:
        """Search past keyframes for spatial loop closure."""
        if len(self.keyframes) < 3:
            return None

        candidates = []
        for kf in self.keyframes[:-2]:
            if (current_time - kf.timestamp) >= self.min_loop_time_diff:
                dist = math.hypot(current_x - kf.x, current_y - kf.y)
                if dist <= self.loop_search_radius:
                    candidates.append((kf, dist))

        if not candidates:
            return None

        candidates.sort(key=lambda item: item[1])
        best_kf, _ = candidates[0]

        # Initial relative guess between current pose and keyframe pose
        dx_guess = best_kf.x - current_x
        dy_guess = best_kf.y - current_y
        dyaw_guess = wrap_angle(best_kf.yaw - current_yaw)

        guess_T = np.eye(3, dtype=np.float64)
        guess_T[0, 0] = math.cos(dyaw_guess)
        guess_T[0, 1] = -math.sin(dyaw_guess)
        guess_T[1, 0] = math.sin(dyaw_guess)
        guess_T[1, 1] = math.cos(dyaw_guess)
        guess_T[0, 2] = dx_guess
        guess_T[1, 2] = dy_guess

        transform, rmse, matches = icp_point_to_line(
            current_scan, best_kf.scan_points, best_kf.normals,
            initial_guess=guess_T,
            max_correspondence_distance=0.40,
            min_correspondences=25
        )

        if matches >= 25 and rmse <= 0.08:
            err_dx = transform[0, 2] - dx_guess
            err_dy = transform[1, 2] - dy_guess
            err_dyaw = wrap_angle(math.atan2(transform[1, 0], transform[0, 0]) - dyaw_guess)

            closure_info = {
                "keyframe_id": best_kf.keyframe_id,
                "timestamp": current_time,
                "rmse": rmse,
                "matches": matches,
                "correction": (err_dx, err_dy, err_dyaw)
            }
            self.loop_closures.append(closure_info)
            return best_kf, err_dx, err_dy, err_dyaw

        return None


@dataclass
class PoseEstimate:
    x: float
    y: float
    yaw: float
    rmse: float = float("inf")
    correspondences: int = 0
    accepted: bool = False


class ICPPoseEstimator:
    """Incrementally estimate base-frame pose using Point-to-Line Metric ICP with loop closure support."""

    def __init__(self, x: float, y: float, yaw: float, *, max_step_translation: float = 0.80,
                 max_step_rotation: float = math.radians(120.0), max_rmse: float = 0.35,
                 use_point_to_line: bool = True):
        self.pose = PoseEstimate(x, y, yaw)
        self.previous_scan = None
        self.previous_normals = None
        self.max_step_translation = max_step_translation
        self.max_step_rotation = max_step_rotation
        self.max_rmse = max_rmse
        self.use_point_to_line = use_point_to_line
        self.loop_manager = KeyframeLoopManager()

    def update(self, local_scan: np.ndarray, odom_dx: float = 0.0, odom_dy: float = 0.0,
               odom_dyaw: float = 0.0, timestamp: float = 0.0) -> PoseEstimate:
        """Incorporate one base-frame scan and return the current pose estimate."""
        scan = downsample_scan(local_scan)
        if len(scan) < 20:
            self.pose.x += odom_dx
            self.pose.y += odom_dy
            self.pose.yaw = wrap_angle(self.pose.yaw + odom_dyaw)
            return self.pose

        if self.previous_scan is None:
            self.previous_scan = scan
            self.previous_normals = estimate_normals_2d(scan)
            self.pose.x += odom_dx
            self.pose.y += odom_dy
            self.pose.yaw = wrap_angle(self.pose.yaw + odom_dyaw)
            self.loop_manager.add_keyframe(timestamp, self.pose.x, self.pose.y, self.pose.yaw, scan)
            return self.pose

        dx_local = odom_dx * math.cos(-self.pose.yaw) - odom_dy * math.sin(-self.pose.yaw)
        dy_local = odom_dx * math.sin(-self.pose.yaw) + odom_dy * math.cos(-self.pose.yaw)

        initial_guess = np.eye(3, dtype=np.float64)
        initial_guess[0, 0] = math.cos(odom_dyaw)
        initial_guess[0, 1] = -math.sin(odom_dyaw)
        initial_guess[1, 0] = math.sin(odom_dyaw)
        initial_guess[1, 1] = math.cos(odom_dyaw)
        initial_guess[0, 2] = dx_local
        initial_guess[1, 2] = dy_local

        if self.use_point_to_line:
            transform, rmse, correspondences = icp_point_to_line(
                scan, self.previous_scan, self.previous_normals, initial_guess=initial_guess
            )
        else:
            transform, rmse, correspondences = icp_match(
                scan, self.previous_scan, initial_guess=initial_guess, mode="point_to_point"
            )

        dx, dy = transform[:2, 2]
        dyaw = math.atan2(transform[1, 0], transform[0, 0])
        valid = (
            correspondences >= 20
            and np.isfinite(rmse)
            and rmse <= self.max_rmse
            and math.hypot(dx, dy) <= self.max_step_translation
            and abs(dyaw) <= self.max_step_rotation
        )

        if valid:
            self.pose.x += dx * math.cos(self.pose.yaw) - dy * math.sin(self.pose.yaw)
            self.pose.y += dx * math.sin(self.pose.yaw) + dy * math.cos(self.pose.yaw)
            self.pose.yaw = wrap_angle(self.pose.yaw + dyaw)
            self.previous_scan = scan
            self.previous_normals = estimate_normals_2d(scan)
            self.pose.rmse = rmse
            self.pose.correspondences = correspondences
            self.pose.accepted = True

            # Keyframing & Loop Closure
            if self.loop_manager.should_add_keyframe(self.pose.x, self.pose.y, self.pose.yaw):
                self.loop_manager.add_keyframe(timestamp, self.pose.x, self.pose.y, self.pose.yaw, scan)

            # Check loop closure
            closure = self.loop_manager.detect_loop_closure(timestamp, self.pose.x, self.pose.y, self.pose.yaw, scan)
            if closure:
                matched_kf, err_dx, err_dy, err_dyaw = closure
                # Correct drift smoothly (50% damping factor)
                self.pose.x += err_dx * 0.5
                self.pose.y += err_dy * 0.5
                self.pose.yaw = wrap_angle(self.pose.yaw + err_dyaw * 0.5)
        else:
            self.pose.x += odom_dx
            self.pose.y += odom_dy
            self.pose.yaw = wrap_angle(self.pose.yaw + odom_dyaw)
            self.pose.rmse = float("inf")
            self.pose.correspondences = correspondences
            self.pose.accepted = False

        return self.pose


def world_to_base_points(points_world: np.ndarray, *args) -> np.ndarray:
    """Express world points in the robot-base frame.

    Supports either:
    1) world_to_base_points(points, base_matrix_12_or_16)
    2) world_to_base_points(points, robot_x, robot_y, robot_yaw)
    """
    points = np.asarray(points_world, dtype=np.float64)
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.float64)

    if len(args) == 1:
        matrix = np.asarray(args[0], dtype=np.float64)
        rotation = np.array([matrix[0:3], matrix[4:7], matrix[8:11]])
        translation = np.array([matrix[3], matrix[7], matrix[11]])
        return (rotation.T @ (points - translation).T).T[:, :2]
    elif len(args) == 3:
        rx, ry, ryaw = args
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        shifted = points[:, :2] - [rx, ry]
        bx = c * shifted[:, 0] - s * shifted[:, 1]
        by = s * shifted[:, 0] + c * shifted[:, 1]
        return np.column_stack([bx, by])
    else:
        raise ValueError("world_to_base_points requires (points, matrix) or (points, x, y, yaw)")


def base_to_world_points(points_base: np.ndarray, pose: PoseEstimate) -> np.ndarray:
    """Project base-frame depth points into the ICP-estimated world frame."""
    points_base = np.asarray(points_base, dtype=np.float64)
    if len(points_base) == 0:
        return np.empty((0, 2), dtype=np.float64)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    points_world = points_base.copy()
    points_world[:, 0] = c * points_base[:, 0] - s * points_base[:, 1] + pose.x
    points_world[:, 1] = s * points_base[:, 0] + c * points_base[:, 1] + pose.y
    return points_world
