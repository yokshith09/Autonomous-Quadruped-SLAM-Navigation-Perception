"""2-D scan-matching SLAM primitives.

The scan matcher estimates the pose increment between consecutive depth scans.
``ICPPoseEstimator`` expects points in the robot-base frame and returns a pose
in the fixed world frame.  It deliberately has no CoppeliaSim dependency, so
the same component can be used with a real depth camera or LiDAR.
"""

from dataclasses import dataclass
import math
import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover - scipy is normally available
    cKDTree = None


def wrap_angle(angle: float) -> float:
    """Normalise an angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _nearest_neighbours(source: np.ndarray, target: np.ndarray):
    if cKDTree is not None:
        return cKDTree(target).query(source)
    # Small, dependency-free fallback for deployment environments without scipy.
    delta = source[:, None, :] - target[None, :, :]
    distances = np.linalg.norm(delta, axis=2)
    indices = np.argmin(distances, axis=1)
    return distances[np.arange(len(source)), indices], indices


def best_fit_transform(source: np.ndarray, target: np.ndarray):
    """Return the rigid 2-D transform that maps *source* onto *target*."""
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


def icp_match(source: np.ndarray, target: np.ndarray, *, initial_guess: np.ndarray = None, max_iterations: int = 30,
              tolerance: float = 1e-4, max_correspondence_distance: float = 0.35,
              min_correspondences: int = 20):
    """Match a current 2-D scan to a previous scan using point-to-point ICP.

    Returns ``(current_to_previous_transform, rmse, correspondence_count)``.
    The correspondence gate prevents unmatched walls and moving objects from
    pulling the estimated pose away from the map.
    """
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if source.ndim != 2 or target.ndim != 2 or source.shape[1] != 2 or target.shape[1] != 2:
        raise ValueError("ICP requires Nx2 source and target point arrays")
    if len(source) < min_correspondences or len(target) < min_correspondences:
        return np.eye(3), float("inf"), 0

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
            return np.eye(3), float("inf"), match_count

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


def downsample_scan(points: np.ndarray, *, voxel_size: float = 0.05,
                    max_points: int = 900) -> np.ndarray:
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
class PoseEstimate:
    x: float
    y: float
    yaw: float
    rmse: float = float("inf")
    correspondences: int = 0
    accepted: bool = False


class ICPPoseEstimator:
    """Incrementally estimate a base-frame pose from consecutive depth scans."""

    def __init__(self, x: float, y: float, yaw: float, *, max_step_translation: float = 0.80,
                 max_step_rotation: float = math.radians(120.0), max_rmse: float = 0.35):
        self.pose = PoseEstimate(x, y, yaw)
        self.previous_scan = None
        self.max_step_translation = max_step_translation
        self.max_step_rotation = max_step_rotation
        self.max_rmse = max_rmse

    def update(self, local_scan: np.ndarray, odom_dx: float = 0.0, odom_dy: float = 0.0, odom_dyaw: float = 0.0) -> PoseEstimate:
        """Incorporate one base-frame scan and return the current pose estimate."""
        scan = downsample_scan(local_scan)
        if len(scan) < 20:
            self.pose.x += odom_dx
            self.pose.y += odom_dy
            self.pose.yaw = wrap_angle(self.pose.yaw + odom_dyaw)
            return self.pose
            
        if self.previous_scan is None:
            self.previous_scan = scan
            self.pose.x += odom_dx
            self.pose.y += odom_dy
            self.pose.yaw = wrap_angle(self.pose.yaw + odom_dyaw)
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

        transform, rmse, correspondences = icp_match(scan, self.previous_scan, initial_guess=initial_guess)
        dx, dy = transform[:2, 2]
        dyaw = math.atan2(transform[1, 0], transform[0, 0])
        valid = (
            correspondences >= 20
            and np.isfinite(rmse)
            and rmse <= self.max_rmse
            and math.hypot(dx, dy) <= self.max_step_translation
            and abs(dyaw) <= self.max_step_rotation
            and math.hypot(dx - dx_local, dy - dy_local) <= 0.05
            and abs(wrap_angle(dyaw - odom_dyaw)) <= math.radians(5.0)
        )

        if valid:
            self.pose.x += dx * math.cos(self.pose.yaw) - dy * math.sin(self.pose.yaw)
            self.pose.y += dx * math.sin(self.pose.yaw) + dy * math.cos(self.pose.yaw)
            self.pose.yaw = wrap_angle(self.pose.yaw + dyaw)
            self.previous_scan = scan
            self.pose.rmse = rmse
            self.pose.correspondences = correspondences
            self.pose.accepted = True
        else:
            self.pose.x += odom_dx
            self.pose.y += odom_dy
            self.pose.yaw = wrap_angle(self.pose.yaw + odom_dyaw)
            self.pose.rmse = float("inf")
            self.pose.correspondences = correspondences
            self.pose.accepted = False
            
        return self.pose


def world_to_base_points(points_world: np.ndarray, base_matrix) -> np.ndarray:
    """Express CoppeliaSim world points in the robot-base coordinate frame."""
    matrix = np.asarray(base_matrix, dtype=np.float64)
    rotation = np.array([
        matrix[0:3], matrix[4:7], matrix[8:11]
    ])
    translation = np.array([matrix[3], matrix[7], matrix[11]])
    return (rotation.T @ (np.asarray(points_world) - translation).T).T


def base_to_world_points(points_base: np.ndarray, pose: PoseEstimate) -> np.ndarray:
    """Project base-frame depth points into the ICP-estimated world frame."""
    points_base = np.asarray(points_base, dtype=np.float64)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    points_world = points_base.copy()
    points_world[:, 0] = c * points_base[:, 0] - s * points_base[:, 1] + pose.x
    points_world[:, 1] = s * points_base[:, 0] + c * points_base[:, 1] + pose.y
    return points_world
