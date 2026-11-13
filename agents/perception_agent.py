"""Perception Agent: RGB-D acquisition, pinhole 3D back-projection, and spatial gating."""
import numpy as np
from typing import Optional
from .base import BaseAgent
from .bus import MessageBus, RGBDMsg, ScanMsg
from core.sensors import depth_to_base_points, grab_depth_frame, grab_rgb_frame


SCAN_MAX_Z = 0.60      # top of the band used for scan matching (metres above the body origin)
OBSTACLE_MAX_Z = 0.20  # metres above the body origin; see PerceptionAgent.max_height
CLOUD_VOXEL = 0.06   # 3D view kept for 3D matching: one point per 6 cm voxel
CLOUD_MAX_Z = 2.5    # metres above the body origin


# Drop detection (floor edge, stairwell). Body-frame z is height above the floor plus ~1.5 cm (T_base_cam takes the
# camera's world height), so the floor reads z ~ 0.01 and the camera sees it from ~0.19 m. Where the floor ends inside
# the window straight ahead and nothing stands there, the ground drops away.
DROP_WINDOW_X = (0.25, 0.50)   # metres ahead of the body origin
DROP_HALF_WIDTH = 0.10         # metres either side of the heading (body ~0.10 m wide)
DROP_FLOOR_X = 0.45            # the floor must reach this far ahead, or it ends at a drop
DROP_SEARCH_X = 0.80           # floor searched this far: image rows land at discrete distances (~0.38, 0.47, 0.6 m),
                               # so a window ending at 0.50 m left one row of margin and gait pitch could drop it
GROUND_Z = (-0.08, 0.04)       # body-frame z of floor points (below OBSTACLE band)


def drop_ahead(points: np.ndarray) -> Optional[float]:
    """Distance ahead (m) where the floor ends with nothing standing there, or None. Run 13 walked off the unwalled
    rescue floor with 35 cm of position error; this looks at the floor itself, whatever the map says."""
    if len(points) == 0:
        return None
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    win = (x >= DROP_WINDOW_X[0]) & (x <= DROP_WINDOW_X[1]) & (np.abs(y) <= DROP_HALF_WIDTH)
    if np.any(win & (z >= GROUND_Z[1]) & (z <= SCAN_MAX_Z)):
        return None                                   # something stands there: the bumper handles it
    ground = ((x >= DROP_WINDOW_X[0]) & (x <= DROP_SEARCH_X) & (np.abs(y) <= DROP_HALF_WIDTH)
              & (z > GROUND_Z[0]) & (z < GROUND_Z[1]))
    reach = float(x[ground].max()) if ground.any() else DROP_WINDOW_X[0]
    return reach if reach < DROP_FLOOR_X else None


def virtual_wall(x_edge: float) -> np.ndarray:
    """Obstacle points along a drop edge (body frame, mid obstacle band)."""
    ys = np.linspace(-0.20, 0.20, 9)
    return np.column_stack([np.full(len(ys), x_edge + 0.03), ys, np.full(len(ys), 0.10)])


def voxel_down(points: np.ndarray, voxel: float) -> np.ndarray:
    if len(points) == 0:
        return points
    _, first = np.unique(np.floor(points / voxel).astype(np.int32), axis=0, return_index=True)
    return points[np.sort(first)]


class PerceptionAgent(BaseAgent):
    def __init__(self, bus: MessageBus, sim=None, depth_cam=None, rgb_cam=None, camera_matrix=None, update_rate_hz: float = 10.0):
        super().__init__("PerceptionAgent", bus, update_rate_hz)
        self.sim = sim
        self.depth_cam = depth_cam
        self.rgb_cam = rgb_cam
        self.camera_matrix = camera_matrix
        self.T_base_cam = camera_matrix.get('T_base_cam', None) if camera_matrix else None
        self.point_stride = 4
        self.self_mask_radius = 0.22  # 22cm radial deadzone around base origin
        self.min_height = 0.04
        # Top of the obstacle band, above the body origin (about 5.5 cm over the floor). The robot is about 7 cm tall:
        # anything higher than ~25 cm above the floor (a 40-50 cm tabletop) is not an obstacle, so it can walk under
        # tables and only the legs are mapped. Was 0.60, which mapped the whole tabletop as a solid block.
        self.max_height = OBSTACLE_MAX_Z
        self.drop_ahead_m = None    # distance to a drop ahead (m), None when the floor continues
        self.publish_cloud = False  # full 3D view for offline 3D-matching studies (main_agentic --record-3d)

    def initialize(self) -> bool:
        if self.camera_matrix and self.T_base_cam is None:
            self.T_base_cam = self.camera_matrix.get('T_base_cam', None)
        self.is_running = True
        return True

    def process_frame(self, depth_matrix: np.ndarray, rgb_matrix: Optional[np.ndarray], timestamp: float):
        """Process raw depth and RGB into body-frame 3D points and 2D obstacle scans."""
        if depth_matrix is None:
            return

        # Publish raw sensory buffer
        self.bus.publish("sensor/rgbd", RGBDMsg(
            timestamp=timestamp,
            rgb_image=rgb_matrix,
            depth_matrix=depth_matrix
        ))

        # Project directly into robot base coordinate frame (zero world matrix reliance)
        base_points = depth_to_base_points(depth_matrix, self.camera_matrix, self.T_base_cam)
        if len(base_points) == 0:
            return

        # Radial self-masking in robot base frame: discard returns within chassis/leg radius
        dist_b = np.hypot(base_points[:, 0], base_points[:, 1])
        z = base_points[:, 2]
        outside = dist_b > self.self_mask_radius
        # Two bands. Scan matching keeps the tall band (walls up to SCAN_MAX_Z): its structure is what SLAM needs, and
        # it keeps SLAM's input identical to earlier recordings. The map and the bumper use the low band (up to
        # max_height), so a tabletop the robot fits under is not a wall. Live run 32 used the low band for both:
        # 15 cm error at 150 s against 5-8 cm in runs 19, 24, 31.
        pts_map = base_points[outside & (z >= -0.08) & (z <= self.max_height)]
        pts_scan = base_points[outside & (z >= self.min_height) & (z <= SCAN_MAX_Z)]
        edge = drop_ahead(base_points[outside])
        self.drop_ahead_m = edge
        if edge is not None:  # wall off the drop for the bumper and the map; never in the scan used for localization
            pts_map = np.vstack([pts_map, virtual_wall(edge)]) if len(pts_map) else virtual_wall(edge)
            if timestamp - getattr(self, "_drop_log_t", -1e9) > 5.0:
                self._drop_log_t = timestamp
                print(f"  [PERCEPTION] Drop ahead at {edge:.2f} m (floor ends, nothing stands there) -> virtual wall")
        if len(pts_map) == 0 and len(pts_scan) == 0:
            return
        pts_obs = pts_map[pts_map[:, 2] >= self.min_height] if len(pts_map) else np.empty((0, 3))
        dist_obs = np.hypot(pts_obs[:, 0], pts_obs[:, 1]) if len(pts_obs) > 0 else np.array([99.0])

        # Downsample: scan points for matching, low-band obstacle points for the bumper
        sampled_2d = pts_scan[::self.point_stride, :2] if len(pts_scan) > 0 else np.empty((0, 2), dtype=np.float64)
        obstacle_2d = pts_obs[::self.point_stride, :2] if len(pts_obs) > 0 else np.empty((0, 2), dtype=np.float64)
        if edge is not None:  # every virtual point, not a strided sample of them
            obstacle_2d = np.vstack([obstacle_2d, virtual_wall(edge)[:, :2]])

        # Publish filtered body-frame scan (base_points_3d includes ground + obstacles for raycasting)
        self.bus.publish("sensor/scan", ScanMsg(
            timestamp=timestamp,
            points_2d=sampled_2d,
            obstacle_2d=obstacle_2d,
            base_points_3d=pts_map,
            world_points_3d=pts_map,  # Backwards compatibility
            cloud_3d=voxel_down(base_points[(dist_b > self.self_mask_radius) & (base_points[:, 2] <= CLOUD_MAX_Z)],
                                CLOUD_VOXEL).astype(np.float32) if self.publish_cloud else None
        ))

        if (timestamp - getattr(self, 'last_log_time', 0.0)) > 30.0:
            self.last_log_time = timestamp
            min_d = float(np.min(dist_obs)) if len(pts_obs) > 0 else 99.0
            print(f"  [PERCEPTION] Body-frame Scan: {len(sampled_2d)} pts | Closest obstacle: {min_d:.2f}m")

    def step(self, current_time: float) -> None:
        if self.sim is None or self.depth_cam is None:
            return
        try:
            depth_mat = grab_depth_frame(self.sim, self.depth_cam, self.camera_matrix)
            rgb_mat = grab_rgb_frame(self.sim, self.rgb_cam, self.camera_matrix) if self.rgb_cam else None
            self.process_frame(depth_mat, rgb_mat, current_time)
        except Exception as e:
            self.handle_error(e)
