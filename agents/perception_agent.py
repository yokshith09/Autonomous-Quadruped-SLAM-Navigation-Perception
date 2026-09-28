"""Perception Agent: RGB-D acquisition, pinhole 3D back-projection, and spatial gating."""
import numpy as np
from typing import Optional
from .base import BaseAgent
from .bus import MessageBus, RGBDMsg, ScanMsg
from sensors import depth_to_base_points, grab_depth_frame, grab_rgb_frame


class PerceptionAgent(BaseAgent):
    def __init__(self, bus: MessageBus, sim=None, depth_cam=None, rgb_cam=None, camera_matrix=None, update_rate_hz: float = 10.0):
        super().__init__("PerceptionAgent", bus, update_rate_hz)
        self.sim = sim
        self.depth_cam = depth_cam
        self.rgb_cam = rgb_cam
        self.camera_matrix = camera_matrix
        self.T_base_cam = camera_matrix.get('T_base_cam', None) if camera_matrix else None
        self.point_stride = 4
        self.self_mask_radius = 0.40  # 40cm radial deadzone to eliminate leg mapping
        self.min_height = 0.04
        self.max_height = 0.60

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

        # Spatial band filtering in robot body frame: eliminate floor and high reflections
        valid_mask = (base_points[:, 2] >= self.min_height) & (base_points[:, 2] <= self.max_height)
        pts_b = base_points[valid_mask]
        if len(pts_b) == 0:
            return

        # Radial self-masking in robot base frame: discard returns within chassis/leg radius
        dist_b = np.hypot(pts_b[:, 0], pts_b[:, 1])
        pts_b = pts_b[dist_b > self.self_mask_radius]
        if len(pts_b) == 0:
            return

        # Downsample for 2D scan matching
        sampled_2d = pts_b[::self.point_stride, :2]

        # Publish filtered body-frame scan
        self.bus.publish("sensor/scan", ScanMsg(
            timestamp=timestamp,
            points_2d=sampled_2d,
            base_points_3d=pts_b,
            world_points_3d=pts_b  # Backwards compatibility
        ))

        if (timestamp - getattr(self, 'last_log_time', 0.0)) > 3.0:
            self.last_log_time = timestamp
            min_d = float(np.min(dist_b)) if len(pts_b) > 0 else 99.0
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
