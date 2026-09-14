"""Perception Agent: RGB-D acquisition, pinhole 3D back-projection, and spatial gating."""
import numpy as np
from typing import Optional, Tuple
from .base import BaseAgent
from .bus import MessageBus, RGBDMsg, ScanMsg
from sensors import depth_to_world_points, grab_depth_frame, grab_rgb_frame


class PerceptionAgent(BaseAgent):
    def __init__(self, bus: MessageBus, sim=None, depth_cam=None, rgb_cam=None, camera_matrix=None, update_rate_hz: float = 10.0):
        super().__init__("PerceptionAgent", bus, update_rate_hz)
        self.sim = sim
        self.depth_cam = depth_cam
        self.rgb_cam = rgb_cam
        self.camera_matrix = camera_matrix
        self.point_stride = 4
        self.self_mask_radius = 0.40  # 40cm radial deadzone to eliminate leg mapping
        self.min_height = 0.04
        self.max_height = 0.60

    def initialize(self) -> bool:
        self.is_running = True
        return True

    def process_frame(self, depth_matrix: np.ndarray, rgb_matrix: Optional[np.ndarray], cam_pose_matrix: np.ndarray, robot_x: float, robot_y: float, timestamp: float):
        """Process raw depth and RGB into filtered 3D points and 2D obstacle scans."""
        if depth_matrix is None:
            return

        # Publish raw sensory buffer
        self.bus.publish("sensor/rgbd", RGBDMsg(
            timestamp=timestamp,
            rgb_image=rgb_matrix,
            depth_matrix=depth_matrix
        ))

        world_points = depth_to_world_points(depth_matrix, self.camera_matrix, cam_pose_matrix)
        if len(world_points) == 0:
            return

        # Spatial band filtering: eliminate floor and ceiling returns
        valid_mask = (world_points[:, 2] >= self.min_height) & (world_points[:, 2] <= self.max_height)
        pts_w = world_points[valid_mask]
        if len(pts_w) == 0:
            return

        # Radial self-masking: discard points within robot chassis/legs
        dist_w = np.hypot(pts_w[:, 0] - robot_x, pts_w[:, 1] - robot_y)
        pts_w = pts_w[dist_w > self.self_mask_radius]
        if len(pts_w) == 0:
            return

        # Downsample for 2D scan matching
        sampled_2d = pts_w[::self.point_stride, :2]

        # Publish filtered scan
        self.bus.publish("sensor/scan", ScanMsg(
            timestamp=timestamp,
            points_2d=sampled_2d,
            world_points_3d=pts_w
        ))

    def step(self, current_time: float) -> None:
        if self.sim is None or self.depth_cam is None:
            return
        try:
            raw_matrix = self.sim.getObjectMatrix(self.depth_cam, self.sim.handle_world)
            depth_mat = grab_depth_frame(self.sim, self.depth_cam, self.camera_matrix)
            rgb_mat = grab_rgb_frame(self.sim, self.rgb_cam, self.camera_matrix) if self.rgb_cam else None
            
            # Fetch latest pose from bus
            latest_pose = self.bus.get_latest("pose/state")
            rx = latest_pose.x if latest_pose else 0.0
            ry = latest_pose.y if latest_pose else 0.0
            
            self.process_frame(depth_mat, rgb_mat, raw_matrix, rx, ry, current_time)
        except Exception as e:
            self.handle_error(e)
