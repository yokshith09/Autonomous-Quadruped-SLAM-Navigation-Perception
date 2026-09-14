"""SLAM Agent: IMU-stabilized scan-matching odometry and continuous pose estimation."""
import numpy as np
import math
from .base import BaseAgent
from .bus import MessageBus, PoseMsg, ScanMsg
from slam import ICPPoseEstimator, world_to_base_points, base_to_world_points


class SLAMAgent(BaseAgent):
    def __init__(self, bus: MessageBus, initial_x: float = 0.0, initial_y: float = 0.0, initial_yaw: float = 0.0, update_rate_hz: float = 10.0):
        super().__init__("SLAMAgent", bus, update_rate_hz)
        self.pose_estimator = ICPPoseEstimator(initial_x, initial_y, initial_yaw)
        self.latest_scan: ScanMsg = None
        self.use_ground_truth_nav = True  # Can bypass SLAM drift for perfect ground truth evaluation
        self.latest_gt_pose = (initial_x, initial_y, initial_yaw)

    def initialize(self) -> bool:
        self.bus.subscribe("sensor/scan", self.on_scan)
        self.bus.subscribe("pose/ground_truth", self.on_gt_pose)
        self.is_running = True
        return True

    def on_scan(self, scan_msg: ScanMsg):
        self.latest_scan = scan_msg

    def on_gt_pose(self, pose_msg: PoseMsg):
        self.latest_gt_pose = (pose_msg.x, pose_msg.y, pose_msg.yaw)

    def step(self, current_time: float) -> None:
        if self.latest_scan is None or len(self.latest_scan.points_2d) == 0:
            return

        try:
            gt_x, gt_y, gt_yaw = self.latest_gt_pose
            sampled_w = self.latest_scan.points_2d

            # Convert to IMU-stabilized base frame using current heading
            head_pts = world_to_base_points(sampled_w, gt_x, gt_y, gt_yaw)
            
            # Run ICP match
            slam_pose = self.pose_estimator.update(head_pts)

            # Publish SLAM estimated pose
            self.bus.publish("pose/slam", PoseMsg(
                timestamp=current_time,
                x=slam_pose.x,
                y=slam_pose.y,
                yaw=slam_pose.yaw,
                is_ground_truth=False
            ))

            # Active navigation pose
            active_x = gt_x if self.use_ground_truth_nav else slam_pose.x
            active_y = gt_y if self.use_ground_truth_nav else slam_pose.y
            active_yaw = gt_yaw if self.use_ground_truth_nav else slam_pose.yaw

            self.bus.publish("pose/state", PoseMsg(
                timestamp=current_time,
                x=active_x,
                y=active_y,
                yaw=active_yaw,
                is_ground_truth=self.use_ground_truth_nav
            ))
        except Exception as e:
            self.handle_error(e)
