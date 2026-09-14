"""Locomotion Agent: Biomimetic sinusoidal quadruped gait generation and heading control."""
import math
from typing import List, Tuple, Optional
from .base import BaseAgent
from .bus import MessageBus, ActuationMsg, PathMsg, PoseMsg
from robot import LizardRobot, norm_angle


class LocomotionAgent(BaseAgent):
    def __init__(self, bus: MessageBus, robot_driver: Optional[LizardRobot] = None, wp_radius: float = 0.20, update_rate_hz: float = 20.0):
        super().__init__("LocomotionAgent", bus, update_rate_hz)
        self.robot = robot_driver
        self.wp_radius = wp_radius
        self.waypoints: List[Tuple[float, float]] = []
        self.wp_idx: int = 0
        self.latest_pose: Optional[PoseMsg] = None
        self.speed: float = 0.0
        self.state_str: str = "IDLE"

    def initialize(self) -> bool:
        self.bus.subscribe("plan/path", self.on_path)
        self.bus.subscribe("pose/state", self.on_pose)
        self.is_running = True
        return True

    def on_path(self, path_msg: PathMsg):
        self.waypoints = path_msg.waypoints
        self.wp_idx = 0

    def on_pose(self, pose_msg: PoseMsg):
        self.latest_pose = pose_msg

    def step(self, current_time: float) -> None:
        if self.latest_pose is None:
            return

        rx, ry, r_yaw = self.latest_pose.x, self.latest_pose.y, self.latest_pose.yaw

        # Waypoint tracking
        if self.waypoints and self.wp_idx < len(self.waypoints):
            target_wx, target_wy = self.waypoints[self.wp_idx]
            dist_to_wp = math.hypot(target_wx - rx, target_wy - ry)

            if dist_to_wp < self.wp_radius:
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    self.speed = 0.0
                    self.state_str = "ARRIVED"
                else:
                    target_wx, target_wy = self.waypoints[self.wp_idx]

            if self.wp_idx < len(self.waypoints):
                target_heading = math.atan2(target_wy - ry, target_wx - rx)
                heading_err = norm_angle(target_heading - r_yaw)

                # Heading steering logic
                if abs(heading_err) > 0.45:
                    self.speed = 0.0  # Pivot turn in place
                    self.state_str = "PIVOT"
                else:
                    self.speed = 1.0  # Walk forward
                    self.state_str = "WALK"

                if self.robot is not None:
                    self.robot.steer_heading(target_heading, current_time)
                    self.robot.apply_gait(current_time, self.speed)

                self.bus.publish("control/actuation", ActuationMsg(
                    timestamp=current_time,
                    joint_targets={},
                    heading_error=heading_err,
                    speed=self.speed,
                    state_str=self.state_str
                ))
        else:
            self.speed = 0.0
            self.state_str = "WAITING_PATH"
            if self.robot is not None:
                self.robot.apply_gait(current_time, 0.0)
