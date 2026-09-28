"""Locomotion Agent: Biomimetic sinusoidal quadruped gait generation and heading control."""
import math
import numpy as np
from typing import List, Tuple, Optional
from .base import BaseAgent
from .bus import MessageBus, ActuationMsg, PathMsg, PoseMsg, ScanMsg, MissionEventMsg
from robot import LizardRobot, norm_angle


class LocomotionAgent(BaseAgent):
    def __init__(self, bus: MessageBus, robot_driver: Optional[LizardRobot] = None, wp_radius: float = 0.25, update_rate_hz: float = 20.0):
        super().__init__("LocomotionAgent", bus, update_rate_hz)
        self.robot = robot_driver
        self.wp_radius = wp_radius
        self.waypoints: List[Tuple[float, float]] = []
        self.wp_idx: int = 0
        self.latest_pose: Optional[PoseMsg] = None
        self.latest_scan: Optional[ScanMsg] = None
        self.speed: float = 0.0
        self.state_str: str = "IDLE"

        # Anti-stuck watchdog state
        self.last_stuck_check_time: float = 0.0
        self.last_stuck_pos: Tuple[float, float] = (0.0, 0.0)
        self.stuck_ticks: int = 0
        self.recovery_countdown: int = 0

    def initialize(self) -> bool:
        self.bus.subscribe("plan/path", self.on_path)
        self.bus.subscribe("pose/state", self.on_pose)
        self.bus.subscribe("sensor/scan", self.on_scan)
        self.is_running = True
        return True

    def on_path(self, path_msg: PathMsg):
        self.waypoints = path_msg.waypoints
        self.wp_idx = 0

    def on_pose(self, pose_msg: PoseMsg):
        self.latest_pose = pose_msg

    def on_scan(self, scan_msg: ScanMsg):
        self.latest_scan = scan_msg

    def step(self, current_time: float) -> None:
        if self.latest_pose is None:
            return

        rx, ry, r_yaw = self.latest_pose.x, self.latest_pose.y, self.latest_pose.yaw

        # 1. Anti-stuck active recovery maneuver (reverse step)
        if self.recovery_countdown > 0:
            self.recovery_countdown -= 1
            self.speed = -0.5  # Reverse gait swing
            self.state_str = "RECOVER_BACKUP"
            if self.robot is not None:
                try:
                    self.robot.apply_gait(current_time, self.speed)
                except Exception as e:
                    self.handle_error(e)
            self.bus.publish("control/actuation", ActuationMsg(
                timestamp=current_time,
                joint_targets={},
                heading_error=0.0,
                speed=self.speed,
                state_str=self.state_str
            ))
            return

        # 2. Waypoint tracking & navigation
        if self.waypoints and self.wp_idx < len(self.waypoints):
            # Advance past start/intermediate waypoints already within reach or behind the next segment
            while self.wp_idx < len(self.waypoints) - 1:
                cur_d = math.hypot(self.waypoints[self.wp_idx][0] - rx, self.waypoints[self.wp_idx][1] - ry)
                nxt_d = math.hypot(self.waypoints[self.wp_idx + 1][0] - rx, self.waypoints[self.wp_idx + 1][1] - ry)
                if cur_d < max(self.wp_radius, 0.32) or nxt_d < cur_d:
                    self.wp_idx += 1
                else:
                    break

            target_wx, target_wy = self.waypoints[self.wp_idx]
            dist_to_wp = math.hypot(target_wx - rx, target_wy - ry)

            if dist_to_wp < self.wp_radius:
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    self.speed = 0.0
                    self.state_str = "ARRIVED"
                    self.waypoints = []
                    self.wp_idx = 0
                else:
                    target_wx, target_wy = self.waypoints[self.wp_idx]

            if self.waypoints and self.wp_idx < len(self.waypoints):
                target_heading = math.atan2(target_wy - ry, target_wx - rx)

                # Reactive obstacle repulsion using 2D sensory scan
                avoidance_yaw = 0.0
                if self.latest_scan is not None and self.latest_scan.points_2d is not None and len(self.latest_scan.points_2d) > 0:
                    pts = self.latest_scan.points_2d
                    # Coordinates in robot body frame: forward (x_b), lateral (y_b)
                    x_b = pts[:, 0]
                    y_b = pts[:, 1]

                    # Forward safety envelope: 15cm to 58cm ahead, 30cm half-width
                    crit = (x_b > 0.15) & (x_b < 0.58) & (np.abs(y_b) < 0.30)
                    if np.any(crit):
                        close_yb = y_b[crit]
                        mean_yb = float(np.mean(close_yb))
                        # Steer laterally away from closest obstacle cluster
                        avoidance_yaw = -0.60 if mean_yb >= 0 else 0.60

                steer_target = norm_angle(target_heading + avoidance_yaw)
                heading_err = norm_angle(steer_target - r_yaw)
                abs_err = abs(heading_err)

                # Continuous speed modulation
                if avoidance_yaw != 0.0:
                    self.speed = 0.30
                    self.state_str = "AVOID"
                    if (current_time - getattr(self, 'last_avoid_log_time', 0.0)) > 2.5:
                        self.last_avoid_log_time = current_time
                        print(f"  [LOCOMOTION] Proximity safety envelope: Repelling from obstacle cluster (steer bias: {avoidance_yaw:+.2f} rad)")
                elif abs_err > 0.85:
                    self.speed = 0.12  # Turn on the spot instead of walking forward into obstacles
                    self.state_str = "TURN"
                elif abs_err > 0.45:
                    self.speed = 0.45  # Continuous turn-while-walking
                    self.state_str = "TURN"
                elif abs_err > 0.20:
                    self.speed = 0.75  # Arc turn
                    self.state_str = "WALK"
                else:
                    self.speed = 1.0   # Full forward walk
                    self.state_str = "WALK"

                # Anti-stuck watchdog: verify physical progress every 1.0s
                if current_time - self.last_stuck_check_time >= 1.0:
                    dist_moved = math.hypot(rx - self.last_stuck_pos[0], ry - self.last_stuck_pos[1])
                    if self.speed > 0.2 and dist_moved < 0.05:
                        self.stuck_ticks += 1
                    else:
                        self.stuck_ticks = 0
                    self.last_stuck_pos = (rx, ry)
                    self.last_stuck_check_time = current_time

                if self.stuck_ticks >= 2:  # Stuck for ~2 seconds
                    print(f"  [LOCOMOTION] Watchdog detected obstacle stall at ({rx:.2f}, {ry:.2f}) -> Executing reverse recovery step")
                    self.recovery_countdown = 16  # 16 steps (0.8s) of reverse recovery
                    self.stuck_ticks = 0
                    self.waypoints = []
                    self.wp_idx = 0
                    self.bus.publish("mission/event", MissionEventMsg(
                        timestamp=current_time,
                        event_type="ROBOT_STUCK",
                        details={"x": rx, "y": ry}
                    ))

                d_fwd, d_lat, dyaw_gyro = 0.16 * self.speed * 0.05, 0.0, 0.0
                if self.robot is not None:
                    try:
                        orient = self.robot.sim.getObjectOrientation(self.robot.base, self.robot.sim.handle_world)
                        self.robot.steer_heading(orient, r_yaw, steer_target)
                        self.robot.apply_gait(current_time, self.speed)
                        dt_step = max(0.01, current_time - getattr(self, "_prev_cmd_time", current_time - 0.05))
                        self._prev_cmd_time = current_time
                        d_fwd, dyaw_gyro = self.robot.get_odometry_imu_delta(dt_step, self.speed)
                        d_lat = getattr(self.robot, "last_odom_lateral", 0.0)
                    except Exception as e:
                        self.handle_error(e)

                self.bus.publish("control/actuation", ActuationMsg(
                    timestamp=current_time,
                    joint_targets={},
                    heading_error=heading_err,
                    speed=self.speed,
                    state_str=self.state_str,
                    odom_forward=d_fwd,
                    odom_lateral=d_lat,
                    gyro_dyaw=dyaw_gyro
                ))
        else:
            # When no waypoints are active: hold position stably (DO NOT spin in circles)
            self.speed = 0.0
            self.state_str = "IDLE"
            d_fwd, dyaw_gyro = 0.0, 0.0
            if self.robot is not None:
                try:
                    self.robot.apply_gait(current_time, 0.0)
                    _, dyaw_gyro = self.robot.get_odometry_imu_delta(0.05, 0.0)
                except Exception:
                    pass
            self.bus.publish("control/actuation", ActuationMsg(
                timestamp=current_time,
                joint_targets={},
                heading_error=0.0,
                speed=0.0,
                state_str="IDLE",
                odom_forward=d_fwd,
                gyro_dyaw=dyaw_gyro
            ))

        if (current_time - getattr(self, 'last_status_log_time', 0.0)) > 2.5:
            self.last_status_log_time = current_time
            n_wps = len(self.waypoints) - self.wp_idx if self.waypoints else 0
            print(f"  [LOCOMOTION] State: [{self.state_str}] | Speed: {self.speed:.2f} | Remaining Waypoints: {n_wps}")
