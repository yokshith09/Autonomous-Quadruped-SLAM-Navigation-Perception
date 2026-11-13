"""Locomotion Agent: Biomimetic sinusoidal quadruped gait generation and heading control."""
import math
from collections import deque
import numpy as np
from typing import List, Tuple, Optional
from .base import BaseAgent
from .bus import MessageBus, ActuationMsg, PathMsg, PoseMsg, ScanMsg, MissionEventMsg
import core.robot as robot_module
from core.robot import LizardRobot, gait_body_speed, norm_angle

TURN_IN_PLACE_RAD = 0.45  # heading error above which the robot stops and turns before walking on
LOOKAHEAD_M = 0.40        # pure-pursuit lookahead along the path
# Gait command for walking. 0.75 beats 1.0 on every measure (gait_bench, 20 s straight walks): 0.0824 vs 0.0782 m/s,
# 0 % vs 6 % of samples sliding backward, 12.7 vs 18.4 deg/s yaw wobble; at 1.0 the legs overdrive and slip.
WALK_CMD = 0.75
# Contact detection from the front hip load (core.robot.LizardRobot.leg_load), normalised by the gait command:
# free walking <= 0.0097, blocked >= 0.0222 at every command from 0.35 to 1.0 (stall_probe). The depth camera is blind
# inside about 0.22 m and odometry is dead-reckoned from the leg commands, so without this a robot pushing a wall
# reports 8 cm/s of motion it does not make (live run 10: 16 s against a wall, position error 11 -> 122 cm).
STALL_WINDOW = 12         # control steps averaged (0.6 s at 20 Hz)
STALL_LOAD_RATIO = 0.015  # mean load / command above this: the legs are pushing on something
STALL_MIN_CMD = 0.3       # below this command the load is too small to judge
# Steps already blocked before the window mean crosses the threshold: with the measured loads (free 0.0083, blocked
# about 0.027 normalised) it crosses on the 5th blocked step, so the 4 steps before it reported motion that did not
# happen and are taken back.
STALL_DELAY_STEPS = 4
STALL_MOVE_M = 0.05        # a walking robot that moves less than this in a second (scaled by the floor calibration) is
                           # stalled. Rescue floor: true speed 0.060 m/s vs 0.090 m/s in scene 1, so ordinary walking
                           # dipped under a fixed 5 cm and raised three false stalls in 150 s (rescue run 1).
RECOVERY_TURN_RAD = 1.2    # recovery turns the body this far towards the open side (about 70 deg)
TIPPED_UP_Z = 0.7071      # body tilted more than 45 deg from upright

class LocomotionAgent(BaseAgent):
    def __init__(self, bus: MessageBus, robot_driver: Optional[LizardRobot] = None, wp_radius: float = 0.18, update_rate_hz: float = 20.0):
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
        self.recovery_heading: float = 0.0
        self.calibrate_until: float = 0.0   # seconds of stand-still gyro calibration at the start (scene profile)
        self._walk_steps: int = 0   # steps commanded to walk since the last stall check
        self._window_steps: int = 0
        # Progress watchdog: a goal the robot cannot get closer to (wall in the way, mis-registered map) is dropped.
        self.progress_timeout_s: float = 15.0
        self._goal_best_dist = float("inf")
        self._goal_best_time = None
        self._load_hist = deque(maxlen=STALL_WINDOW)  # hip load / gait command, one sample per walking step
        self._odom_hist = deque(maxlen=STALL_WINDOW)  # forward odometry reported over the same steps
        self.last_load = float("nan")  # latest hip load / command, recorded for offline contact studies

    def initialize(self) -> bool:
        self.bus.subscribe("plan/path", self.on_path)
        self.bus.subscribe("pose/state", self.on_pose)
        self.bus.subscribe("sensor/scan", self.on_scan)
        self.is_running = True
        return True

    def on_path(self, path_msg: PathMsg):
        if not self.waypoints or not path_msg.waypoints or                 math.hypot(path_msg.waypoints[-1][0] - self.waypoints[-1][0],
                           path_msg.waypoints[-1][1] - self.waypoints[-1][1]) > 0.3:
            self._goal_best_dist, self._goal_best_time = float("inf"), None
        self.waypoints = path_msg.waypoints
        self.wp_idx = 0

    def _leg_blocked(self) -> bool:
        """True when the legs have been loaded well above free walking for the whole window."""
        self.last_load = float("nan")
        if self.robot is None or self.speed < STALL_MIN_CMD or self.recovery_countdown > 0:
            self._load_hist.clear()
            self._odom_hist.clear()
            return False
        try:
            self.last_load = self.robot.leg_load() / self.speed
            self._load_hist.append(self.last_load)
        except Exception as e:
            self.handle_error(e)
            return False
        return len(self._load_hist) == STALL_WINDOW and sum(self._load_hist) / STALL_WINDOW > STALL_LOAD_RATIO

    def _enter_recovery(self, current_time: float, rx: float, ry: float, why: str) -> None:
        print(f"  [LOCOMOTION] {why} at ({rx:.2f}, {ry:.2f}) -> Executing turn-away recovery")
        self.recovery_countdown = 14
        self.recovery_heading = norm_angle(self.latest_pose.yaw + RECOVERY_TURN_RAD * self._open_side())
        self.stuck_ticks = 0
        self._load_hist.clear()
        self.waypoints = []
        self.wp_idx = 0
        self.bus.publish("mission/event", MissionEventMsg(
            timestamp=current_time, event_type="ROBOT_STUCK", details={"x": rx, "y": ry, "contact": True,
                                                                    "yaw": self.latest_pose.yaw}))

    def _obstacle_points(self):
        """Low-band obstacle points (what the body can hit); scan-matching points when perception sent no low band."""
        o = getattr(self.latest_scan, "obstacle_2d", None)
        return o if o is not None else self.latest_scan.points_2d

    def _open_side(self) -> float:
        """+1 to turn left, -1 to turn right: towards the side whose scan returns are farther (more open)."""
        pts = None if self.latest_scan is None else self._obstacle_points()
        if pts is None or len(pts) == 0:
            return 1.0
        ahead = pts[(pts[:, 0] > 0.0) & (pts[:, 0] < 1.5)]
        left, right = ahead[ahead[:, 1] > 0.0], ahead[ahead[:, 1] <= 0.0]
        mean_range = lambda p: float(np.mean(np.hypot(p[:, 0], p[:, 1]))) if len(p) else 99.0
        return 1.0 if mean_range(left) >= mean_range(right) else -1.0

    def _remaining_path(self, rx: float, ry: float) -> float:
        wps = self.waypoints[self.wp_idx:]
        if not wps:
            return 0.0
        d = math.hypot(wps[0][0] - rx, wps[0][1] - ry)
        return d + sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(wps, wps[1:]))

    def on_pose(self, pose_msg: PoseMsg):
        self.latest_pose = pose_msg

    def on_scan(self, scan_msg: ScanMsg):
        self.latest_scan = scan_msg

    def step(self, current_time: float) -> None:
        if self.latest_pose is None:
            return

        rx, ry, r_yaw = self.latest_pose.x, self.latest_pose.y, self.latest_pose.yaw
        dt_step = max(0.01, current_time - getattr(self, "_prev_cmd_time", current_time - 0.05))
        self._prev_cmd_time = current_time

        # Stand still at the start so SLAM can measure the gyro bias (calibrate_until, from the scene profile).
        if current_time < self.calibrate_until:
            self.speed, self.state_str = 0.0, "CALIBRATE"
            d_fwd, d_lat, dyaw_gyro = 0.0, 0.0, 0.0
            if self.robot is not None:
                try:
                    self.robot.stop_motors()
                    _, dyaw_gyro = self.robot.get_odometry_imu_delta(dt_step, 0.0)
                except Exception as e:
                    self.handle_error(e)
            self.bus.publish("control/actuation", ActuationMsg(
                timestamp=current_time, joint_targets={}, heading_error=0.0, speed=0.0, state_str="CALIBRATE",
                odom_forward=0.0, odom_lateral=0.0, gyro_dyaw=dyaw_gyro))
            return

        # 0. Tipped over (body more than TIPPED_UP_Z from upright): stop the legs and report. SLAM and mapping read the
        # TIPPED state and ignore the camera, which then faces the floor or ceiling and would corrupt the map.
        if self.robot is not None:
            try:
                tipped = self.robot.body_up() < TIPPED_UP_Z
            except Exception as e:
                self.handle_error(e)
                tipped = False
            if tipped:
                if self.state_str != "TIPPED":
                    print(f"  [LOCOMOTION] Robot tipped over at ({rx:.2f}, {ry:.2f}) -> legs stopped")
                    self.bus.publish("mission/event", MissionEventMsg(
                        timestamp=current_time, event_type="ROBOT_TIPPED", details={"x": rx, "y": ry}))
                self.speed, self.state_str = 0.0, "TIPPED"
                self.waypoints, self.wp_idx = [], 0
                try:
                    self.robot.stop_motors()
                except Exception:
                    pass
                self.bus.publish("control/actuation", ActuationMsg(
                    timestamp=current_time, joint_targets={}, heading_error=0.0, speed=0.0, state_str="TIPPED",
                    odom_forward=0.0, odom_lateral=0.0, gyro_dyaw=0.0))
                return

        # 1. Anti-stuck recovery: stop the legs and turn the body away from the obstacle. It used to walk backwards
        # (speed -0.6) with the legs jammed; in live runs 13 and 14 the body flipped within 0.4 s of the first
        # reverse step, which ended the mission. A turn is a heading change, it cannot flip the body.
        if self.recovery_countdown > 0:
            self.recovery_countdown -= 1
            self.speed = 0.0
            self.state_str = "RECOVER_TURN"
            d_fwd, d_lat, dyaw_gyro = 0.0, 0.0, 0.0
            if self.robot is not None:
                try:
                    orient = self.robot.sim.getObjectOrientation(self.robot.base, self.robot.sim.handle_world)
                    self.robot.steer_heading(orient, r_yaw, self.recovery_heading)
                    self.robot.stop_motors()
                    d_fwd, dyaw_gyro = self.robot.get_odometry_imu_delta(dt_step, 0.0)
                    d_lat = getattr(self.robot, "last_odom_lateral", 0.0)
                except Exception as e:
                    self.handle_error(e)
            self.bus.publish("control/actuation", ActuationMsg(
                timestamp=current_time,
                joint_targets={},
                heading_error=0.0,
                speed=self.speed,
                state_str=self.state_str,
                odom_forward=d_fwd,
                odom_lateral=d_lat,
                gyro_dyaw=dyaw_gyro
            ))
            return

        # 2. Waypoint tracking & navigation (tight adherence to the planned blue line)
        if self.waypoints and self.wp_idx < len(self.waypoints):
            # Progress = remaining path length, not straight-line distance: a path around a wall moves away from the
            # goal for a while, and the straight-line test called that "no progress" (16 of 17 stuck events, run 4).
            goal_dist = self._remaining_path(rx, ry)
            if self._goal_best_time is None or goal_dist < self._goal_best_dist - 0.2:
                self._goal_best_dist, self._goal_best_time = goal_dist, current_time
            elif current_time - self._goal_best_time > self.progress_timeout_s:
                print(f"  [LOCOMOTION] No progress toward goal for {self.progress_timeout_s:.0f}s -> abandoning target")
                self.bus.publish("mission/event", MissionEventMsg(
                    timestamp=current_time, event_type="ROBOT_STUCK", details={"x": rx, "y": ry}))
                self.waypoints, self.wp_idx = [], 0
                self._goal_best_dist, self._goal_best_time = float("inf"), None
                return
            # Only skip the start cell (wp_idx == 0) if the robot is already standing on it
            if self.wp_idx == 0 and len(self.waypoints) > 1:
                if math.hypot(self.waypoints[0][0] - rx, self.waypoints[0][1] - ry) < 0.24:
                    self.wp_idx = 1

            # Advance if within wp_radius or if waypoint was just passed behind the robot
            while self.wp_idx < len(self.waypoints) - 1:
                wx, wy = self.waypoints[self.wp_idx]
                nx, ny = self.waypoints[self.wp_idx + 1]
                cur_d = math.hypot(wx - rx, wy - ry)
                # Vector from current waypoint to next waypoint vs robot to current waypoint
                seg_dx, seg_dy = nx - wx, ny - wy
                to_wp_dx, to_wp_dy = wx - rx, wy - ry
                passed_wp = (seg_dx * to_wp_dx + seg_dy * to_wp_dy) < 0.0 and cur_d < 0.26
                if cur_d < self.wp_radius or passed_wp:
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
                # Pure-pursuit: steer at the first waypoint at least LOOKAHEAD_M away, not at the next one 0.2 m
                # ahead; small pose noise and waypoint switches then barely move the commanded heading (run 5:
                # 12.7 turn-direction reversals per minute, 10 deg/s yaw wobble while walking straight).
                # Never look past a corner (path turn > 30 deg): the chord would cut it, and doorframes are tight.
                for j in range(self.wp_idx, len(self.waypoints)):
                    target_wx, target_wy = self.waypoints[j]
                    if math.hypot(target_wx - rx, target_wy - ry) >= LOOKAHEAD_M:
                        break
                    if 0 < j < len(self.waypoints) - 1:
                        (ax, ay), (bx, by) = self.waypoints[j - 1], self.waypoints[j + 1]
                        turn = norm_angle(math.atan2(by - target_wy, bx - target_wx) - math.atan2(target_wy - ay, target_wx - ax))
                        if abs(turn) > math.radians(30):
                            break
                target_heading = math.atan2(target_wy - ry, target_wx - rx)

                # Close-range frontal bumper safety envelope (narrowed so lateral doorposts don't trigger false swerving)
                avoidance_yaw = 0.0
                obst = self._obstacle_points() if self.latest_scan is not None else None
                if obst is not None and len(obst) > 0:
                    pts = obst
                    x_b = pts[:, 0]
                    y_b = pts[:, 1]

                    crit = (x_b > 0.14) & (x_b < 0.28) & (np.abs(y_b) < 0.10)
                    if np.any(crit):
                        close_yb = y_b[crit]
                        mean_yb = float(np.mean(close_yb))
                        avoidance_yaw = -0.22 if mean_yb >= 0 else 0.22

                steer_target = norm_angle(target_heading + avoidance_yaw)
                heading_err = norm_angle(steer_target - r_yaw)
                abs_err = abs(heading_err)

                # High-efficiency speed modulation
                if avoidance_yaw != 0.0:
                    self.speed = 0.70
                    self.state_str = "AVOID"
                    if (current_time - getattr(self, 'last_avoid_log_time', 0.0)) > 2.5:
                        self.last_avoid_log_time = current_time
                        print(f"  [LOCOMOTION] Proximity safety envelope: Repelling from obstacle cluster (steer bias: {avoidance_yaw:+.2f} rad)")
                elif abs_err > TURN_IN_PLACE_RAD:
                    # Turn in place, then walk straight: rotating while translating destabilises scan matching
                    # (Zhang et al., "stepwise and consistent motions", arXiv 2503.17005). This used to walk at 0.35
                    # above 0.95 rad and at 0.75 between 0.45 and 0.95 rad.
                    self.speed = 0.0
                    self.state_str = "TURN"
                else:
                    self.speed = WALK_CMD  # forward walk along the path line
                    self.state_str = "WALK"

                # Anti-stuck watchdog: verify physical progress every 1.0s
                self._window_steps += 1
                self._walk_steps += 1 if self.speed > 0.2 else 0
                if current_time - self.last_stuck_check_time >= 1.0:
                    dist_moved = math.hypot(rx - self.last_stuck_pos[0], ry - self.last_stuck_pos[1])
                    # Turn-then-walk dithering samples as "walking" now and then without ever walking a second: only a
                    # robot that walked most of the window can be stalled (live runs 13/14: false stall -> flip).
                    walked_window = self._walk_steps >= 0.8 * max(1, self._window_steps)
                    self._walk_steps = self._window_steps = 0
                    if walked_window and dist_moved < STALL_MOVE_M * robot_module.ODOM_SCALE:
                        self.stuck_ticks += 1
                    else:
                        self.stuck_ticks = 0
                    self.last_stuck_pos = (rx, ry)
                    self.last_stuck_check_time = current_time

                blocked = self._leg_blocked()
                phantom = sum(list(self._odom_hist)[-STALL_DELAY_STEPS:])  # reported while already blocked
                if blocked:
                    self._enter_recovery(current_time, rx, ry, "Legs loaded (blocked by an obstacle)")
                    self._odom_hist.clear()
                elif self.stuck_ticks >= 2:  # pose not moving for ~2 seconds
                    self._enter_recovery(current_time, rx, ry, "Watchdog detected obstacle stall")

                d_fwd, d_lat, dyaw_gyro = gait_body_speed(self.speed) * dt_step, 0.0, 0.0
                if self.robot is not None:
                    try:
                        orient = self.robot.sim.getObjectOrientation(self.robot.base, self.robot.sim.handle_world)
                        self.robot.steer_heading(orient, r_yaw, steer_target)
                        if self.speed == 0.0:
                            self.robot.stop_motors()  # a zero-speed gait still creeps sideways unmeasured
                        else:
                            self.robot.apply_gait(current_time, self.speed)
                        d_fwd, dyaw_gyro = self.robot.get_odometry_imu_delta(dt_step, self.speed)
                        d_lat = getattr(self.robot, "last_odom_lateral", 0.0)
                    except Exception as e:
                        self.handle_error(e)
                if blocked:
                    # The body was not moving for the detection window either: take back what was reported then (live
                    # run 11: about 5 cm per contact, 1 m over 20 contacts).
                    d_fwd, d_lat = -phantom, 0.0
                elif self._load_hist:
                    self._odom_hist.append(d_fwd)

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
            d_fwd, d_lat, dyaw_gyro = 0.0, 0.0, 0.0
            if self.robot is not None:
                try:
                    # Hold a still stance: a zero-speed gait still lifts the legs and makes the body creep
                    # sideways while odometry reports no motion.
                    self.robot.stop_motors()
                    d_fwd, dyaw_gyro = self.robot.get_odometry_imu_delta(dt_step, 0.0)
                    d_lat = getattr(self.robot, "last_odom_lateral", 0.0)
                except Exception:
                    pass
            self.bus.publish("control/actuation", ActuationMsg(
                timestamp=current_time,
                joint_targets={},
                heading_error=0.0,
                speed=0.0,
                state_str="IDLE",
                odom_forward=d_fwd,
                odom_lateral=d_lat,
                gyro_dyaw=dyaw_gyro
            ))

        if (current_time - getattr(self, 'last_status_log_time', 0.0)) > 30.0:
            self.last_status_log_time = current_time
            n_wps = len(self.waypoints) - self.wp_idx if self.waypoints else 0
            print(f"  [LOCOMOTION] State: [{self.state_str}] | Speed: {self.speed:.2f} | Remaining Waypoints: {n_wps}")
