"""SLAM Agent: IMU-stabilized scan-matching odometry and continuous pose estimation."""
import math
from .base import BaseAgent
from .bus import MessageBus, PoseMsg, ScanMsg, ActuationMsg, MissionEventMsg
from core.slam import ICPPoseEstimator, wrap_angle


class SLAMAgent(BaseAgent):
    def __init__(self, bus: MessageBus, initial_x: float = 0.0, initial_y: float = 0.0, initial_yaw: float = 0.0, 
                 use_ground_truth_nav: bool = False, update_rate_hz: float = 10.0):
        super().__init__("SLAMAgent", bus, update_rate_hz)
        self.pose_estimator = ICPPoseEstimator(initial_x, initial_y, initial_yaw)
        self.latest_scan: ScanMsg = None
        self.use_ground_truth_nav = use_ground_truth_nav  # Closed-loop SLAM is default (False)
        self.latest_gt_pose = (initial_x, initial_y, initial_yaw)
        self.accum_odom_dx = 0.0
        self.accum_odom_dy = 0.0
        self.accum_odom_dyaw = 0.0
        self.track_times_ms = []
        self.odom_distance_m = 0.0

    def initialize(self) -> bool:
        self.bus.subscribe("sensor/scan", self.on_scan)
        if self.use_ground_truth_nav:
            self.bus.subscribe("pose/ground_truth", self.on_gt_pose)
        self.bus.subscribe("control/actuation", self.on_actuation)
        self.is_running = True
        return True

    def on_scan(self, scan_msg: ScanMsg):
        self.latest_scan = scan_msg

    def on_gt_pose(self, pose_msg: PoseMsg):
        self.latest_gt_pose = (pose_msg.x, pose_msg.y, pose_msg.yaw)

    def on_actuation(self, act_msg: ActuationMsg):
        gyro_dyaw = act_msg.gyro_dyaw
        mid_yaw = wrap_angle(self.pose_estimator.pose.yaw + self.accum_odom_dyaw + 0.5 * gyro_dyaw)
        odom_lat = getattr(act_msg, "odom_lateral", 0.0)
        self.odom_distance_m += math.hypot(act_msg.odom_forward, odom_lat)
        self.accum_odom_dx += act_msg.odom_forward * math.cos(mid_yaw) - odom_lat * math.sin(mid_yaw)
        self.accum_odom_dy += act_msg.odom_forward * math.sin(mid_yaw) + odom_lat * math.cos(mid_yaw)
        self.accum_odom_dyaw = wrap_angle(self.accum_odom_dyaw + gyro_dyaw)

    def step(self, current_time: float) -> None:
        if self.latest_scan is None or len(self.latest_scan.points_2d) == 0:
            return

        try:
            import time as _time
            t_start = _time.perf_counter()
            gt_x, gt_y, gt_yaw = self.latest_gt_pose
            
            # Points are already strictly in the robot base frame (kinematic extrinsics)
            base_pts = self.latest_scan.points_2d
            
            # Run Point-to-Line Metric ICP with kinematic odometry & gyro prior + loop closure PGO
            prev_closures = len(self.pose_estimator.loop_manager.loop_closures)
            slam_pose = self.pose_estimator.update(
                base_pts,
                odom_dx=self.accum_odom_dx,
                odom_dy=self.accum_odom_dy,
                odom_dyaw=self.accum_odom_dyaw,
                timestamp=current_time
            )
            self.track_times_ms.append((_time.perf_counter() - t_start) * 1000.0)
            self.accum_odom_dx = 0.0
            self.accum_odom_dy = 0.0
            self.accum_odom_dyaw = 0.0
            
            # Check for new loop closures
            if len(self.pose_estimator.loop_manager.loop_closures) > prev_closures:
                latest_c = self.pose_estimator.loop_manager.loop_closures[-1]
                self.bus.publish("mission/event", MissionEventMsg(
                    timestamp=current_time,
                    event_type="LOOP_CLOSURE_DETECTED",
                    details={
                        "matched_keyframe": latest_c["keyframe_id"],
                        "rmse": latest_c["rmse"],
                        "matches": latest_c["matches"]
                    }
                ))

            # Publish SLAM estimated pose
            self.bus.publish("pose/slam", PoseMsg(
                timestamp=current_time,
                x=slam_pose.x,
                y=slam_pose.y,
                yaw=slam_pose.yaw,
                is_ground_truth=False,
                rmse=float(getattr(slam_pose, 'rmse', 0.0))
            ))

            # Active navigation pose: SLAM estimate by default (closed-loop)
            active_x = gt_x if self.use_ground_truth_nav else slam_pose.x
            active_y = gt_y if self.use_ground_truth_nav else slam_pose.y
            active_yaw = gt_yaw if self.use_ground_truth_nav else slam_pose.yaw

            self.bus.publish("pose/state", PoseMsg(
                timestamp=current_time,
                x=active_x,
                y=active_y,
                yaw=active_yaw,
                is_ground_truth=self.use_ground_truth_nav,
                rmse=float(getattr(slam_pose, 'rmse', 0.0))
            ))

            if (current_time - getattr(self, "last_log_time", 0.0)) > 30.0:
                self.last_log_time = current_time
                n_kfs = len(self.pose_estimator.loop_manager.keyframes)
                rmse = float(getattr(slam_pose, 'rmse', 0.0))
                mode_str = "GROUND_TRUTH" if self.use_ground_truth_nav else "CLOSED_LOOP_SLAM"
                print(f"  [SLAM ({mode_str})] Pose ({slam_pose.x:.2f}, {slam_pose.y:.2f}, {math.degrees(slam_pose.yaw):.1f}°) | Keyframes: {n_kfs} | ICP Residual: {rmse:.3f}m")
        except Exception as e:
            self.handle_error(e)
