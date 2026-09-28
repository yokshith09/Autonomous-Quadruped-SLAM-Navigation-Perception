"""Planning Agent: Vector-dilated A* search and line-of-sight shortcut smoothing."""
import time
import math
from typing import List, Tuple, Optional
from .base import BaseAgent
from .bus import MessageBus, PathMsg, FrontierTargetMsg, PoseMsg, GridMapMsg, MissionEventMsg
from planner import a_star_search, smooth_path
from grid import find_nearest_free, ROBOT_RADIUS


class PlanningAgent(BaseAgent):
    def __init__(self, bus: MessageBus, grid_ref=None, replan_cooldown: float = 1.0, update_rate_hz: float = 5.0):
        super().__init__("PlanningAgent", bus, update_rate_hz)
        self.grid = grid_ref
        self.replan_cooldown = replan_cooldown
        self.last_replan_time = -999.0
        self.replan_count = 0
        
        self.latest_target: Optional[FrontierTargetMsg] = None
        self.latest_pose: Optional[PoseMsg] = None
        self.current_path: List[Tuple[float, float]] = []
        self.last_planned_goal: Optional[Tuple[int, int]] = None

    def initialize(self) -> bool:
        self.bus.subscribe("frontier/target", self.on_target)
        self.bus.subscribe("pose/state", self.on_pose)
        self.bus.subscribe("mission/event", self.on_mission_event)
        self.is_running = True
        return True

    def on_mission_event(self, event_msg: MissionEventMsg):
        if event_msg.event_type in ["ROBOT_STUCK", "ROOM_COMPLETED"]:
            self.current_path = []
            self.last_replan_time = -999.0
            self.latest_target = None
            self.last_planned_goal = None

    def on_target(self, target_msg: FrontierTargetMsg):
        self.latest_target = target_msg

    def on_pose(self, pose_msg: PoseMsg):
        self.latest_pose = pose_msg

    def _is_path_blocked(self) -> bool:
        if not self.current_path or self.grid is None:
            return True
        for wx, wy in self.current_path[1:]:
            gx, gy = self.grid.world_to_grid(wx, wy)
            if self.grid.is_occupied_inflated(gx, gy, radius_cells=3):
                return True
        return False

    def plan_path(self, current_time: float) -> Optional[List[Tuple[float, float]]]:
        if self.grid is None or self.latest_target is None or self.latest_pose is None:
            return None

        rx, ry = self.latest_pose.x, self.latest_pose.y
        dist_to_goal = math.hypot(self.latest_target.target_wx - rx, self.latest_target.target_wy - ry)
        if dist_to_goal < 0.28:
            self.latest_target = None
            return None

        goal_raw = (self.latest_target.target_gx, self.latest_target.target_gy)
        same_goal = (
            self.last_planned_goal is not None and
            math.hypot(goal_raw[0] - self.last_planned_goal[0], goal_raw[1] - self.last_planned_goal[1]) <= 2.0
        )

        # Keep active path if goal hasn't changed, path is clear of obstacles, and within 4.0s window
        if same_goal and self.current_path and not self._is_path_blocked() and (current_time - self.last_replan_time) < 4.0:
            return self.current_path

        # Cooldown guard for same goal when unblocked
        if same_goal and (current_time - self.last_replan_time) < self.replan_cooldown and not self._is_path_blocked():
            return self.current_path

        start_gx, start_gy = self.grid.world_to_grid(rx, ry)
        goal_gx, goal_gy = goal_raw

        # Ensure start and goal are on free navigable cells
        start_gx, start_gy = find_nearest_free(self.grid, start_gx, start_gy)
        goal_gx, goal_gy = find_nearest_free(self.grid, goal_gx, goal_gy)

        if start_gx == goal_gx and start_gy == goal_gy:
            self.latest_target = None
            return None

        # a_star_search takes (grid, (start_gx, start_gy), (goal_gx, goal_gy)) and returns world waypoints [(x, y), ...]
        world_path = a_star_search(self.grid, (start_gx, start_gy), (goal_gx, goal_gy), smooth=False)
        if not world_path:
            print(f"  [PLANNER] Target ({self.latest_target.target_wx:.2f}, {self.latest_target.target_wy:.2f}) unreachable! Blacklisting & replanning...")
            self.bus.publish("mission/event", MissionEventMsg(
                timestamp=current_time,
                event_type="PATH_UNREACHABLE",
                details={
                    "target_gx": self.latest_target.target_gx,
                    "target_gy": self.latest_target.target_gy
                }
            ))
            self.latest_target = None
            self.current_path = []
            return None

        # Line-of-sight shortcut smoothing on world coordinates
        world_waypoints = smooth_path(self.grid, world_path)

        self.last_replan_time = current_time
        self.replan_count += 1
        self.current_path = world_waypoints
        self.last_planned_goal = goal_raw

        print(f"  [PLANNER] A* Path computed: {len(world_waypoints)} safe waypoints -> Target ({self.latest_target.target_wx:.2f}, {self.latest_target.target_wy:.2f})")

        # Publish planned path
        self.bus.publish("plan/path", PathMsg(
            timestamp=current_time,
            waypoints=world_waypoints,
            active_idx=0,
            replan_count=self.replan_count
        ))
        return world_waypoints

    def step(self, current_time: float) -> None:
        try:
            self.plan_path(current_time)
        except Exception as e:
            self.handle_error(e)
