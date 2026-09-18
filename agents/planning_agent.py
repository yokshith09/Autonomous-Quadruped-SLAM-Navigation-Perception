"""Planning Agent: Vector-dilated A* search and line-of-sight shortcut smoothing."""
import time
import math
from typing import List, Tuple, Optional
from .base import BaseAgent
from .bus import MessageBus, PathMsg, FrontierTargetMsg, PoseMsg, GridMapMsg
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

    def initialize(self) -> bool:
        self.bus.subscribe("frontier/target", self.on_target)
        self.bus.subscribe("pose/state", self.on_pose)
        self.is_running = True
        return True

    def on_target(self, target_msg: FrontierTargetMsg):
        self.latest_target = target_msg

    def on_pose(self, pose_msg: PoseMsg):
        self.latest_pose = pose_msg

    def plan_path(self, current_time: float) -> Optional[List[Tuple[float, float]]]:
        if self.grid is None or self.latest_target is None or self.latest_pose is None:
            return None

        # Cooldown guard: prevent CPU over-utilization
        if (current_time - self.last_replan_time) < self.replan_cooldown:
            return self.current_path

        rx, ry = self.latest_pose.x, self.latest_pose.y
        start_gx, start_gy = self.grid.world_to_grid(rx, ry)
        goal_gx, goal_gy = self.latest_target.target_gx, self.latest_target.target_gy

        # Ensure start and goal are on free navigable cells
        start_gx, start_gy = find_nearest_free(self.grid, start_gx, start_gy)
        goal_gx, goal_gy = find_nearest_free(self.grid, goal_gx, goal_gy)

        if start_gx == goal_gx and start_gy == goal_gy:
            return None

        # a_star_search takes (grid, (start_gx, start_gy), (goal_gx, goal_gy)) and returns world waypoints [(x, y), ...]
        world_path = a_star_search(self.grid, (start_gx, start_gy), (goal_gx, goal_gy), smooth=False)
        if not world_path:
            return None

        # Line-of-sight shortcut smoothing on world coordinates
        world_waypoints = smooth_path(self.grid, world_path)

        self.last_replan_time = current_time
        self.replan_count += 1
        self.current_path = world_waypoints

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
