"""Frontier Exploration Agent: Hierarchical room-by-room frontier selection and doorway transitions."""
import math
import numpy as np
from typing import List, Tuple, Set, Optional
from .base import BaseAgent
from .bus import MessageBus, FrontierTargetMsg, GridMapMsg, PoseMsg, MissionEventMsg
from grid import find_nearest_free


class FrontierAgent(BaseAgent):
    def __init__(self, bus: MessageBus, grid_ref=None, min_dist: float = 0.5, max_dist: float = 6.0, update_rate_hz: float = 1.0):
        super().__init__("FrontierAgent", bus, update_rate_hz)
        self.grid = grid_ref
        self.min_dist = min_dist
        self.max_dist = max_dist
        self.latest_pose: Optional[PoseMsg] = None
        self.latest_grid_msg: Optional[GridMapMsg] = None
        
        # Room-by-Room Exploration State
        self.active_room_id: int = -1
        self.explored_rooms: Set[int] = set()
        self.visited_doorways: Set[Tuple[float, float]] = set()
        self.blacklist_frontiers: Set[Tuple[int, int]] = set()
        self.visited_positions: List[Tuple[float, float]] = []
        self.state: str = "INTRA_ROOM_SWEEP"  # INTRA_ROOM_SWEEP | INTER_ROOM_TRANSIT | MISSION_COMPLETE

    def initialize(self) -> bool:
        self.bus.subscribe("pose/state", self.on_pose)
        self.bus.subscribe("map/grid", self.on_grid)
        self.is_running = True
        return True

    def on_pose(self, pose_msg: PoseMsg):
        self.latest_pose = pose_msg
        if not self.visited_positions or math.hypot(pose_msg.x - self.visited_positions[-1][0], pose_msg.y - self.visited_positions[-1][1]) > 0.4:
            self.visited_positions.append((pose_msg.x, pose_msg.y))

    def on_grid(self, grid_msg: GridMapMsg):
        self.latest_grid_msg = grid_msg

    def select_frontier(self, current_time: float) -> Optional[FrontierTargetMsg]:
        if self.grid is None or self.latest_pose is None:
            return None

        rx, ry = self.latest_pose.x, self.latest_pose.y
        rgx, rgy = self.grid.world_to_grid(rx, ry)

        # Detect current room if unassigned
        room_labels = self.latest_grid_msg.room_labels if self.latest_grid_msg else None
        if room_labels is not None and 0 <= rgy < room_labels.shape[0] and 0 <= rgx < room_labels.shape[1]:
            current_detected_room = int(room_labels[rgy, rgx])
            if current_detected_room > 1 and self.active_room_id == -1:
                self.active_room_id = current_detected_room
                self.bus.publish("mission/event", MissionEventMsg(
                    timestamp=current_time,
                    event_type="ROOM_ENTERED",
                    details={"room_id": self.active_room_id}
                ))

        all_frontiers = self.grid.detect_frontiers()
        if not all_frontiers:
            self.state = "MISSION_COMPLETE"
            return None

        # 1. Collect valid candidates, respecting blacklist
        candidates = []
        for f in all_frontiers:
            gx, gy, area = f
            if (gx, gy) in self.blacklist_frontiers:
                continue
            wx, wy = self.grid.grid_to_world(gx, gy)
            dist = math.hypot(wx - rx, wy - ry)
            candidates.append((f, dist))

        if not candidates:
            # If all frontiers were blacklisted, clear blacklist and re-try
            if self.blacklist_frontiers:
                self.blacklist_frontiers.clear()
                for f in all_frontiers:
                    wx, wy = self.grid.grid_to_world(f[0], f[1])
                    dist = math.hypot(wx - rx, wy - ry)
                    candidates.append((f, dist))
            if not candidates:
                self.state = "MISSION_COMPLETE"
                return None

        # Filter by min_dist (prefer frontiers > min_dist, but fall back to closer if none exist)
        dist_candidates = [c for c in candidates if c[1] >= self.min_dist]
        if not dist_candidates:
            dist_candidates = candidates  # Use all available candidates

        # Classify into active room vs other rooms
        room_frontiers = []
        other_frontiers = []
        for f, dist in dist_candidates:
            gx, gy, area = f
            f_room = -1
            if room_labels is not None and 0 <= gy < room_labels.shape[0] and 0 <= gx < room_labels.shape[1]:
                f_room = int(room_labels[gy, gx])

            if self.active_room_id > 1 and f_room == self.active_room_id:
                room_frontiers.append((f, dist))
            else:
                other_frontiers.append((f, dist))

        # Decision Logic:
        # Case A: Current room still has unexplored frontiers -> Clear it!
        if room_frontiers:
            self.state = "INTRA_ROOM_SWEEP"
            target_frontier = self._score_frontiers(room_frontiers, rx, ry)
            gx, gy, area = target_frontier
            wx, wy = self.grid.grid_to_world(gx, gy)
            
            target_msg = FrontierTargetMsg(
                timestamp=current_time,
                target_gx=gx,
                target_gy=gy,
                target_wx=wx,
                target_wy=wy,
                room_id=self.active_room_id,
                area=area,
                is_doorway=False
            )
            self.bus.publish("frontier/target", target_msg)
            return target_msg

        # Case B: Current room is CLEARED! Transition to next room via Doorway
        doorways = self.latest_grid_msg.doorways if self.latest_grid_msg else []
        unvisited_doorways = [d for d in doorways if d not in self.visited_doorways]

        if unvisited_doorways:
            self.state = "INTER_ROOM_TRANSIT"
            unvisited_doorways.sort(key=lambda d: math.hypot(d[0] - rx, d[1] - ry))
            dw_target = unvisited_doorways[0]
            self.visited_doorways.add(dw_target)
            
            if self.active_room_id > 0:
                self.explored_rooms.add(self.active_room_id)
                self.bus.publish("mission/event", MissionEventMsg(
                    timestamp=current_time,
                    event_type="ROOM_COMPLETED",
                    details={"room_id": self.active_room_id}
                ))

            dgx, dgy = self.grid.world_to_grid(dw_target[0], dw_target[1])
            target_msg = FrontierTargetMsg(
                timestamp=current_time,
                target_gx=dgx,
                target_gy=dgy,
                target_wx=dw_target[0],
                target_wy=dw_target[1],
                room_id=-1,
                area=50,
                is_doorway=True
            )
            self.bus.publish("frontier/target", target_msg)
            return target_msg

        # Case C: Global exploration if other frontiers exist
        if other_frontiers:
            self.state = "GLOBAL_EXPLORATION"
            target_frontier = self._score_frontiers(other_frontiers, rx, ry)
            gx, gy, area = target_frontier
            wx, wy = self.grid.grid_to_world(gx, gy)
            
            target_msg = FrontierTargetMsg(
                timestamp=current_time,
                target_gx=gx,
                target_gy=gy,
                target_wx=wx,
                target_wy=wy,
                room_id=-1,
                area=area,
                is_doorway=False
            )
            self.bus.publish("frontier/target", target_msg)
            return target_msg

        self.state = "MISSION_COMPLETE"
        return None

    def _score_frontiers(self, frontier_list, rx, ry):
        best_f = None
        best_score = -float('inf')
        for f, dist in frontier_list:
            gx, gy, area = f
            wx, wy = self.grid.grid_to_world(gx, gy)
            
            # Distance penalty
            score = area - (dist * 4.0)
            
            # Novelty bonus: reward distance from historical positions
            if self.visited_positions:
                min_vis_dist = min(math.hypot(wx - vx, wy - vy) for vx, vy in self.visited_positions)
                score += min(min_vis_dist * 2.0, 10.0)
                
            if score > best_score:
                best_score = score
                best_f = f
        return best_f

    def step(self, current_time: float) -> None:
        try:
            self.select_frontier(current_time)
        except Exception as e:
            self.handle_error(e)
