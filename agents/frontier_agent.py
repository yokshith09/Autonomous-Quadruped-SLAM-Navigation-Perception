"""Frontier Exploration Agent: Hierarchical room-by-room frontier selection and doorway transitions."""
import math
import numpy as np
from typing import List, Tuple, Set, Optional
from .base import BaseAgent
from .bus import MessageBus, FrontierTargetMsg, GridMapMsg, PoseMsg, MissionEventMsg
from grid import find_nearest_free, bresenham_line
from robot import norm_angle


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

        # Goal Commitment & Hysteresis State (Prevents target thrashing / path confusion)
        self.committed_target: Optional[Tuple[int, int]] = None
        self.committed_doorway: Optional[Tuple[float, float]] = None
        self.commitment_time: float = 0.0

    def initialize(self) -> bool:
        self.bus.subscribe("pose/state", self.on_pose)
        self.bus.subscribe("map/grid", self.on_grid)
        self.bus.subscribe("mission/event", self.on_mission_event)
        self.is_running = True
        return True

    def _is_doorway_visited(self, dw: Tuple[float, float]) -> bool:
        return any(math.hypot(dw[0] - vd[0], dw[1] - vd[1]) < 0.65 for vd in self.visited_doorways)

    def _is_blacklisted(self, gx: int, gy: int, radius: float = 2.5) -> bool:
        return any(math.hypot(gx - bgx, gy - bgy) <= radius for bgx, bgy in self.blacklist_frontiers)

    def on_mission_event(self, event_msg: MissionEventMsg):
        if event_msg.event_type in ["PATH_UNREACHABLE", "ROBOT_STUCK"]:
            tgx = event_msg.details.get("target_gx")
            tgy = event_msg.details.get("target_gy")
            if tgx is not None and tgy is not None:
                self.blacklist_frontiers.add((tgx, tgy))
            # Always drop stuck or unreachable targets and doorways to prevent deadlocks
            if self.committed_doorway is not None:
                self.visited_doorways.add(self.committed_doorway)
                self.committed_doorway = None
            if self.committed_target is not None:
                self.blacklist_frontiers.add(self.committed_target)
                self.committed_target = None
                print("  [FRONTIER] Cleared stalled target from watchdog alert. Re-evaluating alternatives.")
            if len(self.blacklist_frontiers) > 40:
                self.blacklist_frontiers.clear()
        elif event_msg.event_type == "ROOM_COMPLETED":
            self.committed_target = None
            self.committed_doorway = None

    def _get_room_at(self, gx: int, gy: int, room_labels: Optional[np.ndarray], radius: int = 8) -> int:
        if room_labels is None:
            return self.active_room_id if self.active_room_id > 0 else 1
        h, w = room_labels.shape
        if not (0 <= gy < h and 0 <= gx < w):
            return self.active_room_id if self.active_room_id > 0 else 1
        val = int(room_labels[gy, gx])
        if val > 0:
            return val
        
        # Sample neighborhood around cell (essential for frontiers on unsegmented boundary)
        y0, y1 = max(0, gy - radius), min(h, gy + radius + 1)
        x0, x1 = max(0, gx - radius), min(w, gx + radius + 1)
        sub = room_labels[y0:y1, x0:x1]
        pos = sub[sub > 0]
        if len(pos) == 0:
            return self.active_room_id if self.active_room_id > 0 else 1
        vals, counts = np.unique(pos, return_counts=True)
        return int(vals[np.argmax(counts)])

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
        r_yaw = self.latest_pose.yaw
        rgx, rgy = self.grid.world_to_grid(rx, ry)

        # Check if committed target was reached
        if self.committed_target is not None:
            cwx, cwy = self.grid.grid_to_world(self.committed_target[0], self.committed_target[1])
            if math.hypot(cwx - rx, cwy - ry) < 0.40:
                if self.committed_doorway is not None:
                    self.visited_doorways.add(self.committed_doorway)
                    self.committed_doorway = None
                    print(f"  [FRONTIER] Successfully transited through doorway to ({cwx:.2f}, {cwy:.2f})")
                else:
                    self.blacklist_frontiers.add(self.committed_target)
                self.committed_target = None

        # Detect current room (do NOT prematurely mark previous room explored until its frontiers are 0)
        room_labels = self.latest_grid_msg.room_labels if self.latest_grid_msg else None
        current_detected_room = self._get_room_at(rgx, rgy, room_labels)
        if current_detected_room > 0:
            if self.active_room_id <= 0:
                self.active_room_id = current_detected_room
                print(f"  [FRONTIER] Initialized in Room {self.active_room_id} -> Starting Intra-Room Sweep")
                self.bus.publish("mission/event", MissionEventMsg(
                    timestamp=current_time,
                    event_type="ROOM_ENTERED",
                    details={"room_id": self.active_room_id}
                ))
            elif current_detected_room != self.active_room_id and current_detected_room not in self.explored_rooms:
                if self.committed_doorway is not None:
                    self.visited_doorways.add(self.committed_doorway)
                    self.committed_doorway = None
                self.active_room_id = current_detected_room
                self.state = "INTRA_ROOM_SWEEP"
                self.committed_target = None
                print(f"  [FRONTIER] >>> ENTERED NEW ROOM {self.active_room_id} <<< Commencing INTRA_ROOM_SWEEP!")
                self.bus.publish("mission/event", MissionEventMsg(
                    timestamp=current_time,
                    event_type="ROOM_ENTERED",
                    details={"room_id": self.active_room_id}
                ))
        elif self.active_room_id <= 0:
            self.active_room_id = 1

        # If currently transiting a doorway, maintain commitment until penetrating into the new room (with timeout watchdog)
        if self.state == "INTER_ROOM_TRANSIT" and self.committed_target is not None:
            cwx, cwy = self.grid.grid_to_world(self.committed_target[0], self.committed_target[1])
            dist_to_transit = math.hypot(cwx - rx, cwy - ry)
            if (current_time - self.commitment_time) > 8.0:
                print(f"  [FRONTIER] Doorway transit timed out (>8s) -> Aborting transit and falling back to open exploration.")
                if self.committed_doorway is not None:
                    self.visited_doorways.add(self.committed_doorway)
                    self.committed_doorway = None
                self.blacklist_frontiers.add(self.committed_target)
                self.committed_target = None
                self.state = "GLOBAL_EXPLORATION"
            elif dist_to_transit >= 0.40:
                target_msg = FrontierTargetMsg(
                    timestamp=current_time,
                    target_gx=self.committed_target[0],
                    target_gy=self.committed_target[1],
                    target_wx=cwx,
                    target_wy=cwy,
                    room_id=-1,
                    area=50,
                    is_doorway=True
                )
                self.bus.publish("frontier/target", target_msg)
                return target_msg

        all_frontiers = self.grid.detect_frontiers()
        if not all_frontiers:
            self.state = "MISSION_COMPLETE"
            print("  [FRONTIER] All frontiers cleared across entire map! MISSION_COMPLETE.")
            return None

        # Hold committed frontier target until reached, cleared by camera rays, or 8s timeout
        if self.state in ("INTRA_ROOM_SWEEP", "GLOBAL_EXPLORATION") and self.committed_target is not None:
            cwx, cwy = self.grid.grid_to_world(self.committed_target[0], self.committed_target[1])
            dist_to_comm = math.hypot(cwx - rx, cwy - ry)
            still_has_frontier = any(
                math.hypot(f[0] - self.committed_target[0], f[1] - self.committed_target[1]) <= 10.0
                for f in all_frontiers
            )
            if (current_time - self.commitment_time) <= 8.0 and dist_to_comm >= 0.42 and still_has_frontier:
                target_msg = FrontierTargetMsg(
                    timestamp=current_time,
                    target_gx=self.committed_target[0],
                    target_gy=self.committed_target[1],
                    target_wx=cwx,
                    target_wy=cwy,
                    room_id=self.active_room_id,
                    area=30,
                    is_doorway=False
                )
                self.bus.publish("frontier/target", target_msg)
                return target_msg
            else:
                self.blacklist_frontiers.add(self.committed_target)
                self.committed_target = None

        # 1. Collect all non-blacklisted candidates (never orphan frontiers due to Voronoi label shifts)
        candidates = []
        for f in all_frontiers:
            gx, gy, area = f
            if self._is_blacklisted(gx, gy):
                continue
            wx, wy = self.grid.grid_to_world(gx, gy)
            dist = math.hypot(wx - rx, wy - ry)
            candidates.append((f, dist))

        if not candidates and self.blacklist_frontiers:
            self.blacklist_frontiers.clear()
            for f in all_frontiers:
                wx, wy = self.grid.grid_to_world(f[0], f[1])
                dist = math.hypot(wx - rx, wy - ry)
                candidates.append((f, dist))

        if not candidates:
            self.state = "MISSION_COMPLETE"
            print("  [FRONTIER] All rooms and frontiers explored. MISSION_COMPLETE.")
            return None

        # Filter by min_dist (prefer frontiers > min_dist, but fall back to closer if none exist)
        dist_candidates = [c for c in candidates if c[1] >= self.min_dist]
        if not dist_candidates:
            dist_candidates = candidates

        # Classify into active room vs other rooms
        room_frontiers = []
        other_frontiers = []
        for f, dist in dist_candidates:
            gx, gy, area = f
            f_room = self._get_room_at(gx, gy, room_labels)
            if self.active_room_id > 0 and self.active_room_id not in self.explored_rooms and f_room == self.active_room_id:
                room_frontiers.append((f, dist))
            else:
                other_frontiers.append((f, dist))

        # Collect valid forward doorways (unvisited and navigable outside obstacle inflation)
        doorways = self.latest_grid_msg.doorways if self.latest_grid_msg else []
        valid_doorways = []
        for d in doorways:
            if self._is_doorway_visited(d):
                continue
            dgx, dgy = self.grid.world_to_grid(d[0], d[1])
            if self._is_blacklisted(dgx, dgy, radius=3.5):
                continue
            if not self.grid.is_free(dgx, dgy) or self.grid.is_occupied_inflated(dgx, dgy, radius_cells=3):
                continue
            valid_doorways.append(d)

        # Decision Logic:
        # Case A: Current room still has unexplored frontiers -> Clear it!
        if room_frontiers and self.active_room_id not in self.explored_rooms:
            self.state = "INTRA_ROOM_SWEEP"
            target_frontier = self._score_frontiers(room_frontiers, rx, ry, r_yaw)
            gx, gy, area = target_frontier
            gx, gy = find_nearest_free(self.grid, gx, gy)
            self.committed_target = (gx, gy)
            self.commitment_time = current_time
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
            print(f"  [FRONTIER] [INTRA_ROOM_SWEEP] Target: ({wx:.2f}, {wy:.2f}) [Room {self.active_room_id}] | Dist: {math.hypot(wx-rx, wy-ry):.2f}m")
            self.bus.publish("frontier/target", target_msg)
            return target_msg

        # Mark active room explored ONLY now that room_frontiers is genuinely empty
        if self.active_room_id > 0 and self.active_room_id not in self.explored_rooms:
            self.explored_rooms.add(self.active_room_id)
            print(f"  [FRONTIER] ★ Room {self.active_room_id} 100% EXPLORED! Marked closed.")
            self.bus.publish("mission/event", MissionEventMsg(
                timestamp=current_time,
                event_type="ROOM_COMPLETED",
                details={"room_id": self.active_room_id}
            ))

        # Case B: Current room is CLEARED! Transition to next room via forward Doorway if reachable
        if valid_doorways:
            self.state = "INTER_ROOM_TRANSIT"
            valid_doorways.sort(key=lambda d: math.hypot(d[0] - rx, d[1] - ry))
            dw_target = valid_doorways[0]
            self.committed_doorway = dw_target

            dgx, dgy = self.grid.world_to_grid(dw_target[0], dw_target[1])
            dgx, dgy = find_nearest_free(self.grid, dgx, dgy)
            target_wx, target_wy = self.grid.grid_to_world(dgx, dgy)

            self.committed_target = (dgx, dgy)
            self.commitment_time = current_time
            print(f"  [FRONTIER] [INTER_ROOM_TRANSIT] Heading to Doorway ({target_wx:.2f}, {target_wy:.2f})")
            target_msg = FrontierTargetMsg(
                timestamp=current_time,
                target_gx=dgx,
                target_gy=dgy,
                target_wx=target_wx,
                target_wy=target_wy,
                room_id=-1,
                area=50,
                is_doorway=True
            )
            self.bus.publish("frontier/target", target_msg)
            return target_msg

        # Case C: Global exploration into any remaining unexplored frontiers
        if other_frontiers:
            self.state = "GLOBAL_EXPLORATION"
            target_frontier = self._score_frontiers(other_frontiers, rx, ry, r_yaw)
            gx, gy, area = target_frontier
            gx, gy = find_nearest_free(self.grid, gx, gy)
            self.committed_target = (gx, gy)
            self.commitment_time = current_time
            wx, wy = self.grid.grid_to_world(gx, gy)
            
            f_room = self._get_room_at(gx, gy, room_labels)
            target_msg = FrontierTargetMsg(
                timestamp=current_time,
                target_gx=gx,
                target_gy=gy,
                target_wx=wx,
                target_wy=wy,
                room_id=f_room,
                area=area,
                is_doorway=False
            )
            print(f"  [FRONTIER] [GLOBAL_EXPLORATION] Target: ({wx:.2f}, {wy:.2f}) [Room {f_room}] | Dist: {math.hypot(wx-rx, wy-ry):.2f}m")
            self.bus.publish("frontier/target", target_msg)
            return target_msg

        self.state = "MISSION_COMPLETE"
        return None

    def _score_frontiers(self, frontier_list, rx, ry, r_yaw: float = 0.0):
        best_f = None
        best_score = -float('inf')
        rgx, rgy = self.grid.world_to_grid(rx, ry)
        h, w = self.grid.height, self.grid.width

        for f, dist in frontier_list:
            gx, gy, area = f
            wx, wy = self.grid.grid_to_world(gx, gy)

            # 1. Volumetric unknown information gain in local 9x9 neighborhood
            y0, y1 = max(0, gy - 4), min(h, gy + 5)
            x0, x1 = max(0, gx - 4), min(w, gx + 5)
            patch = self.grid.grid_log_odds[y0:y1, x0:x1]
            unk_count = int(np.count_nonzero((patch > self.grid.thresh_free) & (patch < self.grid.thresh_occ)))
            info_gain = min(float(area), 18.0) + 0.25 * unk_count

            # 2. Geodesic wall-crossing penalty: penalize frontiers behind solid walls
            wall_hits = 0
            for cx, cy in bresenham_line(rgx, rgy, gx, gy):
                if 0 <= cx < w and 0 <= cy < h and self.grid.grid_log_odds[cy, cx] >= self.grid.thresh_occ:
                    wall_hits += 1
                    if wall_hits >= 2:
                        break
            eff_dist = dist * (1.85 if wall_hits >= 2 else 1.0)

            score = info_gain - (eff_dist * 3.5)

            # 3. Obstacle clearance bonus: prefer open viewpoints over tight corners
            if not self.grid.is_occupied_inflated(gx, gy, radius_cells=4):
                score += 5.0
            else:
                score -= 6.0

            # 4. Novelty bonus: strongly reward distance from historical positions
            if self.visited_positions:
                min_vis_dist = min(math.hypot(wx - vx, wy - vy) for vx, vy in self.visited_positions)
                score += min(min_vis_dist * 4.0, 14.0)

            # 5. Heading alignment bonus: strongly favor frontiers ahead of the robot
            target_angle = math.atan2(wy - ry, wx - rx)
            angle_err = abs(norm_angle(target_angle - r_yaw))
            score += math.cos(angle_err) * 11.0

            # 6. Goal commitment & hysteresis bonus
            if self.committed_target is not None:
                d_to_committed = math.hypot(gx - self.committed_target[0], gy - self.committed_target[1])
                if d_to_committed <= 6.0:
                    score += 16.0

            if score > best_score:
                best_score = score
                best_f = f
        return best_f

    def step(self, current_time: float) -> None:
        try:
            target = self.select_frontier(current_time)
            if target is None and (current_time - getattr(self, 'last_log_time', 0.0)) > 3.0:
                self.last_log_time = current_time
                print(f"  [FRONTIER] State: [{self.state}] | Active: Room {self.active_room_id} | Explored Rooms: {list(self.explored_rooms)}")
        except Exception as e:
            self.handle_error(e)
