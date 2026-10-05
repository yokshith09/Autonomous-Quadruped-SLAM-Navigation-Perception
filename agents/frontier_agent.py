"""Frontier Exploration Agent: Geodesic-Wavefront Topological Next-Best-View (GW-NBV) with Monotonic Angular Sweep."""
import math
from collections import deque
import cv2
import numpy as np
from typing import List, Tuple, Set, Optional
from .base import BaseAgent
from .bus import MessageBus, FrontierTargetMsg, GridMapMsg, PoseMsg, MissionEventMsg
from grid import find_nearest_free, bresenham_line
from robot import norm_angle


class FrontierAgent(BaseAgent):
    def __init__(self, bus: MessageBus, grid_ref=None, min_dist: float = 0.45, max_dist: float = 6.0, update_rate_hz: float = 1.0):
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
        self.state: str = "INTRA_ROOM_SWEEP"  # INTRA_ROOM_SWEEP | INTER_ROOM_TRANSIT | GLOBAL_EXPLORATION | MISSION_COMPLETE

        # Goal Commitment, Dynamic Cone Tracking & Monotonic Angular Sweep State
        self.committed_target: Optional[Tuple[int, int]] = None
        self.committed_doorway: Optional[Tuple[float, float]] = None
        self.commitment_time: float = 0.0
        self.sweep_direction: int = 1  # +1 CCW, -1 CW monotonic room sweep

    def initialize(self) -> bool:
        self.bus.subscribe("pose/state", self.on_pose)
        self.bus.subscribe("map/grid", self.on_grid)
        self.bus.subscribe("mission/event", self.on_mission_event)
        self.is_running = True
        return True

    def _is_doorway_visited(self, dw: Tuple[float, float]) -> bool:
        return any(math.hypot(dw[0] - vd[0], dw[1] - vd[1]) < 0.65 for vd in self.visited_doorways)

    def _is_blacklisted(self, gx: int, gy: int, radius: float = 3.0) -> bool:
        return any(math.hypot(gx - bgx, gy - bgy) <= radius for bgx, bgy in self.blacklist_frontiers)

    def _compute_geodesic_field(self, start_gx: int, start_gy: int) -> Tuple[np.ndarray, np.ndarray]:
        """Compute 2D obstacle-inflated Geodesic (Wavefront) distance map in meters from (start_gx, start_gy).
        ponytail: 8-connected BFS wavefront on 220x220 grid (<2ms); upgrade to Fast Marching Method if grid >1000x1000.
        """
        h, w = self.grid.height, self.grid.width
        occ = (self.grid.grid_log_odds > self.grid.thresh_occ).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        inflated = cv2.dilate(occ, kernel, iterations=1)

        # Snap start outside inflation margin if robot is close to a wall
        sx, sy = start_gx, start_gy
        if not (0 <= sx < w and 0 <= sy < h) or inflated[sy, sx] > 0:
            found = False
            for r in range(1, 12):
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        nx, ny = start_gx + dx, start_gy + dy
                        if 0 <= nx < w and 0 <= ny < h and inflated[ny, nx] == 0 and self.grid.is_free(nx, ny):
                            sx, sy = nx, ny
                            found = True
                            break
                    if found:
                        break
                if found:
                    break

        dist_map = np.full((h, w), np.inf, dtype=np.float32)
        if not (0 <= sx < w and 0 <= sy < h):
            return dist_map, inflated

        dist_map[sy, sx] = 0.0
        q = deque([(sx, sy)])
        res = self.grid.res
        diag_res = res * 1.41421356
        neighbors = [
            (-1, 0, res), (1, 0, res), (0, -1, res), (0, 1, res),
            (-1, -1, diag_res), (-1, 1, diag_res), (1, -1, diag_res), (1, 1, diag_res)
        ]

        # Only traverse known free space (or immediate robot footprint) outside inflated walls
        free_mask = (self.grid.grid_log_odds <= self.grid.thresh_free) & (inflated == 0)
        free_mask[sy, sx] = True

        while q:
            cx, cy = q.popleft()
            cd = dist_map[cy, cx]
            for dx, dy, step_c in neighbors:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h and free_mask[ny, nx]:
                    nd = cd + step_c
                    if nd < dist_map[ny, nx]:
                        dist_map[ny, nx] = nd
                        q.append((nx, ny))

        return dist_map, inflated

    def _query_geodesic_dist(self, dist_map: np.ndarray, gx: int, gy: int, search_radius: int = 6) -> Tuple[float, Tuple[int, int]]:
        """Return (geodesic_distance_m, best_navigable_grid_cell) in neighborhood of (gx, gy)."""
        h, w = dist_map.shape
        best_d = float("inf")
        best_cell = (gx, gy)
        for r in range(0, search_radius + 1):
            y0, y1 = max(0, gy - r), min(h, gy + r + 1)
            x0, x1 = max(0, gx - r), min(w, gx + r + 1)
            sub = dist_map[y0:y1, x0:x1]
            min_val = float(np.min(sub))
            if np.isfinite(min_val):
                idx = np.unravel_index(int(np.argmin(sub)), sub.shape)
                best_d = min_val
                best_cell = (x0 + int(idx[1]), y0 + int(idx[0]))
                break
        return best_d, best_cell

    def on_mission_event(self, event_msg: MissionEventMsg):
        if event_msg.event_type in ["PATH_UNREACHABLE", "ROBOT_STUCK"]:
            tgx = event_msg.details.get("target_gx")
            tgy = event_msg.details.get("target_gy")
            if tgx is not None and tgy is not None:
                self.blacklist_frontiers.add((tgx, tgy))
            # Temporary back-off: clear committed doorway without marking it permanently visited
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
        if not self.visited_positions or math.hypot(pose_msg.x - self.visited_positions[-1][0], pose_msg.y - self.visited_positions[-1][1]) > 0.35:
            self.visited_positions.append((pose_msg.x, pose_msg.y))

    def on_grid(self, grid_msg: GridMapMsg):
        self.latest_grid_msg = grid_msg

    def select_frontier(self, current_time: float) -> Optional[FrontierTargetMsg]:
        if self.grid is None or self.latest_pose is None:
            return None

        rx, ry = self.latest_pose.x, self.latest_pose.y
        r_yaw = self.latest_pose.yaw
        rgx, rgy = self.grid.world_to_grid(rx, ry)

        # Compute fast Geodesic Wavefront distance map from robot's current cell
        dist_map, inflated_mask = self._compute_geodesic_field(rgx, rgy)

        # Check if committed target was physically reached
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

        # Detect current room
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

        all_frontiers = self.grid.detect_frontiers()
        if not all_frontiers:
            self.state = "MISSION_COMPLETE"
            print("  [FRONTIER] All frontiers cleared across entire map! MISSION_COMPLETE.")
            return None

        # Maintain or Dynamically Slide Committed Target in Forward Viewing Cone
        if self.committed_target is not None:
            cwx, cwy = self.grid.grid_to_world(self.committed_target[0], self.committed_target[1])
            dist_to_comm = math.hypot(cwx - rx, cwy - ry)
            geo_d, nav_cell = self._query_geodesic_dist(dist_map, self.committed_target[0], self.committed_target[1], search_radius=4)

            if (current_time - self.commitment_time) <= 9.0 and dist_to_comm >= 0.38 and np.isfinite(geo_d):
                if self.state == "INTER_ROOM_TRANSIT":
                    target_msg = FrontierTargetMsg(
                        timestamp=current_time,
                        target_gx=nav_cell[0],
                        target_gy=nav_cell[1],
                        target_wx=cwx,
                        target_wy=cwy,
                        room_id=-1,
                        area=50,
                        is_doorway=True
                    )
                    self.bus.publish("frontier/target", target_msg)
                    return target_msg

                # Check if frontier still exists near committed target OR receded forward inside camera cone
                still_has_frontier = any(
                    math.hypot(f[0] - self.committed_target[0], f[1] - self.committed_target[1]) <= 9.0
                    for f in all_frontiers
                )
                if still_has_frontier:
                    target_msg = FrontierTargetMsg(
                        timestamp=current_time,
                        target_gx=nav_cell[0],
                        target_gy=nav_cell[1],
                        target_wx=cwx,
                        target_wy=cwy,
                        room_id=self.active_room_id,
                        area=30,
                        is_doorway=False
                    )
                    self.bus.publish("frontier/target", target_msg)
                    return target_msg
                else:
                    # Dynamic View-Cone Slide: as camera rays clear the immediate frontier, slide target forward
                    # along the receding boundary in the same viewing cone instead of oscillating to another wall!
                    best_slide = None
                    best_slide_d = 18.0  # Max 18 cells (0.90m) slide radius
                    for f in all_frontiers:
                        fgx, fgy, farea = f
                        if self._is_blacklisted(fgx, fgy):
                            continue
                        d_shift = math.hypot(fgx - self.committed_target[0], fgy - self.committed_target[1])
                        if d_shift <= best_slide_d:
                            fwx, fwy = self.grid.grid_to_world(fgx, fgy)
                            ang_err = abs(norm_angle(math.atan2(fwy - ry, fwx - rx) - r_yaw))
                            f_geo, f_nav = self._query_geodesic_dist(dist_map, fgx, fgy, search_radius=5)
                            if ang_err <= 0.65 and np.isfinite(f_geo) and f_geo >= 0.35:
                                best_slide_d = d_shift
                                best_slide = (f_nav[0], f_nav[1], fwx, fwy, farea)
                    if best_slide is not None:
                        self.committed_target = (best_slide[0], best_slide[1])
                        target_msg = FrontierTargetMsg(
                            timestamp=current_time,
                            target_gx=best_slide[0],
                            target_gy=best_slide[1],
                            target_wx=best_slide[2],
                            target_wy=best_slide[3],
                            room_id=self.active_room_id,
                            area=best_slide[4],
                            is_doorway=False
                        )
                        self.bus.publish("frontier/target", target_msg)
                        return target_msg

            # Target expired, cleared, or became unreachable
            if self.committed_doorway is not None:
                self.visited_doorways.add(self.committed_doorway)
                self.committed_doorway = None
            self.blacklist_frontiers.add(self.committed_target)
            self.committed_target = None

        # 1. Collect all REACHABLE non-blacklisted candidates using Geodesic Wavefront
        candidates = []
        for f in all_frontiers:
            gx, gy, area = f
            if self._is_blacklisted(gx, gy):
                continue
            geo_dist, nav_cell = self._query_geodesic_dist(dist_map, gx, gy, search_radius=6)
            if not np.isfinite(geo_dist):
                continue  # Provably unreachable by A* right now -> skip!
            candidates.append(((nav_cell[0], nav_cell[1], area), geo_dist))

        if not candidates and self.blacklist_frontiers:
            self.blacklist_frontiers.clear()
            for f in all_frontiers:
                gx, gy, area = f
                geo_dist, nav_cell = self._query_geodesic_dist(dist_map, gx, gy, search_radius=7)
                if np.isfinite(geo_dist):
                    candidates.append(((nav_cell[0], nav_cell[1], area), geo_dist))

        if not candidates:
            # Fallback if robot is temporarily boxed in: pick closest raw frontier
            for f in all_frontiers:
                wx, wy = self.grid.grid_to_world(f[0], f[1])
                candidates.append((f, math.hypot(wx - rx, wy - ry)))

        if not candidates:
            self.state = "MISSION_COMPLETE"
            print("  [FRONTIER] All rooms and frontiers explored. MISSION_COMPLETE.")
            return None

        # Filter by min_dist (prefer frontiers >= min_dist, fall back if only close ones remain)
        dist_candidates = [c for c in candidates if c[1] >= self.min_dist]
        if not dist_candidates:
            dist_candidates = candidates

        # Classify into active room vs other rooms
        room_frontiers = []
        other_frontiers = []
        for f, geo_dist in dist_candidates:
            gx, gy, area = f
            f_room = self._get_room_at(gx, gy, room_labels)
            if self.active_room_id > 0 and self.active_room_id not in self.explored_rooms and f_room == self.active_room_id:
                room_frontiers.append((f, geo_dist))
            else:
                other_frontiers.append((f, geo_dist))

        # Collect valid forward doorways (unvisited and navigable on Geodesic map)
        doorways = self.latest_grid_msg.doorways if self.latest_grid_msg else []
        valid_doorways = []
        for d in doorways:
            if self._is_doorway_visited(d):
                continue
            dgx, dgy = self.grid.world_to_grid(d[0], d[1])
            if self._is_blacklisted(dgx, dgy, radius=3.5):
                continue
            dw_geo, dw_nav = self._query_geodesic_dist(dist_map, dgx, dgy, search_radius=4)
            if not np.isfinite(dw_geo):
                continue
            valid_doorways.append((d, dw_geo, dw_nav))

        # Decision Logic:
        # Case A: Current room still has unexplored reachable frontiers -> Clear it systematically!
        if room_frontiers and self.active_room_id not in self.explored_rooms:
            self.state = "INTRA_ROOM_SWEEP"
            target_frontier = self._score_frontiers(room_frontiers, rx, ry, r_yaw)
            gx, gy, area = target_frontier
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
            print(f"  [FRONTIER] [INTRA_ROOM_SWEEP] Target: ({wx:.2f}, {wy:.2f}) [Room {self.active_room_id}] | GeoDist: {dist_map[gy, gx]:.2f}m")
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

        # Case B: Current room is CLEARED! Transition to next room via Doorway Penetration Vector
        if valid_doorways:
            self.state = "INTER_ROOM_TRANSIT"
            valid_doorways.sort(key=lambda item: item[1])
            dw_target, dw_geo, dw_nav = valid_doorways[0]
            self.committed_doorway = dw_target

            # Project Doorway Penetration Vector 0.55m past the doorway into the next room
            vec_x, vec_y = dw_target[0] - rx, dw_target[1] - ry
            vec_len = math.hypot(vec_x, vec_y)
            if vec_len > 0.1:
                pen_wx = dw_target[0] + (vec_x / vec_len) * 0.55
                pen_wy = dw_target[1] + (vec_y / vec_len) * 0.55
                pgx, pgy = self.grid.world_to_grid(pen_wx, pen_wy)
                p_geo, p_nav = self._query_geodesic_dist(dist_map, pgx, pgy, search_radius=5)
                if np.isfinite(p_geo):
                    dw_nav = p_nav

            dgx, dgy = dw_nav
            target_wx, target_wy = self.grid.grid_to_world(dgx, dgy)

            self.committed_target = (dgx, dgy)
            self.commitment_time = current_time
            print(f"  [FRONTIER] [INTER_ROOM_TRANSIT] Penetrating Doorway -> ({target_wx:.2f}, {target_wy:.2f})")
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

        # Case C: Global exploration into any remaining unexplored frontiers across all rooms
        if other_frontiers:
            self.state = "GLOBAL_EXPLORATION"
            target_frontier = self._score_frontiers(other_frontiers, rx, ry, r_yaw)
            gx, gy, area = target_frontier
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
            print(f"  [FRONTIER] [GLOBAL_EXPLORATION] Target: ({wx:.2f}, {wy:.2f}) [Room {f_room}] | GeoDist: {dist_map[gy, gx]:.2f}m")
            self.bus.publish("frontier/target", target_msg)
            return target_msg

        self.state = "MISSION_COMPLETE"
        return None

    def _score_frontiers(self, frontier_list, rx, ry, r_yaw: float = 0.0):
        """Score reachable frontiers using Geodesic Cost-to-Go, Volumetric Gain, and Hemisphere-First Monotonic Sweep."""
        best_f = None
        best_score = -float('inf')
        h, w = self.grid.height, self.grid.width

        # Check if any candidate lies in the forward hemisphere (|angle_err| <= 95 deg)
        has_forward_candidate = False
        for f, _ in frontier_list:
            fwx, fwy = self.grid.grid_to_world(f[0], f[1])
            ang_err = abs(norm_angle(math.atan2(fwy - ry, fwx - rx) - r_yaw))
            if ang_err <= 1.65:  # ~95 degrees
                has_forward_candidate = True
                break

        best_signed_err = 0.0
        for f, geo_dist in frontier_list:
            gx, gy, area = f
            wx, wy = self.grid.grid_to_world(gx, gy)

            # 1. Volumetric unknown information gain in local 9x9 neighborhood
            y0, y1 = max(0, gy - 4), min(h, gy + 5)
            x0, x1 = max(0, gx - 4), min(w, gx + 5)
            patch = self.grid.grid_log_odds[y0:y1, x0:x1]
            unk_count = int(np.count_nonzero((patch > self.grid.thresh_free) & (patch < self.grid.thresh_occ)))
            info_gain = min(float(area), 20.0) + 0.30 * unk_count

            # 2. True Geodesic Walking Distance Penalty (replaces Euclidean wall-blind distance)
            score = info_gain - (geo_dist * 3.8)

            # 3. Obstacle clearance bonus
            if not self.grid.is_occupied_inflated(gx, gy, radius_cells=4):
                score += 6.0
            else:
                score -= 6.0

            # 4. Novelty bonus: reward distance from historical positions
            if self.visited_positions:
                min_vis_dist = min(math.hypot(wx - vx, wy - vy) for vx, vy in self.visited_positions)
                score += min(min_vis_dist * 4.0, 14.0)

            # 5. Hemisphere-First & Monotonic Angular Sweep Continuity
            target_angle = math.atan2(wy - ry, wx - rx)
            signed_err = norm_angle(target_angle - r_yaw)
            angle_err = abs(signed_err)

            # Forward view-cone bonus
            score += math.cos(angle_err) * 14.0

            # If forward frontiers exist, strongly suppress 180-degree turn reversals (eliminates front/back confusion)
            if has_forward_candidate and angle_err > 1.90:
                score -= 18.0

            # Monotonic angular sweep bonus (consistent rotational clearing direction)
            if (signed_err >= 0 and self.sweep_direction >= 0) or (signed_err < 0 and self.sweep_direction < 0):
                score += 4.5

            # 6. Goal commitment & hysteresis bonus
            if self.committed_target is not None:
                d_to_committed = math.hypot(gx - self.committed_target[0], gy - self.committed_target[1])
                if d_to_committed <= 7.0:
                    score += 18.0

            if score > best_score:
                best_score = score
                best_f = f
                best_signed_err = signed_err

        if abs(best_signed_err) > 0.15:
            self.sweep_direction = 1 if best_signed_err > 0 else -1

        return best_f

    def step(self, current_time: float) -> None:
        try:
            target = self.select_frontier(current_time)
            if target is None and (current_time - getattr(self, 'last_log_time', 0.0)) > 3.0:
                self.last_log_time = current_time
                print(f"  [FRONTIER] State: [{self.state}] | Active: Room {self.active_room_id} | Explored Rooms: {list(self.explored_rooms)}")
        except Exception as e:
            self.handle_error(e)

