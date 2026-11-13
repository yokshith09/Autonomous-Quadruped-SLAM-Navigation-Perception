"""Frontier Exploration Agent: Geodesic-Wavefront Topological Next-Best-View (GW-NBV) with Monotonic Angular Sweep."""
import math
from collections import deque
import cv2
import numpy as np
from typing import List, Tuple, Set, Optional
from .base import BaseAgent
from .bus import MessageBus, FrontierTargetMsg, GridMapMsg, PoseMsg, MissionEventMsg
from .spatial_memory import SpatialMemory
from core.robot import norm_angle
from core.grid import bresenham_line
from core import planner
from core.planner import CLEARANCE_KERNEL, clear_margin_near, kernel_for

try:
    from numba import njit
except ImportError:  # optional: without numba the Python wavefront gives the same field, much slower
    njit = None

if njit is not None:
    @njit(cache=True)
    def _wavefront_kernel(free_mask, sx, sy, step, diag, dist):
        """Compiled twin of the Python wavefront in _compute_geodesic_field: same FIFO order, neighbour order and
        float32 arithmetic, so the distance field is bit-identical."""
        h, w = free_mask.shape
        cap = 8 * h * w
        qx = np.empty(cap, np.int32)
        qy = np.empty(cap, np.int32)
        head, tail = 0, 0
        qx[tail], qy[tail] = sx, sy
        tail += 1
        ddx = (-1, 1, 0, 0, -1, -1, 1, 1)
        ddy = (0, 0, -1, 1, -1, 1, -1, 1)
        while head != tail:
            cx, cy = qx[head % cap], qy[head % cap]
            head += 1
            cd = dist[cy, cx]
            for k in range(8):
                nx, ny = cx + ddx[k], cy + ddy[k]
                if 0 <= nx < w and 0 <= ny < h and free_mask[ny, nx]:
                    nd = cd + (step if k < 4 else diag)
                    if nd < dist[ny, nx]:
                        dist[ny, nx] = nd
                        if tail - head >= cap:
                            raise RuntimeError("wavefront queue overflow")
                        qx[tail % cap], qy[tail % cap] = nx, ny
                        tail += 1
        return dist
else:
    _wavefront_kernel = None

MIN_GLOBAL_AREA = 10       # cells: a smaller frontier is only worth a trip if it is near
NEAR_FRONTIER_M = 3.0      # ... this close (m); far tiny ones are left to the floor-coverage patrol
MIN_VISIBLE_UNKNOWN = 3   # a frontier must show at least this many unknown cells to its viewpoint, or it is unobservable
RETIRE_RADIUS = 6.0       # cells (0.3 m): a retired frontier also covers its cluster's medoid drifting as it shrinks
COMMIT_PROGRESS_M = 0.25  # a committed target is kept while the walking distance to it shrinks by this much ...
DOOR_MAX_FAILS = 2           # doorway transits ending stuck or unreachable before that doorway is skipped
DOOR_TURN_COST_M = 1.5      # a doorway straight behind the robot counts this much further than one ahead
PATROL_STEP = 8             # cells (0.40 m) between candidate patrol points
COVER_RADIUS_M = 0.35       # a patrol point the robot has never been within this distance of counts as unwalked
COVER_BONUS_S = 900.0       # unwalked floor (under a table, a narrow gap) outranks merely stale floor
PATROL_SECONDS_PER_M = 20.0  # staleness worth one metre of walking (the robot walks about 12 s per metre)
EXPLORED_OVERLAP = 0.7    # a new room label whose free floor is this much inside finished rooms is already covered
COMMIT_STALL_S = 20.0     # ... within this many seconds; otherwise the attempt counts as a failure


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
        # Retired frontiers, in grid cells; never cleared. Spatially hashed: a lookup reads only nearby buckets, so it
        # stays constant-time however many frontiers a long run retires (a full scan per candidate did not).
        self.retired = SpatialMemory(bucket=RETIRE_RADIUS)
        self.explored_floor = None  # free cells of finished rooms (bool grid), kept by place across re-labelling
        # Post-exploration patrol (the mission runs to its time limit): where and when the robot has been, in metres
        self.patrol = True
        self.visits = SpatialMemory(bucket=1.0)
        self.patrol_failed = SpatialMemory(bucket=1.0)
        self.failed_doorways = SpatialMemory(bucket=1.0)  # doorway transits that ended stuck or unreachable
        self.patrol_target: Optional[Tuple[int, int]] = None
        self._patrol_best = (float("inf"), 0.0)
        self._complete_log_t = -1e9
        self.strikes: dict = {}
        self.committed_frontier: Optional[Tuple[int, int]] = None  # raw frontier cell behind committed_target (a nav cell)
        self._raw_of: dict = {}
        self.visited_positions: List[Tuple[float, float]] = []
        self.state: str = "INTRA_ROOM_SWEEP"  # INTRA_ROOM_SWEEP | INTER_ROOM_TRANSIT | GLOBAL_EXPLORATION | MISSION_COMPLETE

        # Goal Commitment, Dynamic Cone Tracking & Monotonic Angular Sweep State
        self.committed_target: Optional[Tuple[int, int]] = None
        self.committed_doorway: Optional[Tuple[float, float]] = None
        self._commit_best_d: float = float("inf")  # shortest walking distance to the committed target so far
        self._commit_best_t: float = 0.0           # when it last shrank by COMMIT_PROGRESS_M
        self.sweep_direction: int = 1  # +1 CCW, -1 CW monotonic room sweep

    def initialize(self) -> bool:
        self.bus.subscribe("pose/state", self.on_pose)
        self.bus.subscribe("map/grid", self.on_grid)
        self.bus.subscribe("mission/event", self.on_mission_event)
        self.is_running = True
        return True

    def _is_doorway_visited(self, dw: Tuple[float, float]) -> bool:
        return any(math.hypot(dw[0] - vd[0], dw[1] - vd[1]) < 0.65 for vd in self.visited_doorways)

    def _is_blacklisted(self, gx: int, gy: int, radius: float = RETIRE_RADIUS) -> bool:
        return self.retired.any_near("frontier", gx, gy, radius)

    def _strike(self, cell, limit: int) -> None:
        """Count a failed attempt at a frontier; retire it permanently after `limit` failures."""
        if cell is None:
            return
        k = (int(cell[0]) // 6, int(cell[1]) // 6)
        self.strikes[k] = self.strikes.get(k, 0) + 1
        if self.strikes[k] >= limit:
            self.retired.add("frontier", int(cell[0]), int(cell[1]))

    def _retire_committed(self, limit: int) -> None:
        self._strike(self.committed_target, limit)
        self._strike(self.committed_frontier, limit)

    def _remember_explored(self, room_id: int, room_labels) -> None:
        """Keep the floor of a finished room by place, not by label: labels change when the map is re-segmented."""
        if room_labels is None:
            return
        floor = (room_labels == room_id) & (self.grid.grid_log_odds <= self.grid.thresh_free)
        self.explored_floor = floor if self.explored_floor is None else (self.explored_floor | floor)

    def _already_covered(self, room_id: int, room_labels) -> bool:
        if room_labels is None or self.explored_floor is None:
            return False
        floor = (room_labels == room_id) & (self.grid.grid_log_odds <= self.grid.thresh_free)
        n = int(np.count_nonzero(floor))
        return n > 0 and np.count_nonzero(floor & self.explored_floor) >= EXPLORED_OVERLAP * n

    def _start_commitment(self, current_time: float) -> None:
        self._commit_best_d, self._commit_best_t = float("inf"), current_time
        self.patrol_target = None  # a reachable frontier came back: exploration takes over from patrolling

    def _visible_unknown(self, vx: int, vy: int, gx: int, gy: int, r: int = 3) -> int:
        """Unknown cells around frontier (gx, gy) that a straight ray from viewpoint (vx, vy) reaches without crossing an
        occupied cell. Zero for frontiers behind a wall (depth leaks) or inside objects: going there cannot reveal anything."""
        lo, g = self.grid.grid_log_odds, self.grid
        n = 0
        for y in range(max(0, gy - r), min(g.height, gy + r + 1)):
            for x in range(max(0, gx - r), min(g.width, gx + r + 1)):
                if g.thresh_free < lo[y, x] < g.thresh_occ and \
                        all(lo[cy, cx] < g.thresh_occ for cx, cy in list(bresenham_line(vx, vy, x, y))[:-1]):
                    n += 1
        return n

    def _compute_geodesic_field(self, start_gx: int, start_gy: int) -> Tuple[np.ndarray, np.ndarray]:
        """Compute 2D obstacle-inflated Geodesic (Wavefront) distance map in meters from (start_gx, start_gy).
        ponytail: 8-connected BFS wavefront on 220x220 grid (<2ms); upgrade to Fast Marching Method if grid >1000x1000.
        """
        h, w = self.grid.height, self.grid.width
        occ = (self.grid.grid_log_odds > self.grid.thresh_occ).astype(np.uint8)
        cells = planner.NARROW_CLEARANCE_CELLS if planner.NARROW_GAPS else planner.CLEARANCE_CELLS
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_for(cells))
        inflated = cv2.dilate(occ, kernel, iterations=1)
        clear_margin_near(inflated, occ, start_gx, start_gy)  # same rule as the planner: margins do not trap the robot

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

        if _wavefront_kernel is not None:
            # 87 % of a decision in pure Python (306 of 351 ms on the run-4 map)
            return _wavefront_kernel(free_mask, sx, sy, np.float32(res), np.float32(diag_res), dist_map), inflated

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
        if event_msg.event_type in ["PATH_UNREACHABLE", "ROBOT_STUCK"] and self.patrol_target is not None:
            # A patrol point is not a frontier: mark it failed, never strike a frontier for it
            wx, wy = self.grid.grid_to_world(*self.patrol_target)
            self.patrol_failed.add("fail", wx, wy, t=event_msg.timestamp)
            self.patrol_target = None
            return
        if event_msg.event_type in ["PATH_UNREACHABLE", "ROBOT_STUCK"]:
            tgx = event_msg.details.get("target_gx")
            tgy = event_msg.details.get("target_gy")
            if tgx is not None and tgy is not None:
                self._strike((tgx, tgy), limit=2)
            # Back off from the doorway without marking it traversed, but count the failure by place: run 11 retried one
            # doorway along the same blocked path more than 20 times because these failures were never counted.
            if self.committed_doorway is not None:
                self.failed_doorways.add("fail", self.committed_doorway[0], self.committed_doorway[1],
                                         t=event_msg.timestamp)
            self.committed_doorway = None
            if self.committed_target is not None:
                self._retire_committed(limit=2)
                self.committed_target = None
                print("  [FRONTIER] Cleared stalled target from watchdog alert. Re-evaluating alternatives.")
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
            self.visits.add("visit", pose_msg.x, pose_msg.y, t=pose_msg.timestamp)

    def _finish(self, current_time, rx, ry, dist_map, inflated, reason: str) -> Optional[FrontierTargetMsg]:
        """Nothing reachable is left to explore. The mission keeps running (owner's rule: no early finish), so instead
        of standing idle the robot patrols: it walks to the reachable place it has not seen for longest, which
        re-checks the map for new frontiers (exploration resumes on its own when one appears) and retraces the route
        for loop closures (Zhang et al., arXiv 2503.17005). Returns None only when patrolling is off."""
        if self.state != "MISSION_COMPLETE" or current_time - self._complete_log_t > 30.0:
            print(f"  [FRONTIER] {reason} MISSION_COMPLETE{' -> patrolling stale areas' if self.patrol else ''}.")
            self._complete_log_t = current_time
        self.state = "MISSION_COMPLETE"
        self.committed_target = None
        if not self.patrol:
            return None
        if self.patrol_target is not None:
            gx, gy = self.patrol_target
            wx, wy = self.grid.grid_to_world(gx, gy)
            geo_d = float(dist_map[gy, gx])
            if math.hypot(wx - rx, wy - ry) < 0.40:
                self.patrol_target = None                     # reached
            elif not np.isfinite(geo_d) or current_time - self._patrol_best[1] > COMMIT_STALL_S:
                self.patrol_failed.add("fail", wx, wy, t=current_time)
                self.patrol_target = None                     # unreachable now, or no progress
            elif geo_d < self._patrol_best[0] - COMMIT_PROGRESS_M:
                self._patrol_best = (geo_d, current_time)
        if self.patrol_target is None:
            self.patrol_target = self._pick_patrol_target(current_time, rx, ry, dist_map, inflated)
            if self.patrol_target is None:
                return None
            self._patrol_best = (float("inf"), current_time)
            wx, wy = self.grid.grid_to_world(*self.patrol_target)
            print(f"  [FRONTIER] [PATROL] Target: ({wx:.2f}, {wy:.2f}) | GeoDist: {dist_map[self.patrol_target[1], self.patrol_target[0]]:.2f}m")
        gx, gy = self.patrol_target
        wx, wy = self.grid.grid_to_world(gx, gy)
        msg = FrontierTargetMsg(timestamp=current_time, target_gx=gx, target_gy=gy, target_wx=wx, target_wy=wy,
                                room_id=-1, area=0, is_doorway=False)
        self.bus.publish("frontier/target", msg)
        return msg

    def _pick_patrol_target(self, current_time, rx, ry, dist_map, inflated) -> Optional[Tuple[int, int]]:
        """Reachable lattice point maximising (seconds since last walked nearby) - PATROL_SECONDS_PER_M * walk, with a
        bonus for floor the robot has never stood on."""
        reach = np.isfinite(dist_map) & (inflated == 0)
        ys, xs = np.nonzero(reach[::PATROL_STEP, ::PATROL_STEP])
        best, best_score = None, -float("inf")
        for gy, gx in zip(ys * PATROL_STEP, xs * PATROL_STEP):
            wx, wy = self.grid.grid_to_world(int(gx), int(gy))
            if math.hypot(wx - rx, wy - ry) < 1.0 or self.patrol_failed.any_near("fail", wx, wy, 0.6):
                continue
            last = max((it["t"] for it in self.visits.near("visit", wx, wy, 1.0)), default=-1e9)
            score = min(current_time - last, current_time) - PATROL_SECONDS_PER_M * float(dist_map[gy, gx])
            # Cover the floor, not only the view: the camera saw under the table from 0.73 m (run 33), so it counted
            # as explored and was never walked. Floor the robot has never stood near gets a large bonus.
            if not self.visits.any_near("visit", wx, wy, COVER_RADIUS_M):
                score += COVER_BONUS_S
            if score > best_score:
                best, best_score = (int(gx), int(gy)), score
        return best

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
                    self._retire_committed(limit=1)  # stood at its viewpoint; whatever is still unknown there is unobservable
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
            # Switch only once the active room is finished. Room labels near a boundary change under a moving robot,
            # and switching on every change made it alternate between two rooms without finishing either (F3).
            # A room whose label vanishes has no frontiers left under its ID, so it is closed below and this
            # cannot deadlock.
            elif (current_detected_room != self.active_room_id and current_detected_room not in self.explored_rooms
                  and self.active_room_id in self.explored_rooms
                  and self._already_covered(current_detected_room, room_labels)):
                # Re-segmentation gave an area the robot has already swept a new label: close it, do not sweep again
                self.explored_rooms.add(current_detected_room)
                print(f"  [FRONTIER] Room {current_detected_room} lies in already explored floor -> closed without a sweep")
            elif (current_detected_room != self.active_room_id and current_detected_room not in self.explored_rooms
                  and self.active_room_id in self.explored_rooms):
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
            return self._finish(current_time, rx, ry, dist_map, inflated_mask, "All frontiers cleared across entire map!")

        # Maintain or Dynamically Slide Committed Target in Forward Viewing Cone
        if self.committed_target is not None:
            cwx, cwy = self.grid.grid_to_world(self.committed_target[0], self.committed_target[1])
            dist_to_comm = math.hypot(cwx - rx, cwy - ry)
            geo_d, nav_cell = self._query_geodesic_dist(dist_map, self.committed_target[0], self.committed_target[1], search_radius=4)

            # Keep the target until it is reached, its frontier is gone, or it stops getting closer. (A fixed 9 s
            # window here expired after about 0.8 m of walking, and re-scoring by the current heading then
            # swapped targets mid-route.)
            if np.isfinite(geo_d) and geo_d < self._commit_best_d - COMMIT_PROGRESS_M:
                self._commit_best_d, self._commit_best_t = geo_d, current_time
            stalled = current_time - self._commit_best_t > COMMIT_STALL_S
            if not stalled and dist_to_comm >= 0.38 and np.isfinite(geo_d):
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
                                best_slide = (f_nav[0], f_nav[1], fwx, fwy, farea, (fgx, fgy))
                    if best_slide is not None:
                        self.committed_target = (best_slide[0], best_slide[1])
                        self.committed_frontier = best_slide[5]
                        self._start_commitment(current_time)
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

            # Target cleared (its frontier was seen from afar: done), unreachable or stalled (failed attempts).
            if self.committed_doorway is not None:
                self.visited_doorways.add(self.committed_doorway)
                self.committed_doorway = None
            if not np.isfinite(geo_d) or stalled:
                self._retire_committed(limit=2)
            self.committed_target = None

        # 1. Collect all REACHABLE, OBSERVABLE, non-retired candidates using Geodesic Wavefront
        candidates = []
        self._raw_of = {}
        for f in all_frontiers:
            gx, gy, area = f
            if self._is_blacklisted(gx, gy):
                continue
            geo_dist, nav_cell = self._query_geodesic_dist(dist_map, gx, gy, search_radius=6)
            if not np.isfinite(geo_dist):
                continue  # Provably unreachable by A* right now -> skip!
            if self._visible_unknown(nav_cell[0], nav_cell[1], gx, gy) < MIN_VISIBLE_UNKNOWN:
                continue  # behind a wall or inside an object: nothing to see from where the robot can stand
            self._raw_of[nav_cell] = (gx, gy)
            candidates.append(((nav_cell[0], nav_cell[1], area), geo_dist))

        if not candidates and int(np.count_nonzero(np.isfinite(dist_map))) > 400:
            # Robot has room to move (>1 m^2 reachable) yet no frontier is reachable: exploration is done.
            return self._finish(current_time, rx, ry, dist_map, inflated_mask,
                                f"{len(all_frontiers)} frontier(s) left, none reachable.")

        if not candidates:
            # Fallback if robot is temporarily boxed in: pick closest raw frontier
            for f in all_frontiers:
                wx, wy = self.grid.grid_to_world(f[0], f[1])
                candidates.append((f, math.hypot(wx - rx, wy - ry)))

        if not candidates:
            return self._finish(current_time, rx, ry, dist_map, inflated_mask, "All rooms and frontiers explored.")

        # Filter by min_dist (prefer frontiers >= min_dist, fall back if only close ones remain)
        dist_candidates = [c for c in candidates if c[1] >= self.min_dist]
        if not dist_candidates:
            dist_candidates = candidates
        # Run 34 spent its last ~13 min (1000-1785 s, coverage 80.0 -> 81.0 %) crossing the map for 3-8 cell frontier
        # slivers along walls; 51 of them were left at the end. Far tiny ones are left to the floor-coverage patrol;
        # small gaps near the robot are still explored.
        dist_candidates = [(f, d) for f, d in dist_candidates if f[2] >= MIN_GLOBAL_AREA or d <= NEAR_FRONTIER_M]

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
        door_rooms = self.latest_grid_msg.doorway_rooms if self.latest_grid_msg else []
        valid_doorways = []
        for i, d in enumerate(doorways):
            if self._is_doorway_visited(d):
                continue
            if sum(1 for _ in self.failed_doorways.near("fail", d[0], d[1], 0.6)) >= DOOR_MAX_FAILS:
                continue  # tried and failed repeatedly: other doorways or frontiers lead on
            if i < len(door_rooms) and all(r in self.explored_rooms for r in door_rooms[i]):
                continue  # joins two finished rooms: nothing new behind it (run 5 toured such doorways for 600 s)
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
            self.committed_frontier = self._raw_of.get((gx, gy), (gx, gy))
            self._start_commitment(current_time)
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
            self._remember_explored(self.active_room_id, room_labels)
            print(f"  [FRONTIER] * Room {self.active_room_id} 100% EXPLORED! Marked closed.")
            self.bus.publish("mission/event", MissionEventMsg(
                timestamp=current_time,
                event_type="ROOM_COMPLETED",
                details={"room_id": self.active_room_id}
            ))

        # Case B: Current room is CLEARED! Transition to next room via Doorway Penetration Vector
        if valid_doorways:
            self.state = "INTER_ROOM_TRANSIT"
            # Walking distance plus a turning cost: a doorway straight behind counts DOOR_TURN_COST_M further. By
            # distance alone the robot turned round for a doorway behind it that was only slightly nearer (run 7:
            # 2 of 13 doorway choices were more than 135 deg from the direction of travel).
            def door_cost(item):
                (dwx, dwy), geo, _ = item
                turn = abs(norm_angle(math.atan2(dwy - ry, dwx - rx) - r_yaw))
                return geo + DOOR_TURN_COST_M * turn / math.pi
            valid_doorways.sort(key=door_cost)
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
            self.committed_frontier = None
            self._start_commitment(current_time)
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
            self.committed_frontier = self._raw_of.get((gx, gy), (gx, gy))
            self._start_commitment(current_time)
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

        return self._finish(current_time, rx, ry, dist_map, inflated_mask, "No candidate left.")

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

