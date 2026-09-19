"""Mapping Agent: Bayesian log-odds occupancy grid, Bresenham raycasting, and Watershed room segmentation."""
import math
import numpy as np
import cv2
from scipy.ndimage import distance_transform_edt
from typing import List, Tuple, Dict, Optional
from .base import BaseAgent
from .bus import MessageBus, GridMapMsg, ScanMsg, PoseMsg
from grid import BayesianOccupancyGrid, GRID_RES


class MappingAgent(BaseAgent):
    def __init__(self, bus: MessageBus, x_min: float = -5.5, x_max: float = 5.5,
                 y_min: float = -5.5, y_max: float = 5.5, resolution: float = GRID_RES,
                 update_rate_hz: float = 10.0):
        super().__init__("MappingAgent", bus, update_rate_hz)
        self.grid = BayesianOccupancyGrid(x_min, x_max, y_min, y_max, resolution)
        self.latest_scan: Optional[ScanMsg] = None
        self.latest_pose: Optional[PoseMsg] = None
        self.room_labels: Optional[np.ndarray] = None
        self.doorways: List[Tuple[float, float]] = []
        self.segmentation_interval = 20  # Re-segment rooms periodically
        self.step_counter = 0

        # Persistent Room Identity Tracking (Prevents room IDs from flipping across updates)
        self.room_centroids: Dict[int, Tuple[float, float]] = {}
        self.next_room_id: int = 1

    def initialize(self) -> bool:
        self.bus.subscribe("sensor/scan", self.on_scan)
        self.bus.subscribe("pose/state", self.on_pose)
        self.is_running = True
        return True

    def on_scan(self, scan_msg: ScanMsg):
        self.latest_scan = scan_msg

    def on_pose(self, pose_msg: PoseMsg):
        self.latest_pose = pose_msg

    def segment_rooms(self) -> Tuple[np.ndarray, List[Tuple[float, float]]]:
        """Topological Room Segmentation using Euclidean Distance Transform and Voronoi partitioning.
        Identifies rooms as major free-space convex basins and doorways as interface bottlenecks.
        """
        free_mask = (self.grid.grid_log_odds <= self.grid.thresh_free).astype(np.uint8) * 255
        
        num_free_cells = int(np.count_nonzero(free_mask))
        if num_free_cells < 400:
            return np.zeros_like(free_mask, dtype=np.int32), []

        # 1. Distance transform on free space
        dist_transform = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5)
        max_val = np.max(dist_transform)
        if max_val < 4:  # At least 20cm clearance
            return np.zeros_like(free_mask, dtype=np.int32), []

        # 2. Adaptive multi-scale distance transform thresholding to separate distinct room cores
        valid_components = []
        markers = np.zeros_like(free_mask, dtype=np.int32)
        for t_ratio in [0.65, 0.60, 0.56, 0.50, 0.45, 0.40]:
            thresh = t_ratio * max_val
            sure_fg = (dist_transform >= thresh).astype(np.uint8)
            num_markers, cur_markers, stats, cents = cv2.connectedComponentsWithStats(sure_fg)
            comps = []
            for i in range(1, num_markers):
                if stats[i, cv2.CC_STAT_AREA] >= 12:
                    gx, gy = cents[i][0], cents[i][1]
                    wx, wy = self.grid.grid_to_world(int(round(gx)), int(round(gy)))
                    comps.append((i, (wx, wy)))
            if len(comps) >= 2:
                valid_components = comps
                markers = cur_markers
                break
            elif len(comps) == 1 and not valid_components:
                valid_components = comps
                markers = cur_markers

        num_rooms = len(valid_components)
        if num_rooms <= 1:
            # Single room detected so far (Room 1): whole map belongs to Room 1
            voronoi_map = np.ones_like(free_mask, dtype=np.int32)
            self.room_labels = voronoi_map
            self.doorways = []
            if num_rooms == 1:
                self.room_centroids[1] = valid_components[0][1]
            return voronoi_map, []

        # Persistent Room ID Matching: match new cores to previous room centroids
        seeds = np.zeros_like(markers, dtype=np.int32)
        used_persistent_ids = set()
        for comp_idx, (cwx, cwy) in valid_components:
            best_id = None
            best_dist = 2.0  # 2.0-meter matching radius
            for rid, (rcx, rcy) in self.room_centroids.items():
                if rid in used_persistent_ids:
                    continue
                d = math.hypot(cwx - rcx, cwy - rcy)
                if d < best_dist:
                    best_dist = d
                    best_id = rid
            if best_id is not None:
                assigned_id = best_id
                used_persistent_ids.add(assigned_id)
                self.room_centroids[assigned_id] = (cwx, cwy)
            else:
                assigned_id = self.next_room_id
                self.next_room_id += 1
                used_persistent_ids.add(assigned_id)
                self.room_centroids[assigned_id] = (cwx, cwy)
            
            seeds[markers == comp_idx] = assigned_id

        # 4. Multi-room Voronoi partitioning via Euclidean distance transform (whole-grid partitioning)
        _, indices = distance_transform_edt(seeds == 0, return_indices=True)
        voronoi_map = seeds[indices[0], indices[1]]

        # Free space room map for doorway interface detection
        free_room_map = voronoi_map.copy()
        free_room_map[free_mask == 0] = 0

        # 5. Extract doorways: interface where two distinct rooms touch in free space
        door_mask = np.zeros_like(free_mask, dtype=np.uint8)
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            shifted = np.roll(np.roll(free_room_map, dy, axis=0), dx, axis=1)
            interface = (free_room_map > 0) & (shifted > 0) & (free_room_map != shifted)
            door_mask |= interface.astype(np.uint8)

        # Eliminate boundary roll wrap-around artifacts
        door_mask[0:3, :] = 0
        door_mask[-3:, :] = 0
        door_mask[:, 0:3] = 0
        door_mask[:, -3:] = 0

        # Require clearance: doorway bottleneck must be in safely navigable free space (>= 15cm from walls)
        door_mask &= (dist_transform >= 3).astype(np.uint8)

        num_d, d_labels, d_stats, d_cents = cv2.connectedComponentsWithStats(door_mask)
        doorway_points = []
        for i in range(1, num_d):
            if d_stats[i, cv2.CC_STAT_AREA] >= 3:
                dgx, dgy = int(round(d_cents[i][0])), int(round(d_cents[i][1]))
                if self.grid.is_free(dgx, dgy):
                    wx, wy = self.grid.grid_to_world(dgx, dgy)
                    doorway_points.append((wx, wy))

        self.room_labels = voronoi_map
        self.doorways = doorway_points
        return voronoi_map, doorway_points

    def step(self, current_time: float) -> None:
        if self.latest_scan is None or self.latest_pose is None:
            return

        try:
            rx, ry = self.latest_pose.x, self.latest_pose.y
            pts_w = self.latest_scan.world_points_3d

            if pts_w is not None and len(pts_w) > 0:
                # Update Bayesian Occupancy Grid via Bresenham raycasting
                self.grid.update_from_depth_points(pts_w, (rx, ry))

            # Periodic Room Segmentation
            self.step_counter += 1
            if self.step_counter % self.segmentation_interval == 0:
                self.segment_rooms()

            free, occ, unk, explored_pct = self.grid.get_grid_stats()

            # Publish updated Grid Map Message
            self.bus.publish("map/grid", GridMapMsg(
                timestamp=current_time,
                log_odds=self.grid.grid_log_odds,
                free_cells=int(free),
                occupied_cells=int(occ),
                unknown_cells=int(unk),
                explored_pct=float(explored_pct),
                room_labels=self.room_labels,
                doorways=self.doorways
            ))

            if (current_time - getattr(self, 'last_log_time', 0.0)) > 3.0:
                self.last_log_time = current_time
                n_rooms = len(self.room_centroids) if self.room_centroids else 1
                n_dws = len(self.doorways)
                print(f"  [MAPPING] Grid: {free:,d} free ({explored_pct:.1f}% explored) | {n_rooms} Rooms segmented | {n_dws} Doorways")
        except Exception as e:
            self.handle_error(e)
