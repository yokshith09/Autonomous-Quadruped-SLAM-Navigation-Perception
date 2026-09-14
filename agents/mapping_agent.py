"""Mapping Agent: Bayesian log-odds occupancy grid, Bresenham raycasting, and Watershed room segmentation."""
import numpy as np
import cv2
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
        """Topological Room Segmentation using Euclidean Distance Transform and Watershed.
        Identifies rooms as major free-space convex basins and doorways as narrow bottlenecks.
        """
        free_mask = (self.grid.grid_log_odds <= self.grid.thresh_free).astype(np.uint8) * 255
        occupied_mask = (self.grid.grid_log_odds >= self.grid.thresh_occ).astype(np.uint8) * 255
        
        if np.sum(free_mask) < 200:
            return np.zeros_like(free_mask, dtype=np.int32), []

        # 1. Distance transform on free space
        dist_transform = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5)

        # 2. Find local maxima (room centers) with distance threshold
        max_val = np.max(dist_transform)
        if max_val < 4:  # At least 20cm clearance
            return np.zeros_like(free_mask, dtype=np.int32), []

        # Threshold to get reliable room core regions
        _, sure_fg = cv2.threshold(dist_transform, 0.45 * max_val, 255, 0)
        sure_fg = np.uint8(sure_fg)

        # 3. Label room marker components
        num_markers, markers = cv2.connectedComponents(sure_fg)
        if num_markers <= 1:
            # Single room detected so far
            return (free_mask > 0).astype(np.int32), []

        # Add 1 to all labels so that background is not 0
        markers = markers + 1
        # Mark unknown and occupied as background 0
        markers[free_mask == 0] = 0

        # 4. Apply watershed on 3-channel visual representation
        cv_img = cv2.cvtColor(free_mask, cv2.COLOR_GRAY2BGR)
        cv2.watershed(cv_img, markers)

        # Watershed boundary lines are -1
        doorway_points = []
        boundary_mask = (markers == -1) & (free_mask > 0)
        
        # Doorway centroids from boundary intersections
        num_doors, door_labels, stats, centroids = cv2.connectedComponentsWithStats(boundary_mask.astype(np.uint8))
        for i in range(1, num_doors):
            if stats[i, cv2.CC_STAT_AREA] >= 2:
                gx, gy = int(centroids[i][0]), int(centroids[i][1])
                wx, wy = self.grid.grid_to_world(gx, gy)
                doorway_points.append((wx, wy))

        self.room_labels = markers
        self.doorways = doorway_points
        return markers, doorway_points

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
        except Exception as e:
            self.handle_error(e)
