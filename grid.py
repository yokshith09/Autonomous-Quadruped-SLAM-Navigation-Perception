import math
import numpy as np
import cv2
from collections import deque

GRID_RES = 0.05
ROBOT_RADIUS = 0.25

def bresenham_line(x1, y1, x2, y2):
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy
    x, y = x1, y1
    while True:
        yield (x, y)
        if x == x2 and y == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy

class BayesianOccupancyGrid:
    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 1

    def __init__(self, x_min, x_max, y_min, y_max, resolution):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.res = resolution
        self.width = int((x_max - x_min) / resolution) + 1
        self.height = int((y_max - y_min) / resolution) + 1
        
        # Log-odds probabilities: 0.0 is 50% (Unknown)
        self.grid_log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        
        # Thresholds
        self.l_occ = 0.85     # Two hits mark occupied
        self.l_free = -0.4    # decrease when free
        self.l_max = 2.0      # Max certainty occupied
        self.l_min = -2.0     # Max certainty free
        self.thresh_occ = 1.0 # threshold to consider cell as OCCUPIED
        self.thresh_free = -0.5 # threshold to consider cell as FREE

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.x_min) / self.res)
        gy = int((wy - self.y_min) / self.res)
        return max(0, min(gx, self.width - 1)), max(0, min(gy, self.height - 1))

    def grid_to_world(self, gx, gy):
        return self.x_min + gx * self.res, self.y_min + gy * self.res

    def get_state(self, gx, gy):
        val = self.grid_log_odds[gy, gx]
        if val >= self.thresh_occ:
            return self.OCCUPIED
        elif val <= self.thresh_free:
            return self.FREE
        else:
            return self.UNKNOWN

    def is_free(self, gx, gy):
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.get_state(gx, gy) == self.FREE
        return False

    def is_occupied(self, gx, gy):
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.get_state(gx, gy) == self.OCCUPIED
        return True # out of bounds is implicitly occupied

    def add_wall_segment(self, x1, y1, x2, y2, inflate=0.0):
        lo_x = min(x1, x2) - inflate
        hi_x = max(x1, x2) + inflate
        lo_y = min(y1, y2) - inflate
        hi_y = max(y1, y2) + inflate
        gx1, gy1 = self.world_to_grid(lo_x, lo_y)
        gx2, gy2 = self.world_to_grid(hi_x, hi_y)
        self.grid_log_odds[gy1:gy2 + 1, gx1:gx2 + 1] = self.l_max

    def update_from_depth_points(self, points_world, robot_pos, sample_rate=24):
        changed_cells = set()
        cells_updated = 0
        rgx, rgy = self.world_to_grid(*robot_pos[:2])

        # Free space at robot base
        if self.grid_log_odds[rgy, rgx] > self.thresh_free:
            self.grid_log_odds[rgy, rgx] = max(self.l_min, self.grid_log_odds[rgy, rgx] + self.l_free)
            changed_cells.add((rgx, rgy))
            cells_updated += 1

        # Downsample points for performance
        points_to_process = points_world[::sample_rate]

        for pt in points_to_process:
            px, py, pz = pt[0], pt[1], pt[2]
            
            # distance from robot
            dist = math.hypot(px - robot_pos[0], py - robot_pos[1])
            if dist < 0.65:
                continue # Ignore robot's own body
            if not (self.x_min <= px <= self.x_max and self.y_min <= py <= self.y_max):
                continue
            pgx, pgy = self.world_to_grid(px, py)

            # Height filter:
            # Ground points (< 4cm) are FREE space.
            # Obstacle points (4cm to 60cm) are OCCUPIED space.
            # Points above 60cm are ignored
            if pz > 0.6:
                continue
                
            is_obstacle = (pz >= 0.04)

            # Raycast: Mark line between robot and point as FREE
            # We do this FIRST to ensure we don't update endpoints that are 'behind' 2D walls
            hit_wall = False
            ray_cells = list(bresenham_line(rgx, rgy, pgx, pgy))
            
            for i, (cx, cy) in enumerate(ray_cells):
                if not (0 <= cx < self.width and 0 <= cy < self.height):
                    break
                    
                is_endpoint = (i == len(ray_cells) - 1)
                
                if not is_endpoint:
                    # Stop if our 2D ray hits a known wall. This prevents the camera from 
                    # "seeing over" low walls and mapping the outside floor as free space.
                    if self.get_state(cx, cy) == self.OCCUPIED:
                        hit_wall = True
                        break
                        
                    # Clear free space along the ray
                    self.grid_log_odds[cy, cx] = max(self.l_min, self.grid_log_odds[cy, cx] + self.l_free)
                    changed_cells.add((cx, cy))
                    cells_updated += 1
                else:
                    # Process the endpoint ONLY if the 2D ray wasn't blocked by a wall
                    if not hit_wall:
                        if is_obstacle:
                            self.grid_log_odds[cy, cx] = min(self.l_max, self.grid_log_odds[cy, cx] + self.l_occ)
                        else:
                            self.grid_log_odds[cy, cx] = max(self.l_min, self.grid_log_odds[cy, cx] + self.l_free)
                        changed_cells.add((cx, cy))
                        cells_updated += 1

        return cells_updated, list(changed_cells)

    def is_occupied_inflated(self, gx, gy, radius_cells=4):
        """Check if cell (gx, gy) or any cell within radius_cells is OCCUPIED."""
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return True
        if self.get_state(gx, gy) == self.OCCUPIED:
            return True
        r_sq = radius_cells * radius_cells
        for dy in range(-radius_cells, radius_cells + 1):
            ny = gy + dy
            if 0 <= ny < self.height:
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy <= r_sq:
                        nx = gx + dx
                        if 0 <= nx < self.width:
                            if self.grid_log_odds[ny, nx] >= self.thresh_occ:
                                return True
        return False

    def detect_frontiers(self):
        # 1. Create a boolean mask of FREE and UNKNOWN cells
        is_free_mask = self.grid_log_odds <= self.thresh_free
        is_unknown_mask = (self.grid_log_odds > self.thresh_free) & (self.grid_log_odds < self.thresh_occ)
        
        # 2. Find edges where FREE touches UNKNOWN using morphological dilation
        kernel = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8)
        free_dilated = cv2.dilate(is_free_mask.astype(np.uint8), kernel)
        frontier_edges = (free_dilated == 1) & is_unknown_mask
        
        # 3. Cluster into regions using connectedComponents
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier_edges.astype(np.uint8), connectivity=8)
        
        frontiers = []
        for i in range(1, num_labels):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area >= 3: # Filter out tiny noise clusters
                cx, cy = int(centroids[i][0]), int(centroids[i][1])
                frontiers.append((cx, cy, area))
                
        return frontiers

    def get_grid_stats(self):
        free = np.sum(self.grid_log_odds <= self.thresh_free)
        occupied = np.sum(self.grid_log_odds >= self.thresh_occ)
        unknown = np.sum((self.grid_log_odds > self.thresh_free) & (self.grid_log_odds < self.thresh_occ))
        total = self.width * self.height
        explored_pct = (free + occupied) / total * 100.0
        return free, occupied, unknown, explored_pct

    def get_occupied_world_points(self):
        ys, xs = np.where(self.grid_log_odds >= self.thresh_occ)
        if len(ys) == 0:
            return []
        
        pts = []
        for x, y in zip(xs, ys):
            wx, wy = self.grid_to_world(x, y)
            pts.append((wx, wy, 0.2)) # Render them at 20cm height
        return pts

    def to_debug_image(self, slam_pos, slam_heading, true_pos=None, true_heading=None,
                       frontiers=None, path=None, trajectory=None,
                       blacklist_frontiers=None, scale=3):
        if frontiers is None: frontiers = []
        if trajectory is None: trajectory = []
        if blacklist_frontiers is None: blacklist_frontiers = set()

        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Crisp Color Palette:
        # UNKNOWN -> Dark Charcoal Grey (50, 50, 50)
        # FREE    -> Clean Crisp Light Gray / White (235, 235, 235)
        # OCCUPIED-> Bright Crimson Red (20, 20, 220)
        img[(self.grid_log_odds > self.thresh_free) & (self.grid_log_odds < self.thresh_occ)] = (50, 50, 50)
        img[self.grid_log_odds <= self.thresh_free] = (235, 235, 235)
        img[self.grid_log_odds >= self.thresh_occ] = (20, 20, 220)
        
        # Planned path is yellow.  The actual SLAM pose trail is green, so it
        # is immediately clear to an evaluator where the robot has travelled.
        if path:
            for i in range(len(path) - 1):
                p1_gx, p1_gy = self.world_to_grid(path[i][0], path[i][1])
                p2_gx, p2_gy = self.world_to_grid(path[i+1][0], path[i+1][1])
                cv2.line(img, (p1_gx, p1_gy), (p2_gx, p2_gy), (0, 220, 255), 1)

        if len(trajectory) > 1:
            for i in range(len(trajectory) - 1):
                p1_gx, p1_gy = self.world_to_grid(*trajectory[i])
                p2_gx, p2_gy = self.world_to_grid(*trajectory[i + 1])
                cv2.line(img, (p1_gx, p1_gy), (p2_gx, p2_gy), (0, 255, 0), 2)
                
        # Draw frontiers
        for f in frontiers:
            fx, fy = f[0], f[1]
            cv2.circle(img, (fx, fy), 2, (255, 200, 0), -1) # Cyan frontier centroid

        # Draw blacklisted frontiers
        for f in blacklist_frontiers:
            bx, by = f[0], f[1]
            cv2.circle(img, (bx, by), 2, (0, 200, 255), -1) # Orange blacklisted
            cv2.line(img, (bx-1, by-1), (bx+1, by+1), (0, 0, 0), 1)
            cv2.line(img, (bx-1, by+1), (bx+1, by-1), (0, 0, 0), 1)

        # 1. SLAM Estimated Robot Body & Pose (MAGENTA / DEEP VIOLET)
        sgx, sgy = self.world_to_grid(slam_pos[0], slam_pos[1])
        r_cells = max(3, int(ROBOT_RADIUS / self.res))
        cv2.circle(img, (sgx, sgy), r_cells, (180, 0, 180), 2) # SLAM footprint outline
        cv2.circle(img, (sgx, sgy), 4, (255, 0, 255), -1)      # Bright Magenta center dot
        sax = int(sgx + 10 * math.cos(slam_heading))
        say = int(sgy + 10 * math.sin(slam_heading))
        cv2.line(img, (sgx, sgy), (sax, say), (255, 0, 255), 2) # Magenta heading vector arrow

        # 2. Ground Truth Robot Body & Pose (CYAN / ELECTRIC BLUE) & Drift Vector (YELLOW)
        if true_pos is not None:
            tgx, tgy = self.world_to_grid(true_pos[0], true_pos[1])
            cv2.circle(img, (tgx, tgy), r_cells, (200, 200, 0), 2) # Ground truth footprint outline
            cv2.circle(img, (tgx, tgy), 4, (255, 255, 0), -1)      # Bright Cyan center dot
            if true_heading is not None:
                tax = int(tgx + 10 * math.cos(true_heading))
                tay = int(tgy + 10 * math.sin(true_heading))
                cv2.line(img, (tgx, tgy), (tax, tay), (255, 255, 0), 2) # Cyan heading vector arrow
            
            # Yellow connecting line (Position Drift Vector)
            cv2.line(img, (tgx, tgy), (sgx, sgy), (0, 255, 255), 2)
        
        # Proximity sensor lookahead ring around SLAM pose
        look_dist = 0.30
        look_cells = int(look_dist / self.res)
        cv2.circle(img, (sgx, sgy), look_cells, (255, 120, 0), 1) # Blue sensor ring
        
        # Flip vertically for bottom-left grid origin
        img = cv2.flip(img, 0)
        
        if scale > 1:
            img = cv2.resize(img, (self.width * scale, self.height * scale), interpolation=cv2.INTER_NEAREST)
        
        # Add visual Legend overlay banner at top (Dark background, clear text)
        banner_h = 28
        legend_banner = np.ones((banner_h, img.shape[1], 3), dtype=np.uint8) * 15
        cv2.putText(legend_banner, "CYAN: Ground Truth | MAGENTA: SLAM | GREEN: Travel | YELLOW: Plan | RED: Wall | WHITE: Free",
                    (10, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        img = np.vstack([legend_banner, img])
        
        return img
        
        return img

def find_nearest_free(grid, gx, gy):
    q = deque([(gx, gy)])
    visited = {(gx, gy)}
    while q:
        cx, cy = q.popleft()
        if grid.is_free(cx, cy):
            return cx, cy
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < grid.width and 0 <= ny < grid.height and (nx, ny) not in visited:
                if grid.get_state(nx, ny) == grid.OCCUPIED:
                    continue
                visited.add((nx, ny))
                q.append((nx, ny))
    return gx, gy
