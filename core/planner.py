import math
import heapq
import cv2
import numpy as np
from core.grid import bresenham_line

def a_star_search(grid, start, goal, smooth=False):
    """
    A* search algorithm for grid path planning with obstacle clearance margin.
    start: (gx, gy) grid coordinates
    goal: (gx, gy) grid coordinates
    Returns list of world coordinates [(x, y), ...]
    """
    if start == goal:
        return [grid.grid_to_world(*start)]

    # Precompute 15cm hard obstacle clearance for collision avoidance (3 cells radius at 5cm resolution)
    occ = (grid.grid_log_odds > grid.thresh_occ).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    inflated_mask = cv2.dilate(occ, kernel, iterations=1)
    
    # Precompute soft wall clearance field using Euclidean Distance Transform (meters)
    # Allows robot to center in open spaces while seamlessly traversing tight 60cm doorways
    free_space_binary = (1 - occ).astype(np.uint8)
    dist_to_wall = cv2.distanceTransform(free_space_binary, cv2.DIST_L2, 3) * grid.res

    # Traversable = known free space with 1-cell raycasting pinholes closed. Large unknown regions (e.g. behind a
    # wall the camera has not seen) stay blocked, so paths cannot cut through unexplored space.
    free = (grid.grid_log_odds <= grid.thresh_free).astype(np.uint8)
    traversable = cv2.morphologyEx(free, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)) > 0
    
    def is_inflated(x, y):
        if 0 <= x < grid.width and 0 <= y < grid.height:
            return inflated_mask[y, x] > 0
        return True

    # If start is inside inflation margin (robot near wall), find closest non-inflated free cell
    if is_inflated(*start):
        best_dist = float('inf')
        alt_start = start
        for r in range(1, 12):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    cand = (start[0] + dx, start[1] + dy)
                    if 0 <= cand[0] < grid.width and 0 <= cand[1] < grid.height:
                        if not is_inflated(*cand) and grid.is_free(*cand):
                            d = math.hypot(dx, dy)
                            if d < best_dist:
                                best_dist = d
                                alt_start = cand
            if best_dist < float('inf'):
                start = alt_start
                break

    # If goal is inside inflation margin (frontier on wall edge), find closest non-inflated free neighbor
    if is_inflated(*goal):
        best_dist = float('inf')
        alt_goal = goal
        for r in range(1, 8):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    cand = (goal[0] + dx, goal[1] + dy)
                    if 0 <= cand[0] < grid.width and 0 <= cand[1] < grid.height:
                        if not is_inflated(*cand) and grid.is_free(*cand):
                            d = math.hypot(dx, dy)
                            if d < best_dist:
                                best_dist = d
                                alt_goal = cand
            if best_dist < float('inf'):
                goal = alt_goal
                break

    if start == goal:
        return [grid.grid_to_world(*start)]

    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def get_neighbors_with_cost(s):
        neighbors = []
        cx, cy = s
        dirs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
        for dx, dy, base_cost in dirs:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < grid.width and 0 <= ny < grid.height:
                if is_inflated(nx, ny):
                    continue
                # Prevent diagonal corner clipping
                if abs(dx) == 1 and abs(dy) == 1:
                    if is_inflated(nx, cy) or is_inflated(cx, ny):
                        continue
                
                # Plan through known free space only: unknown cells may lie behind a wall the camera has not seen.
                if not traversable[ny, nx] and (nx, ny) != goal:
                    continue
                cost = base_cost
                
                # Soft clearance penalty: prefer staying away from walls in open areas,
                # but seamlessly passes through narrow 60cm doorways without blocking
                d_wall = dist_to_wall[ny, nx]
                if d_wall < 0.28:
                    cost += (0.28 - d_wall) * 1.5

                neighbors.append(((nx, ny), cost))
        return neighbors

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(grid.grid_to_world(*current))
                current = came_from[current]
            path.append(grid.grid_to_world(*start))
            path.reverse()
            return path

        for neighbor, cost in get_neighbors_with_cost(current):
            tentative_g = g_score[current] + cost
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return []  # No path found

def _line_of_sight(grid, p1, p2, inflated_mask=None):
    gx1, gy1 = grid.world_to_grid(*p1)
    gx2, gy2 = grid.world_to_grid(*p2)
    path = list(bresenham_line(gx1, gy1, gx2, gy2))
    for i in range(len(path)):
        cx, cy = path[i]
        if not (0 <= cx < grid.width and 0 <= cy < grid.height):
            return False
        if not grid.is_free(cx, cy):
            return False
        if inflated_mask is not None and inflated_mask[cy, cx] > 0:
            return False
        # Diagonal check to prevent corner cutting
        if i > 0:
            px, py = path[i-1]
            dx, dy = cx - px, cy - py
            if abs(dx) == 1 and abs(dy) == 1:
                if grid.get_state(cx, py) == grid.OCCUPIED or grid.get_state(px, cy) == grid.OCCUPIED:
                    return False
    return True

def smooth_path(grid, path, min_spacing=0.20):
    if len(path) <= 2:
        return path
    
    # Safety clearance mask for line of sight
    occ = (grid.grid_log_odds > grid.thresh_occ).astype(np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    inflated_mask = cv2.dilate(occ, kernel, iterations=1)

    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        farthest = i + 1
        for j in range(len(path) - 1, i, -1):
            if _line_of_sight(grid, path[i], path[j], inflated_mask):
                farthest = j
                break
        smoothed.append(path[farthest])
        i = farthest

    final = [smoothed[0]]
    for k in range(1, len(smoothed)):
        px, py = final[-1]
        nx, ny = smoothed[k]
        d = math.hypot(nx - px, ny - py)
        if d > min_spacing * 2:
            n_segments = int(d / min_spacing)
            for s in range(1, n_segments):
                t = s / n_segments
                final.append((px + t * (nx - px), py + t * (ny - py)))
        final.append((nx, ny))
    return final
