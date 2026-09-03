import math
import heapq
import time
import cv2
import numpy as np
from grid import bresenham_line

def a_star_search(grid, start, goal, smooth=False):
    """
    A* search algorithm for grid path planning.
    start: (gx, gy) grid coordinates
    goal: (gx, gy) grid coordinates
    Returns list of world coordinates [(x, y), ...]
    """
    if start == goal:
        return [grid.grid_to_world(*start)]

    start_time = time.time()
    
    # Precompute inflated mask for 100x speedup
    occ = (grid.grid_log_odds > grid.thresh_occ).astype(np.uint8)
    kernel = np.zeros((11, 11), dtype=np.uint8)
    for i in range(11):
        for j in range(11):
            if (i-5)**2 + (j-5)**2 <= 25:
                kernel[i, j] = 1
    inflated_mask = cv2.dilate(occ, kernel, iterations=1)
    
    def is_inflated(x, y):
        if 0 <= x < grid.width and 0 <= y < grid.height:
            return inflated_mask[y, x] > 0
        return True

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
                
                val = grid.get_state(nx, ny)
                cost = base_cost
                
                # Penalize unknown space to prefer exploring known free space
                if val == grid.UNKNOWN:
                    cost *= 5.0
                    
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
            
            if smooth:
                simplified_world_path = [path[0]]
                for p in path[1:-1:3]:
                    simplified_world_path.append(p)
                simplified_world_path.append(path[-1])
                print(f"  [PLANNER] A* found path in {(time.time() - start_time)*1000:.1f}ms")
                return simplified_world_path
            
            print(f"  [PLANNER] A* found path in {(time.time() - start_time)*1000:.1f}ms")
            return path

        for neighbor, cost in get_neighbors_with_cost(current):
            tentative_g = g_score[current] + cost
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return []  # No path found

def _line_of_sight(grid, p1, p2):
    gx1, gy1 = grid.world_to_grid(*p1)
    gx2, gy2 = grid.world_to_grid(*p2)
    path = list(bresenham_line(gx1, gy1, gx2, gy2))
    for i in range(len(path)):
        cx, cy = path[i]
        if not grid.is_free(cx, cy):
            return False
        # Diagonal check to prevent corner cutting
        if i > 0:
            px, py = path[i-1]
            dx, dy = cx - px, cy - py
            if abs(dx) == 1 and abs(dy) == 1:
                if grid.get_state(cx, py) == grid.OCCUPIED or grid.get_state(px, cy) == grid.OCCUPIED:
                    return False
    return True

def smooth_path(grid, path, min_spacing=0.3):
    if len(path) <= 2:
        return path
    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        farthest = i + 1
        for j in range(len(path) - 1, i, -1):
            if _line_of_sight(grid, path[i], path[j]):
                farthest = j
                break
        smoothed.append(path[farthest])
        i = farthest

    final = [smoothed[0]]
    for k in range(1, len(smoothed)):
        px, py = final[-1]
        nx, ny = smoothed[k]
        d = math.hypot(nx - px, ny - py)
        if d > min_spacing * 3:
            n_segments = int(d / min_spacing)
            for s in range(1, n_segments):
                t = s / n_segments
                final.append((px + t * (nx - px), py + t * (ny - py)))
        final.append((nx, ny))
    return final
