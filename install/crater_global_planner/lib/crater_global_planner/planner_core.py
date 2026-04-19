import heapq
import itertools
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.ndimage import maximum_filter

GridCell = Tuple[int, int]  # (row=y, col=x)

_SQRT2 = math.sqrt(2.0)
_STEP_COST = {
    (-1, 0): 1.0,
    (1, 0): 1.0,
    (0, -1): 1.0,
    (0, 1): 1.0,
    (-1, -1): _SQRT2,
    (-1, 1): _SQRT2,
    (1, -1): _SQRT2,
    (1, 1): _SQRT2,
}


def world_to_grid(pts: np.ndarray, min_x: float, min_y: float, resolution: float) -> np.ndarray:
    g = np.zeros_like(pts, dtype=float)
    g[:, 0] = (pts[:, 0] - min_x) / resolution
    g[:, 1] = (pts[:, 1] - min_y) / resolution
    return g


def grid_to_world(path_cells: Sequence[GridCell], min_x: float, min_y: float, resolution: float) -> np.ndarray:
    path_world = []
    for gy, gx in path_cells:
        path_world.append((min_x + gx * resolution, min_y + gy * resolution))
    return np.array(path_world, dtype=float)


def dilate_obstacles(grid: np.ndarray, radius: int) -> np.ndarray:
    inv = 1 - grid
    dilated = maximum_filter(inv, size=2 * radius + 1)
    return 1 - dilated


def _path_cost(path: Sequence[GridCell]) -> float:
    if len(path) < 2:
        return 0.0
    return float(
        sum(
            math.hypot(path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1])
            for k in range(len(path) - 1)
        )
    )


def _snap_to_nearest_free(grid: np.ndarray, cell: GridCell, max_radius: int = 8) -> Optional[GridCell]:
    ny, nx = grid.shape
    cy = int(np.clip(np.rint(cell[0]), 0, ny - 1))
    cx = int(np.clip(np.rint(cell[1]), 0, nx - 1))
    if grid[cy, cx] == 1:
        return (cy, cx)

    best = None
    best_d2 = math.inf
    for r in range(1, max_radius + 1):
        y0, y1 = max(0, cy - r), min(ny - 1, cy + r)
        x0, x1 = max(0, cx - r), min(nx - 1, cx + r)
        for yy in range(y0, y1 + 1):
            for xx in range(x0, x1 + 1):
                if grid[yy, xx] == 0:
                    continue
                d2 = (yy - cy) ** 2 + (xx - cx) ** 2
                if d2 < best_d2:
                    best = (yy, xx)
                    best_d2 = d2
        if best is not None:
            return best
    return None


def astar(grid: np.ndarray, start: GridCell, goal: GridCell) -> Optional[List[GridCell]]:
    ny, nx = grid.shape
    for pt in (start, goal):
        if not (0 <= pt[0] < ny and 0 <= pt[1] < nx):
            return None
    if grid[start[0], start[1]] == 0 or grid[goal[0], goal[1]] == 0:
        return None
    if start == goal:
        return [start]

    open_set = []
    counter = 0
    heapq.heappush(open_set, (0.0, counter, start))
    g = {start: 0.0}
    came = {}
    visited = set()

    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            path = []
            while current in came:
                path.append(current)
                current = came[current]
            path.append(start)
            return path[::-1]

        cy, cx = current
        for (dy, dx), step in _STEP_COST.items():
            nyy, nxx = cy + dy, cx + dx
            if not (0 <= nyy < ny and 0 <= nxx < nx):
                continue
            if grid[nyy, nxx] == 0:
                continue
            if dy != 0 and dx != 0 and (grid[cy + dy, cx] == 0 or grid[cy, cx + dx] == 0):
                continue

            tentative = g[current] + step
            neigh = (nyy, nxx)
            if tentative < g.get(neigh, math.inf):
                g[neigh] = tentative
                f = tentative + math.hypot(nyy - goal[0], nxx - goal[1])
                came[neigh] = current
                counter += 1
                heapq.heappush(open_set, (f, counter, neigh))
    return None


def _distance_matrix(grid: np.ndarray, waypoints: Sequence[GridCell]):
    n = len(waypoints)
    dist = np.full((n, n), 0.0)
    path_cache = {}
    for i in range(n):
        for j in range(i + 1, n):
            p = astar(grid, waypoints[i], waypoints[j])
            path_cache[(i, j)] = p
            c = _path_cost(p) if p is not None else math.inf
            dist[i, j] = c
            dist[j, i] = c
    return dist, path_cache


def _best_waypoint_order_closed(dist: np.ndarray) -> List[int]:
    n = dist.shape[0]
    mid = list(range(1, n - 1))
    best_cost = math.inf
    best_perm = None
    for perm in itertools.permutations(mid):
        order = [0] + list(perm) + [n - 1]
        cost = sum(dist[order[i], order[i + 1]] for i in range(len(order) - 1))
        if cost < best_cost:
            best_cost = cost
            best_perm = order
    return best_perm


def build_total_path(grid: np.ndarray, wp_grid: np.ndarray):
    raw_waypoints = [(w[1], w[0]) for w in wp_grid]
    waypoints = []
    for rc in raw_waypoints:
        snapped = _snap_to_nearest_free(grid, rc)
        if snapped is None:
            return None, None
        waypoints.append(snapped)

    dist, path_cache = _distance_matrix(grid, waypoints)
    order = _best_waypoint_order_closed(dist)
    if order is None:
        return None, None

    final_path = []
    for i in range(len(order) - 1):
        a, b = order[i], order[i + 1]
        if a < b:
            p = path_cache[(a, b)]
        else:
            p = path_cache[(b, a)]
            if p is not None:
                p = p[::-1]
        if p is None:
            return None, order
        final_path.extend(p if i == 0 else p[1:])
    return final_path, order
