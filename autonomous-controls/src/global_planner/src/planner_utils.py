import math
import numpy as np
import heapq
import itertools
import json
import os
from scipy.ndimage import maximum_filter

# --------------------------------------------------------------
# 0. CONFIG & COORDINATE HELPERS
# --------------------------------------------------------------
def load_config(path="config.json"):
    with open(path, 'r') as f:
        return json.load(f)

def transform_raw_points(points, flip_y=True, flip_z=True):
    """Consistent coordinate flipping (Y and Z)."""
    p = points.copy()
    if flip_y:
        p[:, 1] *= -1
    if flip_z:
        p[:, 2] *= -1
        p[:, 2] -= p[:, 2].min()
    return p

def world_to_grid(pts, min_x, min_y, resolution):
    """World (m) -> Grid (index). Expects pts as (N, 2)."""
    g = np.zeros_like(pts)
    g[:, 0] = (pts[:, 0] - min_x) / resolution
    g[:, 1] = (pts[:, 1] - min_y) / resolution
    return g

def grid_to_world(path_cells, min_x, min_y, resolution):
    """Grid (index) -> World (m). path_cells as [(gy, gx), ...]."""
    path_world = []
    for (gy, gx) in path_cells:
        x_world = min_x + gx * resolution
        y_world = min_y + gy * resolution
        path_world.append((x_world, y_world))
    return np.array(path_world)

# Precomputed step costs for 8-connected grid (avoids np.linalg.norm in hot loop)
_SQRT2 = math.sqrt(2)
_STEP_COST = {
    (-1,  0): 1.0,   (1,  0): 1.0,
    ( 0, -1): 1.0,   (0,  1): 1.0,
    (-1, -1): _SQRT2, (-1, 1): _SQRT2,
    ( 1, -1): _SQRT2, ( 1, 1): _SQRT2,
}


def _path_cost(path):
    """Geometric cost of a grid path (supports 8-connected moves)."""
    return sum(
        math.hypot(path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1])
        for k in range(len(path) - 1)
    )


def _snap_to_nearest_free(grid, cell, max_radius=8):
    """
    Keep waypoint on valid terrain:
    1) round+clip to grid
    2) if blocked, search nearest free cell in expanding radius
    """
    ny, nx = grid.shape
    cy, cx = cell

    cy = int(np.clip(np.rint(cy), 0, ny - 1))
    cx = int(np.clip(np.rint(cx), 0, nx - 1))
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

# --------------------------------------------------------------
# 1. DILATION
# --------------------------------------------------------------
def dilate_obstacles(grid, radius=1):
    inv     = 1 - grid
    dilated = maximum_filter(inv, size=2 * radius + 1)
    return 1 - dilated


# --------------------------------------------------------------
# 2. A*  (8-connected, Euclidean step cost + heuristic)
# --------------------------------------------------------------
def astar(grid, start, goal):
    ny, nx = grid.shape

    # FIX 1: validate start and goal before searching
    for pt in (start, goal):
        if not (0 <= pt[0] < ny and 0 <= pt[1] < nx):
            return None
    if grid[start[0], start[1]] == 0 or grid[goal[0], goal[1]] == 0:
        return None
    if start == goal:
        return [start]

    open_set = []
    counter  = 0          # FIX 2: tie-breaker avoids comparing tuples on equal f
    heapq.heappush(open_set, (0.0, counter, start))

    g    = {start: 0.0}
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
        for (dy, dx), step in _STEP_COST.items():    # FIX 3: precomputed cost
            nyy, nxx = cy + dy, cx + dx
            if not (0 <= nyy < ny and 0 <= nxx < nx):
                continue
            if grid[nyy, nxx] == 0:
                continue
            # Prevent diagonal corner-cutting through blocked cells.
            if dy != 0 and dx != 0:
                if grid[cy + dy, cx] == 0 or grid[cy, cx + dx] == 0:
                    continue

            tentative = g[current] + step
            neigh     = (nyy, nxx)

            if tentative < g.get(neigh, math.inf):
                g[neigh] = tentative
                # FIX 4: math.hypot instead of np.linalg.norm in hot loop
                f = tentative + math.hypot(nyy - goal[0], nxx - goal[1])
                came[neigh] = current
                counter += 1
                heapq.heappush(open_set, (f, counter, neigh))

    return None


# --------------------------------------------------------------
# 3. DISTANCE MATRIX
# --------------------------------------------------------------
def compute_distance_matrix(grid, waypoints):
    """
    FIX 5: exploit symmetry → only n*(n-1)/2 A* calls (was n²).
    FIX 6: store actual geometric path cost (sum of step lengths)
           instead of number of cells, which misweights diagonals.
    """
    n         = len(waypoints)
    dist      = np.full((n, n), 0.0)
    path_cache = {}

    for i in range(n):
        for j in range(i + 1, n):
            p = astar(grid, waypoints[i], waypoints[j])
            path_cache[(i, j)] = p
            if p is not None:
                cost = _path_cost(p)
            else:
                cost = math.inf
            dist[i, j] = cost
            dist[j, i] = cost   # symmetric grid → d(A,B) == d(B,A)

    return dist, path_cache


# --------------------------------------------------------------
# 4. TSP  (brute-force, optimal for small k)
# --------------------------------------------------------------
def best_waypoint_order_closed(dist):
    """
    index 0   = base start
    index N-1 = base end  (same coordinate as start)
    1..N-2    = waypoints to permute

    Complexity O(k!) – practical up to ~8 intermediate waypoints.
    For larger k, replace with Held-Karp (O(2^k · k²)) or nearest-neighbour.
    """
    n   = dist.shape[0]
    mid = list(range(1, n - 1))

    best_cost = math.inf
    best_perm = None

    for perm in itertools.permutations(mid):
        order = [0] + list(perm) + [n - 1]
        cost  = sum(dist[order[i], order[i+1]] for i in range(len(order) - 1))
        if cost < best_cost:
            best_cost = cost
            best_perm = order

    return best_perm


# --------------------------------------------------------------
# 5. BUILD COMPLETE PATH
# --------------------------------------------------------------
def build_total_path(grid, wp_grid):
    """
    wp_grid must be shaped (K+2, 2):
        column 0 = gx,  column 1 = gy
        row 0    = base start
        rows 1…K = intermediate waypoints
        row K+1  = base end (same location as row 0)

    Returns (path_cells, visit_order) or (None, order) on failure.
    """
    # (row=gy, col=gx) convention expected by astar / grid indexing
    raw_waypoints = [(w[1], w[0]) for w in wp_grid]
    waypoints = []
    for i, rc in enumerate(raw_waypoints):
        snapped = _snap_to_nearest_free(grid, rc)
        if snapped is None:
            print(f"  [!] Waypoint {i} has no traversable cell within search radius.")
            return None, None
        waypoints.append(snapped)

    D, path_cache = compute_distance_matrix(grid, waypoints)
    order = best_waypoint_order_closed(D)

    final_path = []
    for i in range(len(order) - 1):
        a = order[i]
        b = order[i + 1]
        if a < b:
            p = path_cache[(a, b)]
        else:
            p = path_cache[(b, a)]
            if p is not None:
                p = p[::-1]

        if p is None:
            print(f"  [!] No path between waypoint indices {a} and {b}")
            return None, order

        # Skip the first cell of each subsequent segment to avoid duplicates
        final_path.extend(p if i == 0 else p[1:])

    return final_path, order
