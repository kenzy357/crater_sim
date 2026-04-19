import os
import numpy as np
import matplotlib.pyplot as plt
from planner_utils import (
    dilate_obstacles, build_total_path, load_config, 
    world_to_grid, grid_to_world
)

# ──────────────────────────────────────────────────────────────────────────────
# 0. LOAD CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────
cfg = load_config()
COORD = cfg["coordinates"]
PLAN  = cfg["planning"]
PATHS = cfg["paths"]

# ──────────────────────────────────────────────────────────────────────────────
# 1. LOAD PRECOMPUTED GRID + METADATA
# ──────────────────────────────────────────────────────────────────────────────
grid = np.load("grid.npy")
min_x, min_y, resolution = np.load("metadata.npy")

# ──────────────────────────────────────────────────────────────────────────────
# 2. WAYPOINTS & ROUTE
# ──────────────────────────────────────────────────────────────────────────────
base_s4 = np.array(COORD["base_s4"])
if COORD["flip_y"]:
    base_s4[1] *= -1

all_waypoints = np.array(COORD["all_waypoints"])
if COORD["flip_y"]:
    all_waypoints[:, 1] *= -1

selected_idx = COORD["selected_indices"]
selected_wps = all_waypoints[selected_idx]

# FINAL LIST: base → selected → base
route = np.vstack([base_s4, selected_wps, base_s4])

# Convert to grid coords using helper
wp_grid      = world_to_grid(route, min_x, min_y, resolution)
all_wp_grid  = world_to_grid(all_waypoints, min_x, min_y, resolution)
base_grid    = world_to_grid(base_s4[np.newaxis], min_x, min_y, resolution)[0]

# ──────────────────────────────────────────────────────────────────────────────
# 3. DILATE OBSTACLES & PLANNING
# ──────────────────────────────────────────────────────────────────────────────
safe_grid = dilate_obstacles(grid, radius=PLAN["dilation_radius"])

print(f"Running A* / TSP planner (2D mode) …")
path, order = build_total_path(safe_grid, wp_grid)

if path is None:
    raise RuntimeError(
        "Planner failed – no valid path found. "
        "Check that all selected waypoints are on traversable terrain."
    )

print("Optimal order:", order)
print("Path length (cells):", len(path))

# ──────────────────────────────────────────────────────────────────────────────
# 4. CONVERT GRID PATH → WORLD COORDINATES
# ──────────────────────────────────────────────────────────────────────────────
path_world = grid_to_world(path, min_x, min_y, resolution)

# Save to CSV
os.makedirs(PATHS["output_dir"], exist_ok=True)
csv_out = os.path.join(PATHS["output_dir"], "global_path_world.csv")
np.savetxt(csv_out, path_world, delimiter=",", header="x,y", comments="")
print(f"Saved path to {csv_out}")

# ──────────────────────────────────────────────────────────────────────────────
# 5. PLOT EVERYTHING
# ──────────────────────────────────────────────────────────────────────────────
plt.figure(figsize=(12,10))
plt.imshow(safe_grid, cmap="gray", origin="lower")

# Markers
plt.scatter(base_grid[0], base_grid[1], c='cyan', s=120, label="BASE S4")
plt.scatter(all_wp_grid[:,0], all_wp_grid[:,1], c="orange", s=40, label="All 9 WPs")
plt.scatter(wp_grid[1:-1,0], wp_grid[1:-1,1], c="red", s=60, label="Selected")

# Path
py, px = zip(*path)
plt.plot(px, py, "lime", linewidth=2, label="Optimal Path")

plt.legend()
plt.title(f"Global A* Optimal Path | Selected WPs: {selected_idx}")
plt.show()
