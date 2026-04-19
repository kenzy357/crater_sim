import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from planner_utils import (
    dilate_obstacles, build_total_path, load_config, 
    transform_raw_points, world_to_grid
)

# ──────────────────────────────────────────────────────────────────────────────
# 0. LOAD CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────
cfg = load_config()
PLAN     = cfg["planning"]
COORD    = cfg["coordinates"]
PATHS    = cfg["paths"]

RESOLUTION   = PLAN["resolution"]
SLOPE_THRESH = PLAN["slope_threshold"]
DILATION_R   = PLAN["dilation_radius"]
SURF_STEP    = PLAN["surf_step"]
PATH_LIFT    = PLAN["path_lift"]

# ──────────────────────────────────────────────────────────────────────────────
# 1. LOAD POINT CLOUD
# ──────────────────────────────────────────────────────────────────────────────
print("Loading point cloud …")
df     = pd.read_csv(PATHS["points_csv"])
points = df.iloc[:, :3].values       # X, Y, Z

# Use unified transformation
points = transform_raw_points(points, flip_y=COORD["flip_y"], flip_z=COORD["flip_z"])

x, y, z = points[:, 0], points[:, 1], points[:, 2]

min_x, max_x = x.min(), x.max()
min_y, max_y = y.min(), y.max()

nx = int((max_x - min_x) / RESOLUTION) + 1
ny = int((max_y - min_y) / RESOLUTION) + 1

# ──────────────────────────────────────────────────────────────────────────────
# 2. HEIGHT MAP
# ──────────────────────────────────────────────────────────────────────────────
print("Building height map …")
gx_idx = np.clip(((x - min_x) / RESOLUTION).astype(int), 0, nx - 1)
gy_idx = np.clip(((y - min_y) / RESOLUTION).astype(int), 0, ny - 1)
flat   = gy_idx * nx + gx_idx

order_z    = np.argsort(-z)
flat_sort  = flat[order_z]
z_sort     = z[order_z]
_, first   = np.unique(flat_sort, return_index=True)

height_map = np.full(ny * nx, np.nanmin(z))
height_map[flat_sort[first]] = z_sort[first]
height_map = height_map.reshape(ny, nx)

# ──────────────────────────────────────────────────────────────────────────────
# 3. SLOPE MAP
# ──────────────────────────────────────────────────────────────────────────────
print("Computing slope map …")
gy_grad, gx_grad = np.gradient(height_map, RESOLUTION)
slope_deg = np.degrees(np.arctan(np.sqrt(gx_grad**2 + gy_grad**2)))

traversable = (slope_deg < SLOPE_THRESH).astype(int)

# ──────────────────────────────────────────────────────────────────────────────
# 4. WAYPOINTS & ROUTE
# ──────────────────────────────────────────────────────────────────────────────
base_s4 = np.array(COORD["base_s4"])
if COORD["flip_y"]:
    base_s4[1] *= -1

all_waypoints = np.array(COORD["all_waypoints"])
if COORD["flip_y"]:
    all_waypoints[:, 1] *= -1

selected_idx = COORD["selected_indices"]
selected_wps  = all_waypoints[selected_idx]

route = np.vstack([base_s4, selected_wps, base_s4])

# Use unified world_to_grid
wp_grid      = world_to_grid(route, min_x, min_y, RESOLUTION)
all_wp_grid  = world_to_grid(all_waypoints, min_x, min_y, RESOLUTION)
base_grid    = world_to_grid(base_s4[np.newaxis], min_x, min_y, RESOLUTION)[0]

# ──────────────────────────────────────────────────────────────────────────────
# 5. OBSTACLE DILATION & PLANNING
# ──────────────────────────────────────────────────────────────────────────────
print(f"Dilating obstacles (radius={DILATION_R}) …")
safe_grid = dilate_obstacles(traversable, radius=DILATION_R)

print("Running A* / TSP planner …")
path, order = build_total_path(safe_grid, wp_grid)

if path is None:
    raise RuntimeError("Planner failed – no path found. Try increasing SLOPE_THRESH.")

# ──────────────────────────────────────────────────────────────────────────────
# 6. PATH → WORLD + HEIGHT
# ──────────────────────────────────────────────────────────────────────────────
POLE_EVERY  = 20
path_arr = np.array(path)
path_x   = min_x + path_arr[:, 1] * RESOLUTION
path_y   = min_y + path_arr[:, 0] * RESOLUTION
path_z_ground = height_map[path_arr[:, 0], path_arr[:, 1]]
path_z        = path_z_ground + PATH_LIFT

def wp_height(wp_world):
    gx = int(np.clip((wp_world[0] - min_x) / RESOLUTION, 0, nx - 1))
    gy = int(np.clip((wp_world[1] - min_y) / RESOLUTION, 0, ny - 1))
    return height_map[gy, gx]

# ──────────────────────────────────────────────────────────────────────────────
# 7. SURFACE SUBSAMPLING
# ──────────────────────────────────────────────────────────────────────────────
ss = SURF_STEP
gx_idx = np.arange(0, nx, ss)
gy_idx = np.arange(0, ny, ss)
GX, GY  = np.meshgrid(gx_idx, gy_idx)

X_surf = min_x + GX * RESOLUTION
Y_surf = min_y + GY * RESOLUTION
Z_surf = height_map[::ss, ::ss]
S_surf = slope_deg[::ss, ::ss]

norm_slope  = np.clip(S_surf / (SLOPE_THRESH * 1.5), 0.0, 1.0)
face_colors = cm.RdYlGn_r(norm_slope)

# ──────────────────────────────────────────────────────────────────────────────
# 8. FIGURE
# ──────────────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(22, 10))
ax3 = fig.add_subplot(121, projection="3d")
ax2 = fig.add_subplot(122)

# 3-D Terrain
ax3.plot_surface(X_surf, Y_surf, Z_surf, facecolors=face_colors, alpha=0.65)
ax3.plot(path_x, path_y, path_z, color="yellow", linewidth=3.5, label="Planned path")

# Scatter points and poles
DOT_EVERY = 8
ax3.scatter(path_x[::DOT_EVERY], path_y[::DOT_EVERY], path_z[::DOT_EVERY], c="yellow", s=10)
for idx in range(0, len(path_x), POLE_EVERY):
    ax3.plot([path_x[idx], path_x[idx]], [path_y[idx], path_y[idx]], [path_z_ground[idx], path_z[idx]], color="yellow", alpha=0.55)

# Markers
bh = wp_height(base_s4) + PATH_LIFT
ax3.scatter([base_s4[0]], [base_s4[1]], [bh], c="cyan", s=300, marker="*")
for i, wp in enumerate(selected_wps):
    h = wp_height(wp) + PATH_LIFT
    ax3.scatter([wp[0]], [wp[1]], [h], c="red", s=200, marker="o")
    ax3.text(wp[0], wp[1], h + 0.12, f"W{selected_idx[i]}", color="red", fontweight="bold")

# 2-D Top-down
ax2.imshow(safe_grid, cmap="gray", origin="lower", extent=[min_x, max_x, min_y, max_y], alpha=0.6)
ax2.imshow(cm.RdYlGn_r(norm_slope), origin="lower", extent=[min_x, max_x, min_y, max_y], alpha=0.5)
ax2.plot(path_x, path_y, color="yellow", linewidth=2.5, label="Planned path")

# Save & Show
plt.suptitle(f"Rover Path Planner | Slope: {SLOPE_THRESH}° | WPs: {[f'W{i}' for i in selected_idx]}", fontsize=13, fontweight="bold")
os.makedirs(PATHS["output_dir"], exist_ok=True)
plt.savefig(f"{PATHS['output_dir']}/planner_3d_view.png", dpi=150)
plt.show()
