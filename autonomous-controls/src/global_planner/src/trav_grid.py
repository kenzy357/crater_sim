import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.ndimage import uniform_filter

# --------------------------------------------------------------
# 0. PARAMETERS
# --------------------------------------------------------------
slope_thresh = 20.0
rough_thresh = 0.1
resolution = 0.1  # meters per cell (10 cm) numbere of cells 

# --------------------------------------------------------------
# 1. LOAD POINT CLOUD CSV
# --------------------------------------------------------------
df = pd.read_csv("media/marsyard_raw_points.csv")
points = df.iloc[:, :3].values  # X, Y, Z

points[:, 1] *= -1  # Flip Y
points[:, 2] *= -1  # Flip Z
points[:, 2] -= np.min(points[:, 2])

# --------------------------------------------------------------
# 2. CONFIGURE GRID
# --------------------------------------------------------------
x = points[:, 0]
y = points[:, 1]
z = points[:, 2]

min_x, max_x = x.min(), x.max()
min_y, max_y = y.min(), y.max()

nx = int((max_x - min_x) / resolution) + 1
ny = int((max_y - min_y) / resolution) + 1

height_map = np.full((ny, nx), np.nan)

# --------------------------------------------------------------
# 3. FILL HEIGHT MAP (use highest point per cell)
# --------------------------------------------------------------
for px, py, pz in points:
    gx = int((px - min_x) / resolution)
    gy = int((py - min_y) / resolution)

    if np.isnan(height_map[gy, gx]) or pz > height_map[gy, gx]:
        height_map[gy, gx] = pz

# Replace NaNs with minimum height
height_map[np.isnan(height_map)] = np.nanmin(height_map)

# --------------------------------------------------------------
# 4. COMPUTE SLOPE MAP
# --------------------------------------------------------------
gy, gx = np.gradient(height_map, resolution)
slope_deg = np.degrees(np.arctan(np.sqrt(gx**2 + gy**2)))

# --------------------------------------------------------------
# 5. COMPUTE ROUGHNESS  (vectorized – replaces slow Python double-loop)
# --------------------------------------------------------------
kernel = 3
# std = sqrt(E[x²] - E[x]²) computed with sliding-window mean filters
_mean_sq = uniform_filter(height_map ** 2, size=kernel)
_sq_mean = uniform_filter(height_map,      size=kernel) ** 2
roughness_map = np.sqrt(np.maximum(_mean_sq - _sq_mean, 0.0))

# --------------------------------------------------------------
# 6. TRAVERSABILITY RULES
# --------------------------------------------------------------
traversable = np.logical_and(
    slope_deg < slope_thresh,
    roughness_map < rough_thresh
)

grid = traversable.astype(int)

# --------------------------------------------------------------
# 7. DEFINE WAYPOINTS
# --------------------------------------------------------------
waypoints = np.array([
    [15.1159, -3.0854],
    [6.8073, 10.3746],
    [12.3282, 6.7797],
    [19.7272, 5.2386],
    [25.2349, 2.0235], 
    [10.0126, 0.0000],
    [20.1270, 0.0000],
    [24.4818, 7.9578], # nogood
    [17.9612, 2.8924]  # nogood
])

# Same inversion as applied to point cloud
waypoints[:, 1] *= -1

# Convert world → grid coordinates
wp_grid = np.zeros_like(waypoints)
wp_grid[:, 0] = (waypoints[:, 0] - min_x) / resolution
wp_grid[:, 1] = (waypoints[:, 1] - min_y) / resolution

# --------------------------------------------------------------
# 8. PLOT
# --------------------------------------------------------------
plt.figure(figsize=(12, 10))
plt.title("2D Traversability Grid (White=safe, Black=unsafe)")
plt.imshow(grid, cmap="gray", origin="lower")

plt.xlabel("Grid X")
plt.ylabel("Grid Y")

plt.scatter(
    wp_grid[:, 0],
    wp_grid[:, 1],
    c='red',
    s=50,
    marker='o',
    label="Waypoints"
)

plt.legend()

# --------------------------------------------------------------
# 9. SAVE FOR PLANNER
# --------------------------------------------------------------
np.save("grid.npy", grid)
np.save("metadata.npy", np.array([min_x, min_y, resolution]))

print("Saved grid.npy and metadata.npy")

plt.show()
