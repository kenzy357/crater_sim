import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm

# --------------------------------------------------------------
# 0. PARAMETERS
# --------------------------------------------------------------
resolution = 0.1  # meters per cell
max_slope_display = 60  # cap visualization (degrees)

# --------------------------------------------------------------
# 1. LOAD POINT CLOUD
# --------------------------------------------------------------
df = pd.read_csv("media/marsyard_raw_points.csv")
points = df.iloc[:, :3].values

points[:, 1] *= -1
points[:, 2] *= -1
points[:, 2] -= np.min(points[:, 2])

x, y, z = points[:, 0], points[:, 1], points[:, 2]

# --------------------------------------------------------------
# 2. GRID SETUP
# --------------------------------------------------------------
min_x, max_x = x.min(), x.max()
min_y, max_y = y.min(), y.max()

nx = int((max_x - min_x) / resolution) + 1
ny = int((max_y - min_y) / resolution) + 1

height_map = np.full((ny, nx), np.nan)

# --------------------------------------------------------------
# 3. HEIGHT MAP
# --------------------------------------------------------------
for px, py, pz in points:
    gx = int((px - min_x) / resolution)
    gy = int((py - min_y) / resolution)

    if np.isnan(height_map[gy, gx]) or pz > height_map[gy, gx]:
        height_map[gy, gx] = pz

height_map[np.isnan(height_map)] = np.nanmin(height_map)

# --------------------------------------------------------------
# 4. SLOPE MAP
# --------------------------------------------------------------
gy, gx = np.gradient(height_map, resolution)
slope_deg = np.degrees(np.arctan(np.sqrt(gx**2 + gy**2)))

# Clip only for visualization
slope_vis = np.clip(slope_deg, 0, max_slope_display)

# --------------------------------------------------------------
# 5. DISCRETE COLOR BINS (every 10°)
# --------------------------------------------------------------
bins = np.arange(0, max_slope_display + 10, 10)  # 0,10,20,...
cmap = plt.get_cmap("viridis", len(bins) - 1)
norm = BoundaryNorm(bins, cmap.N)

# --------------------------------------------------------------
# 6. WAYPOINTS
# --------------------------------------------------------------
waypoints = np.array([
    [15.1159, -3.0854],
    [6.8073, 10.3746],
    [12.3282, 6.7797],
    [19.7272, 5.2386],
    [25.2349, 2.0235], 
    [10.0126, 0.0000],
    [20.1270, 0.0000],
    [24.4818, 7.9578],
    [17.9612, 2.8924]
])

waypoints[:, 1] *= -1

wp_grid = np.zeros_like(waypoints)
wp_grid[:, 0] = (waypoints[:, 0] - min_x) / resolution
wp_grid[:, 1] = (waypoints[:, 1] - min_y) / resolution

# --------------------------------------------------------------
# 7. PLOT
# --------------------------------------------------------------
plt.figure(figsize=(12, 10))
plt.title("Slope Map (10° Discrete Bands)")

im = plt.imshow(
    slope_vis,
    origin="lower",
    cmap=cmap,
    norm=norm
)

plt.xlabel("Grid X")
plt.ylabel("Grid Y")

# Discrete colorbar with labels
cbar = plt.colorbar(im, ticks=bins)
cbar.set_label("Slope (degrees)")

# Waypoints
plt.scatter(
    wp_grid[:, 0],
    wp_grid[:, 1],
    c="red",
    s=60,
    edgecolors="black",
    label="Waypoints"
)

plt.legend()
plt.tight_layout()
plt.show()
