import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# --------------------------------------------------------------
# 0. PARAMETERS
# --------------------------------------------------------------
resolution = 0.1       # meters per cell
slope_threshold = 25.0 # degrees

# --------------------------------------------------------------
# 1. LOAD POINT CLOUD
# --------------------------------------------------------------
df = pd.read_csv("media/marsyard_raw_points.csv")
points = df.iloc[:, :3].values  # X, Y, Z

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
# 4. SLOPE
# --------------------------------------------------------------
gy, gx = np.gradient(height_map, resolution)
slope_deg = np.degrees(np.arctan(np.sqrt(gx**2 + gy**2)))

# --------------------------------------------------------------
# 5. THRESHOLD MAP (0 = safe, 1 = unsafe)
# --------------------------------------------------------------
slope_mask = slope_deg >= slope_threshold

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
# 7. PLOT (CLEAR BINARY MAP)
# --------------------------------------------------------------
plt.figure(figsize=(12, 10))
plt.title("Slope Threshold Map ( < 25° SAFE | ≥ 25° UNSAFE )")

# White = safe, Red = unsafe
cmap = ListedColormap(["white", "red"])

plt.imshow(
    slope_mask.astype(int),
    origin="lower",
    cmap=cmap
)

plt.xlabel("Grid X")
plt.ylabel("Grid Y")

# Legend (manual, explicit)
import matplotlib.patches as mpatches
safe_patch = mpatches.Patch(color="white", label="Slope < 25° (SAFE)")
unsafe_patch = mpatches.Patch(color="red", label="Slope ≥ 25° (UNSAFE)")
plt.legend(handles=[safe_patch, unsafe_patch], loc="upper right")

# Waypoints
plt.scatter(
    wp_grid[:, 0],
    wp_grid[:, 1],
    c="blue",
    s=60,
    edgecolors="black",
    label="Waypoints"
)

plt.tight_layout()
plt.show()
