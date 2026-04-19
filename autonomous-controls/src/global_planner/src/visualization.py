import open3d as o3d
import pandas as pd

file = 'media/marsyard_raw_points.csv'

df = pd.read_csv(file)
points = df.iloc[:, :3].values

point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points)

o3d.visualization.draw_geometries([point_cloud])