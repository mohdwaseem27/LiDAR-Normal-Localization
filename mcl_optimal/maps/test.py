import open3d as o3d
import numpy as np

# Load the PCD file
pcd = o3d.io.read_point_cloud("map_output.pcd")

# Check if the point cloud has normals
if pcd.has_normals():
    print("Point cloud has normals. Visualizing normals.")
else:
    print("Point cloud does not have normals. Estimating normals...")
    # Estimate normals if they are not available
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# Visualize the point cloud along with normals
o3d.visualization.draw_geometries([pcd], point_show_normal=True)
