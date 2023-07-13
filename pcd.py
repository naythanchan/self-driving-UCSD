import json
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

# Open the JSON file
with open('depth_output.json', 'r') as f:
    data = json.load(f)

# Access specific values from the JSON data
depth_img = data['depth_img']
view_matrix = data['view_matrix']
proj_matrix = np.array(data['proj_matrix']).reshape(4, 4)
depth = np.array(depth_img)

print(proj_matrix)

# Get dimensions
height, width = depth.shape
jj = np.tile(range(width), height)
ii = np.repeat(range(height), width)
# Camera intrinsics
fx = proj_matrix[0][0]
fy = proj_matrix[1][1]
cx = proj_matrix[0][2]
cy = proj_matrix[1][2]

# Generate point cloud
point_cloud = []
for v in range(76):
    for u in range(256):
        z = depth[v, u]
        if z > 0:
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            point_cloud.append([x, y, z])

# Convert to NumPy array
pcd = np.array(point_cloud)
# print(pcd)

# # Reshape depth image
# z = depth.reshape(height * width)

# # Camera intrinsics
# FX_DEPTH = proj_matrix[0][0]
# FY_DEPTH = proj_matrix[1][1]
# CX_DEPTH = proj_matrix[0][2]
# CY_DEPTH = proj_matrix[1][2]

# # Compute point cloud
# pcd = np.dstack([((ii - CX_DEPTH) * z / FX_DEPTH),
#                  ((jj - CY_DEPTH) * z / FY_DEPTH),
#                  z]).reshape((height * width, 3))

pcd_o3d = o3d.geometry.PointCloud()  # create point cloud object
pcd_o3d.points = o3d.utility.Vector3dVector(pcd)  # set pcd_np as the point cloud points
# Visualize:
o3d.visualization.draw_geometries([pcd_o3d])