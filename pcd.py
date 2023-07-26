import json
import numpy as np
import open3d as o3d
import math

# Open the JSON file
with open('depth_output.json', 'r') as f:
    data = json.load(f)

# Retrieve depth matrices
depth_img = data['depth_img']
view_matrix = data['view_matrix']
proj_matrix = np.array(data['proj_matrix']).reshape(4, 4)
depth = np.array(depth_img)

# Camera intrinsics
fx = proj_matrix[0][0]
fy = -proj_matrix[1][1]
cx = proj_matrix[0][2]
cy = proj_matrix[1][2]

# Generate point cloud
v, u = np.indices(depth.shape)
z = depth.copy()
valid_mask = (z > 0)

x = (u - cx) * z / fx
y = (v - cy) * z / fy + 150
point_cloud = np.column_stack(
    (x[valid_mask], y[valid_mask], 10000 - z[valid_mask] * 10000))

# Convert the angle to radians
theta = math.radians(12)

# Define the rotation matrix around the x-axis
R_x = np.array([
    [1, 0, 0],
    [0, np.cos(theta), -np.sin(theta)],
    [0, np.sin(theta), np.cos(theta)]
])

rotated_points = np.dot(point_cloud, R_x)

# Filter background and ground
obstacles = rotated_points[(rotated_points[:, 2] > 0)
                           & (rotated_points[:, 1] > 0.5)]

# Save output
np.set_printoptions(threshold=np.inf)
data = {
    'obstacles_pcd': obstacles.tolist()
}
with open('point_cloud.json', 'w') as f:
    json.dump(data, f)

# Axis array
x_axis = np.array([[0, 0, 0], [200, 0, 0], [250, 0, 0], [300, 0, 0]])
y_axis = np.array([[0, 0, 0], [0, 100, 0], [0, 150, 0], [
                  0, 200, 0], [0, 250, 0], [0, 300, 0]])
z_axis = np.array([[0, 0, 0], [0, 0, 200], [0, 0, 250], [0, 0, 300]])

# 2D map
# Create a copy to avoid modifying the original array
swapped_coordinates = obstacles.copy()
swapped_coordinates[:, 1], swapped_coordinates[:,
                                               2] = obstacles[:, 2], obstacles[:, 1]
swapped_coordinates[:, 2] = 0
swapped_coordinates[:, 1] *= 3

# Save output
np.set_printoptions(threshold=np.inf)
data = {
    'pcd_map': swapped_coordinates.tolist()
}
with open('pcd_map.json', 'w') as f:
    json.dump(data, f)

# Generate visualization
pcd_o3d = o3d.geometry.PointCloud()
pcd_o3d.points = o3d.utility.Vector3dVector(swapped_coordinates)
o3d.visualization.draw_geometries([pcd_o3d])
