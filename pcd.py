import json
import numpy as np
import open3d as o3d
import math
import matplotlib.pyplot as plt

# Tuning
translation = [0, 3]
stretching = [1, 1]
dimensions = 25

# Open the JSON file
with open('depth_output.json', 'r') as f:
    data = json.load(f)

# Retrieve depth matrices
depth_img = data['depth']
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
    (x[valid_mask] * 0.7, y[valid_mask], 10000 - z[valid_mask] * 10000))

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

# 2D map
map = obstacles.copy()
map[:, 1], map[:, 2] = obstacles[:, 2], obstacles[:, 1]
map[:, 2] = 0
map = map[:, :2]

# Transform map
scaled_map = map * ((dimensions - 1) / 238) # scale it down
local_map = np.array([[((dimensions - 1) / 2) -
                       (x - ((dimensions - 1) / 2)), y] for x, y in scaled_map]) # flip the y axis along its middle

local_map[:, 0] += translation[0]
local_map[:, 1] += translation[1]

local_map[:, 0] = (local_map[:, 0] - 12) * stretching[0] + 12
local_map[:, 1] = (local_map[:, 1] - 12) * stretching[1] + 12

# Save output to JSON
# 2D map
np.set_printoptions(threshold=np.inf)
map_data = {
    'map': local_map.tolist()
}
with open('pcd_map.json', 'w') as f:
    json.dump(map_data, f)

# # 3D obstacles
# obstacles_data = {
#     'obstacles_pcd': obstacles.tolist()
# }
# with open('point_cloud.json', 'w') as f:
#     json.dump(obstacles_data, f)

# Visual the map
def plot_map(array):
    plt.clf()
    plt.scatter(array[:, 0], array[:, 1], s=1)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.xlim(0, dimensions - 1)
    plt.ylim(0, dimensions - 1)
    plt.title('Obstacles')
    plt.show()

# Save depth image for comparison
plt.imsave(f'depth_map_images/depth_img.png', depth)

# Generate visualization
plot_map(local_map)

# # Archive open3d
# pcd_o3d = o3d.geometry.PointCloud()
# pcd_o3d.points = o3d.utility.Vector3dVector(obstacles)
# o3d.visualization.draw_geometries([pcd_o3d])
