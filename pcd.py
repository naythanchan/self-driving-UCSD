import json
import numpy as np
import open3d as o3d

# Open the JSON file
with open('depth_output.json', 'r') as f:
    data = json.load(f)

# Access specific values from the JSON data
depth_img = data['depth_img']
view_matrix = data['view_matrix']
proj_matrix = np.array(data['proj_matrix']).reshape(4, 4)
depth = np.array(depth_img)

# Check the shape of the depth array
print(depth.shape)
print(proj_matrix)


# Camera intrinsics
fx = -proj_matrix[0][0]
fy = -proj_matrix[1][1]
cx = proj_matrix[0][2]
cy = proj_matrix[1][2]

# Generate point cloud
point_cloud = []
for v in range(depth.shape[0]):
    for u in range(depth.shape[1]):
        z = depth[v, u]
        if z > 0:
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            point_cloud.append([x, y, z * 10000])

# Convert to NumPy array
point_cloud = np.array(point_cloud)

# filtered_point_cloud = point_cloud[point_cloud[:, 2] != -1]

# np.set_printoptions(threshold=np.inf)
# data = {
#     'point_cloud': point_cloud.tolist()
# }
# with open('point_cloud.json', 'w') as f:
#     json.dump(data, f)

pcd_o3d = o3d.geometry.PointCloud()  # create point cloud object
pcd_o3d.points = o3d.utility.Vector3dVector(
    point_cloud)  # set pcd_np as the point cloud points
# Visualize:
o3d.visualization.draw_geometries([pcd_o3d])
