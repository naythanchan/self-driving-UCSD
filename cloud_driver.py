import pybullet as p
import time
import pybullet_data
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import json

# Physics
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.resetSimulation()
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)  # either this

# Camera
p.resetDebugVisualizerCamera(
    cameraDistance=0.1,
    cameraYaw=- 90,
    cameraPitch=10,
    cameraTargetPosition=[
        0, 0, 0.2],
    physicsClientId=0
)
# Path
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Assets
p.loadSDF("stadium.sdf")
car = p.loadURDF("racecar/racecar.urdf")
cube = p.loadURDF("cube.urdf", basePosition=[0, 3, 0.5], useFixedBase=1)

# Obstacles
zone_width = 6  # meters
zone_depth = 10  # meters

# Define the race position
race_position = [0, 0, 0]  # [x, y, z]

# Generate a random position within the zone
def generate_random_position():
    x = random.uniform(race_position[0] + 2, race_position[0] + 2 + zone_depth)
    y = random.uniform(
        race_position[1] - zone_width/2, race_position[1] + zone_width/2)
    z = race_position[2] + 0.5  # Height of the cube
    return [x, y, z]

num_cubes = 10  # Number of cubes to load
box_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.4, height=1)

for _ in range(num_cubes):
    cube_position = generate_random_position()
    box_body = p.createMultiBody(
        baseMass=1.0,
        baseCollisionShapeIndex=box_shape,
        basePosition=cube_position,
    )

# While Loop Init
start_time = time.time()
last_process_time = 0
i = 0

steering = [4, 6]
wheels = [2, 3]
target_pos = 0

# Loop
while (True):
    elapsed_time = time.time() - start_time

    # Depth Camera
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    cam_info = p.getDebugVisualizerCamera()
    view_matrix = cam_info[2]
    proj_matrix = cam_info[3]

    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
        256, 256, viewMatrix=view_matrix, projectionMatrix=proj_matrix
    )
    depth = np.array(depth_img).reshape((256, 256))

    # Driving
    for wheel in wheels:
        p.setJointMotorControl2(car,
                                wheel,
                                p.VELOCITY_CONTROL,
                                targetVelocity=25,
                                force=2)
    for steer in steering:
        p.setJointMotorControl2(
            car, steer, p.POSITION_CONTROL, targetPosition=0)

    
    # Localization
    pos, hquat = p.getBasePositionAndOrientation(car)
    h = p.getEulerFromQuaternion(hquat)
    x = pos[0]
    y = pos[1]

    if elapsed_time - last_process_time >= 2.0:
        # Point Cloud
        projection = np.array(proj_matrix).reshape(4, 4)
        fx = projection[0][0]
        fy = -projection[1][1]
        cx = projection[0][2]
        cy = projection[1][2]

        # Generate point cloud
        v, u = np.indices(depth.shape)
        z = depth.copy()
        valid_mask = (z > 0)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy + 150
        point_cloud = np.column_stack((x[valid_mask], y[valid_mask], 10000 - z[valid_mask] * 10000))
        obstacles = point_cloud[(point_cloud[:, 2] != 0) & (point_cloud[:, 1] > 0)]

        # Remove outliers
        closest_point = np.max(obstacles[:, 2])
        car_buffer = 20
        close_obstacles = obstacles[obstacles[:, 2] >= closest_point - car_buffer]

        # Make it 2d
        plt.scatter(close_obstacles[:, 0], close_obstacles[:, 1], s=1)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Close Obstacles')
        plt.savefig(f'pcd_images/pcd_{i}.png')

        # Update
        i += 1
        last_process_time = elapsed_time

    # Follow Robot With Camera
    p.resetDebugVisualizerCamera(
        cameraDistance=0.1,
        cameraYaw=math.degrees(h[2]) - 90,
        cameraPitch=10,
        cameraTargetPosition=[
            pos[0] + math.cos(h[2]), pos[1] + math.sin(h[2]), pos[2] + 0.2],
        physicsClientId=0
    )
