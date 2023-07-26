import pybullet as p
import time
import pybullet_data
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import json

# Tuning paramaters
gap_threshold = 40  # min gap a car fits through
car_buffer = 20  # close obstacles
steering_constant = 0.005  # nerf correction
frames = 2  # takes snapshots every {frames} seconds
capture = True  # boolean to save images

zone_width = 30  # obstacle meters
zone_depth = 50  # obstacle meters
num_cubes = 60  # Number of cubes to load

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

# Define the race position
race_position = [0, 0, 0]  # [x, y, z]

# Generate a random position within the zone


def generate_random_position():
    x = random.uniform(race_position[0] + 2.3,
                       race_position[0] + 2.3 + zone_depth)
    y = random.uniform(
        race_position[1] - zone_width/2, race_position[1] + zone_width/2)
    z = race_position[2] + 0.5  # Height of the cube
    return [x, y, z]


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
first_shot = True
i = 0

steering = [4, 6]
wheels = [2, 3]
target_pos = 0
direction = 0

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
                                targetVelocity=35,
                                force=10)
    for steer in steering:
        p.setJointMotorControl2(
            car, steer, p.POSITION_CONTROL, targetPosition=direction * steering_constant)

    # Localization
    pos, hquat = p.getBasePositionAndOrientation(car)
    h = p.getEulerFromQuaternion(hquat)
    x = pos[0]
    y = pos[1]

    if (elapsed_time - last_process_time >= frames) or (first_shot == True and elapsed_time - last_process_time >= 0.1):
        first_shot = False

        print(f"Frame {i}")
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
        point_cloud = np.column_stack(
            (x[valid_mask], y[valid_mask], 10000 - z[valid_mask] * 10000))
        
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

        if (len(obstacles) == 0):
            continue
        # Remove outliers
        closest_point = np.max(obstacles[:, 2])
        close_obstacles = obstacles[obstacles[:, 2]
                                    >= closest_point - car_buffer]
        # End points
        bl_point = np.array([0, 0, 70])
        br_point = np.array([350, 0, 70])
        tl_point = np.array([0, 350, 70])
        tr_point = np.array([350, 350, 70])
        # Append end points to the bottom of close_obstacles array
        close_obstacles = np.vstack(
            (close_obstacles, bl_point, br_point, tl_point, tr_point))

        if capture:
            plt.clf()
            plt.scatter(close_obstacles[:, 0], close_obstacles[:, 1], s=1)
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('Close Obstacles')
            plt.savefig(f'pcd_images/pcd_{i}.png')
            plt.imsave(f'depth_images/depth_img_{i}.png', depth)

        # Determine gaps
        def find_horizontal_gaps(map_points, threshold):
            gaps = []

            # Sort map_points based on x-coordinate
            sorted_points = map_points[np.argsort(map_points[:, 0])]

            for j in range(len(sorted_points) - 1):
                curr_point = sorted_points[j]
                next_point = sorted_points[j + 1]
                gap_size = next_point[0] - curr_point[0]

                if gap_size > threshold:
                    gap_center = (curr_point[0] + next_point[0]) / 2
                    gaps.append(np.array([gap_center, gap_size]))
            return np.array(gaps)

        gaps = find_horizontal_gaps(close_obstacles, 5)
        if len(gaps) > 0:
            boolean_gaps = np.where(gaps[:, 1] >= gap_threshold)

            if len(boolean_gaps) > 0:
                big_gaps = gaps[boolean_gaps]

                if (len(big_gaps > 0)):
                    optimal_idx = np.argmin(np.abs(big_gaps[:, 0] - 175))
                    optimal_center = big_gaps[optimal_idx, 0]
                    optimal_gap = big_gaps[optimal_idx, 1]

                    print("Optimal Center:", optimal_center)
                    print("Optimal Gap:", optimal_gap)
                    direction = 175 - optimal_center
                else:
                    print(
                        f"No gaps with a size of {gap_threshold} or more found.")
                    direction = -1
            else:
                print(f"No gaps with a size of {gap_threshold} or more found.")
                direction = -1
        else:
            print("No horizontal gaps found.")
            direction = -1

        # Update
        i += 1
        last_process_time = elapsed_time
        print("\n\n")

    # Follow Robot With Camera
    p.resetDebugVisualizerCamera(
        cameraDistance=0.1,
        cameraYaw=math.degrees(h[2]) - 90,
        cameraPitch=10,
        cameraTargetPosition=[
            pos[0] + math.cos(h[2]), pos[1] + math.sin(h[2]), pos[2] + 0.2],
        physicsClientId=0
    )

    # Telemetry
    # print(direction * steering_constant)
