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
p.loadURDF("plane.urdf")
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


# Load the cubes randomly within the defined zone
num_cubes = 10  # Number of cubes to load
box_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.4, 0.4, 0.4])

for _ in range(num_cubes):
    cube_position = generate_random_position()
    box_body = p.createMultiBody(
        baseMass=1.0,
        baseCollisionShapeIndex=box_shape,
        basePosition=cube_position,
    )

# Loop
target = [5, 5]
steering = [4, 6]
wheels = [2, 3]
target_pos = 0

start_time = time.time()
flag = True

while (True):
    elapsed_time = time.time() - start_time
    # Enable rendering in the debug visualizer
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    # Get the car's camera properties
    cam_info = p.getDebugVisualizerCamera()
    view_matrix = cam_info[2]
    proj_matrix = cam_info[3]

    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
        256, 256, viewMatrix=view_matrix, projectionMatrix=proj_matrix
    )
    depth = np.array(depth_img)
    reshaped_depth_img = np.reshape(depth_img, (256, 256))

    cut_depth = reshaped_depth_img[100:-80, :]
    left_cut = cut_depth[:, :85]
    middle_cut = cut_depth[:, 85:176]
    right_cut = cut_depth[:, 176:255]

    left_mean = np.mean(left_cut)
    middle_mean = np.mean(middle_cut)
    right_mean = np.mean(right_cut)
    results = {'left_mean': left_mean,
               'middle_mean': middle_mean, 'right_mean': right_mean}

    # Find the variable with the maximum value
    max_variable = max(results, key=results.get)

    # view = np.array(view_matrix)
    # reshaped_view = np.reshape(view, (4, 4))

    # proj = np.array(proj_matrix)
    # reshaped_proj = np.reshape(proj, (4, 4))

    if flag and elapsed_time >= (random.uniform(1.5, 3.5)):
        np.set_printoptions(threshold=np.inf)
        data = {
            'depth_img': cut_depth.tolist()
            # 'view_matrix': view_matrix,
            # 'proj_matrix': proj_matrix
        }
        with open('depth_output.json', 'w') as f:
            json.dump(data, f)
        plt.imsave(f'depth_images/depth_img_{1}.png', cut_depth)
        plt.imsave(f'depth_images/left_cut_depth_img_{1}.png', left_cut)
        plt.imsave(f'depth_images/middle_cut_depth_img_{1}.png', middle_cut)
        plt.imsave(f'depth_images/right_cut_depth_img_{1}.png', right_cut)
        print(left_mean)
        print(middle_mean)
        print(right_mean)
        print(max_variable)
        flag = False

    # Driving
    for wheel in wheels:
        p.setJointMotorControl2(car,
                                wheel,
                                p.VELOCITY_CONTROL,
                                targetVelocity=25,
                                force=2)
    pos, hquat = p.getBasePositionAndOrientation(car)
    h = p.getEulerFromQuaternion(hquat)
    x = pos[0]
    y = pos[1]
    distance = math.sqrt((target[0] - x)**2 + (target[1] - y)**2)

    max_mean = max(left_mean, middle_mean, right_mean)
    correction = 5
    if max_mean == middle_mean:
        target_pos = math.radians(0)
    elif max_mean == left_mean:
        target_pos += math.radians(correction)
    else:
        target_pos -= math.radians(correction)

    for steer in steering:
        p.setJointMotorControl2(
            car, steer, p.POSITION_CONTROL, targetPosition=target_pos)

    p.resetDebugVisualizerCamera(
        cameraDistance=0.1,
        cameraYaw=math.degrees(h[2]) - 90,
        cameraPitch=10,
        cameraTargetPosition=[
            pos[0] + math.cos(h[2]), pos[1] + math.sin(h[2]), pos[2] + 0.2],
        physicsClientId=0
    )


