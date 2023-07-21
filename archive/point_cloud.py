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

# Loop
steering = [4, 6]
wheels = [2, 3]
target_pos = 0

start_time = time.time()
flag = True

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

    # Process Depth Images
    depth = np.array(depth_img)
    reshaped_depth_img = np.reshape(depth_img, (256, 256))
    cut_depth = reshaped_depth_img[100:-80, :]
    left_cut = cut_depth[:, :85]
    middle_cut = cut_depth[:, 85:176]
    right_cut = cut_depth[:, 176:255]

    # Analyze Depth Images
    left_mean = np.mean(left_cut)
    middle_mean = np.mean(middle_cut)
    right_mean = np.mean(right_cut)
    results = {'left_mean': left_mean,
               'middle_mean': middle_mean, 'right_mean': right_mean}
    target_cut = max(results, key=results.get)

    # Take Random Snapshot
    if flag and elapsed_time >= (random.uniform(1.5, 3.5)):
        np.set_printoptions(threshold=np.inf)
        data = {
            'depth_img': reshaped_depth_img.tolist(),
            'view_matrix': view_matrix,
            'proj_matrix': proj_matrix
        }
        with open('depth_output.json', 'w') as f:
            json.dump(data, f)

        # Save Images
        plt.imsave(f'depth_images/depth_img_{1}.png', reshaped_depth_img)
        plt.imsave(f'depth_images/left_cut_depth_img_{1}.png', left_cut)
        plt.imsave(f'depth_images/middle_cut_depth_img_{1}.png', middle_cut)
        plt.imsave(f'depth_images/right_cut_depth_img_{1}.png', right_cut)

        # Print
        print(f"Left: {left_mean}\nMiddle: {middle_mean}\nRight: {right_mean}")
        print(target_cut)
        flag = False
        break

    # Driving
    for wheel in wheels:
        p.setJointMotorControl2(car,
                                wheel,
                                p.VELOCITY_CONTROL,
                                targetVelocity=25,
                                force=2)
    
    # Localization
    pos, hquat = p.getBasePositionAndOrientation(car)
    h = p.getEulerFromQuaternion(hquat)
    x = pos[0]
    y = pos[1]

    correction = 10
    if target_cut == "middle_mean":
        target_pos = math.radians(0)
    elif target_cut == "left_mean":
        target_pos = math.radians(correction)
    else:
        target_pos = -math.radians(correction)

    for steer in steering:
        p.setJointMotorControl2(
            car, steer, p.POSITION_CONTROL, targetPosition=target_pos)

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
    print(target_cut)
    print(target_pos)

