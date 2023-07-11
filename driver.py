import pybullet as p
import time
import pybullet_data
import math
import random

# Physics
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.resetSimulation()
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)  # either this

# Camera
p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=-90, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

# Path
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Assets
p.loadURDF("plane.urdf")
car = p.loadURDF("racecar/racecar.urdf")
cube = p.loadURDF("cube.urdf", basePosition=[0,3,0.5], useFixedBase=1)


# Obstacles
zone_width = 6  # meters
zone_depth = 10  # meters

# Define the race position
race_position = [0, 0, 0]  # [x, y, z]

# Generate a random position within the zone
def generate_random_position():
    x = random.uniform(race_position[0], race_position[0] + zone_depth)
    y = random.uniform(race_position[1] - zone_width/2, race_position[1] + zone_width/2)
    z = race_position[2] + 0.5  # Height of the cube
    return [x, y, z]

# Load the cubes randomly within the defined zone
num_cubes = 10  # Number of cubes to load
for _ in range(num_cubes):
    cube_position = generate_random_position()
    cube = p.loadURDF("cube.urdf", basePosition=cube_position, useFixedBase=0, globalScaling=0.5)

# Driving
target = [5, 5]
steering = [4, 6]
wheels = [2, 3]

start_time = time.time()
while (True):                      
    for wheel in wheels:
        p.setJointMotorControl2(car,
                                wheel,
                                p.VELOCITY_CONTROL,
                                targetVelocity=30,
                                force=1)
    pos, hquat = p.getBasePositionAndOrientation(car)
    h = p.getEulerFromQuaternion(hquat)
    x = pos[0]
    y = pos[1]
    distance = math.sqrt((target[0] - x)**2 + (target[1] - y)**2)
    
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=-90, cameraPitch=-30, cameraTargetPosition=[x, y, 0])

    current_time = time.time()
    if current_time - start_time >= 0.3:
        print(distance)
        start_time = current_time