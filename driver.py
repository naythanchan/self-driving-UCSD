import pybullet as p
import time
import pybullet_data
import math

# Physics
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.resetSimulation()
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(1)  # either this

# Path
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Assets
p.loadURDF("plane.urdf")
car = p.loadURDF("racecar/racecar.urdf")

# Joints
for i in range(p.getNumJoints(car)):
    print(p.getJointInfo(car, i))

# Set Joint Arrays
steering = [4, 6]
wheels = [2, 3]

# Driving
p.resetBasePositionAndOrientation(car, [0,0,0], [0,0,0,1])
target = [5, 5]

start_time = time.time()
while (True):
    for wheel in wheels:
        p.setJointMotorControl2(car,
                                wheel,
                                p.VELOCITY_CONTROL,
                                targetVelocity=5,
                                force=10)
    pos, hquat = p.getBasePositionAndOrientation(car)
    h = p.getEulerFromQuaternion(hquat)
    x = pos[0]
    y = pos[1]
    distance = math.sqrt((target[0] - x)**2 + (target[1] - y)**2)
    
    current_time = time.time()
    if current_time - start_time >= 0.3:
        print(distance)
        start_time = current_time

