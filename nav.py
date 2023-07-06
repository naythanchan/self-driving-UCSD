import os, inspect
import pybullet as p
import pybullet_data
import math
import time

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
parentdir = os.path.join(currentdir, "../gym")

os.sys.path.insert(0, parentdir)



cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -10)

useRealTimeSim = 1

#for video recording (works best on Mac and Linux, not well on Windows)
#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "racecar.mp4")
p.setRealTimeSimulation(useRealTimeSim)  # either this
#p.loadURDF("plane.urdf")
p.loadSDF(os.path.join(pybullet_data.getDataPath(), "stadium.sdf"))

car = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "racecar/racecar.urdf"))
for i in range(p.getNumJoints(car)):
  print(p.getJointInfo(car, i))

inactive_wheels = [5, 7]
wheels = [2, 3]

for wheel in inactive_wheels:
  p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

steering = [4, 6]

#targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0) #-10 to 10, start at 0
#maxForceSlider = p.addUserDebugParameter("maxForce", 0, 20, 20) #0 to 10, start at 10
#steeringSlider = p.addUserDebugParameter("steering", -0.5, 0.5, 0) #-0.5 to 0.5, start at 0


#BOX IMPLEMENTATION 
box_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5])

# Create the box body
box_body = p.createMultiBody(
    baseMass=1.0,  # Mass of the box
    baseCollisionShapeIndex=box_shape,  # Corrected argument name
    basePosition=[7, 7, 1],  # Initial position of the box
)

def moveTo(targetX, targetY):
  pos, hquat = p.getBasePositionAndOrientation(car)
  h = p.getEulerFromQuaternion(hquat)
  x = pos[0]
  y = pos[1]
  distance = math.sqrt((targetX - x)**2 + (targetY - y)**2)
  theta = math.atan2((targetY - y), (targetX - x))
  while distance > 1:
    pos, hquat = p.getBasePositionAndOrientation(car)
    h = p.getEulerFromQuaternion(hquat)
    x = pos[0]
    y = pos[1]
    distance = math.sqrt((targetX - x)**2 + (targetY - y)**2)
    theta = math.atan2((targetY - y), (targetX - x))
    maxForce = 20
    # targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
    targetVelocity = 5*distance
    #steeringAngle = p.readUserDebugParameter(steeringSlider)


    
    steeringAngle = theta - h[2]


    # FIXED ANGLE ISSUE
    if steeringAngle > (math.pi / 2) or steeringAngle < -(math.pi / 2):
       steeringAngle = h[2] - theta
    else:
       steeringAngle = theta - h[2]

    for wheel in wheels:
        p.setJointMotorControl2(car,
                            wheel,
                            p.VELOCITY_CONTROL,
                            targetVelocity=targetVelocity,
                            force=maxForce)

    for steer in steering:
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)

    steering
    if (useRealTimeSim == 0):
        p.stepSimulation()
    time.sleep(0.01)

    # print(h[2] / math.pi, theta / math.pi)
    print(steeringAngle)

points = [(-10, 0)]

for i in points:
   moveTo(i[0], i[1])
   

