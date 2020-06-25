import pybullet as p
import time
import math as m
import numpy as np

p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")

cubeStartPos = [-2.65,0,.4]

cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#pendulum = p.loadURDF("r2d2_Dario.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1)
pendulum = p.loadURDF("Pendulum.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1)
p.setGravity(0,0,-10)

for joint in range(p.getNumJoints(pendulum)):
    p.setJointMotorControl2(pendulum, joint, p.VELOCITY_CONTROL, targetVelocity=10, force=0)
    p.getJointInfo(pendulum, joint)



for i in range (20000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()


"""

#p.setAdditionalSearchPath(data_dario.getDataPath())
#p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

p.setTimeStep(1./500)
#planeId = p.loadURDF("plane.urdf")

cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
quadruped = p.loadURDF("r2d2_Dario",cubeStartPos, cubeStartOrientation, useFixedBase=1) 

"""
