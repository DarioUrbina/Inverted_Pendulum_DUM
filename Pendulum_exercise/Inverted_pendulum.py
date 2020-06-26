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

for i in range(p.getNumJoints(pendulum)):
    info=p.getJointInfo(pendulum, i)
    print(p.getJointInfo(pendulum, i))
    print(info[0])
    print(p.getJointState(pendulum,info[0]))
    #print(info([ info[i] for i in [0,1] ]))

for joint in range(p.getNumJoints(pendulum)):
    p.setJointMotorControl2(pendulum, joint, p.VELOCITY_CONTROL, targetVelocity=10, force=0)
    p.getJointInfo(pendulum, joint)



for i in range (20000):
    p.stepSimulation()
    info=p.getJointInfo(pendulum, 3)
    info2=p.getJointState(pendulum,info[0])
    print(info2[0])
    
    time.sleep(1./240.)
    #p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity=5, force=(activation_profile[i][0])*force_scale)
p.disconnect()


"""

#p.setAdditionalSearchPath(data_dario.getDataPath())
#p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

p.setTimeStep(1./500)
#planeId = p.loadURDF("plane.urdf")

cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
quadruped = p.loadURDF("r2d2_Dario",cubeStartPos, cubeStartOrientation, useFixedBase=1) 

"""
