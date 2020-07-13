import pybullet as p
import time
import math as m
import numpy as np
import pybullet_data

p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")

cubeStartPos = [-2.15,0,.5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

base_1 = p.loadURDF("Base_1.urdf",cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_SELF_COLLISION)
base_2 = p.loadURDF("Base_2.urdf",cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_SELF_COLLISION)
#base_1 = p.loadURDF("Cyclic_body_chain.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)

cid = p.createConstraint(base_1, -1, base_1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 1], [0, 0, 1])
cid2 = p.createConstraint(base_2, -1, base_2, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
#cid2 = p.createConstraint(base_1, -1, base_1, 0, p.JOINT_FIXED, [0, 0, 1], [0, 0, 1], [0, 0, 1]

p.changeConstraint(cid, [1,1,0], [1,0,0], maxForce=50)
#p.changeConstraint(cid, pivot, jointChildFrameOrientation=orn, maxForce=50)
p.setGravity(0,0,-10)



for i in range(p.getNumJoints(base_1)):
    """If more links or joints are added, the joint indexes change"""
    info=p.getJointInfo(base_1, i)
    print(p.getJointInfo(base_1, i))
    print('Hello')
    #print(info[0])    #print(p.getJointState(pendulum,info[0]))    #print(info([ info[i] for i in [0,1] ]))

for i in range (20000):
    p.stepSimulation()
    p.changeConstraint(cid, [0,0,0], [1,0,0], maxForce=50)

    time.sleep(1./240.)
    
p.disconnect()



