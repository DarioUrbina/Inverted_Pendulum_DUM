import pybullet as p
import time
import math as m
import numpy as np
import pybullet_data
#so far revised
p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")

cubeStartPos = [-2.15,0,.5]
cubeStartPos2 = [0,0,.9]
cubeStartPos3 = [2.15,0,.5]
PulleyStartPos = [-2.15,.21,.65]


PulleyStartOrientation = p.getQuaternionFromEuler([1.570796, 0, 0])  
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
cubeStartOrientation2 = p.getQuaternionFromEuler([0,1.570796,0])

base_1 = p.loadURDF("Base_1.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
rail = p.loadURDF("Rail.urdf",cubeStartPos2, cubeStartOrientation2, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
base_2 = p.loadURDF("Base_1.urdf",cubeStartPos3, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
#moving_mechanism = p.loadURDF("Cart_tendons_and_pulleys.urdf",cubeStartPos4, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
#Pulley_1 = p.loadURDF("Pulley.urdf",PulleyStartPos, PulleyStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)



#base_1 = p.loadURDF("Cyclic_body_chain.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)

cid = p.createConstraint(base_1, -1, base_1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
#cid2 = p.createConstraint(Pulley_1, -1, base_1, -1, p.JOINT_REVOLUTE, [0, 0, 0], [0, 0, 0], [0, 0, 0])
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



