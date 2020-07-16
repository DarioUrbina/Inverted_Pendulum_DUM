import pybullet as p
import time
import math as m
import numpy as np
import pybullet_data

p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")

cubeStartPos = [-2.15,0,.75]
cubeStartPos2 = [0,0,1.4]
cubeStartPos3 = [2.15,0,.75]

PulleyStartOrientation = p.getQuaternionFromEuler([1.570796, 0, 0])  
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0]) #**
cubeStartOrientation2 = p.getQuaternionFromEuler([0,-1.570796,0])

base_1 = p.loadURDF("Base_2.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
rail = p.loadURDF("Tendons_Cart_Rail.urdf",cubeStartPos2, cubeStartOrientation2, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION) #**
base_2 = p.loadURDF("Base_1.urdf",cubeStartPos3, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)           #**

""""Getting access and information from specific joints in each body (each body has links and joint described in the URDF files):"""
nJoints = p.getNumJoints(base_2)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(base_2, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
Base_pulley1_2 = jointNameToId['Base_pulley1']

nJoints = p.getNumJoints(rail)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(rail, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
last_tendon_link_1 = jointNameToId['tendon1_13_tendon1_14']
cart = jointNameToId['slider_cart']

nJoints = p.getNumJoints(base_1)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(base_1, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]  
last_tendon_link_2 = jointNameToId['tendon1_13_tendon1_14']

#print("LAST TENDON LINK:")
#print(tendon1_4_tendon1_5)

cid2 = p.createConstraint(base_2, Base_pulley1_2, rail, last_tendon_link_1, p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [-.56, 0, 0])

cid4 = p.createConstraint(base_1, last_tendon_link_2, rail, cart, p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0,-.55, 0])


""" #********REFERENCE: ***********
cid = p.createConstraint(quadruped, knee_front_rightR_link, quadruped, knee_front_rightL_link,
                           p.JOINT_POINT2POINT, [0, 0, 0], [0, 0.005, 0.1], [0, 0.01, 0.1])
"""
#cid2 = p.createConstraint(base_1, -1, base_1, 0, p.JOINT_FIXED, [0, 0, 1], [0, 0, 1], [0, 0, 1]

"""p.changeConstraint(cid, [1,1,0], [1,0,0], maxForce=50)"""
#p.changeConstraint(cid, pivot, jointChildFrameOrientation=orn, maxForce=50)
p.setGravity(0,0,-10)



#for i in range(p.getNumJoints(base_1)):
"""If more links or joints are added, the joint indexes change"""
#    info=p.getJointInfo(base_1, i)
#    print(p.getJointInfo(base_1, i))
#    print('Hello')
    #print(info[0])    #print(p.getJointState(pendulum,info[0]))    #print(info([ info[i] for i in [0,1] ]))

for i in range (20000):
    p.stepSimulation()
    #p.changeConstraint(cid, [0,0,0], [1,0,0], maxForce=50)

    time.sleep(1./240.)
    
p.disconnect()



