import pybullet as p
import time
import math as m
import numpy as np
import pybullet_data
#so far revised
p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")

cubeStartPos = [-2.15,0,.75]
cubeStartPos2 = [0,0,1.4]
cubeStartPos3 = [2.15,0,.75]

PulleyStartOrientation = p.getQuaternionFromEuler([1.570796, 0, 0])  
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
cubeStartOrientation2 = p.getQuaternionFromEuler([0,-1.570796,0])
#cubeStartOrientation2

base_1 = p.loadURDF("Base_1.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
rail = p.loadURDF("Tendons_Cart_Rail.urdf",cubeStartPos2, cubeStartOrientation2, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
base_2 = p.loadURDF("Base_1.urdf",cubeStartPos3, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
"""cart = p.loadURDF("Cart.urdf",flags=p.URDF_USE_SELF_COLLISION)"""


nJoints = p.getNumJoints(base_2)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(base_2, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
Base_pulley1_2 = jointNameToId['Base_pulley1']

nJoints3 = p.getNumJoints(rail)
jointNameToId = {}
for i in range(nJoints3):
  jointInfo3 = p.getJointInfo(rail, i)
  jointNameToId[jointInfo3[1].decode('UTF-8')] = jointInfo3[0]
last_tendon_link_1 = jointNameToId['tendon1_13_tendon1_14']

last_tendon_link_2 = jointNameToId['tendon2_13_tendon2_14']



nJoints2 = p.getNumJoints(base_1)
jointNameToId = {}
for i in range(nJoints2):
  jointInfo2 = p.getJointInfo(base_1, i)
  jointNameToId[jointInfo2[1].decode('UTF-8')] = jointInfo2[0]  
Base_pulley1_1 = jointNameToId['Base_pulley1']

"""nJoints4 = p.getNumJoints(rail)
jointNameToId = {}
for i in range(nJoints4):
  jointInfo4 = p.getJointInfo(rail, i)
  jointNameToId[jointInfo3[1].decode('UTF-8')] = jointInfo4[0]
last_tendon_link_2 = jointNameToId['tendon2_13_tendon2_14']
"""





""" #********REFERENCE: ***********
knee_front_rightL_link = jointNameToId['knee_front_rightL_link']

p.setJointMotorControl2(quadruped, knee_front_rightL_link, p.POSITION_CONTROL,
                            motordir[4] * (kneeangle) * float(aa) / steps)

p.resetJointState(quadruped, knee_front_rightL_link, motordir[4] * kneeangle)

cid = p.createConstraint(quadruped, knee_front_rightR_link, quadruped, knee_front_rightL_link,
                           p.JOINT_POINT2POINT, [0, 0, 0], [0, 0.005, 0.1], [0, 0.01, 0.1])

if(1):
    p.setJointMotorControl(quadruped, knee_front_rightL_link, p.VELOCITY_CONTROL, 0,
                         kneeFrictionForce)
                         
"""


#tendon1_4_tendon1_5 = jointNameToId['tendon1_4_tendon1_5']
#tendon1_4_tendon1_5_2 = jointNameToId['tendon1_4_tendon1_5']

#print("LAST TENDON LINK:")
#print(tendon1_4_tendon1_5)


#moving_mechanism = p.loadURDF("Cart_tendons_and_pulleys.urdf",cubeStartPos4, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)


#cid = p.createConstraint(base_1, -1, base_1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])

#WORKING WELL:
cid2 = p.createConstraint(base_2, Base_pulley1_2, rail, last_tendon_link_1, p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [-.56, 0, 0])

cid4 = p.createConstraint(base_1, Base_pulley1_1, rail, last_tendon_link_2, p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [-.56, 0, 0])
"""
cid3 = p.createConstraint(base_2, tendon1_4_tendon1_5_2, cart, -1, p.JOINT_FIXED, [0, 0, 0], [.45, .05, 0], [.1, .05, 0])
"""

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



