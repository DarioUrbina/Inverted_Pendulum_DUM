import pybullet as p
import time
import math as m
import numpy as np
import pybullet_data

p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")

"""_____________________________________________________________________________________________________________________________"""
"""Gains and motor forces"""
motorForce=700
proportional_gain = -1500
integral_gain = -1000
derivative_gain = 3000

previous_pendulum_angle = 0
previous_cart_position = 0

"""_____________________________________________________________________________________________________________________________"""
cubeStartPos = [-2.15,0,.75]
cubeStartPos2 = [0,0,1.4]
cubeStartPos3 = [2.15,0,.75]

PulleyStartOrientation = p.getQuaternionFromEuler([1.570796, 0, 0])  
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0]) 
cubeStartOrientation2 = p.getQuaternionFromEuler([0,-1.570796,0])

base_2 = p.loadURDF("Base_2.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
pendulum = p.loadURDF("Pendulum_Tendon_1_Cart_Rail.urdf",cubeStartPos2, cubeStartOrientation2, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION) 
base_1 = p.loadURDF("Base_1.urdf",cubeStartPos3, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)


"""_____________________________________________________________________________________________________________________________"""
"""Getting access and information from specific joints in each body (each body has links and joint described in the URDF files):"""
nJoints = p.getNumJoints(base_1)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(base_1, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
Base_pulley_1 = jointNameToId['Base_pulley1']

nJoints = p.getNumJoints(pendulum)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(pendulum, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
last_tendon_link_1 = jointNameToId['tendon1_13_tendon1_14']
cart_pendulumAxis = jointNameToId['cart_pendulumAxis']
cart = jointNameToId['slider_cart']

nJoints = p.getNumJoints(base_2)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(base_2, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
last_tendon_link_2 = jointNameToId['tendon1_13_tendon1_14']
Base_pulley_2 = jointNameToId['Base_pulley1']
"""_____________________________________________________________________________________________________________________________"""
"""Creating new contraints (joints), with the information obtained in the previous step"""

pulley_1_tendon_magenta = p.createConstraint(base_1, Base_pulley_1, pendulum, last_tendon_link_1, p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [-.56, 0, 0])
tendon_white_cart = p.createConstraint(base_2, last_tendon_link_2, pendulum, cart, p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0,-.55, 0])

"""_____________________________________________________________________________________________________________________________"""
"""Defining some motor conditions"""
p.setJointMotorControl2(pendulum, cart_pendulumAxis, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

#FROM REFERENCE: p.changeConstraint(cid, [1,1,0], [1,0,0], maxForce=50)
#p.changeConstraint(cid, pivot, jointChildFrameOrientation=orn, maxForce=50)

#p.changeConstraint(pulley_1_tendon_magenta, [0,0,0], [0,0,0], maxForce=100)
#p.changeConstraint(pulley_1_tendon_magenta, [0,0,0], [0,0,0], maxForce=100)


"""_____________________________________________________________________________________________________________________________"""


p.setGravity(0,0,-10)
p.setJointMotorControl2(base_1, Base_pulley_1, p.VELOCITY_CONTROL, targetVelocity=10, force=1000)
p.setJointMotorControl2(base_2, Base_pulley_2, p.VELOCITY_CONTROL, targetVelocity=10, force=-1000)
    

for i in range (20000):
    p.stepSimulation()
    pendulum_angle = p.getJointState(pendulum,cart_pendulumAxis)
    pendulum_angle = pendulum_angle[0]
    print(pendulum_angle)

    angle_delta_error = -pendulum_angle

    #PROPPORTIONAL
    p_correction = proportional_gain * pendulum_angle

    #INTEGRAL
    i_correction = integral_gain * (previous_pendulum_angle + pendulum_angle)
    previous_pendulum_angle = pendulum_angle

    #DERIVATIVE
    d_correction = derivative_gain * angle_delta_error

    u = p_correction + i_correction + d_correction + 10
    print(u)
    u = u*.2
    if u<4000:
      u=4000
    print(u)
    
    #p.changeConstraint(cid, [0,0,0], [1,0,0], maxForce=50)
    if pendulum_angle > 0:
      p.setJointMotorControl2(base_1, Base_pulley_1, p.VELOCITY_CONTROL, targetVelocity=100, force=u)
      p.setJointMotorControl2(base_2, Base_pulley_2, p.VELOCITY_CONTROL, targetVelocity=100, force=1000)
      print(">0")
    else:
      p.setJointMotorControl2(base_1, Base_pulley_1, p.VELOCITY_CONTROL, targetVelocity=100, force=1000)
      p.setJointMotorControl2(base_2, Base_pulley_2, p.VELOCITY_CONTROL, targetVelocity=100, force=-u)
      print("<0")
    

    time.sleep(1./240.)

    
p.disconnect()



