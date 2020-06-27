import pybullet as p
import time
import math as m
import numpy as np

p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")

cubeStartPos = [-2.65,0,.4]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#pendulum = p.loadURDF("r2d2_Dario.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1)
pendulum = p.loadURDF("Pendulum_for_cart_actuation.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1)
p.setGravity(0,0,-10)

motorForce=700

proportional_gain = -1500
integral_gain = -1000
derivative_gain = 3000

previous_pendulum_angle = 0

for i in range(p.getNumJoints(pendulum)):
    info=p.getJointInfo(pendulum, i)
    print(p.getJointInfo(pendulum, i))
    print(info[0])
    print(p.getJointState(pendulum,info[0]))
    #print(info([ info[i] for i in [0,1] ]))

for joint in range(p.getNumJoints(pendulum)):
    p.setJointMotorControl2(pendulum, joint, p.VELOCITY_CONTROL, targetVelocity=10, force=0)
    p.getJointInfo(pendulum, joint)

p.changeDynamics(pendulum, linkIndex=1 , mass=100, restitution=.1)


for i in range (20000):
    p.stepSimulation()
    #info=p.getJointInfo(pendulum, 3)
    pendulum_angle=p.getJointState(pendulum,3)
    print(pendulum_angle[0])
    delta_error = 0-pendulum_angle[0]
    print("delta_Error:")
    print(delta_error)

    #PROPPORTIONAL
    p_correction = proportional_gain * pendulum_angle[0]

    #INTEGRAL
    i_correction = integral_gain * (previous_pendulum_angle + pendulum_angle[0])
    previous_pendulum_angle = pendulum_angle[0]

    #DERIVATIVE
    d_correction = derivative_gain * delta_error
    
    #Input Signal
    u = p_correction + i_correction + d_correction 
    
    p.setJointMotorControl2(pendulum, 2, p.VELOCITY_CONTROL, targetVelocity=10, force=u)
        
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
