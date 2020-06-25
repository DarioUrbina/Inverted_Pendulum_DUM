#Things to consider:
#Motors can be activated with different force levels; but considering that they go through activation cycles, the limb present "tremour" when
#moving if the force is too big. THis might be die to leg inertia.

import pybullet as p
import time
import pybullet_data

#$#$#
import math as m
import numpy as np

motors=8
force_scale = 80
power_blocks=100
steps=100
rows= steps*power_blocks
activation_profile = np.zeros(shape=(rows,motors))
line_count=0
left_motors=[2,3,4,5] #motor numbers in xml are differnet, but in python arrays they are treated like this

"""
for i in range (steps):
    for j in range(power_blocks):
        for mot in range(motors):
            activation_profile[line_count][mot]=m.sin(i)
        line_count=line_count+1
"""

for i in range (steps):
    for j in range(power_blocks):
        for mot in range(motors):
            if (mot % 2) == 0:
                #print ("a")
                #if mot == 2:
                #    activation = m.cos(i)
                #if mot == 4:
                #    activation = m.cos(i)
                #else:
                activation = (m.sin(i))
                if activation < 0:
                    if (activation) > -.7:
                        activation = -.7
                if activation > 0:
                    if (activation) > .7:
                        activation = .7
                activation_profile[line_count][mot]=activation
            else:
                #print ("b")
                #if mot == 3:
                    #print("hhhhhhhhhhhhhhhhhhhhhh")
                #    activation = m.sin(i)
                #if mot == 5:
                    #print("iiiiiiiiiiiiiiiiiiiii")
                #    activation = m.sin(i)
                #else:
                    #print("jjjjjjjjjjjjjjjjjjjjj")
                    #activation = m.cos(i)
                activation = (m.sin(i+(3.14/2)))
                if activation < 0:
                    if (activation) > -.7:
                        activation = -.7
                if activation > 0:
                    if (activation) > .7:
                        activation = .7
                activation_profile[line_count][mot]=activation
            #print (p)
        line_count=line_count+1

#print (activation_profile)        
#$#$#

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,.05]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2_Dario.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1) 


#for i in range(p.getNumJoints(boxId)):
  #print(p.getJointInfo(boxId, i))

#for joint in range(p.getNumJoints(boxId)):
  #p.setJointMotorControl2(boxId, joint, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
  ##p.getJointInfo(boxId, joint)

for i in range (rows):
    p.stepSimulation()
    time.sleep(1./240.)
    """for ii in range (1,9):
      #print (ii*2)
      p.setJointMotorControl2(boxId, ii*2, p.VELOCITY_CONTROL, targetVelocity=5, force=(activation_profile[i][ii-1])*500)"""
    p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity=5, force=(activation_profile[i][0])*force_scale)
    p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, targetVelocity=5, force=(activation_profile[i][1])*force_scale)
    #left starts:
    p.setJointMotorControl2(boxId, 6, p.VELOCITY_CONTROL, targetVelocity=5, force=(activation_profile[i][2])*force_scale)
    p.setJointMotorControl2(boxId, 8, p.VELOCITY_CONTROL, targetVelocity=5, force=(activation_profile[i][3])*force_scale)
    p.setJointMotorControl2(boxId, 10, p.VELOCITY_CONTROL, targetVelocity=5, force=(activation_profile[i][4])*force_scale)
    p.setJointMotorControl2(boxId, 12, p.VELOCITY_CONTROL, targetVelocity=5, force=(activation_profile[i][5])*force_scale)
    #left finishes
    p.setJointMotorControl2(boxId, 14, p.VELOCITY_CONTROL, targetVelocity=5, force=(activation_profile[i][6])*force_scale)
    p.setJointMotorControl2(boxId, 16, p.VELOCITY_CONTROL, targetVelocity=5, force=(activation_profile[i][7])*force_scale)
      
    
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()

