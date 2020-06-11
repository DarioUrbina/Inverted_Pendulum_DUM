import pybullet as p
import time
import pybullet_data

#$#$#
import math as m
import numpy as np

motors=8
power_blocks=100
steps=100
rows= steps*power_blocks
activation_profile = np.zeros(shape=(rows,motors))
line_count=0
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
                activation_profile[line_count][mot]=m.sin(i)
            else:
                activation_profile[line_count][mot]=m.sin(i+3.14)
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


for i in range(p.getNumJoints(boxId)):
  print(p.getJointInfo(boxId, i))

#for joint in range(p.getNumJoints(boxId)):
  #p.setJointMotorControl2(boxId, joint, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
  ##p.getJointInfo(boxId, joint)

for i in range (rows):
    p.stepSimulation()
    time.sleep(1./240.)
    for ii in range (1,9):
      #print (ii*2)
      p.setJointMotorControl2(boxId, ii*2, p.VELOCITY_CONTROL, targetVelocity=5, force=(activation_profile[i][ii-1])*500)
    
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()

