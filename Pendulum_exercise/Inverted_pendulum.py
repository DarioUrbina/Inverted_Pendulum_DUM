import pybullet as p
import time

p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")

cubeStartPos = [0,0,.05]

cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
quadruped = p.loadURDF("Pendulum.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1)


"""

#import data_dario
#import pybullet_data
#import time

import math as m
import numpy as np


#p.setAdditionalSearchPath(data_dario.getDataPath())
#p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
p.setTimeStep(1./500)
#planeId = p.loadURDF("plane.urdf")

cubeStartPos = [0,0,.05]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
quadruped = p.loadURDF("r2d2_Dario",cubeStartPos, cubeStartOrientation, useFixedBase=1) 

"""
