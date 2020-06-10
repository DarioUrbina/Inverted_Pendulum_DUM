import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,.05]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#With fixed base (Can I do it fixed in a certain plane?)
boxId = p.loadURDF("r2d2_Dario.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1) 
#boxId = p.loadURDF("r2d2_Dario.urdf",cubeStartPos, cubeStartOrientation)

for i in range(p.getNumJoints(boxId)):
  print(p.getJointInfo(boxId, i))

for joint in range(p.getNumJoints(boxId)):
  p.setJointMotorControl2(boxId, joint, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
  #p.getJointInfo(boxId, joint)

for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
    
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
