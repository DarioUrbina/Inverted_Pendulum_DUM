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
    """
    if i<300:
      p.setJointMotorControl2(boxId, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
      p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, targetVelocity=100, force=-10)
      p.setJointMotorControl2(boxId, 6, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 8, p.VELOCITY_CONTROL, targetVelocity=100, force= -10)
      p.setJointMotorControl2(boxId, 10, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 12, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 14, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 16, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      print("b")
    if 300<i<500:
      p.setJointMotorControl2(boxId, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
      p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity=100, force=-100)
      p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, targetVelocity=100, force=10)
      p.setJointMotorControl2(boxId, 6, p.VELOCITY_CONTROL, targetVelocity=100, force=-100)
      p.setJointMotorControl2(boxId, 8, p.VELOCITY_CONTROL, targetVelocity=100, force= 10)
      p.setJointMotorControl2(boxId, 10, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 12, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 14, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 16, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      print("a")
    if 500<i<800:
      p.setJointMotorControl2(boxId, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
      p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity=10, force=100)
      p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, targetVelocity=100, force=-10)
      p.setJointMotorControl2(boxId, 6, p.VELOCITY_CONTROL, targetVelocity=10, force=100)
      p.setJointMotorControl2(boxId, 8, p.VELOCITY_CONTROL, targetVelocity=100, force= -10)
      p.setJointMotorControl2(boxId, 10, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 12, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 14, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 16, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      print("a")
      
    if 800<i<1000:
      p.setJointMotorControl2(boxId, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
      p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, targetVelocity=100, force=-10)
      p.setJointMotorControl2(boxId, 6, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 8, p.VELOCITY_CONTROL, targetVelocity=100, force= -10)
      p.setJointMotorControl2(boxId, 10, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 12, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 14, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 16, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      print("b")
    if 1100<i<1400:
      p.setJointMotorControl2(boxId, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
      p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity=100, force=-100)
      p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, targetVelocity=100, force=10)
      p.setJointMotorControl2(boxId, 6, p.VELOCITY_CONTROL, targetVelocity=100, force=-100)
      p.setJointMotorControl2(boxId, 8, p.VELOCITY_CONTROL, targetVelocity=100, force= 10)
      p.setJointMotorControl2(boxId, 10, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 12, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 14, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 16, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      print("a")
    if 1400<i<1700:
      p.setJointMotorControl2(boxId, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
      p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity=10, force=100)
      p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, targetVelocity=100, force=-10)
      p.setJointMotorControl2(boxId, 6, p.VELOCITY_CONTROL, targetVelocity=10, force=100)
      p.setJointMotorControl2(boxId, 8, p.VELOCITY_CONTROL, targetVelocity=100, force= -10)
      p.setJointMotorControl2(boxId, 10, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 12, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 14, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 16, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      print("a")
      
    if 1700<i<2000:
      p.setJointMotorControl2(boxId, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
      p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, targetVelocity=100, force=-10)
      p.setJointMotorControl2(boxId, 6, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 8, p.VELOCITY_CONTROL, targetVelocity=100, force= -10)
      p.setJointMotorControl2(boxId, 10, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 12, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 14, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 16, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      print("b")
    if 2000<i<2300:
      p.setJointMotorControl2(boxId, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
      p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity=100, force=-100)
      p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, targetVelocity=100, force=10)
      p.setJointMotorControl2(boxId, 6, p.VELOCITY_CONTROL, targetVelocity=100, force=-100)
      p.setJointMotorControl2(boxId, 8, p.VELOCITY_CONTROL, targetVelocity=100, force= 10)
      p.setJointMotorControl2(boxId, 10, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 12, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 14, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 16, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      print("a")
    else:
      p.setJointMotorControl2(boxId, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
      p.setJointMotorControl2(boxId, 2, p.VELOCITY_CONTROL, targetVelocity=10, force=100)
      p.setJointMotorControl2(boxId, 4, p.VELOCITY_CONTROL, targetVelocity=100, force=-10)
      p.setJointMotorControl2(boxId, 6, p.VELOCITY_CONTROL, targetVelocity=10, force=100)
      p.setJointMotorControl2(boxId, 8, p.VELOCITY_CONTROL, targetVelocity=100, force= -10)
      p.setJointMotorControl2(boxId, 10, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 12, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 14, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      p.setJointMotorControl2(boxId, 16, p.VELOCITY_CONTROL, targetVelocity=10, force=10)
      print("a")
    print (i)
    """
    
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
