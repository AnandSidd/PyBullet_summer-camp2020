import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")
cubeStartPos = [2,2,1]
cubeStartPos1 = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("sample.urdf",cubeStartPos, cubeStartOrientation)
boxId1=p.loadURDF("dabba.urdf",cubeStartPos1, cubeStartOrientation)
i=0
while True:
  p.setGravity(0.707*i,0.707*i,0)
  p.stepSimulation()
  time.sleep(1./240.)
  i=i+1
  if(i>=9.8):
    i=0
  cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
  cubePos1, cubeOrn1 = p.getBasePositionAndOrientation(boxId1)

  print(cubePos,cubeOrn)
  print(cubePos1,cubeOrn1)
p.disconnect()
