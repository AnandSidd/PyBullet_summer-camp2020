
import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

planeId = p.loadURDF("plane.urdf")
cubeStartPos1 = [2,2,1]
cubeStartPos2 = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
sampleId = p.loadURDF("sample.urdf",cubeStartPos1, cubeStartOrientation)
dabbaId=p.loadURDF("dabba.urdf",cubeStartPos2,cubeStartOrientation)
i=0
while True:
    p.setGravity(0.707*i,0.707*i,0)    
    p.stepSimulation()
    time.sleep(1./490.)
    i=i+0.02
    if(i>=9.8):
        i=0    
cubePos2, cubeOrn2 = p.getBasePositionAndOrientation(dabbaId)            
cubePos1, cubeOrn1 = p.getBasePositionAndOrientation(sampleId)
print(cubePos2,cubeOrn2)
print(cubePos1,cubeOrn1)
p.disconnect()
