import pybullet as p
import time
import pybullet_data
import os
''''
press n release 'r' to apply external force and start the simulation

'''
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
cameraDistance = 1
cameraYaw = 90
cameraPitch = -25
cameraTargetPosition=[2,1,2]
p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)
  #camInfo = p.getDebugVisualizerCamera()
  #camForward = camInfo[5]

pin1 =p.loadURDF("cube.urdf",[1,0.5,2],p.getQuaternionFromEuler([0,0,0]))
sph1 =p.loadURDF("sphere.urdf",[1,0.5,1],p.getQuaternionFromEuler([0,0,0]))
pin2 =p.loadURDF("cube.urdf",[1,0.5+0.2,2],p.getQuaternionFromEuler([0,0,0]))
sph2 =p.loadURDF("sphere.urdf",[1,0.5+0.2,1],p.getQuaternionFromEuler([0,0,0]))
pin3 =p.loadURDF("cube.urdf",[1,0.5+0.4,2],p.getQuaternionFromEuler([0,0,0]))
sph3 =p.loadURDF("sphere.urdf",[1,0.5+0.4,1],p.getQuaternionFromEuler([0,0,0]))
pin4 =p.loadURDF("cube.urdf",[1,0.5+0.6,2],p.getQuaternionFromEuler([0,0,0]))
sph4 =p.loadURDF("sphere.urdf",[1,0.5+0.587,1],p.getQuaternionFromEuler([0,0,0]))
pin5 =p.loadURDF("cube.urdf",[1,0.5+0.8,2],p.getQuaternionFromEuler([0,0,0]))
sph5 =p.loadURDF("sphere.urdf",[1,0.5+0.79,1],p.getQuaternionFromEuler([0,0,0]))

p.changeDynamics(sph1,-1,restitution=1)
p.changeDynamics(sph2,-1,restitution=1)
p.changeDynamics(sph3,-1,restitution=1)
p.changeDynamics(sph4,-1,restitution=1)
p.changeDynamics(sph5,-1,restitution=1)

#sphPos, sphOrn = p.getBasePositionAndOrientation(sph)

#cubePos, cubeOrn = p.getBasePositionAndOrientation(pin)
print("PRESS AND RELEASE 'r' TO APPLY EXTERNAL FORCE AND START THE SIMULATION")
while 1:
 p.createConstraint(pin1,-1,-1,-1, p.JOINT_FIXED,[0, 0, 0], [0,0,0],[1,0.5,2])
 p.createConstraint(pin2,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[1,0.5+0.2,2])
 p.createConstraint(pin3,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[1,0.5+0.4,2])
 p.createConstraint(pin4,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[1,0.5+0.6,2])
 p.createConstraint(pin5,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[1,0.5+0.8,2])
 
 p.createConstraint(pin1,-1,sph1,-1, p.JOINT_POINT2POINT,[1, 0.5, 2], [0,0,0],[0,0,1])
 p.createConstraint(pin2,-1,sph2,-1,p.JOINT_POINT2POINT,[1,0.5+0.2,2],[0,0,0],[0,0,1])
 p.createConstraint(pin3,-1,sph3,-1,p.JOINT_POINT2POINT,[1,0.5+0.4,2],[0,0,0],[0,0,1])
 p.createConstraint(pin4,-1,sph4,-1,p.JOINT_POINT2POINT,[1,0.5+0.6,2],[0,0,0],[0,0,1])
 p.createConstraint(pin5,-1,sph5,-1,p.JOINT_POINT2POINT,[1,0.5+0.8,2],[0,0,0],[0,0,1])

 keys=p.getKeyboardEvents()

 for k in keys:
   if ord('r') in keys and keys[ord('r')]&p.KEY_WAS_TRIGGERED:
     p.applyExternalForce(sph1,-1,[0,-800,1000],[1,0.5,1],p.WORLD_FRAME)
 p.setRealTimeSimulation(1)
  

p.disconnect()
