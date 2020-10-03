import matplotlib.pyplot as plt
import cv2
import numpy as np
import pybullet as p
import pybullet_data
import time
import math
p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
carpos = [0, 0, 0.1]
p.loadURDF("dabba.urdf",[-1,3,0.1])
p.loadURDF("dabba.urdf",[1,2,0.1])
p.loadURDF("dabba.urdf",[-0.5,6,0.1])
car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])
cameraDistance = 5
cameraYaw = 35
cameraPitch = -35
height=512
width=512
aspect=width/height
near=0.02
far=5
fov=60
targetvel = 5  #rad/s
maxForce = 50 #Newton
#jointI=[2,3,4,5]
#maxForceL=[100,100,100,100]
wheels = [2, 3, 4, 5]
wheelVelocities = [0, 0, 0, 0]
wheelDeltasTurn = [1, -1, 1, -1]
wheelDeltasFwd = [1, 1, 1, 1]
while 1:
  huskyPos, orn = p.getBasePositionAndOrientation(car)
  #cameraTargetPosition = huskyPos
  orn1=p.getEulerFromQuaternion(orn)
  #print(orn1)
  a=math.degrees(orn1[0])
  b=math.degrees(orn1[1])
  c=math.degrees(orn1[2])
  #p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)
  #camInfo = p.getDebugVisualizerCamera()
  #camForward = camInfo[5]
  #c=cameraTargetPosition
  #x=cameraTargetPosition[0]*100
  #y=cameraTargetPosition[1]*100
  viewmatrix=p.computeViewMatrixFromYawPitchRoll([huskyPos[0],huskyPos[1],0.6],0.2,c-90,b,a,2)
  #p.computeViewMatrix([huskyPos[0],huskyPos[1],0.6],[x,y,0],[0,0,1])
  projectionmatrix=p.computeProjectionMatrixFOV(fov,aspect,near,far)
  image=p.getCameraImage(width,height,viewmatrix,projectionmatrix,shadow=True,renderer=p.ER_BULLET_HARDWARE_OPENGL)
  keys = p.getKeyboardEvents()
  shift = 0.01
  wheelVelocities = [0, 0, 0, 0]
  for k in keys:
    if ord('r') in keys and keys[ord('r')]&p.KEY_WAS_TRIGGERED:
      while True:
        for i in range(len(wheels)):
          wheelVelocities[i] = wheelVelocities[i] - targetvel * wheelDeltasTurn[i]
          for i in range(len(wheels)):
             p.setJointMotorControl2(car,
                            wheels[i],
                            p.VELOCITY_CONTROL,
                            targetVelocity=wheelVelocities[i],
                            force=maxForce)
        p.setRealTimeSimulation(1)
    if ord('c') in keys and keys[ord('c')]&p.KEY_WAS_TRIGGERED:
      rgb = np.reshape(image[2], (height, width, 4)) * 1. / 255.
      cv2.imshow('img',rgb)
      cv2.waitKey(2000)
      cv2.destroyAllWindows()
      
    if ord('a') in keys and keys[ord('a')]&p.KEY_WAS_TRIGGERED:
      targetvel=targetvel+1  
      break
    if p.B3G_LEFT_ARROW in keys:
      for i in range(len(wheels)):
        wheelVelocities[i] = wheelVelocities[i] - targetvel * wheelDeltasTurn[i]
    if p.B3G_RIGHT_ARROW in keys:
      for i in range(len(wheels)):
        wheelVelocities[i] = wheelVelocities[i] + targetvel * wheelDeltasTurn[i]
    if p.B3G_UP_ARROW in keys:
      for i in range(len(wheels)):
        wheelVelocities[i] = wheelVelocities[i] + targetvel * wheelDeltasFwd[i]
    if p.B3G_DOWN_ARROW in keys:
      for i in range(len(wheels)):
        wheelVelocities[i] = wheelVelocities[i] - targetvel * wheelDeltasFwd[i]
  
    #baseorn = p.getQuaternionFromEuler([0, 0, ang])
  for i in range(len(wheels)):
    p.setJointMotorControl2(car,
                            wheels[i],
                            p.VELOCITY_CONTROL,
                            targetVelocity=wheelVelocities[i],
                            force=maxForce)
  p.setRealTimeSimulation(1)
  #time.sleep(1./240.)
p.getContactPoints(car)

p.disconnect()
