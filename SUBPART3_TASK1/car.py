import pybullet as p
import pybullet_data
import time
import os
p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadSDF(os.path.join(pybullet_data.getDataPath(), "stadium.sdf"))
p.setGravity(0, 0, -10)
carpos = [-1,0,0]
carpos2=[1,0,0]
car2 = p.loadURDF("husky2.urdf", carpos2[0], carpos2[1], carpos2[2])
car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])
numJoints = p.getNumJoints(car)
cameraDistance = 5
cameraYaw = 35
cameraPitch = -35
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))
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
  cameraTargetPosition = huskyPos
  p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)
  camInfo = p.getDebugVisualizerCamera()
  camForward = camInfo[5]
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
