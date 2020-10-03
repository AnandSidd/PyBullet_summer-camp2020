import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
#these are the pre required conditions for the task.
ramp=p.loadURDF("wedge.urdf")
p.setGravity(0, 0, -10)
p.changeDynamics(ramp,-1,lateralFriction=0.5)

huskypos = [2, 0, 0.1]
husky = p.loadURDF("husky/husky.urdf", huskypos)

'''
1.print Number of Joints and joint info and state
2.Get the user input about the control function they 
want to simulate and see.(its upto you, it can be a string / int anything but just leave
a comment with the legend to your menu)
'''
jointno=p.getNumJoints(husky)
print(jointno)
#for joint index//
for i in range(jointno):
  print(p.getJointInfo(husky,i))
  print(p.getJointState(husky,i))


#USER CAN GIVE INPUT ABOUT THE CONTROL FUNCTION HE WANTS TO SIMULATE :::0 FOR VELOCITY:::1 FOR TORQUE:::


v=int(input("ENTER 0 FOR VELOCITY_CONTROL:\nENTER 1 FOR TORQUE_CONTROL:\n"))
jointindexes=[2,3,4,5]
cameraDistance = 5
cameraYaw = 35
cameraPitch = -35

maxForce=[40,40,40,40]
optimal_velocity_value=[-30,-30,-30,-30]
optimal_torque_value = [-250,-250,-250,-250]
      
def Torque_control():
      p.setJointMotorControlArray(husky,jointindexes,controlMode=p.TORQUE_CONTROL,forces=optimal_torque_value)

def Velocity_control():
      p.setJointMotorControlArray(husky,jointindexes,controlMode=p.VELOCITY_CONTROL,targetVelocities=optimal_velocity_value,forces=maxForce)

x=0
while True:
  huskyPos, orn = p.getBasePositionAndOrientation(husky)
  cameraTargetPosition = huskyPos
  p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)
  camInfo = p.getDebugVisualizerCamera()
  camForward = camInfo[5]
  if v==1:
    Torque_control()    
  elif v==0:
    Velocity_control()
  p.stepSimulation()
  time.sleep(.01)
  x=x+1
  if x==100:
     print(p.getLinkState(husky,linkIndex=0))#state and velocity info
     print("\n")
     x=0

p.disconnect()
