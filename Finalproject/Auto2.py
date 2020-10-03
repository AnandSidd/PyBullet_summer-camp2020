import pybullet as p
import pybullet_data
import time
import os
import numpy as np
import math
import cv2
from playsound import playsound

def get_line(goal_point,ball_pos):
    slope_angle=math.atan((ball_pos[1]-goal_point[1])/(ball_pos[0]-goal_point[0]))
    point_carx=ball_pos[0]+math.cos(slope_angle)
    point_cary=ball_pos[1]+math.sin(slope_angle)

    return point_carx,point_cary,slope_angle

def get_linehusky(carpos,carori,point1x,point1y):
    slope_angle=math.atan((point1y-carpos[1])/(point1x-carpos[0]))
    if point1x>0 and point1y>0:
        angle=-3.14+slope_angle
    elif point1x<0 and point1y>0:
        angle=slope_angle
    elif point1x<0 and point1y<0:
        angle=slope_angle
    elif point1x>0 and point1y<0:
        angle=3.14+slope_angle
    return angle

def check(carPos,point1):
    if point1[0]-0.1<=carPos[0]<=point1[0]+0.1 and point1[1]-0.1<=carPos[1]<=point1[1]+0.1:
        return True
    else:
        return False
  
p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadSDF(os.path.join(pybullet_data.getDataPath(), "stadium.sdf"))
p.setGravity(0, 0, -10)
carpos1 = [-1,0,0]
carpos=[1,0,0]
car = p.loadURDF("husky2.urdf", carpos[0], carpos[1], carpos[2])
car1 = p.loadURDF("husky/husky.urdf", carpos1[0], carpos1[1], carpos1[2])
numJoints = p.getNumJoints(car1)
cameraDistance = 5
cameraYaw = 35
cameraPitch = -35
#for joint in range(numJoints):
#print(p.getJointInfo(car, joint))
maxforce = 50 #Newton
carPos, carOrn = p.getBasePositionAndOrientation(car)
carorn=p.getEulerFromQuaternion(carOrn)

allowed_values = list(range(-3, -1))

#print(allowed_values)
#can be anything in {-5, ..., 5} \ {0}:
#random_value = random.choice(allowed_values)
w=cv2.imread(os.path.join(os.getcwd(), "youwin.png"),0)
l=cv2.imread(os.path.join(os.getcwd(), "youlose.png"),0)
x=np.random.choice(allowed_values)
allowed_values = list(range(-3, 3+1))
allowed_values.remove(0)
allowed_values.remove(1)
allowed_values.remove(-1)
y=np.random.choice(allowed_values)
box = p.loadURDF("sph.urdf",[x,y,0.2])#x=-21 to -25 y=-5 to 5 for net or goal
'''so point for line between goal and ball is taken as mean of end points of goal i.e. x=-23 and y=0'''
goal_point=[-23,0,0]
#jointI=[2,3,4,5]
#maxForceL=[100,100,100,100]

wheels = [2, 3, 4, 5]
wheelVelocities = [0, 0, 0, 0]
wheelDeltasTurn = [1, -1, 1, -1]
wheelDeltasFwd = [1, 1, 1, 1]
while 1:
    huskyPos, orn = p.getBasePositionAndOrientation(car1)
    carPos, carOrn = p.getBasePositionAndOrientation(car)
    carorn = p.getEulerFromQuaternion(carOrn)    
    box_pos, box_ori = p.getBasePositionAndOrientation(box)
    box_ori = p.getEulerFromQuaternion(box_ori)
    #print(box_pos, box_ori)
    #if box_pos[0]<-23 and -5<box_pos[1]<5:
    if box_pos[0]>23 and -5<box_pos[1]<5:
            print('WIN')
            playsound(os.path.join(os.getcwd(), "youwinsound.wav"))
            cv2.imshow('results',w)
            cv2.waitKey(0)
            #cv2.destroyAllWindows()
            #p.disconnect()
            break;
    elif box_pos[0]<-23 and -5<box_pos[1]<5:
            print("LOOSE")
            playsound(os.path.join(os.getcwd(), "youlosesound.wav"))
            cv2.imshow('results',l)
            cv2.waitKey(0)
            #cv2.destroyAllWindows()
            #p.disconnect()
            break;
    # print(goal_point[0],box_pos[0])
    point1x, point1y, slope = get_line(goal_point, box_pos)
    point1 = [point1x, point1y, 0]
    v=math.dist(box_pos,carPos)
    angle1x = get_linehusky(carPos, carorn, point1x, point1y)
    #print(angle1x)
    maxForceauto = 10
    if math.fabs(angle1x-carorn[2])>1/3.14 and math.dist(box_pos,carPos)>1.5 :
        #cons = p.createConstraint(car, -1, -1, -1, p.JOINT_POINT2POINT, [0, 1, 0], [0, 0, 0], carPos)
        q = p.getQuaternionFromEuler((0, 0, angle1x))
        p.resetBasePositionAndOrientation(car, carPos, q)
        targetVelauto = -5
        #p.removeConstraint(cons)
    if v>3:
        d=carPos[0]-box_pos[0]
        if d<0:  
          targetVelauto=10
        elif d>0:
          targetVelauto=-10
        #else:
         # targetVelauto=-abs(v)
    if check(carPos, point1):
        # print('yes')
        q = p.getQuaternionFromEuler((0, 0, slope))
        p.resetBasePositionAndOrientation(car, carPos, q)
        targetVelauto = -5
  
        #continue
    
    for joint in range(2, 6):
        p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity=targetVelauto, force=maxForceauto)

    # force=(np.array(point1)-np.array(carPos))*60
    # p.applyExternalForce(objectUniqueId=car, linkIndex=-1,
    # forceObj=force, posObj=point1, flags=p.WORLD_FRAME)
    

    targetvel = 10

    cameraTargetPosition = huskyPos
    p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)
    camInfo = p.getDebugVisualizerCamera()
    camForward = camInfo[5]
    keys = p.getKeyboardEvents()
    shift = 0.01
    wheelVelocities = [0, 0, 0, 0]
    for k in keys:      
        if p.B3G_LEFT_ARROW in keys:
            for i in range(len(wheels)):
                wheelVelocities[i] = wheelVelocities[i] - targetvel * wheelDeltasTurn[i]
        if p.B3G_RIGHT_ARROW in keys:
            for i in range(len(wheels)):
                wheelVelocities[i] = wheelVelocities[i] + targetvel * wheelDeltasTurn[i]
        if p.B3G_UP_ARROW in keys:
            #playsound(os.path.join(os.getcwd(), "carsound.wav"))
            for i in range(len(wheels)):
                wheelVelocities[i] = wheelVelocities[i] + targetvel * wheelDeltasFwd[i]
        if p.B3G_DOWN_ARROW in keys:
            for i in range(len(wheels)):
                wheelVelocities[i] = wheelVelocities[i] - targetvel * wheelDeltasFwd[i]

        # baseorn = p.getQuaternionFromEuler([0, 0, ang])
    for i in range(len(wheels)):
        p.setJointMotorControl2(car1,
                                wheels[i],
                                p.VELOCITY_CONTROL,
                                targetVelocity=wheelVelocities[i],
                                force=maxforce)
        
    p.setRealTimeSimulation(1)
    # time.sleep(1./240.)
#p.getContactPoints(car1)
p.disconnect()
