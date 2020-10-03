import pybullet as p
import time
import pybullet_data
import os
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, -2)
def fibo(n):
    if n == 1:
        return 0
    elif n==2:
        return 1
    else:
        return(fibo(n-1) + fibo(n-2))

f=2
while True:
    for j in range(fibo(f)):
         sph = p.loadURDF("sphere.urdf", (j/5.0, 1.8, 5), p.getQuaternionFromEuler([0, 0, 0]))
    for i in range(10000):     
         pos , orient= p.getBasePositionAndOrientation(sph)
         p.setRealTimeSimulation(1)
         print(pos[2])
         if pos[2]<=0:                
            break;
    f=f+1

p.disconnect()
