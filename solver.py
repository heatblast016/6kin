
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import math
import transformations as tfm

class solver:
    def __init__(self, l1: float, l2: float, l3: float, l4: float):
        '''Initializes class, storing variables representing joint lengths'''
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4



    def solve_forward(self, q1: float,q2: float,q3: float,q4: float,q5: float,q6: float): 
        '''Returns end effector and simplified link positions for all links'''
        #Calculates first link transform
        disp1 = np.array([0,0,self.l1])
        r1 = tfm.rotation_z(q1)
        t1 = tfm.generate_transform(r1,disp1)

        #Calculates second link transform
        r2 = tfm.rotation_y(q2)
        t2 = np.matmul(t1,tfm.generate_transform(r2,np.array([0,0,0])))
        
        #Calculates third link transform

        r3 = tfm.rotation_y(q3)
        t3 = np.matmul(t2,tfm.generate_transform(r3,np.array([self.l2,0,0])))

        #Calculates fourth link transform
        r4 = tfm.rotation_x(q4)
        t4 = np.matmul(t3,tfm.generate_transform(r4,np.array([self.l3,0,0])))

        #Calculates fifth link transform
        r5 = tfm.rotation_y(q5)
        t5 = np.matmul(t4,tfm.generate_transform(r5,np.array([0,0,0])))

        #Calculates sixth link transform
        r6 = tfm.rotation_x(q6)
        t6 = np.matmul(t5,tfm.generate_transform(r6,np.array([0,0,0])))

        #Gets position of each joint ("Shoulder", "Elbow", "Wrist", and End effector position)
        x1 = np.matmul(t1,np.array([0,0,0,1]))
        x2 = np.matmul(t3,np.array([0,0,0,1]))
        x3 = np.matmul(t4,np.array([0,0,0,1]))
        x4 = np.matmul(t6, np.array([self.l4,0,0,1]))
        return [x1,x2,x3,x4]

a = solver(1,1,1,0.25)

coords = a.solve_forward(math.pi/4,math.pi/-4,math.pi/4,math.pi/8,math.pi/2,0)

fig = plt.figure(figsize=(4,4))

ax = fig.add_subplot(111, projection='3d')

ax.set_xlim3d(-4, 4)
ax.set_ylim3d(-4, 4)
ax.set_zlim3d(0, 4)

for i in range(0,3):
    start = coords[i]
    end = coords[i+1]
    x, y, z = [start[0], end[0]], [start[1], end[1]], [start[2], end[2]]
    ax.scatter(x, y, z, c='red', s=100)
    ax.plot(x, y, z, color='black')

start = [0,0,0]
end = coords[0]
x, y, z = [start[0], end[0]], [start[1], end[1]], [start[2], end[2]]
ax.scatter(x, y, z, c='red', s=100)
ax.plot(x, y, z, color='black')
plt.show()