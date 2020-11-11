#!/usr/bin/env python3

"""
Alexander Hay
ME449

Chapter 4 - Quiz
"""

import numpy as np
import modern_robotics as mr

xL = 2+np.sqrt(3)
yL = 0
zL = -1+2+np.sqrt(3)

M = np.array([[1,0,0,xL],
              [0,1,0,yL],
              [0,0,1,zL],
              [0,0,0,1]])

S = np.transpose([[0,0,1,0,-1,0]])
j1 = np.transpose([[0,0,1,1,0,0]])
j2 = np.transpose([[0,1,0,0,0,1]])
j3 = np.transpose([[0,1,0,1,0,(1+np.sqrt(3))]])
j4 = np.transpose([[0,1,0,(1-np.sqrt(3)),0,(2+np.sqrt(3))]])
j5 = np.transpose([[0,0,0,0,0,1]]) #(-1+2+np.sqrt(3))
j6 = np.transpose([[0,0,1,0,(-2-np.sqrt(3)),0]])

S = np.append(S, j2, axis=1)
S = np.append(S, j3, axis=1)
S = np.append(S, j4, axis=1)
S = np.append(S, j5, axis=1)
S = np.append(S, j6, axis=1)

j6 = np.transpose([[0,0,1,0,0,0]])
j5 = np.transpose([[0,0,0,0,0,1]])
j4 = np.transpose([[0,1,0,2,0,0]])
j3 = np.transpose([[0,1,0,(2+np.sqrt(3)),0,-1]])
j2 = np.transpose([[0,1,0,(2+np.sqrt(3)),0,-1]])
j2 = np.transpose([[0,1,0,(1+np.sqrt(3)),0,(-1-np.sqrt(3))]])
j1 = np.transpose([[0,0,1,0,(1+np.sqrt(3)),0]])

B = j1
B = np.append(B, j2, axis=1)
B = np.append(B, j3, axis=1)
B = np.append(B, j4, axis=1)
B = np.append(B, j5, axis=1)
B = np.append(B, j6, axis=1)

pi = np.pi

theta_list = np.transpose([[(-pi/2),
                            (pi/2),
                            (pi/3),
                            (-pi/4),
                            1,
                            (pi/6)]])

TSE_space = mr.FKinSpace(M,S,theta_list)
TSE_body = mr.FKinBody(M,B,theta_list)