#!/usr/bin/env python3

"""
Alexander Hay
ME449

Assignment 1
"""

import numpy as np
import modern_robotics as mr

# Given axes
w1 = np.transpose([[0,0,1]])
w2 = np.transpose([[0,1,0]])
w3 = w2
w4 = w2
w5 = np.transpose([[0,0,-1]])
w6 = w2

# Given rotation matricies
r13 = np.array([[0,0,-1],[0,1,0],[1,0,0]])
rs2 = np.array([[0, -1, 0],[-0.5, 0, -0.866],[0.866, 0, -0.5]])
r15 = np.array([[-0.3536, -0.3536, 0.866],[-0.7071, 0.7071, 0],[-0.6124, -0.6124, -0.5]])
r12 = np.array([[-0.5, 0, -0.866],[0, 1, 0],[0.866, 0, -0.5]])
r34 = np.array([[-0.866, 0, -0.5],[0, 1, 0],[0.5, 0, -0.866]])
rs6 = np.array([[-0.3536, -0.7071, 0.6124],[-0.5732, -0.3536, -0.7392],[0.7392, -0.6124, -0.2803]])
rb6 = np.array([[-1,0,0],[0,0,1],[0,1,0]])


# Rotation matricies between frames

# intermediary frames
r2s = np.transpose(rs2)
r1s = np.dot(r12,r2s)
r21 = np.transpose(r12)
r43 = np.transpose(r34)
r31 = np.transpose(r13)
r35 = np.dot(r31,r15)
r26 = np.dot(r2s,rs6)
r51 = np.transpose(r15)
r16 = np.dot(r12,r26)
r6b = np.transpose(rb6)
rs1 = np.transpose(r1s)
rs6 = np.dot(rs1,r16)
rs1 = np.transpose(r1s)

# Rotation matricies Ri,i+1
rs1
r12
r23 = np.dot(r21,r13)
r34
r45 = np.dot(r43,r35)
r56 = np.dot(r51,r16)
r6b = np.transpose(rb6)
rsb = np.dot(rs6,r6b)


# Getting angles of rotation
"""
* mr.MatrixLog3 takes R from SO3 to so3 (skew symmetric) in exponential coordinates
* mr.so3ToVec converts 3x3 skew symmetric matrix to 3-vector
* mr.AxisAng3 returns axis and rotation angle from 3-vector in exponential coordinates
"""

so3mat_s1 = mr.MatrixLog3(rs1)
so3mat_12 = mr.MatrixLog3(r12)
so3mat_23 = mr.MatrixLog3(r23)
so3mat_34 = mr.MatrixLog3(r34)
so3mat_45 = mr.MatrixLog3(r45)
so3mat_56 = mr.MatrixLog3(r56)
#so3mat_6b = mr.MatrixLog3(r6b)

so3ToVec_s1 = mr.so3ToVec(so3mat_s1)
so3ToVec_12 = mr.so3ToVec(so3mat_12)
so3ToVec_23 = mr.so3ToVec(so3mat_23)
so3ToVec_34 = mr.so3ToVec(so3mat_34)
so3ToVec_45 = mr.so3ToVec(so3mat_45)
so3ToVec_56 = mr.so3ToVec(so3mat_56)
#so3ToVec_6b = mr.so3ToVec(so3mat_6b)

theta_s1 = mr.AxisAng3(so3ToVec_s1)[1]
theta_12 = mr.AxisAng3(so3ToVec_12)[1]
theta_23 = mr.AxisAng3(so3ToVec_23)[1]
theta_34 = mr.AxisAng3(so3ToVec_34)[1]
theta_45 = mr.AxisAng3(so3ToVec_45)[1]
theta_56 = mr.AxisAng3(so3ToVec_56)[1]
#theta_6b = mr.AxisAng3(so3ToVec_6b)[1]

thetas = np.transpose([[theta_s1,
                        theta_12,
                        theta_23,
                        theta_34,
                        theta_45,
                        theta_56]])
#                        theta_6b]])

print("Thetas:")
print(thetas)
print()
print("Rsb")
print(rsb)