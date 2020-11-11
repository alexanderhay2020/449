"""
Alexander Hay
ME449 - HW3,
Problem 5.26
"""

import modern_robotics as mr
import math
import numpy as np
from numpy import linalg as la
import pprint

l1 = 550
l2 = 300
l3 = 60
w1 = 45

m1 = [1,0,0,0]
m2 = [0,1,0,0]
m3 = [0,0,1,l1+l2+l3]
m4 = [0,0,0,1]

# Screw axis
b1 = [0,0,1,0,0,0]
b2 = [0,1,0,l1+l2+l3,0,0]
b3 = [0,0,1,0,0,0]
b4 = [0,1,0,l2+l3,0,w1]
b5 = [0,0,1,0,0,0]
b6 = [0,1,0,l3,0,0]
b7 = [0,0,1,0,0,0]
blist = np.array(([b1,b2,b3,b4,b5,b6,b7]))

theta = math.pi/2
theta_list = np.array(([theta],[theta],[theta],[theta],[theta],[theta],[theta]))

"""************************************
a) Numerical body jacobian
"""
jb = mr.JacobianBody(blist.T,theta_list)

print("a)")
print ("Jacobian: ")
pprint.pprint(jb)
print
print


"""************************************
b) principle axes and lengths
"""

"""split jacobian into jb_w and jb_v"""

jbw, jbv = np.rint(np.split(jb,2))


"""calculate A"""

a_w = np.dot(jbw,jbw.T)
a_v = np.dot(jbv,jbv.T)

"""determine eigenvalues and eigenvectors"""
eval_w,evec_w = la.eig(a_w)
eval_v,evec_v = la.eig(a_v)

"""determine lengths via sqrt of eigenvalues"""
len_w = np.sqrt(eval_w)
len_v = np.sqrt(eval_v)

print("b)")
print("lengths of principle semi-axes for angular velocity manipulability ellipsoid: ")
print(str(len_w))
print()
print("lengths of principle semi-axes for linear velocity manipulability ellipsoid: ")
print(str(len_v))
print()
print("directions of principle semi-axes for angular velocity manipulability ellipsoid: ")
print(str(evec_w))
print()
print("directions of principle semi-axes for linear velocity manipulability ellipsoid: ")
print(str(evec_v))
print()
print()


"""************************************
c) principle axes and lengths
"""

"""determine inverse of A"""
a_w_inv = la.inv(a_w)
a_v_inv = la.inv(a_v)

"""determine eigenvalues and eigenvectors"""
eval_w_inv,evec_w_inv = la.eig(a_w)
eval_v_inv,evec_v_inv = la.eig(a_v)

"""determine lengths via sqrt of eigenvalues"""
len_w_inv = np.sqrt(eval_w_inv)
len_v_inv = np.sqrt(eval_v_inv)

print("c)")
print("lengths of principle semi-axes of 3D moment ellipsoid: ")
print(str(len_w_inv))
print()
print("lengths of principle semi-axes of 3D force ellipsoid: ")
print(str(len_v_inv))
print()
print("directions of principle semi-axes of 3D moment ellipsoid: ")
print(str(evec_w_inv))
print()
print("directions of principle semi-axes of 3D force ellipsoid: ")
print(str(evec_v_inv))
