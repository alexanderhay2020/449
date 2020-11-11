#!/usr/bin/env python3

"""
Alexander Hay
ME449

Assignment 2
"""

import numpy as np
import modern_robotics as mr


def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """
    This function was cloned from the Modern Robotics library
    Modifications to the function track robot configuration and error
    """
    
    # Store information to track changes (this is new)
    # new information is put into an array
    j_history = []
    se3_history= []
    error_twist_history = []
    ang_err_mag_history = []
    lin_err_mag_history = []
    
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    
    
    while err and i < maxiterations:
        
        # changes are appended to corresponding array
        j_history.append(thetalist)
        se3_history.append(mr.FKinBody(M,Blist,thetalist))
        error_twist_history.append(Vb)
        ang_err_mag_history.append(np.linalg.norm([Vb[0], Vb[1], Vb[2]]))
        lin_err_mag_history.append(np.linalg.norm([Vb[3], Vb[4], Vb[5]]))

        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)
        i = i + 1
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    return (thetalist, not err, j_history,se3_history,error_twist_history,ang_err_mag_history,lin_err_mag_history)

# Given measurements, converted to meters
w1=.109
w2=.082
l1=.425
l2=.392
h1=.099
h2=.095

Blist = np.array([[ 0, 1, 0,  w1+w2,  0,       l1+l2],
                  [ 0, 0, 1,  h2,     -l1-l2,  0],
                  [ 0, 0, 1,  h2,     -l2,     0],
                  [ 0, 0, 1,  h2,     0,       0],
                  [ 0,-1, 0, -w2,     0,       0],
                  [ 0, 0, 1,  0,      0,       0]]).T

M = np.array([[-1, 0, 0, l1+l2],
              [ 0, 0, 1, w1+w2],
              [ 0, 1, 0, h1-h2],
              [ 0, 0, 0, 1]])

T = np.array([[ 0, 1, 0,-0.5], # Desired end-effector configuration Tsd
              [ 0, 0,-1, 0.1],
              [-1, 0, 0, 0.1],
              [ 0, 0, 0, 1]])

eomg = 0.001 # (rad) = 0.057 degrees
ev = 0.0001 # (meters) = 0.1mm

# init guess
#thetalist0=np.array([0.135, -0.943, 1.482, -0.458, -2.992, -1.482])
thetalist0=np.array([0, np.pi/2, -3*np.pi/2, 2*np.pi, -np.pi, -3*np.pi/2])

# calling fuction, defining historical variables
[thetalist, success, j_history, se3_history, error_twist_history, ang_err_mag_history, lin_err_mag_history] = IKinBodyIterates(Blist,M,T,thetalist0,eomg,ev)

print("Success: " + str(success)) 
np.savetxt("joint_history.csv",j_history,delimiter=",")