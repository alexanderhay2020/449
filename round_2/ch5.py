#!/usr/bin/env python3

"""
Alexander Hay
ME449

Chapter 5 - Quiz
"""
import numpy as np
import modern_robotics as mr

# Given:

l1,l2,l3=1,1,1

th1 = 0
th2 = np.pi/2
th3 = 0
th = np.array([[th1,th2,th3]])

j1 = np.transpose([[0,0,1,0,0,0]])

j2v1 = l1*np.sin(th1)
j2v2 = -l1*np.cos(th1)
j2 = np.transpose([[0,0,1,j2v1,j2v2,0]])

j3v1 = (l1*np.sin(th1)) + (l2*np.sin(th1+th2))
j3v2 = (-l1*np.cos(th1)) - (l2*np.cos(th1+th2))
j3 = np.transpose([[0,0,1,j3v1,j3v2,0]])

J = np.append(j1,j2,axis=1)
J = np.append(J,j3,axis=1)

Fs = np.transpose([[0,0,0,-2,0,0]])

l1,l2,l3,l4=1,1,1,1
th1,th2=0,0
th3=np.pi/2
th4=-np.pi/2

th = np.array([[th1,th2,th3,th4]])

jb1 = np.array([[1,1,1,1]])

jb_21 = (l3*np.sin(th4)) + (l2*np.sin(th3+th4)) + (l1*np.sin(th2+th3+th4))
jb_22 = (l3*np.sin(th4)) + (l2*np.sin(th3+th4))
jb_23 = (l3*np.sin(th4))
jb2 = np.array([[jb_21,jb_22,jb_23,0]])

jb_31 = l4 + (l3*np.cos(th4)) + (l2*np.cos(th3+th4)) + (l1*np.cos(th2+th3+th4))
jb_32 = l4 + (l3*np.cos(th4)) + (l2*np.cos(th3+th4))
jb_33 = l4 + (l3*np.cos(th4))
jb3 = np.array([[jb_31, jb_32, jb_33, l4]])

Jb = jb1
Jb = np.append(Jb,jb2,axis=0)
Jb = np.append(Jb,jb3,axis=0)

Fb = np.transpose([[0,0,10,10,10,0]])

# T = Js.T(0)*Fs

T = np.dot(np.transpose(Jb),Fb)