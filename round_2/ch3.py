#!/usr/bin/env python3

"""
Alexander Hay
ME449

Chapter 3 - Quiz
"""

import numpy as np
import modern_robotics as mr

xa = np.transpose([[0,0,1]])
ya = np.transpose([[-1,0,0]])
za = np.transpose([[0,-1,0]])

p_sa = np.transpose([[0,0,1]])

Rsa = xa
Rsa = np.append(Rsa, ya,axis=1)
Rsa = np.append(Rsa, za,axis=1)
Tsa = np.append(Rsa,p_sa,axis=1)
Tsa = np.append(Tsa,[[0,0,0,1]],axis=0)
Tas = mr.TransInv(Tsa)

xb = np.transpose([[1,0,0]])
yb = np.transpose([[0,0,-1]])
zb = np.transpose([[0,1,0]])

p_sb = np.transpose([[0,2,0]])

Rsb = xb
Rsb = np.append(Rsb, yb,axis=1)
Rsb = np.append(Rsb, zb,axis=1)
Tsb = np.append(Rsb,p_sb,axis=1)
Tsb = np.append(Tsb,[[0,0,0,1]],axis=0)

Tbs = mr.TransInv(Tsb)

Tab = np.dot(Tas,Tsb)

p_bs = np.dot(Rsb, p_sb)

V_s = np.transpose([[3,2,1,-1,-2,-3]])

V_a = np.dot(mr.Adjoint(Tas),V_s)