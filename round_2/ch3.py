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

xb = np.transpose([[1,0,0]])
yb = np.transpose([[0,0,-1]])
zb = np.transpose([[0,1,0]])

o_a = np.transpose([[0,0,1]])
o_b = np.transpose([[0,2,0]])

Tsa = xa
Tsa = np.append(Tsa, ya,axis=1)
Tsa = np.append(Tsa, za,axis=1)
Tsa = np.append(Tsa,o_a,axis=1)
Tsa = np.append(Tsa,[[0,0,0,1]],axis=0)
Tas = mr.RotInv(Tsa)

Tsb = xb
Tsb = np.append(Tsb, yb,axis=1)
Tsb = np.append(Tsb, zb,axis=1)
Tsb = np.append(Tsb,o_b,axis=1)
Tsb = np.append(Tsb,[[0,0,0,1]],axis=0)

Tsb_inv = mr.RotInv(Tsb)
Tbs = np.transpose(Tsb)

Tab = np.dot(Tas,Tsb)