#!/usr/bin/env python3

"""
Alexander Hay
ME449

Assignment 3
"""

import numpy as np
import modern_robotics as mr

# Given: Mlist, Glist, Slist
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 

G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]

Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

# Gravity
g = [0, 0, -9.81]

# Time stuff
t_end = 3                       # seconds
dt = 1/100                      # 100 steps per second

# No forces, no torques (freefall)
Ftip = [0,0,0,0,0,0]
taulist = [0,0,0,0,0,0]

# # Home configuration at rest
# thetalist = [0, 0, 0, 0, 0, 0]
# dthetalist = [0, 0, 0, 0, 0, 0]
# ddthetalist = mr.ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist)

# path = np.zeros([300,6])        # 3 seconds x 100 steps/sec, 6 joint angles

# print("Part 1 Running...")

# for i in range(path.shape[0]):
#     [thetalistNext, dthetalistNext] = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)
#     thetalist = thetalistNext
#     dthetalist = dthetalistNext
#     ddthetalist = mr.ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist)
#     path[i] = thetalist
#     if i%10==0:
#         print("Iter: " + str(i) + "/" + str(path.shape[0]))
    
# np.savetxt("path.csv",path,delimiter=",")
# print("Done")
# print(".csv file saved as path.csv")
# print("************************")

# Second configuration
thetalist = [-1, -1, 0, 0, 0, 0]
dthetalist = [0, 0, 0, 0, 0, 0]
ddthetalist = mr.ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist)

path_2 = np.zeros([500,6])        # 5 seconds x 100 steps/sec, 6 joint angles

print()
print("Part 2 Running...")

for i in range(path_2.shape[0]):
    [thetalistNext, dthetalistNext] = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)
    thetalist = thetalistNext
    dthetalist = dthetalistNext
    ddthetalist = mr.ForwardDynamics(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist)
    path_2[i] = thetalist
    if i%10==0:
        print("Iter: " + str(i) + "/" + str(path_2.shape[0]))
    
np.savetxt("path_2.csv",path_2,delimiter=",")
print("Done")
print(".csv file saved as path_2.csv")