# def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
#     """Computes inverse kinematics in the body frame for an open chain robot
#        and prints a report of each iteration
#
#     :param Blist: The joint screw axes in the end-effector frame when the
#                   manipulator is at the home position, in the format of a
#                   matrix with axes as the columns
#     :param M: The home configuration of the end-effector
#     :param T: The desired end-effector configuration Tsd
#     :param thetalist0: An initial guess of joint angles that are close to
#                        satisfying Tsd
#     :param eomg: A small positive tolerance on the end-effector orientation
#                  error. The returned joint angles must give an end-effector
#                  orientation error less than eomg
#     :param ev: A small positive tolerance on the end-effector linear position
#                error. The returned joint angles must give an end-effector
#                position error less than ev
#     :return thetalist: Joint angles that achieve T within the specified
#                        tolerances,
#     :return success: A logical value where TRUE means that the function found
#                      a solution and FALSE means that it ran through the set
#                      number of maximum iterations without finding a solution
#                      within the tolerances eomg and ev.
#     Uses an iterative Newton-Raphson root-finding method.
#     The maximum number of iterations before the algorithm is terminated has
#     been hardcoded in as a variable called maxiterations. It is set to 20 at
#     the start of the function, but can be changed if needed.
#
#     Example Input:
#         import numpy as np
#         import hw4 as mr
#         Blist = np.array([[0, 0, -1, 2, 0,   0],
#                           [0, 0,  0, 0, 1,   0],
#                           [0, 0,  1, 0, 0, 0.1]]).T
#         M = np.array([[-1, 0,  0, 0],
#                       [ 0, 1,  0, 6],
#                       [ 0, 0, -1, 2],
#                       [ 0, 0,  0, 1]])
#         T = np.array([[0, 1,  0,     -5],
#                       [1, 0,  0,      4],
#                       [0, 0, -1, 1.6858],
#                       [0, 0,  0,      1]])
#         thetalist0 = np.array([1.5, 2.5, 3])
#         eomg = 0.01
#         ev = 0.001
#     Output:
#         (np.array([1.57073819, 2.999667, 3.14153913]), True)
#     """
#     thetalist = np.array(thetalist0).copy()
#     theta_array = np.empty([1,len(thetalist)])
#     theta_array[0] = thetalist
#
#     i = 0
#     maxiterations = 20
#     Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
#                                                       thetalist)), T)))
#     err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
#           or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
#     while err and i < maxiterations:
#
#         theta_array = np.append(theta_array,[thetalist],axis=0)
#
#         print ("Iteration: " + str(i))
#         print (" ")
#
#         thetalist = thetalist + np.dot(np.linalg.pinv(JacobianBody(Blist, thetalist)), Vb)
#         theta_array[i] = thetalist
#
#         i = i + 1
#         Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, thetalist)), T)))
#         err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
#
#         print ("joint vector:")
#         print (thetalist)
#         print (" ")
#
#         print ("SE(3) end-effector config:")
#
#         T_th = FKinBody(M, Blist, thetalist)
#
#         print (T_th)
#         print (" ")
#
#         print ("error twist V_b:")
#         print (Vb)
#         print (" ")
#
#         print ("error magnitude ||w_b||:")
#         print (np.linalg.norm([Vb[0], Vb[1], Vb[2]]))
#         print (" ")
#
#         print ("error magnitude ||v_b||:")
#         print (np.linalg.norm([Vb[3], Vb[4], Vb[5]]))
#         print (" ")
#
#         print ("joint vector matrix:")
#         print (theta_array)
#         print (" ")
#
#     np.savetxt("iterates.csv", theta_array, delimiter=",")
#     print ("Joint Vector Matrix Saved")
#     return (thetalist, not err)

import numpy as np
import hw4 as mr

w1 = 0.109
w2 = 0.082
l1 = 0.425
l2 = 0.392
h1 = 0.089
h2 = 0.095

m1 = [-1, 0, 0, l1+l2]
m2 = [0, 0, 1, w1+w2]
m3 = [0, 1, 0, h1-h2]
m4 = [0, 0, 0, 1]

M = np.array([m1, m2, m3, m4])

b1 = [0, 0, 1, 0, 0, 0]
b2 = [0, 1, 0, -h1, 0, 0]
b3 = [0, 1, 0, -h1, 0, l1]
b4 = [0, 1, 0, -h1, 0, l1+l2]
b5 = [0, 0, -1, -w1, l1+l2, 0]
b6 = [0, 1, 0, h2-h1, 0, l1+l2]

Blist = np.array([b1, b2, b3, b4, b5, b6])

T = np.array(([0, 1, 0, -0.5],
             [0, 0, -1, 0.1],
             [-1, 0, 0, 0.1],
             [0, 0, 0, 1]))

eomg = 0.001
ev = 0.0001

thetalist = [-6.283, -2.185, -1.859, -5.298, -6.280, -4.845]

mr.IKinBodyIterates(Blist.T, M, T, thetalist, eomg, ev)
