#!/usr/bin/env python3

"""
Alexander Hay
ME449

Capstone - Milestone 2
"""

# Imports
import modern_robotics as mr
import numpy as np

# Global Variables
ref_traj = []

def shuffle(T, gripper_state):
    temp = [T[0][0],T[0][1],T[0][2],\
                      T[1][0],T[1][1],T[1][2],\
                      T[2][0],T[2][1],T[2][2],\
                      T[0][3],T[1][3],T[2][3], gripper_state]
        
    return temp
    # ref_traj = np.append(ref_traj,temp,)

def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_stand, k, t):
    """
    k:                  The number of trajectory reference configurations per 0.01 seconds
                        Ex: k=10, t=0.01; freq_controller = k/t = 1000Hz
    
    t:                  Time of trajectory in seconds.
    
    Tse_init:           The initial configuration of the end-effector in the reference trajectory
    Tsc_init:           The cube's initial configuration
    Tsc_final:          The cube's desired final configuration
    Tce_grasp:          The end-effector's configuration relative to the cube when it is grasping the cube
    Tce_stand:          The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
                        This specifies the configuration of the end-effector {e} relative to the cube frame {c} before lowering to the grasp configuration Tce,grasp, for example. 
    """
    
    N = t*k/0.01
    
    # End-effector standoff configuration above initial cube configuration in world frame
    Tse_si = np.dot(Tsc_init,Tce_stand)
    traj = mr.CartesianTrajectory(Tse_init, Tse_si, t, N, 5)
    
    # for i in range(int(N)):
    #     param.append([traj[i][0][0],traj[i][0][1],traj[i][0][2],traj[i][1][0],traj[i][1][1],traj[i][1][2],\
 			# traj[i][2][0],traj[i][2][1],traj[i][2][2],traj[i][0][3],traj[i][1][3],traj[i][2][3],0])
           
    # np.savetxt("traj.csv", traj, delimiter=",")

    temp = shuffle(traj, 0)
    ref_traj.append(temp)
     
    print("Trajectory Generated")
    # print(temp)
    
def main():
    
    cube_init = np.transpose([[1, 0, 0]])               # (1m, 0m, 0 rad)
    cube_final = np.transpose([[0, -1, -np.pi/2]])      # (0m,-1m, ) pi/2 rad)
    
    M = np.array([[ 1, 0, 0, 0.033 ],
                  [ 0, 1, 0, 0     ],
                  [ 0, 0, 1, 0.6546],
                  [ 0, 0, 0, 1     ]])
    
    Blist = np.array([[ 0, 0, 1,  0,      0.033, 0],
                      [ 0,-1, 0, -0.5076, 0,     0],
                      [ 0,-1, 0, -0.3526, 0,     0],
                      [ 0,-1, 0, -0.2176, 0,     0],
                      [ 0, 0, 1,  0,      0,     0]]).T
    
    """ Default positions """
    Tse_init = np.array([[ 0, 0, 1, 0],
                         [ 0, 1, 0 ,0],
                         [-1, 0, 0, 0.5],
                         [ 0, 0, 0, 1]])
    
    Tsc_init = np.array([[1, 0, 0, 1],
                         [0, 1, 0, 0],
                         [0, 0, 1, 0.025],
                         [0, 0, 0, 1]])
    
    Tsc_final = np.array([[ 0, 1, 0, 0],
                          [-1, 0, 0,-1],
                          [ 0, 0, 1, 0.025],
                          [ 0, 0, 0, 1]])
    
    
    Tce_grasp = np.array([[-0.5, 0, 0.8660254, 0.015],[0, 1, 0, 0],[-0.8660254,0, -0.5,0.02],[0, 0, 0, 1]])
    Tce_stand = np.array([[0, 0, 1, -0.3],[0, 1, 0, 0],[-1, 0, 0, 0.5],[0, 0, 0, 1]])
    	
    k = 1
    t = 2
    
    # N = t*k/0.01
    
    # End-effector standoff configuration above initial cube configuration in world frame
    # Tse_si = np.dot(Tsc_init,Tce_stand)
    # traj = mr.CartesianTrajectory(Tse_init, Tse_si, t, N, 5)
    
    # np.savetxt("traj.csv", traj, delimiter=",")
    TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_stand, k, t)
    
if __name__ == '__main__':
	main()