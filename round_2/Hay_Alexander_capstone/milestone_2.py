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
k = 1

def shuffle(T, gripper_state):
    vec = [T[0][0],T[0][1],T[0][2],\
            T[1][0],T[1][1],T[1][2],\
            T[2][0],T[2][1],T[2][2],\
            T[0][3],T[1][3],T[2][3], gripper_state]
        
    return vec

def close_gripper():
    # Gripper State: 1 = closed
    # Gripper takes 0.625s to open/close
    
    t = 0.63
    N = t*k/0.01

    temp = ref_traj[-1]
    temp[-1] = 1
    temp = np.full((63,13),temp)
    for i in range(len(temp)):
        ref_traj.append(temp[i])

def open_gripper():
    # Gripper State: 0 = closed
    # Gripper takes 0.625s to open/close
    
    t = 0.63
    N = t*k/0.01

    temp = ref_traj[-1]
    temp[-1] = 0
    temp = np.full((63,13),temp)
    for i in range(len(temp)):
        ref_traj.append(temp[i])

def nextState(current_config, control_config):

def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_stand, k):
    """
    k:                  The number of trajectory reference configurations per 0.01 seconds
                        Ex: k=10, t=0.01; freq_controller = k/t = 1000Hz
    
    Tse_init:           The initial configuration of the end-effector for the reference trajectory
    Tsc_init:           The cube's initial configuration
    Tsc_final:          The cube's desired final configuration
    Tce_grasp:          The end-effector's configuration relative to the cube when it is grasping the cube
    Tce_stand:          The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
    """
    t = 2
    N = t*k/0.01
    
    # Move end-effector to cube
    Tse_is = np.dot(Tsc_init, Tce_stand)
    traj = mr.CartesianTrajectory(Tse_init, Tse_is, t, N, 5)
    for i in range(len(traj)):
        temp = shuffle(traj[i], 0)
        ref_traj.append(temp)
    
    # Drop end-effector to cube height
    Tse_ig = np.dot(Tsc_init, Tce_grasp) 
    traj = mr.CartesianTrajectory(Tse_is, Tse_ig, t, N, 5)
    for i in range(len(traj)):
        temp = shuffle(traj[i], 0)
        ref_traj.append(temp)

    # Close gripper
    close_gripper()
    
    # Raise end-effector back up
    traj = mr.CartesianTrajectory(Tse_ig, Tse_is, t, N, 5)
    for i in range(len(traj)):
        temp = shuffle(traj[i], 1)
        ref_traj.append(temp)

    # Move to final position
    t = 4
    N = t*k/0.01
    Tse_fs = np.dot(Tsc_final, Tce_stand)         
    traj = mr.CartesianTrajectory(Tse_is, Tse_fs, t, N, 5)
    for i in range(len(traj)):
        temp = shuffle(traj[i], 1)
        ref_traj.append(temp)
        
    # Drop end-effector to final cube height
    t = 2
    N = t*k/0.01
    Tse_fg = np.dot(Tsc_final, Tce_grasp) 
    traj = mr.CartesianTrajectory(Tse_fs, Tse_fg, t, N, 5)
    for i in range(len(traj)):
        temp = shuffle(traj[i], 1)
        ref_traj.append(temp)
    
    # Open gripper
    open_gripper()
    
    print("Trajectory Generated")

    
def main():
    
    # cube_init = np.transpose([[1, 0, 0]])               # (1m, 0m, 0 rad)
    # cube_final = np.transpose([[0, -1, -np.pi/2]])      # (0m,-1m, ) pi/2 rad)
    
    # M = np.array([[ 1, 0, 0, 0.033 ],
    #               [ 0, 1, 0, 0     ],
    #               [ 0, 0, 1, 0.6546],
    #               [ 0, 0, 0, 1     ]])
    
    # Blist = np.array([[ 0, 0, 1,  0,      0.033, 0],
    #                   [ 0,-1, 0, -0.5076, 0,     0],
    #                   [ 0,-1, 0, -0.3526, 0,     0],
    #                   [ 0,-1, 0, -0.2176, 0,     0],
    #                   [ 0, 0, 1,  0,      0,     0]]).T
    
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
    
    Tce_grasp = np.array([[ 0, 0, 1, 0],
                          [ 0, 1, 0, 0],
                          [-1, 0, 0, 0],
                          [ 0, 0, 0, 1]])
    
    Tce_stand = np.array([[ 0, 0, 1,-0.3],
                          [ 0, 1, 0, 0],
                          [-1, 0, 0, 0.5],
                          [ 0, 0, 0, 1]])
    	
    TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_stand, k)
    
    np.savetxt("ref_traj.csv", ref_traj, delimiter=",")
    print("Trajectory saved as ref_traj.csv")
    
if __name__ == '__main__':
	main()
    