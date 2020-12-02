#!/usr/bin/env python3

"""
Alexander Hay
ME449

Capstone - Milestone 2
"""

# Imports
import modern_robotics as mr
import numpy as np
from numpy import linalg as la


# Global Variables
MAX_SPEED = 12.3                # rad/s
ref_traj = []                   # Saved reference trajectory array
result = []                     # Saved result configurations
k = 1                           # The number of trajectory reference configurations per 0.01 seconds 
dt = 0.01

# Kinematics of youBot
l = 0.47/2                      # length of chassis
w = 0.3/2                       # width of chassis
h = 0.0475                      # height of chassis
r = 0.0475                      # radius of wheels

# Initial Configuration
# phi = -np.pi/5                  # Orientation
# x = 0.1                         # x displacement
# y = 0.2                         # y displancement
# w1,w2,w3,w4 = np.zeros(4)       # wheel speeds
# j1,j2,j3,j4,j5 = np.zeros(5)    # arm joint speeds
# g = 0                           # gripper state

H_pseudo = np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                     [ 1, 1, 1, 1],
                     [-1, 1,-1, 1]])

H = (r/4)* H_pseudo

H = np.append(np.array([[0,0,0,0],[0,0,0,0]]),H,axis=0)
H = np.append(H,np.array([[0,0,0,0]]),axis=0)


M = np.array([[ 1, 0, 0, 0.033 ],
              [ 0, 1, 0, 0     ],
              [ 0, 0, 1, 0.6546],
              [ 0, 0, 0, 1     ]])

Blist = np.array([[ 0, 0, 1,  0,      0.033, 0],
                  [ 0,-1, 0, -0.5076, 0,     0],
                  [ 0,-1, 0, -0.3526, 0,     0],
                  [ 0,-1, 0, -0.2176, 0,     0],
                  [ 0, 0, 1,  0,      0,     0]]).T

def shuffle(T, gripper_state):
    """
    Parameters
    ----------
    T :                             4x4 matrix Tse
    gripper_state :                 0=open; 1=closed

    Returns
    -------
    vec :                           13 row vector

    """
    vec = [T[0][0],T[0][1],T[0][2],\
           T[1][0],T[1][1],T[1][2],\
           T[2][0],T[2][1],T[2][2],\
           T[0][3],T[1][3],T[2][3], gripper_state]
        
    return vec

def close_gripper():
    """
    Gripper State: 1 = closed
    Gripper takes 0.625s to open/close
    """
    # t = 0.63
    # N = t*k/0.01

    temp = ref_traj[-1]
    temp[-1] = 1
    temp = np.full((63,13),temp)
    for i in range(len(temp)):
        ref_traj.append(temp[i])

def open_gripper():
    """
    Gripper State: 0 = closed
    Gripper takes 0.625s to open/close
    """
    
    # t = 0.63
    # N = t*k/0.01

    temp = ref_traj[-1]
    temp[-1] = 0
    temp = np.full((63,13),temp)
    for i in range(len(temp)):
        ref_traj.append(temp[i])

def jacobsLadder(current_config):
    """
    Parameters
    ----------
    phi, x, y :                     robot chassis configuration
    j1, j2, j3, j4, j5              arm configuration
    w1, w2, w3, w4                  wheel speed configuration
    gripper_state :                 0=open; 1=closed.

    Returns
    -------
    x :                             Forward kinematics
    Ja, Jb :                        Jacobian matricies
    """
    phi,x,y,j1,j2,j3,j4,j5,w1,w2,w3,w4,g = current_config
    thetalist = np.array([j1,j2,j3,j4,j5])
    # Tsb = np.array([[]])
    Tsb = np.array([[np.cos(phi),-np.sin(phi), 0, x],
                    [np.sin(phi), np.cos(phi), 0, y],
                    [0       , 0       , 1, 0.0963],
                    [0       , 0       , 0, 1]])
    
    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0, 1]])
    
    Ja = mr.JacobianBody(Blist,thetalist)
    T0e = mr.FKinBody(M,Blist,thetalist)
    X = np.dot(np.dot(Tsb,Tb0),T0e)
    Jb = np.dot(mr.Adjoint(np.dot(la.inv(T0e),Tb0)), H)

    return X,Ja,Jb

def nextState(current_config, control_config):
    """
    current_config:     
        phi, x, y,                  Chassis
        j1, j2, j3, j4, j5          Arm
        w1, w2, w3, w4              Wheels
        g                           Gripper
                        
    control_config:     
        j1d, j2d, j3d, j4d, j5d     Arm joint Velocites
    
    dt:                             Timestamp delta_t

    MAX_SPEED:                      Maximum angular speed of the arm joints and the wheels (rad/s)               
    """
    # Unpack current_config
    phi, x, y, j1, j2, j3, j4, j5, w1, w2, w3, w4, g = current_config    
    # Unpack control_config
    j1d, j2d, j3d, j4d, j5d, w1d, w2d, w3d, w4d = control_config
    
    # Define Tsb
    Tsb = np.array([[np.cos(phi),-np.sin(phi), 0, x],
                    [np.sin(phi), np.cos(phi), 0, y],
                    [          0,           0, 1, 0.0963],
                    [          0,           0, 0, 1]])
   
    # wz, vx, vy
    wdt = np.array([[w1d],[w2d],[w3d],[w4d]]) * dt
    Vb = (r/4) * np.dot(H_pseudo,wdt)
    Vb6 = np.transpose([[0,0,Vb[0],Vb[1],Vb[2],0]])
    
    Tbb = mr.MatrixExp6(mr.VecTose3(Vb6))
    Tb = np.dot(Tsb,Tbb)

    pp = np.arccos(Tb[0][0])
    xx = Tb[0][3]
    yy = Tb[1][3]
    
    j1_new = j1 + (j1d * dt)
    j2_new = j2 + (j2d * dt)
    j3_new = j3 + (j3d * dt)
    j4_new = j4 + (j4d * dt)
    j5_new = j5 + (j5d * dt)
    
    w1_new = w1 + (w1d * dt)
    w2_new = w2 + (w2d * dt)
    w3_new = w3 + (w3d * dt)
    w4_new = w4 + (w4d * dt)
    
    result.append([pp,xx,yy,j1_new,j2_new,j3_new,j4_new,j5_new,w1_new,w2_new,w3_new,w4_new,g])
    
    return pp,xx,yy,j1_new,j2_new,j3_new,j4_new,j5_new,w1_new,w2_new,w3_new,w4_new,g

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
    
    # Raise end-effector back up
    traj = mr.CartesianTrajectory(Tse_fg, Tse_fs, t, N, 5)
    for i in range(len(traj)):
        temp = shuffle(traj[i], 0)
        ref_traj.append(temp)
    
    print("Trajectory Generated")

    
def main():
    phi = -np.pi/5                  # Orientation
    x = 0.1                         # x displacement
    y = 0.2                         # y displancement
    w1,w2,w3,w4 = np.zeros(4)       # wheel speeds
    j1,j2,j3,j4,j5 = np.zeros(5)    # arm joint speeds
    g = 0                           # gripper state
    current_config = ([phi, x, y, j1, j2, j3, j4, j5, w1, w2, w3, w4, g])
    result.append(current_config)
      
    control_config = (0, 0, 0, 0, 0, -10, 10, -10, 10)

    # cube_init = np.transpose([[1, 0, 0]])               # (1m, 0m, 0 rad)
    # cube_final = np.transpose([[0, -1, -np.pi/2]])      # (0m,-1m, ) pi/2 rad)
       
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
    
    for i in range(len(ref_traj)-1):
        current_config = ([phi,x,y,j1,j2,j3,j4,j5,w1,w2,w3,w4,g])
        X, Ja, Jb = jacobsLadder(current_config)
        phi,x,y,j1,j2,j3,j4,j5,w1,w2,w3,w4,g = nextState(current_config,control_config)
        
    np.savetxt("ref_traj.csv", ref_traj, delimiter=",")
    print("Trajectory saved as ref_traj.csv")
    np.savetxt("ref_chassis.csv", result, delimiter=",")
    print("Chassis config saved as ref_chassis.csv")
    
    
if __name__ == '__main__':
	main()
    