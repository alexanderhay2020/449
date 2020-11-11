zip file contents:

path_1.csv ------- joint vectors for robot falling from home configuration
path_2.csv ----- joint vectors for robot falling from [-1,-1,0,0,0,0]
vid_1.mp4 ------ video of robot falling from home configuration (3 sec)
vid_2.mp4 ------ video of robot falling from [-1,-1,0,0,0,0] (5 sec)

Code produces 2 .csv files. Nothing special other than to run it. Since the robot is only affected by gravity and no other forces or torques, Ftip and taulist are defined as:

Ftip = [0,0,0,0,0,0]

taulist = [0,0,0,0,0,0]

ddthetalist is calculated using the ForwardDynamics funciton in the Modern Robotics library. EulerStep is calculated for each dt.
