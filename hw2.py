import math
import numpy as np
import modern_robotics as mr

m = np.array(([1,0,0,0],
              [0,1,0,2],
              [0,0,1,1],
              [0,0,0,1]))

blist = np.array(([0,0,1,-2,0,0],
                  [0,0,1,-1,0,0],
                  [0,0,1,0,0,0],
                  [0,0,0,0,0,1]))

theta = np.array(([0,math.pi/2,-math.pi/2,1]))

x = mr.FKinBody(m,blist.T,theta)
y = mr.FKinSpace(m,blist.T,theta)

print x
print y
