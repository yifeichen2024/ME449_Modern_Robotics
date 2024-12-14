import modern_robotics as mr
import numpy as np
# 2-C 5-1
# 1- D, 3-A 4-B  
# 1
# B

# 2
# 10 * (t/T)^3 - 15 * (t/T)^4 +6*(t/T)^5

# 3
# 7

# 4
# c

# 5
# 0.68
# help(mr.CartesianTrajectory)

# 6 
[[ 0.04229152, -0.04057269,  0.99828116,  0.93313173],
 [ 0.99828116,  0.04229152, -0.04057269,  1.97198558],
 [-0.04057269,  0.99828116,  0.04229152,  2.88912137],
 [ 0.        ,  0.        ,  0.        ,  1.        ]]

# 7
[[ 0.01404133, -0.01384686,  0.99980553,  0.98846721],
 [ 0.99980553,  0.01404133, -0.01384686,  1.97693441],
 [-0.01384686,  0.99980553,  0.01404133,  2.96540162],
 [ 0.        ,  0.        ,  0.        ,  1.        ]]

Xstart = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
X_end = np.array([[0, 0, 1, 1],
                 [1, 0, 0, 2],
                 [0, 1, 0, 3],
                 [0, 0, 0, 1]])
Tf = 10
N = 10
method = 5

ans = mr.CartesianTrajectory(Xstart, X_end, Tf, N, method)
print(ans[-2])

# method = 3

# ans = mr.ScrewTrajectory(Xstart, X_end, Tf, N, method)
# print(ans[-2])

# T = 5
# t = 3
# ans = mr.QuinticTimeScaling(T, t)
# print(ans)