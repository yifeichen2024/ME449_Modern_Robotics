from code import Puppet, filter_nan
import matplotlib.pyplot as plt
import time 
import numpy as np

# config
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

# Part 1
damping_p1 = 0.0
stiffness_p1 = 0.0
restLength_p1 = 0.0
tf_p1 = 5
dt_p1_fine = 0.005
dt_p1_coarse = 0.015

g = np.array([0, 0, -9.81])
thetalist0 = [0,0,0,0,0,0]
dthetalist0 =[0,0,0,0,0,0]
springPosn = [0, 1, 1]  

# Part1 (a)
# traj_p1_fine, d = Puppet(damping_p1, stiffness_p1, restLength_p1, tf_p1, dt_p1_fine, g, Mlist, Glist, Slist, thetalist0, dthetalist0)
# np.savetxt("yifei_chen_ass3/csv/Part1_a.csv", traj_p1_fine, delimiter=",", fmt="%7.6f")
# traj_p1_coarse = Puppet(damping_p1, stiffness_p1, restLength_p1, tf_p1, dt_p1_coarse, g, Mlist, Glist, Slist, thetalist0, dthetalist0)
# traj_p1_coarse = filter_nan(traj_p1_coarse)
# np.savetxt("yifei_chen_ass3/csv/Part1_b.csv", traj_p1_coarse, delimiter=",", fmt="%7.6f")

# Part 2
# Positive damping
damping_p2_pos = 3
stiffness_p2 = 0.0
restLength_p2 = 0.0
springPosn = [0, 1, 1]  
tf_p2 = 5 
dt_p2 = 0.01

# Part 2 (a)
# traj_p2_damping_positive = Puppet(damping_p2_pos, stiffness_p2, restLength_p2, tf_p2, dt_p2, g, Mlist, Glist, Slist, thetalist0, dthetalist0)
# np.savetxt("yifei_chen_ass3/csv/Part2_a.csv", traj_p2_damping_positive, delimiter=",", fmt="%7.6f")

# damping_p2_negative = -1
# traj_p2_damping_negative = Puppet(damping_p2_negative, stiffness_p2, restLength_p2, tf_p2, dt_p2, g, Mlist, Glist, Slist, thetalist0, dthetalist0)
# traj_p2_damping_negative = filter_nan(traj_p2_damping_negative)
# np.savetxt("yifei_chen_ass3/csv/Part2_b.csv", traj_p2_damping_negative, delimiter=",", fmt="%7.6f")

# Part 3
g_p3 = np.array([0.0, 0.0, 0.0])
springPosn3 = [0, 1, 1]  
stiffness_p3 = 8
restLength_p3 = 0
damping_p3 = 0

tf_p3 = 10
dt_p3 = 0.01
# traj_p3_spring = Puppet(damping_p3, stiffness_p3, restLength_p3, tf_p3, dt_p3, g_p3, Mlist, Glist, Slist, thetalist0, dthetalist0)
# np.savetxt("yifei_chen_ass3/csv/Part3_a.csv", traj_p3_spring, delimiter=",", fmt="%7.6f")

# damping_p3_b = 3
# traj_p3_spring_damping = Puppet(damping_p3_b, stiffness_p3, restLength_p3, tf_p3, dt_p3, g_p3, Mlist, Glist, Slist, thetalist0, dthetalist0)
# np.savetxt("yifei_chen_ass3/csv/Part3_b.csv", traj_p3_spring_damping, delimiter=",", fmt="%7.6f")

# Part 4
# need to change the the code for in referncepos function 
g_p4 = np.array([0.0, 0.0, 0.0])
springPosn4 = [1, 1, 1]  
stiffness_p4 = 8
restLength_p4 = 0
damping_p4 = 3

tf_p4 = 10
dt_p4 = 0.01

traj_p4 = Puppet(damping_p4, stiffness_p4, restLength_p4, tf_p4, dt_p4, g_p4, Mlist, Glist, Slist, thetalist0, dthetalist0)
np.savetxt("yifei_chen_ass3/csv/Part4.csv", traj_p4, delimiter=",", fmt="%7.6f")

