import numpy as np
from code import puppet

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



# Example usage
thetalist = np.array([0, 0, 0, 0, 0, 0])
dthetalist = np.array([0, 0, 0, 0, 0, 0])
g = np.array([0, 0, -9.81])

t = 5  # Total simulation time in seconds
dt = 0.01  # Time step in seconds
damping = 0  # Damping coefficient in Nms/rad
stiffness = 1  # Spring stiffness in N/m
restLength = 0.0  # Rest length of the spring in meters

# print(Mlist)
# Run the simulation
thetamat, dthetmat = puppet(thetalist, dthetalist, g, Mlist, Slist, Glist, t, dt, damping, stiffness, restLength)

# # Save the results to a CSV file
np.savetxt("yifei_chen_ass3/Part3_spring_test.csv", thetamat, delimiter=",", fmt="%7.6f")
# # np.savetxt("yifei_chen_ass3/Part1_free_fall_dt_big.csv", thetamat, delimiter=",", fmt="%7.6f")
# # np.savetxt("yifei_chen_ass3/Part2_damping_negative.csv", thetamat, delimiter=",", fmt="%7.6f")
# # np.savetxt("yifei_chen_ass3/Part3_spring.csv", thetamat, delimiter=",", fmt="%7.6f")