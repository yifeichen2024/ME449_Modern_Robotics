import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt

def write_csv_line(csv_filename, data):
    # Append a single line to a CSV file
    with open(csv_filename, 'a') as f:
        f.write(','.join(map(str, data)) + '\n')

def write_csv_mat(csv_filename, mat):
    # Clear and write matrix data to CSV
    open(csv_filename, 'w').close()
    for row in np.matrix(mat).tolist():
        write_csv_line(csv_filename, row)

def gripper_motion(thetalist, N, state):
    # Add gripper state to trajectory data
    return np.tile(np.append(thetalist[:-1], state), (N, 1))

def SE3matToArray13(matrix, gripperState):
    # Convert SE(3) matrix to a 13-element array
    T = np.array(matrix)
    return np.append(np.append(T[:3, :3], T[:3, 3]), gripperState).tolist()

def Array13toSE3Mat(array13):
    # Convert 13-element array to SE(3) matrix and gripper state
    array13 = np.array(array13)
    if len(array13) != 13:
        raise Exception("Array must have size 13")
    T = np.zeros((4, 4))
    T[:3, :3] = array13[:9].reshape(3, 3)
    T[:3, 3] = array13[9:12]
    T[3, 3] = 1
    gripperState = array13[-1]
    return T, gripperState

def ChassisSE3(phi, x, y):
    # Compute chassis SE(3) matrix from phi, x, and y
    R_curr = np.array([
        [np.cos(phi), -np.sin(phi), 0],
        [np.sin(phi),  np.cos(phi), 0],
        [          0,           0,  1],
    ])
    return mr.RpToTrans(R_curr, [x, y, 0.0963])


def PlotTrajectories(dt):
    """
    Plot angular and linear error components from simulation results.
    
    Args:
        dt (float): Simulation time step.
    """
    # Load the data from CSV
    Xerr_array = np.loadtxt('project/csv/Xerr.csv', delimiter=',')
    # Validate data
    assert Xerr_array.shape[1] == 6, "CSV file must have exactly 6 columns!"

    ang_array = Xerr_array[:, 0:3].T  # Angular components (columns 0 to 2 )
    lin_array = Xerr_array[:, 3:6].T  # Linear components (columns 3 to 5)
    
    t_array = np.arange(0, dt * Xerr_array.shape[0], dt)

    labels = ['x','y','z']

    plt.figure(1)
    for i, comp in enumerate(ang_array):
        plt.plot(t_array, comp, label=f'$\omega_{labels[i]}$')
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude of Angular Error (rad/s)')
    plt.legend()
    plt.title("Angular Error Components from Xerr")
    plt.show()

    plt.figure(2)
    for i, comp in enumerate(lin_array):
        plt.plot(t_array, comp, label=f'$v_{labels[i]}$')
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude of Linear Error (m/s)')
    plt.legend()
    plt.title("Linear Error Components from Xerr")

    plt.figure(3)
    for i, comp in enumerate(ang_array):
        plt.plot(t_array, comp, label=f'$\omega_{labels[i]}$')
    for i, comp in enumerate(lin_array):
        plt.plot(t_array, comp, label=f'$v_{labels[i]}$')

    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude of Error (rad/s)')
    plt.legend()
    plt.title("Error vs Time(t)")
    plt.show()

# import numpy as np
# import modern_robotics as mr

# def write_csv_line(csv_filename, data):
#     # Appends a single line of data to a CSV file.
#     with open(csv_filename, 'a') as f:
#         data_str = ','.join([str(i) for i in data]) + '\n'
#         f.write(data_str)

# def write_csv_mat(csv_filename, mat):
#     # clear and write in data 
#     f = open(csv_filename, 'w') 
#     f.close()

#     mat = np.matrix(mat).tolist()
#     for row in mat:
#             write_csv_line(csv_filename, row)

# def gripper_motion(thetalist, N, state):
#     # Add gripper status in the matrix
#     return np.tile(np.append(thetalist[:-1],state), (N,1))

# def SE3matToArray13(matrix, gripperState):
#     '''
#     Turn SE3 to 13 X 1 array, for the output of the function Trajectory
#     '''
#     T = np.array(matrix)
#     return np.append(np.append(T[:3,:3], T[:3,3]), gripperState).tolist()

# def Array13toSE3Mat(array13):
#     '''
#     Takes a 13-by-1 array, and turns it into the SE3 matrix representation of position
#      and a gripper state for the sake of trajectory loading.

#     This is used to turn SE(3) matrices from TrajectoryGenerator()
#     into a 2D array of positions. Do not confuse this with the 13-vector
#     of the robot's configuration: chassis phi+x+y, joint angles, wheel angles, gripper state
#     '''
#     array13 = np.array(array13)
#     if len(array13) != 13:
#         raise Exception("Size incorrect: array should be 1x13 or 13x1")

#     T = np.zeros((4,4))
#     T[:3,:3] = array13[0:9].reshape(3,3)
#     T[:3, 3] = array13[9:12]
#     T[3,3] = 1
#     gripperState = array13[-1]
#     return T, gripperState


# def ChassisSE3(phi, x, y):
#     '''Calculates the chassis transformation matrix, in SE(3), based
#     on the current angle, x, and y of the chassis.
#     '''
#     R_curr = np.array([
# 		[np.cos(phi), -np.sin(phi), 0],
# 		[np.sin(phi),  np.cos(phi), 0],
# 		[          0,           0,  1],
# 	])

#     T_curr = mr.RpToTrans(R_curr, [x, y, 0.0963])
#     return T_curr