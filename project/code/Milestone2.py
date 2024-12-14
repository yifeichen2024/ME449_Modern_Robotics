import numpy as np
import modern_robotics as mr
from tqdm import tqdm

from additional import * 
from config import *


def TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_grasp, Tce_standoff, k):
    '''
    Generate the reference trajectory for the end-effector frame {e}.

    Input:
    - Tse_i: initial config of end effector
    - Tsc_i: cube's initial config
    - Tsc_f: cube's desired final config
    - Tce_grasp: config of end effector rel. to cube while grasping
    - Tce_standoff: end-effector's standoff configuration above the cube, before and after grasping
    - k: The number of trajectory reference configurations per 0.01 seconds

    Output:
    - arr_new: 13 by 1 array 
    '''

    method = 5

    time_grasp = 0.625
    time_standoff = 3

    t_array = [
        8,
        time_standoff,
        time_grasp,
        time_standoff,
        8,
        time_standoff,
        time_grasp,
        time_standoff,
    ] 

    Tf = sum(t_array)
    N = Tf / (k*0.01)

    # define number of data points in each segment of trajectory 
    N_array = N_array = list(map(lambda x: int(x /(k*0.01)), t_array))

    Tse_standoff_i = np.dot(Tsc_i, Tce_standoff)
    Tse_grasp_i    = np.dot(Tsc_i, Tce_grasp)
    Tse_standoff_f = np.dot(Tsc_f, Tce_standoff)
    Tse_grasp_f    = np.dot(Tsc_f, Tce_grasp)

    # Joint limits 
    motion_limits = [
        (Tse_i, Tse_standoff_i),
        (Tse_standoff_i, Tse_grasp_i),
        (None, None),
        (Tse_grasp_i, Tse_standoff_i),
        (Tse_standoff_i, Tse_standoff_f),
        (Tse_standoff_f, Tse_grasp_f),
        (None, None),
        (Tse_grasp_f, Tse_standoff_f)
        ]

    gripperState = 0
    positions_array = []

    print("TrajectoryGenerator():")
    # LOG for visualize the process 
    for i in tqdm(range(len(N_array))):

        if (i == 2 or i == 6):
            gripperState = not gripperState
            array_rep13_list = gripper_motion(array_rep13_list[-1], N_array[i], gripperState)

        else:
            SE3_matrices = mr.ScrewTrajectory(motion_limits[i][0],motion_limits[i][1], \
                t_array[i], N_array[i], method)
            array_rep13_list = [SE3matToArray13(x, gripperState) for x in SE3_matrices]

        gripperState = array_rep13_list[-1][-1]
        positions_array.append(np.array(array_rep13_list))

    arr_new = np.concatenate(positions_array, axis=0)
    write_csv_mat('project/csv/trajectory.csv', arr_new)

    return arr_new

if __name__ == '__main__':
    # choose starting Tse equal to the values found in CoppeliaSim for Scene 8
    Tse_i = mr.RpToTrans(np.identity(3),[9.4E-2, 9.4E-2, 5.94E-1])
    k = 1
    TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_grasp, Tce_standoff, k)