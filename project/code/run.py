# numpy version == 2.1.3 
import modern_robotics as mr
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R

from additional import *
from config import *

from Milestone1 import NextState
from Milestone2 import TrajectoryGenerator
from Milestone3 import FeedbackControl, CalculateJe

    

def testJointLimits(robot_config12):
    """
    Check if joint angles exceed their limits.
    
    Args:
        robot_config12 (list): Current 12-vector robot configuration (base, joints, wheels).
    
    Returns:
        limit_reached (bool): True if any joint violates its limits.
        indices (list): Indices of joints exceeding their limits.
    """

    joint_vals = robot_config12[3:8].flatten()

    joints_beyond_limits = np.logical_or(
            joint_vals < joint_limits_kuka[0].flatten(),
            joint_vals > joint_limits_kuka[1].flatten()
    )

    limit_reached = np.any(joints_beyond_limits)
    indices = np.nonzero(joints_beyond_limits)[0].tolist()

    return limit_reached, indices

    

def main(dt):

    # =====Best=====
    # ===FF===
    # Kp = 0 
    # Ki = 0 

    # init_base    = [np.pi/4, 0, 0]       #chassis phi, x, y
    # init_joints  = [0, -np.pi/4, 0, 0, np.pi/2] # 5 robot joints
    
    # ===FF + P Control=== 
    Kp = np.identity(6) 
    Ki = 0

    init_base    = [np.pi/4, 0, 0]       #chassis phi, x, y
    init_joints  = [0, -np.pi/4, 0, 0, np.pi/2] # 5 robot joints

    # =====overshoot=====
    # ===PI Control===
    # Kp = 4*np.identity(6) 
    # Ki = np.identity(6)

    # init_base    = [np.pi/4, 0, 0]       #chassis phi, x, y
    # init_joints  = [0, -np.pi/12, 0, 0, np.pi/4] # 5 robot joints

    # =====NewTask=====
    # P Control
    # Kp = 1 * np.identity(6)
    # Ki = 0 # np.identity(6)

    # init_base    = [np.pi/4, 0, 0]       #chassis phi, x, y
    # init_joints  = [0, -np.pi/12, 0, 0, np.pi/4] # 5 robot joints
    
    # Tsc_i =  mr.RpToTrans(np.identity(3), [0.5, 0, 0.025]).tolist()
    # Tsc_f =  mr.RpToTrans(np.identity(3), [0, -1, 0.025]).tolist()

    # =====Joint limits=====
    # Kp = np.identity(6)
    # Ki = 0 

    # init_base    = [np.pi/4, 0, 0]       #chassis phi, x, y
    # init_joints  = [0, -np.pi/12, 0, 0, np.pi/4] # 5 robot joints

    # Tsc_i =  mr.RpToTrans(np.identity(3), [0.25, 0.2, 0.025]).tolist()
    # Tsc_f =  mr.RpToTrans(np.identity(3), [0, -1, 0]).tolist()

    # trajectory, error, and motion parameters
    k = 1
    w_max = 10
    rcond = 1E-3 
    Xerr_int = np.zeros((6,1))
    Xerr = 0

    init_wheels  = [0, 0, 0, 0]    # 4 wheel posns
    gripperState = 0               # open the gripper
    robot_config13 = init_base + init_joints + init_wheels + [gripperState]

    # desired initial config
    Tse_0 = np.array([
        [ 0, 0, 1,   0],
        [ 0, 1, 0,   0],
        [-1, 0, 0, 0.5],
        [ 0, 0, 0,   1]
    ])

    # Milestone 2 generate reference trajectory. 
    ref_traj = TrajectoryGenerator(Tse_0, Tsc_i, Tsc_f, Tce_grasp, Tce_standoff, k)

    # iterate through trajectories generated
    N = len(ref_traj)

    # arrays for storage of results
    Xerr_array = np.zeros((N-1, 6)) 
    u_thetad = np.zeros(9)

    robot_config13_array = np.zeros((N,13))
    robot_config13_array[0] = robot_config13[:]

    print("\nMain():")
    for i in tqdm(range(N-1)):

        # calculate current SE(3) config from robot_config13
        base_array    = robot_config13[0:3] 
        joints_array  = robot_config13[3:8]
        robot_config8 = robot_config13[0:8] 
        wheels_array  = robot_config13[8:12]

        # find current position of end effector
        [phi, x, y] = base_array
        Tsb  = ChassisSE3(phi, x, y).tolist()
        T0e = mr.FKinBody(M0e, Blist, joints_array)
        Tse = np.dot(np.dot(Tsb, Tb0), T0e)

        # get desired SE(3) matrix from the reference trajectory
        rep13_array = ref_traj[i] # representation of robot config in 1x13 vector
        Xd, _ = Array13toSE3Mat(rep13_array) 

        rep13_next = ref_traj[i+1]
        Xd_next, gripper_next = Array13toSE3Mat(rep13_next)

        # Milestone3 extract joint and wheel speeds from twist
        V_next, [Xerr, Xerr_int] = FeedbackControl(Tse, Xd, Xd_next, Kp, Ki, Xerr_int, dt)
        Je = CalculateJe(robot_config8, Tb0, M0e, Blist)
        u_thetad = np.dot(np.linalg.pinv(Je, rcond=rcond), V_next)
        u_thetad_clip = np.clip(u_thetad, -w_max, w_max)

        # calculate new phi, x, y, joint angles, wheel angles after following twist
        robot_config12 = NextState(robot_config13[:12], u_thetad_clip, dt, w_max)

        limit_reached, indices = testJointLimits(robot_config12)

        if (limit_reached):
            Je_inds = np.array(indices) + 4
            for j in Je_inds: Je[:,j] = 0

            u_thetad = np.dot(np.linalg.pinv(Je, rcond=rcond), V_next)
            u_thetad_clip = np.clip(u_thetad, -w_max, w_max)

            # calculate new phi, x, y, joint angles, wheel angles after following twist
            robot_config12 = NextState(robot_config13[:12], u_thetad_clip, dt, w_max)
            pass        

        # add to arrays and move to next iteration of loop
        robot_config13[:12] = robot_config12[:]
        robot_config13[-1] = gripper_next
        robot_config13_array[i+1] = robot_config13
        Xerr_array[i,:] = Xerr.T

    # log information 
    print(f'\nData of 13-vector generated.\n')
    print(f'\nError generated.\n')
    # Save robot configuration and error array to CSV file
    np.savetxt('project/csv/robotconfig13.csv', robot_config13_array, delimiter=',')
    np.savetxt('project/csv/Xerr.csv', Xerr_array, delimiter=',')
    
    print("Results written to /csv folder.")


if __name__ == '__main__':
    dt = 0.01
    main(dt)
    PlotTrajectories(dt)