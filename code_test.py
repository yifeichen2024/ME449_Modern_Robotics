import modern_robotics as mr
import numpy as np

def calculate_M(Mlist):
    # 初始化齐次变换矩阵为单位矩阵
    M07 = np.eye(4)
    
    # 从 M01 开始，逐个相乘直到 M67，得到 M07
    for M in Mlist:
        M07 = np.dot(M07, M)
    
    return M07

# help(mr.ForwardDynamics)
# Function to return reference spring position based on current time
def referencePos(current_t):
    # For now, return a constant value for spring position; can be updated to make it time-dependent
    spring_pos = np.array([0, 1, 1])  # for part 3 and 4. Example constant spring position
    return spring_pos


# Main puppet function for simulating robot with damping and spring effect
def puppet(thetalist, dthetalist, g, Mlist, Slist, Glist, t, dt, damping, stiffness, restLength):
    # Initialize lists to store joint values and rates over time
    thetamat = [thetalist.copy()]
    dthetmat = [dthetalist.copy()]
    
    # thetamat = np.array(thetalist.copy())
    # dthetmat = np.array(dthetalist.copy())
    
    # Calculate the number of time steps
    N = int(t / dt)
    # Initial conditions
    taulist = np.zeros(len(thetalist))  # No active joint torques
    Ftip = np.zeros(6)  # No external force at the end-effector

    M = calculate_M(Mlist)

    for i in range(N):
        # Call the referencePos. Get the current spring position
        spring_pos = referencePos(i * dt)
        # Get the current end-effector position
        T_eb = mr.FKinSpace(M, Slist, thetamat[-1])
        end_effector_pos = T_eb[:3, 3] 
        # print(end_effector_pos)

        # Calculate the spring force if the distance is greater than rest length
        displacement = spring_pos - end_effector_pos

        distance = np.linalg.norm(displacement)

        force_magnitude = 0

        if distance > restLength:
            force_magnitude = stiffness * (distance - restLength)
        
        # Calculate force direction
        force_direction = displacement / distance if distance != 0 else np.zeros(3)

        # Calculate spring force vector in {s} frame
        spring_force = force_magnitude * force_direction
        print(spring_force)
        Ftip[:3] = spring_force  # Set the force part of the wrench

        # Calculate the moment due to spring force
        moment_arm = end_effector_pos - np.array([0, 0, 0])  # Assuming base frame origin is [0, 0, 0]
        spring_moment = np.cross(moment_arm, spring_force)
        Ftip[3:] = spring_moment  # Set the moment part of the wrench

        # Add damping torques at each joint
        tau_damping = -damping * dthetalist
        taulist = tau_damping
    
        # For the free fall 
        # Use Forward Dynamics to compute joint accelerations
        ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist)
        # Use Forward Dynamics to compute joint accelerations
        
        # Integrate to find the new state using EulerStep
        thetalist, dthetalist = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)

        # Store the results
        # thetamat = np.vstack([thetamat,thetalist])
        # dthetmat = np.vstack([dthetmat,dthetalist])
        thetamat.append(thetalist.copy())
        dthetmat.append(dthetalist.copy())

    # # Convert the results to numpy arrays for easy handling
    thetamat = np.array(thetamat)
    dthetmat = np.array(dthetmat)

    return thetamat, dthetmat
