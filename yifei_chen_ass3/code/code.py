import numpy as np 
import modern_robotics as mr

# EulerStep 
def ModEulerStep(thetalist, dthetalist, ddthetalist, dt):
    """EulerStep from the MR library, but with an additional
    second-order term from acceleration contributing to changes in position.
    """
    return thetalist + (dt * np.array(dthetalist) + 0.5 * dt**2 * np.array(dthetalist) ), \
           dthetalist + dt * np.array(ddthetalist)

# Nan value due to the errors from mr lib
def filter_nan(array):
    """ 
    Remove the NAN value in csv
    """
    firstnan = None
    for i in range(len(array)):
        row = array[i,:]
        nanmask = np.isnan(row)
        if np.any(nanmask): 
            firstnan = i
            break

    if firstnan:
        array = array[:firstnan]
    return array


def SpringForce(Slist, thetalist, Mlist, stiffness, springPos, restLength):
    '''
    Get the spring force 
    return for the Ftip, which is a 6x1 end-effector wrench caused by the spring force
    '''

    # Get M07
    M_matrices = [np.matrix(M) for M in Mlist]
    M_ee = M_matrices[0] * M_matrices[1] * M_matrices[2] * M_matrices[3] * M_matrices[4] * M_matrices[5] * M_matrices[6]
    
    Tsb = mr.FKinSpace(M_ee, Slist, thetalist)
    springPos_4 = np.append(np.array(springPos), [1]) 
    springPos_4 = np.matrix(springPos_4).T
    
    pb = mr.TransInv(Tsb) * springPos_4
    pb = np.array(pb[0:3].tolist())
    dp = pb[:]
    
    #direction of force 
    spring_dir = -dp / np.linalg.norm(dp)
    
    # magnitude of force 
    force_mag = stiffness * (np.linalg.norm(pb) - restLength)
    force = (force_mag * spring_dir).T[0].tolist()
    Fb_tip = np.array([0, 0, 0, force[0], force[1], force[2]])
    
    return Fb_tip.tolist()

def DampingForce(B, thetad_list):
    '''
    Add damping foce for each joint.
    '''
    tau_list = -B * np.array(thetad_list)
    return tau_list.tolist()

def referencePos(current_t):
    # For part 4: Spring position oscillates sinusoidally along a line
    # Starting from (1, 1, 1) to (1, -1, 1), completing two full back-and-forth cycles in 10 seconds
    amplitude = 1  # Maximum displacement along the y-axis
    frequency = 0.2  # Frequency to complete two full back-and-forth cycles in 10 seconds
    y_position = amplitude * np.sin(2 * np.pi * frequency * current_t)
    spring_pos = np.array([1, y_position, 1])
    
    # For part 1, 2, 3
    # return np.array([0, 1, 1])
    return spring_pos

# the main function 
def Puppet(damping, stiffness, restLength, tf, dt, g, Mlist, Glist, Slist, thetalist0, thetadlist0): 
    
    thetalist = np.array(thetalist0)
    thetadlist = np.array(thetadlist0)  
    t_array = np.arange(0, tf, dt)
    thetamat = np.zeros([len(t_array), len(thetalist)]) # N timesteps, n angles    
    dthetmat = np.zeros([len(t_array), len(thetalist)])
    for i, t in enumerate(t_array):
                
        #calculate forces; call ForwardDynamics() with starting values of t, td, tdd lists
        taulist = DampingForce(damping, thetadlist) 
        
        # problem 4, return for the varied springpos 
        springPos = referencePos(i)
        
        Fb_tip = SpringForce(Slist, thetalist, Mlist, stiffness, springPos, restLength)

        thetaddlist = mr.ForwardDynamics(thetalist, thetadlist, taulist, g, Fb_tip, Mlist, Glist, Slist)
        
        # numerical integration
        thetalist_new, thetadlist_new = mr.EulerStep(thetalist, thetadlist, thetaddlist, dt)
        
        # - store value of theta at next timestep in an array; reset theta and thetad       
        thetamat[i,:] = thetalist_new
        dthetmat[i,:] = thetadlist_new
        thetalist = thetalist_new
        thetadlist = thetadlist_new
        
    return thetamat, dthetmat