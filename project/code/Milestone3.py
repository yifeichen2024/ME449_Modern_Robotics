import modern_robotics as mr
from config import *
from additional import *

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, Xerr_int, dt):
	'''	
	Calculates the end-effector twist for the next timestep.

    Inputs:
    - X: Current end-effector configuration (Tse).
    - Xd: Reference configuration (Tse,d).
    - Xd_next: Next reference configuration (Tse,d,next).
    - Kp, Ki: PI gain matrices.
    - Xerr_int: Accumulated error.
    - dt: Timestep Δt.

    Outputs:
    - V_new: Commanded end-effector twist.
    - Xerr: Current error.
    - Xerr_int: Updated integral of error.
	'''
	
	# Compute adjoint transformation from desired to current frame
	Ted = np.dot(mr.TransInv(X), Xd)
	Ad_ed = mr.Adjoint(Ted)
	
	# Compute error twist
	se3mat_err = mr.MatrixLog6(Ted)
	Xerr = mr.se3ToVec(se3mat_err).reshape(6,1)

	# Compute twist rate
	se3mat_unit = mr.MatrixLog6(np.dot(mr.TransInv(Xd), Xd_next))
	Vd = (1/dt) * mr.se3ToVec(se3mat_unit)

	# Commend twist 
	V_new = (np.dot(Ad_ed, Vd).flatten() + \
			np.dot(Kp, Xerr).flatten() +  \
			np.dot(Ki, Xerr_int).flatten()).reshape((6,1))

	# Add to the integral of error
	Xerr_int += (Xerr * dt)

	return V_new, (Xerr, Xerr_int)


def CalculateJe(robot_config8, Tb0, M0e, Blist):
	'''
	Computes the combined Jacobian Je = [Jbase, Jarm].

    Inputs:
    - robot_config8: Robot configuration (chassis + joint angles).
    - Tb0: Transformation from chassis to arm base.
    - M0e: End-effector home configuration.
    - Blist: Screw axes in the end-effector frame.

    Output:
    - Je: Combined Jacobian.
	'''
	q_array     = robot_config8[0:3]
	theta_array = robot_config8[3:8]

	# Wheel velocity mapping to chassis velocity
	r = 0.0475
	l = 0.47/2.0
	w = 0.3/2.0

	c = 1/(l+w) 
	F = r/4 * np.array([
		[-c, c,  c, -c],
		[ 1, 1,  1,  1],
		[-1, 1, -1,  1]	
	])

	F6 = np.zeros((6,4))
	F6[2:5, :] = F

	# Compute transformations and Jacobians
	[phi, x, y] = q_array
	Tsb = ChassisSE3(phi, x, y)
	Jarm = mr.JacobianBody(Blist, theta_array)
	T0e = mr.FKinBody(M0e, Blist, theta_array)
	Teb = mr.TransInv(np.dot(Tb0, T0e))
	Jbase = np.dot(mr.Adjoint(Teb), F6)

	# Combine Jacobians
	# J_base is 6x4: 4 wheels
	# J_arm  is 6x5: 5 joints
	Je = np.zeros((6, 9))
	Je[:, 0:4] = Jbase
	Je[:, 4:9] = Jarm

	return Je


def TestFeedbackControl():
	Xerr_int = 0

	X = np.array([
		[ 0.170, 0, 0.985, 0.387],
		[     0, 1,     0,     0],
		[-0.985, 0, 0.170, 0.570],
		[     0, 0,     0,     1]
	])

	Xd = np.array([
		[ 0, 0, 1, 0.5],
		[ 0, 1, 0,   0],
		[-1, 0, 0, 0.5],
		[ 0, 0, 0,   1]
	])

	Xd_next = np.array([
		[ 0, 0, 1, 0.6],
		[ 0, 1, 0,   0],
		[-1, 0, 0, 0.3],
		[ 0, 0, 0,   1]
	])

	Kp = 0
	Ki = 0
	dt =  0.01

	V_new, _ = FeedbackControl(X, Xd, Xd_next, Kp, Ki, Xerr_int, dt)

	robot_config8 = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
	Je = CalculateJe(robot_config8, Tb0, M0e, Blist)
	u_thetad = np.dot(np.linalg.pinv(Je, rcond=1e-2), V_new)

	print("\nProjectStep3 debug:")
	print(f"\nu, thetadot: \n{u_thetad.round(1)}")

	# With Kp
	Kp = np.identity(6)
	Ki = 0
	Xerr_int = 0

	print("\nWith nonzero Kp:")
	V_new, _ = FeedbackControl(X, Xd, Xd_next, Kp, Ki, Xerr_int, dt)
	Je = CalculateJe(robot_config8, Tb0, M0e, Blist)
	u_thetad = np.dot(np.linalg.pinv(Je, rcond=1e-2), V_new)
	print(f"\nu, thetadot: \n{u_thetad.round(1)}")

if __name__ == '__main__':
	TestFeedbackControl()