
import modern_robotics as mr
from additional import *
from config import *


def NextState(robot_config12, robot_speeds9, dt, w_max):
	'''	
	Calculates the robot's next configuration based on current state and speeds.

    Inputs:
    - robot_config12: 12-element vector (chassis, arm, wheel configurations).
    - robot_speeds9: 9-element vector (wheel and arm joint speeds).
    - dt: Timestep (Δt).
    - w_max: Maximum angular speed for wheels and joints (limits control speeds).

    Output:
    - Updated 12-element configuration vector.

	'''
	# wheel geometry
	l = 0.47/2.0 
	w = 0.3/2.0  
	r = 0.0475   
	# Checking the data
	robot_config12 = np.array(robot_config12).flatten()
	robot_speeds9 = np.array(robot_speeds9).flatten()

	if len(robot_config12) != 12 or len(robot_speeds9) != 9:
		raise Exception(f"Lengths of arrays: {len(robot_config12)} {len(robot_speeds9)}")

	# coordinates
	q_array      = robot_config12[0:3]
	theta_array  = robot_config12[3:8]
	phi_array    = robot_config12[8:12]

	u_array      = robot_speeds9[0:4]
	thetad_array = robot_speeds9[4:9]

	phi, x, y = q_array

	# wheel vel to vel in world coords
	c = 1/(l+w) 
	F = r/4 * np.array([
		[-c, c,  c, -c],
		[ 1, 1,  1,  1],
		[-1, 1, -1,  1]	
	])

	# Calculate next arm joint angles, wheel angles,
	thetalist = robot_config12[3:12].reshape((1,9))
	dthetalist = np.append(thetad_array, u_array).reshape((1,9))
	ddthetalist = np.zeros(max(robot_speeds9.shape)).reshape((1,9)) 
	[thetanext, _] = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)

	u_array = u_array.reshape((4, 1))
	Vb = np.dot(F, u_array * dt)
	Vb6 = np.zeros((6,1))
	Vb6[2:5] = Vb
	Vb6 = Vb6.flatten()
	se3mat = mr.VecTose3(Vb6)

	# Calculate q_next 
	wbz, vbx, vby = np.array(Vb).flatten().tolist()
	if np.isclose(wbz, 0, atol=1E-4):
		delta_qb = np.array([0, vbx, vby])
	else:
		delta_qb = np.array([
			wbz,
			(vbx * np.sin(wbz) + vby * (np.cos(wbz)-1) )/wbz,
			(vby * np.sin(wbz) + vbx * (1-np.cos(wbz)) )/wbz
		])

	#get from body frame to the space frame
	q_transform = np.array([
		[1,           0,            0],
		[0, np.cos(phi), -np.sin(phi)],
		[0, np.sin(phi),  np.cos(phi)]
	])

	delta_q = np.dot(q_transform, delta_qb)
	q_new = np.array(q_array).flatten() + np.array(delta_q).flatten()
	robot_config12_new = np.append(q_new, thetanext.flatten())

	return robot_config12_new


def RunNextState(robot_config12, u, thetad, dt, w_max):
	'''
    Simulates robot motion under constant controls for one second.

    Inputs:
    - robot_config12: Initial configuration (12-element vector).
    - u: Wheel speed controls (4-element vector).
    - thetad: Arm joint speed controls (5-element vector).
    - dt: Timestep (Δt).
    - w_max: Maximum speed limit.

    Writes the robot's state at each timestep to a CSV file.
    '''
	filename = 'project/csv/next_state.csv'
	f = open(filename, 'w') 
	f.close()

	if (len(u) != 4 or len(thetad) != 5):
		raise Exception("TestNextState: size mismatch in u or thetad")

	robot_speeds9 = np.append(u, thetad)
	csv_data = np.zeros(13)

	for i in range(100):
		robot_config12_new = NextState(robot_config12, robot_speeds9, dt, w_max)
		csv_data[0:12] = robot_config12_new.flatten()
		write_csv_line(filename, csv_data)
		robot_config12 = robot_config12_new


def TestNextState():
	'''
    Test NextState() with sample inputs and log results.
    '''

	test_base_joints = [0, 0, 0, 0, 0, 0.2, -1.6, 0]
	test_wheels = [0,0,0,0]
	robot_config12 = test_base_joints + test_wheels

	u1 = [ 10, 10,  10,  10] # x vel
	u2 = [-10, 10, -10,  10] # y vel
	u3 = [-10, 10,  10, -10] # angle vel

	thetad = np.zeros(5)
	dt = 0.01
	w_max = 5

	RunNextState(robot_config12, u1, thetad, dt, w_max)


if __name__ == '__main__':
    TestNextState()