import modern_robotics as mr
import numpy as np
from scipy.spatial.transform import Rotation as R

# ===== YouBot Configuration =====
# Screw axis list for the robotic arm
Blist = np.array([
    [0,  0, 1,       0, 0.033, 0],
    [0, -1, 0, -0.5076,     0, 0],
    [0, -1, 0, -0.3526,     0, 0],
    [0, -1, 0, -0.2176,     0, 0],
    [0,  0, 1,       0,     0, 0],    
]).T

# Initial cube configuration
Tsc_i = mr.RpToTrans(np.identity(3), [1, 0, 0.025]).tolist()

# Goal cube configuration
Tsc_f = mr.RpToTrans(np.identity(3), [0, -1, 0.025]).tolist()

# Joint limits for the KUKA arm (degrees converted to radians)
joint_limits_kuka = (2 * np.pi / 360) * np.array([
    [-169, 169],
    [ -65,  90],
    [-151, 146],
    [-102.5, 102.5],
    [-167.5, 167.5]    
]).T

# Transformation from chassis frame {b} to arm base frame {0}
Tb0 = mr.RpToTrans(np.identity(3), [0.1662, 0, 0.0026]).tolist()

# Home configuration of the end-effector relative to arm base {0}
M0e = mr.RpToTrans(np.identity(3), [0.033, 0, 0.6546]).tolist()

# Gripper orientation relative to the object (135° about y-axis)
R_ee_to_obj = R.from_euler('y', 135, degrees=True).as_matrix()

# Transformation for the grasp position relative to the object
Tce_grasp = mr.RpToTrans(R_ee_to_obj, [0, 0, 0])

# Transformation for standoff position (offset along z-axis)
dist = 0.08  # Standoff distance
Tce_standoff = np.dot(
    mr.RpToTrans(np.identity(3), dist * np.array([0, 0, 1])),
    mr.RpToTrans(R_ee_to_obj, [0, 0, 0])
).tolist()
