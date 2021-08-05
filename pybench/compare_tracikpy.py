import numpy as np
import tinyfk
from tracikpy import TracIKSolver

ee_pose = np.eye(4)
ee_pose[3, 1:] = np.array([0.7, -0.5, 0.6])

urdf_model_path = tinyfk.pr2_urdfpath()
ik_solver = TracIKSolver(urdf_model_path, "torso_lift_link", "r_gripper_tool_frame")


q_init = np.array([0.564, 0.35, -0.74, -0.7, -0.7, -0.17, -0.63])
qout = ik_solver.ik(ee_pose, qinit=q_init)
