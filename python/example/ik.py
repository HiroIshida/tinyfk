import time

import numpy as np

import tinyfk
from tinyfk import KinematicModel

urdf_model_path = tinyfk.pr2_urdfpath()
kin = KinematicModel(urdf_model_path)
joint_names = [
    "r_shoulder_pan_joint",
    "r_shoulder_lift_joint",
    "r_upper_arm_roll_joint",
    "r_elbow_flex_joint",
    "r_forearm_roll_joint",
    "r_wrist_flex_joint",
    "r_wrist_roll_joint",
]

joint_ids = kin.get_joint_ids(joint_names)
end_link_id = kin.get_link_ids(["r_gripper_tool_frame"])[0]

q_init = np.array([0.564, 0.35, -0.74, -0.7, -0.7, -0.17, -0.63])
end_link_target = np.array([0.5, -0.3, 0.7])

q_now = q_init
ts = time.time()
for _ in range(100):
    P, J = kin.solve_fk([q_init], [end_link_id], joint_ids, with_jacobian=True)
    p = P[0]
    J_sr = J.T @ np.linalg.inv(J @ J.T + 1e-3 * np.eye(3))  # SR-inverse
    residual = end_link_target - p
    dq = J_sr @ residual
    q_now += dq
    if np.linalg.norm(residual) < 1e-3:
        break
print(f"Time to solve IK: {time.time() - ts} sec, Residual: {np.linalg.norm(residual)}")
