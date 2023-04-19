import numpy as np

import tinyfk

urdf_model_path = tinyfk.pr2_urdfpath()
kin_solver = tinyfk.RobotModel(urdf_model_path)
rarm_joint_names = [
    "r_shoulder_pan_joint",
    "r_shoulder_lift_joint",
    "r_upper_arm_roll_joint",
    "r_elbow_flex_joint",
    "r_forearm_roll_joint",
    "r_wrist_flex_joint",
    "r_wrist_roll_joint",
]

rarm_joint_ids = kin_solver.get_joint_ids(rarm_joint_names)
end_link_id = kin_solver.get_link_ids(["r_gripper_tool_frame"])[0]

angle_vector_init = np.array([[0.564, 0.35, -0.74, -0.7, -0.7, -0.17, -0.63]]) * 0.0

# if you set 3 dim vector instead, just rpy componenent is not considered
# and point-ik will be solved
desired_xyzrpy = [0.7, -0.5, 0.6, 0.0, 0.0, 0.0]

xyzrpy, jac = kin_solver.solve_forward_kinematics(
    angle_vector_init,
    [end_link_id],
    rarm_joint_ids,
    tinyfk.RotationType.IGNORE,
    tinyfk.BaseType.FIXED,
)
print(xyzrpy)

ret = kin_solver.solve_com_forward_kinematics(
    angle_vector_init, rarm_joint_ids, tinyfk.BaseType.FIXED
)
print(ret)
