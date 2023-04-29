import numpy as np

import tinyfk
from tinyfk import BaseType, KinematicModel, RotationType

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

# first 7 elements are for joints and the last 3 elements are for x, y, yaw of base.
q = np.array([0.564, 0.35, -0.74, -0.7, -0.7, -0.17, -0.63, 0.1, 0.2, 0.3])

poses, jacobians = kin.solve_fk(
    q,
    end_link_id,
    joint_ids,
    rot_type=RotationType.RPY,
    base_type=BaseType.PLANER,
    with_jacobian=True,
)
