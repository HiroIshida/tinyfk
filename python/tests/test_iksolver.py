from math import sqrt

import numpy as np

import tinyfk

rarm_joint_names = [
    "r_shoulder_pan_joint",
    "r_shoulder_lift_joint",
    "r_upper_arm_roll_joint",
    "r_elbow_flex_joint",
    "r_forearm_roll_joint",
    "r_wrist_flex_joint",
    "r_wrist_roll_joint",
]
larm_joint_names = [
    "l_shoulder_pan_joint",
    "l_shoulder_lift_joint",
    "l_upper_arm_roll_joint",
    "l_elbow_flex_joint",
    "l_forearm_roll_joint",
    "l_wrist_flex_joint",
    "l_wrist_roll_joint",
]

# common setups
urdf_model_path = tinyfk.pr2_urdfpath()
fksolver = tinyfk.RobotModel(urdf_model_path)

# To interact with fksolver, we must get the correspoinding link_ids and joint_ids.
angle_vector = np.array([0.564, 0.35, -0.74, -0.7, -0.7, -0.17, -0.63, 0.3, 0.5, 0.7])


def test_iksolver():
    rarm_joint_ids = fksolver.get_joint_ids(rarm_joint_names)
    dof_base = 3
    angle_vector = np.zeros(len(rarm_joint_ids) + dof_base)

    class Setting:
        def __init__(self, av, pose_desired, with_base):
            self.av_init = av
            self.pose_desired = pose_desired
            self.with_base = with_base

    setting_list = [
        Setting(angle_vector[:-3], np.array([0.7, -0.6, 0.8]), False),
        Setting(angle_vector, np.array([0.7, -0.6, 0.8]), True),
        Setting(angle_vector, np.array([1.7, -0.6, 0.8]), True),
        Setting(angle_vector[:-3], np.array([0.7, -0.6, 0.8, 0, 0, 0]), False),
        Setting(angle_vector, np.array([1.7, -0.6, 0.8, 0, 0, 0]), True),
    ]

    option = {"maxitr": 1000, "ftol": 1e-4, "sr_weight": 1.0}

    end_link_id = fksolver.get_link_ids(["r_gripper_tool_frame"])[0]
    for s in setting_list:
        av_sol = fksolver.solve_inverse_kinematics(
            s.pose_desired,
            s.av_init,
            end_link_id,
            rarm_joint_ids,
            with_base=s.with_base,
            option=option,
        )
        with_rot = len(s.pose_desired) == 6
        P, J = fksolver.solve_forward_kinematics(
            [av_sol],
            [end_link_id],
            rarm_joint_ids,
            with_rot=with_rot,
            with_base=s.with_base,
        )
        pose = P[0]
        acc = np.linalg.norm(s.pose_desired - pose)
        assert acc < option["ftol"]


def test_multiple_iksolver():
    joint_ids = fksolver.get_joint_ids(rarm_joint_names + larm_joint_names)
    dof_base = 3
    angle_vector = np.zeros(len(joint_ids) + dof_base)
    end_link_ids = fksolver.get_link_ids(["r_gripper_tool_frame", "l_gripper_tool_frame"])

    option = {"maxitr": 1000, "ftol": 1e-4, "sr_weight": 1.0}

    pose_desired_list = [
        np.array([0.7, -0.6, 0.8, 0, 0, 0]),
        np.array([0.7, +0.6, 0.8]),
    ]

    av_sol = fksolver.solve_multi_endeffector_inverse_kinematics(
        pose_desired_list,
        angle_vector,
        end_link_ids,
        joint_ids,
        with_base=True,
        option=option,
    )
    pose_list, _ = fksolver.solve_forward_kinematics(
        [av_sol], end_link_ids, joint_ids, with_rot=True, with_base=True
    )

    acc_sq = 0.0
    for pose, pose_desired in zip(pose_list, pose_desired_list):
        if len(pose_desired) == 3:
            diff = pose[:3] - pose_desired
        else:
            diff = pose - pose_desired
        acc_sq += sum(diff**2)
    acc = sqrt(acc_sq)
    assert acc < option["ftol"]
