import copy
import json
from pathlib import Path

import numpy as np
import pytest
from numpy import testing

import tinyfk
from tinyfk import RotationType


@pytest.fixture(scope="session")
def test_data():
    current_file_path = Path(__file__).resolve()
    test_data_path = current_file_path.parent.parent.parent / "test" / "data" / "test_data.json"

    with test_data_path.open(mode="r") as f:
        test_data = json.load(f)
    joint_names = test_data["joint_names"]
    link_names = test_data["link_names"]
    angle_vector = test_data["angle_vector"]
    gt_pose_list_ = np.array(test_data["pose_list"])
    gt_pose_list = copy.copy(gt_pose_list_)
    gt_pose_list[:, 3] = gt_pose_list_[:, 5]  # note in skrobot rpy order is z-y-x
    gt_pose_list[:, 5] = gt_pose_list_[:, 3]  # so, the two lines are swapped.

    # load fkmodel
    urdf_model_path = tinyfk.pr2_urdfpath()
    fksolver = tinyfk.RobotModel(urdf_model_path)

    # adding new link `mylink` to `r_upper_arm_link`
    parent_id = fksolver.get_link_ids(["r_upper_arm_link"])[0]
    fksolver.add_new_link("mylink", parent_id, [0.1, 0.1, 0.1], [0.3, 0.2, 0.1])

    # To interact with fksolver, we must get the correspoinding link_ids and joint_ids.
    link_ids = fksolver.get_link_ids(link_names)
    joint_ids = fksolver.get_joint_ids(joint_names)
    joint_limits = fksolver.get_joint_limits(joint_ids)
    return angle_vector, gt_pose_list, fksolver, link_ids, joint_ids, joint_limits


def test_root_link_id():
    urdf_model_path = tinyfk.pr2_urdfpath()
    fksolver = tinyfk.RobotModel(urdf_model_path)
    assert fksolver.root_link_name == "base_footprint"


def test_joint_limit(test_data):
    angle_vector, gt_pose_list, fksolver, link_ids, joint_ids, joint_limits = test_data

    r_shoulder_pan_joint_limit = joint_limits[0]
    assert abs(r_shoulder_pan_joint_limit[0] - -2.2853981634) < 1e-10
    assert abs(r_shoulder_pan_joint_limit[1] - 0.714601836603) < 1e-10

    r_wrist_roll_joint_limit = joint_limits[-1]
    assert r_wrist_roll_joint_limit[0] is None
    assert r_wrist_roll_joint_limit[1] is None


def test_forward_kinematics(test_data):
    angle_vector, gt_pose_list, fksolver, link_ids, joint_ids, joint_limits = test_data

    # check P (array of poses [pos, rpy] of each link) coincides with the ground truth
    rot_type = RotationType.RPY
    base_type = tinyfk.BaseType.FLOATING
    with_jacobian = True  # If true, jacobian is computed, otherewise returns J = None.
    P, _ = fksolver.solve_forward_kinematics(
        [angle_vector], link_ids, joint_ids, rot_type, base_type, with_jacobian
    )
    testing.assert_almost_equal(P, gt_pose_list)

    # The following test assumes: that the above test without jacobian succeeded.
    # check resulting jacbian J_analytical coincides with J_numerical witch is
    # computed via numerical differentiation.
    for link_id in link_ids:
        P_tmp, J_analytical = fksolver.solve_forward_kinematics(
            [angle_vector], [link_id], joint_ids, rot_type, base_type, True
        )
        P0, _ = fksolver.solve_forward_kinematics(
            [angle_vector], [link_id], joint_ids, rot_type, base_type, False
        )
        testing.assert_almost_equal(P_tmp, P0)  # P computed with and without jacobian must match


def test_jacobian(test_data):
    angle_vector, gt_pose_list, fksolver, link_ids, joint_ids, joint_limits = test_data
    rot_type = RotationType.RPY
    base_type = tinyfk.BaseType.FLOATING
    for link_id in link_ids:
        P0, J_analytical = fksolver.solve_forward_kinematics(
            [angle_vector], [link_id], joint_ids, rot_type, base_type, True
        )
        eps = 1e-7
        J_numerical = np.zeros(J_analytical.shape)
        for i in range(len(angle_vector)):
            angle_vector_p = copy.copy(angle_vector)
            angle_vector_p[i] += eps
            P1, _ = fksolver.solve_forward_kinematics(
                [angle_vector_p], [link_id], joint_ids, rot_type, base_type, False
            )
            P_diff = (P1 - P0) / eps
            J_numerical[:, i] = P_diff.flatten()

        # test position jacobian
        testing.assert_almost_equal(J_numerical, J_analytical)


def test_jacobian_3dof_base(test_data):
    angle_vector, gt_pose_list, fksolver, link_ids, joint_ids, joint_limits = test_data
    n_joint = len(joint_ids)
    extract_indices = np.hstack([np.arange(n_joint), np.array([n_joint, n_joint + 1, n_joint + 5])])
    angle_vector_3dof_base = [angle_vector[i] for i in extract_indices]

    rot_type = RotationType.RPY
    base_type = tinyfk.BaseType.PLANER

    for link_id in link_ids:
        P0, J_analytical = fksolver.solve_forward_kinematics(
            [angle_vector_3dof_base], [link_id], joint_ids, rot_type, base_type, True
        )
        eps = 1e-7
        J_numerical = np.zeros(J_analytical.shape)
        for i in range(len(angle_vector_3dof_base)):
            angle_vector_p = copy.copy(angle_vector_3dof_base)
            angle_vector_p[i] += eps
            P1, _ = fksolver.solve_forward_kinematics(
                [angle_vector_p], [link_id], joint_ids, rot_type, base_type, False
            )
            P_diff = (P1 - P0) / eps
            J_numerical[:, i] = P_diff.flatten()
        testing.assert_almost_equal(J_numerical, J_analytical)


def test_trajectory_fk(test_data):
    # test cases where multiple angles vectors are given
    angle_vector, gt_pose_list, fksolver, link_ids, joint_ids, joint_limits = test_data
    n_dof = len(joint_ids) + 6  # 6 for base
    n_wp = 10

    rot_type = RotationType.RPY
    base_type = tinyfk.BaseType.FLOATING

    angle_vectors = [angle_vector + np.random.randn(n_dof) * 0.1 for _ in range(n_wp)]
    P, J = fksolver.solve_forward_kinematics(
        angle_vectors, link_ids, joint_ids, rot_type, base_type, True
    )
    assert P.shape == (n_wp * len(link_ids), 6)
    assert J.shape == (n_wp * len(link_ids) * 6, n_dof)

    Ps = P.reshape(n_wp, len(link_ids), 6)
    Js = J.reshape(n_wp, len(link_ids) * 6, n_dof)

    # check if multiple av cases is consistent with the single av case
    for i, av in enumerate(angle_vectors):
        P_single, J_single = fksolver.solve_forward_kinematics(
            [av], link_ids, joint_ids, rot_type, base_type, True
        )
        np.testing.assert_almost_equal(Ps[i], P_single)
        np.testing.assert_almost_equal(Js[i], J_single)


def test_hoge(test_data):
    angle_vector, gt_pose_list, fksolver, link_ids, joint_ids, joint_limits = test_data
    q = angle_vector

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

    base_type = tinyfk.BaseType.FLOATING

    link_ids1 = fksolver.get_joint_ids(rarm_joint_names)
    link_ids2 = fksolver.get_joint_ids(larm_joint_names)
    link_id_pairs = list(zip(link_ids1, link_ids2))
    values, J_analytical = fksolver.compute_inter_link_sqdists(
        [q], link_id_pairs, joint_ids, base_type, with_jacobian=True
    )

    eps = 1e-7
    grads = []
    for i in range(len(q)):
        q1 = copy.deepcopy(q)
        q1[i] += eps
        values1, _ = fksolver.compute_inter_link_sqdists(
            [q1], link_id_pairs, joint_ids, base_type, with_jacobian=False
        )
        grads.append((values1 - values) / eps)
    J_numerical = np.array(grads).T
    np.testing.assert_almost_equal(J_numerical, J_analytical, decimal=5)


if __name__ == "__main__":
    data = test_data()
    test_jacobian_3dof_base(data)
