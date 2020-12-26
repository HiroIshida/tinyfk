import os
import copy
import json
import unittest
import numpy as np
from numpy import testing
try:
    import tinyfk
except:
    import _tinyfk as tinyfk
from math import *

here_full_filepath = os.path.join(os.getcwd(), __file__)
here_full_dirpath = os.path.dirname(here_full_filepath)
project_base_path = os.path.join(here_full_dirpath, "..", "..")

urdf_model_path = os.path.join(project_base_path, "data", "pr2.urdf")
test_data_path = os.path.join(project_base_path, "test", "test_data.json")

with open(test_data_path, 'r') as f:
    test_data = json.load(f)
joint_names = test_data['joint_names']
link_names = test_data['link_names']
angle_vector = test_data['angle_vector']
gt_pose_list_ = np.array(test_data['pose_list'])
gt_pose_list = copy.copy(gt_pose_list_)
gt_pose_list[:, 3] = gt_pose_list_[:, 5] # note in skrobot rpy order is z-y-x
gt_pose_list[:, 5] = gt_pose_list_[:, 3] # so, the two lines are swapped.

def test_fksovler():
    fksolver = tinyfk.RobotModel(urdf_model_path)

    # adding new link `mylink` to `r_upper_arm_link`
    parent_id = fksolver.get_link_ids(["r_upper_arm_link"])[0]
    fksolver.add_new_link('mylink', parent_id, [0.1, 0.1, 0.1])

    # To interact with fksolver, we must get the correspoinding link_ids and joint_ids.
    link_ids = fksolver.get_link_ids(link_names)
    joint_ids = fksolver.get_joint_ids(joint_names)

    # check P (array of poses [pos, rpy] of each link) coincides with the ground truth
    use_rotation = True # If true P[i, :] has 6 dim, otherwise has 3 dim.
    use_base = True # If true, assumes that angle_vector takes the form of [q_joints, q_base (3dof)]
    with_jacobian = True # If true, jacobian is computed, otherewise returns J = None.
    P, _ = fksolver.solve_forward_kinematics(
            [angle_vector], link_ids, joint_ids, use_rotation, use_base, with_jacobian)
    testing.assert_almost_equal(P, gt_pose_list)

    print("test start")
    def rpy_kine_mat(rpy):
        a1, a2, a3 = rpy
        a3 = -a3
        a2 = -a2
        a1 = -a1
        mat = np.array([[cos(a3)/cos(a2), -sin(a3)/cos(a2), 0], 
            [sin(a3), cos(a3), 0],
            [-cos(a3)*sin(a2)/cos(a2), sin(a3)*sin(a2)/cos(a2), 1]])
        return mat
    # The following test assumes: that the above test without jacobian succeeded.
    # check resulting jacbian J_analytical coincides with J_numerical witch is 
    # computed via numerical differentiation.
    for link_id, link_name in zip(link_ids, link_names):
        P_tmp, J_analytical = fksolver.solve_forward_kinematics(
                [angle_vector], [link_id], joint_ids, True, True, True)
        P0, _ = fksolver.solve_forward_kinematics(
                [angle_vector], [link_id], joint_ids, True, True, False)
        testing.assert_almost_equal(P_tmp, P0) # P computed with and without jacobian must match

        eps = 1e-7
        J_numerical = np.zeros(J_analytical.shape)
        for i in range(len(angle_vector)):
            angle_vector_p = copy.copy(angle_vector)
            angle_vector_p[i] += eps
            P1, _ = fksolver.solve_forward_kinematics(
                    [angle_vector_p], [link_id], joint_ids, True, True, False)
            P_diff = (P1 - P0)/eps
            J_numerical[:, i] = P_diff.flatten()

        rpy = P0[0][3:]
        mat = rpy_kine_mat(rpy)
        testing.assert_almost_equal(J_numerical[:3, :], J_analytical[:3, :])
        testing.assert_almost_equal(J_numerical[3:, :], mat.dot(J_analytical[3:, :]))
    return J_numerical[3:, :], J_analytical[3:, :], rpy

