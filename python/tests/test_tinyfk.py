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
    with_jacobian = False # If true, jacobian is computed, otherewise returns J = None.
    P, _ = fksolver.solve_forward_kinematics(
            [angle_vector], link_ids, joint_ids, use_rotation, use_base, with_jacobian)
    for p, q, name in zip(P, gt_pose_list, link_names):
        print(name)
        testing.assert_almost_equal(p, q)

    # The following test assumes: that the above test without jacobian succeeded.
    # check resulting jacbian J_analytical coincides with J_numerical witch is 
    # computed via numerical differentiation.
    for link_id, link_name in zip(link_ids, link_names):
        print(link_name)
        P_tmp, J_analytical = fksolver.solve_forward_kinematics(
                [angle_vector], [link_id], joint_ids, False, True, True)
        P0, _ = fksolver.solve_forward_kinematics(
                [angle_vector], [link_id], joint_ids, False, True, False)
        testing.assert_almost_equal(P_tmp, P0) # P computed with and without jacobian must match

        eps = 1e-7
        J_numerical = np.zeros(J_analytical.shape)
        for i in range(len(angle_vector)):
            angle_vector_p = copy.copy(angle_vector)
            angle_vector_p[i] += eps
            P1, _ = fksolver.solve_forward_kinematics(
                    [angle_vector_p], [link_id], joint_ids, False, True, False)
            P_diff = (P1 - P0)/eps
            J_numerical[:, i] = P_diff.flatten()
            testing.assert_almost_equal(J_numerical[:, i], J_analytical[:, i], decimal=5)
