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

urdf_model_path = tinyfk.pr2_urdfpath()
test_data_path = tinyfk._test_data_urdfpath()

with open(test_data_path, 'r') as f:
    test_data = json.load(f)
joint_names = test_data['joint_names']
link_names = test_data['link_names']
angle_vector = test_data['angle_vector']
gt_pose_list_ = np.array(test_data['pose_list'])
gt_pose_list = copy.copy(gt_pose_list_)
gt_pose_list[:, 3] = gt_pose_list_[:, 5] # note in skrobot rpy order is z-y-x
gt_pose_list[:, 5] = gt_pose_list_[:, 3] # so, the two lines are swapped.

# common setups 
fksolver = tinyfk.RobotModel(urdf_model_path)

# adding new link `mylink` to `r_upper_arm_link`
parent_id = fksolver.get_link_ids(["r_upper_arm_link"])[0]
fksolver.add_new_link('mylink', parent_id, [0.1, 0.1, 0.1])

# To interact with fksolver, we must get the correspoinding link_ids and joint_ids.
link_ids = fksolver.get_link_ids(link_names)
joint_ids = fksolver.get_joint_ids(joint_names)

def test_fksovler():
    # check P (array of poses [pos, rpy] of each link) coincides with the ground truth
    use_rotation = True # If true P[i, :] has 6 dim, otherwise has 3 dim.
    use_base = True # If true, assumes that angle_vector takes the form of [q_joints, q_base (3dof)]
    with_jacobian = True # If true, jacobian is computed, otherewise returns J = None.
    P, _ = fksolver.solve_forward_kinematics(
            [angle_vector], link_ids, joint_ids, use_rotation, use_base, with_jacobian)
    testing.assert_almost_equal(P, gt_pose_list)

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
        # test position jacobian
        testing.assert_almost_equal(J_numerical[:3, :], J_analytical[:3, :])
        # test rpy jacobian
        testing.assert_almost_equal(J_numerical[3:, :], J_analytical[3:, :])

