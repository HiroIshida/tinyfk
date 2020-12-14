import os
import copy
import json
import unittest
import numpy as np
from numpy import testing
try:
    import tinyfk
except NameError:
    import _tinyfk as tinyfk

here_full_filepath = os.path.join(os.getcwd(), __file__)
here_full_dirpath = os.path.dirname(here_full_filepath)
project_base_path = os.path.join(here_full_dirpath, "..", "..")

urdf_model_path = os.path.join(project_base_path, "data", "fetch.urdf")
test_data_path = os.path.join(project_base_path, "test", "test_data.json")

with open(test_data_path, 'r') as f:
    test_data = json.load(f)
joint_names = test_data['joint_names']
link_names = test_data['link_names']
angle_vector = test_data['angle_vector']
gt_pose_list_ = np.array(test_data['pose_list'])
gt_pose_list = copy.copy(gt_pose_list_)
gt_pose_list[:, 3] = gt_pose_list_[:, 5] # note in skrobot rpy order is z-y-x
gt_pose_list[:, 5] = gt_pose_list_[:, 3]

def test_fksovler():
    fksolver = tinyfk.RobotModel(urdf_model_path)

    # adding new link `mylink` to `gripper_link`
    parent_id = fksolver.get_link_ids(["gripper_link"])[0]
    fksolver.add_new_link('mylink', parent_id, [0.1, 0.1, 0.1])

    link_ids = fksolver.get_link_ids(link_names)
    joint_ids = fksolver.get_joint_ids(joint_names)

    # check
    P, J = fksolver.solve_forward_kinematics(
            [angle_vector], link_ids, joint_ids, True, True, False)

    testing.assert_almost_equal(P, np.array(gt_pose_list))





