import copy
import pickle

import numpy as np

import tinyfk


def test_pickle():
    urdf_model_path = tinyfk.pr2_urdfpath()
    fksolver = tinyfk.RobotModel(urdf_model_path)
    pickle.dumps(fksolver)
    copy.copy(fksolver)


def test_deepcopy():
    urdf_model_path = tinyfk.pr2_urdfpath()
    fksolver = tinyfk.RobotModel(urdf_model_path)

    # add new link
    parent_id = fksolver.get_link_ids(["r_upper_arm_link"])[0]
    fksolver.add_new_link("mylink", parent_id, [0.1, 0.1, 0.1], [0.3, 0.2, 0.1])
    new_link_id = fksolver.get_link_ids(["mylink"])[0]

    # chaneg some joint angles
    rarm_joint_names = [
        "r_shoulder_pan_joint",
        "r_shoulder_lift_joint",
        "r_upper_arm_roll_joint",
        "r_elbow_flex_joint",
        "r_forearm_roll_joint",
        "r_wrist_flex_joint",
        "r_wrist_roll_joint",
    ]
    rarm_joint_ids = fksolver.get_joint_ids(rarm_joint_names)
    angle_vector = np.array([0.564, 0.35, -0.74, -0.7, -0.7, -0.17, -0.63, 0.1, 0.2, 0.3])
    fksolver.set_joint_angles(rarm_joint_ids, angle_vector, tinyfk.BaseType.PLANER)

    # copy
    fksolver_copied = copy.deepcopy(fksolver)

    # check
    new_link_id_again = fksolver_copied.get_link_ids(["mylink"])[0]
    assert new_link_id_again == new_link_id

    angle_vector_copied = fksolver_copied.get_joint_angles(rarm_joint_ids, tinyfk.BaseType.PLANER)
    np.testing.assert_almost_equal(angle_vector, angle_vector_copied)
