import numpy as np

class IKFail(Exception):
    pass

def solve_inverse_kinematics(self, target_pose, init_angle_vector, elink_id, joint_ids,
        with_base=False, option=None, ignore_fail=False):
    target_pose_list = [target_pose]
    elink_ids = [elink_id]

    return self.solve_multi_endeffector_inverse_kinematics(
            target_pose_list, init_angle_vector, elink_ids, joint_ids,
            with_base=with_base, option=option, ignore_fail=ignore_fail)

def solve_multi_endeffector_inverse_kinematics(self,
        target_pose_list, init_angle_vector, elink_ids, joint_ids,
        with_base=False, option=None, ignore_fail=False):

    if option is None:
        option = {"maxitr": 200, "ftol": 1e-4, "sr_weight":1.0}

    ## assertion stuff
    assert len(target_pose_list) == len(elink_ids)
    n_dof = len(joint_ids) + (3 if with_base else 0)
    assert len(init_angle_vector) == n_dof, "specified angle vector's dim is {0}, but should be {1}".format(len(init_angle_vector), n_dof)

    with_rot_list = []
    pose_dim_list = []
    for target_pose in target_pose_list:
        pose_dim_list.append(len(target_pose))
        with_rot = (len(target_pose) == 6)
        with_rot_list.append(with_rot)

    angle_vector = init_angle_vector
    for i in range(option["maxitr"]):
        pose_diff_list = []
        J_list = []
        self.clear_cache() # because we set use_cache=True
        for target_pose, elink_id, with_rot in zip(target_pose_list, elink_ids, with_rot_list):
            P_single, J_single = self.solve_forward_kinematics(
                    [angle_vector], [elink_id], joint_ids, with_rot, with_base, True, use_cache=True)
            pose_diff_list.append(target_pose - P_single[0])
            J_list.append(J_single)

        pose_diffs = np.hstack(pose_diff_list)
        J = np.vstack(J_list)

        J_sharp = J.T.dot(np.linalg.inv(J.dot(J.T) + option["sr_weight"])) # singular-robust inverse
        angle_vector = angle_vector + J_sharp.dot(pose_diffs)
        if np.linalg.norm(pose_diffs) < option["ftol"]:
            return angle_vector

    if ignore_fail:
        return angle_vector

    raise IKFail
