import numpy as np
from . import _tinyfk

class IKFail(Exception):
    pass

# higher layer wrap
class RobotModel(object):
    def __init__(self, urdfpath):
        self._robot = _tinyfk.RobotModel(urdfpath)

    def set_joint_angles(self, joint_ids, joint_angles_, with_base=False):
        if with_base:
            joint_angles, base_pose = joint_angles_[:-3], joint_angles_[-3:]
            self._robot.set_joint_angles(joint_ids, joint_angles)
            self._robot.set_base_pose(base_pose)
        else:
            self._robot.set_joint_angles(joint_ids, joint_angles_)

    def solve_forward_kinematics(self, joint_angles_sequence, elink_ids, joint_ids,
                                 with_rot=False, with_base=False, with_jacobian=False):
        return self._robot.solve_forward_kinematics(
            joint_angles_sequence, elink_ids, joint_ids,
            with_rot, with_base, with_jacobian)

    def solve_inverse_kinematics(self, target_pose, init_angle_vector, elink_id, joint_ids,
            with_rot=False, with_base=False, option=None):

        if option is None:
            option = {"maxitr": 200, "ftol": 1e-4, "sr_weight":1.0}

        target_pose = np.array(target_pose)
        with_rot = len(target_pose) == 6
        n_dof = len(joint_ids) + (3 if with_base else 0)
        assert len(init_angle_vector) == n_dof

        pose_dim = (6 if with_rot else 3)
        assert len(target_pose) == pose_dim

        angle_vector = init_angle_vector
        for i in range(option["maxitr"]):
            P, J = self._robot.solve_forward_kinematics(
                    [angle_vector], [elink_id], joint_ids, with_rot, with_base, True)
            pose_diff = target_pose - P[0]
            J_sharp = J.T.dot(np.linalg.inv(J.dot(J.T) + option["sr_weight"])) # singular-robust inverse
            angle_vector = angle_vector + J_sharp.dot(pose_diff)
            if np.linalg.norm(pose_diff) < option["ftol"]:
                return angle_vector
        raise IKFail

    def get_joint_ids(self, joint_names):
        return self._robot.get_joint_ids(joint_names)

    def get_link_ids(self, link_names):
        return self._robot.get_link_ids(link_names)

    def add_new_link(self, link_name, parent_id, position):
        return self._robot.add_new_link(link_name, parent_id, position)
