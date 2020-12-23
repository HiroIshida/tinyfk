from . import _tinyfk

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

    def get_joint_ids(self, joint_names):
        return self._robot.get_joint_ids(joint_names)

    def get_link_ids(self, link_names):
        return self._robot.get_link_ids(link_names)

    def add_new_link(self, link_name, parent_id, position):
        return self._robot.add_new_link(link_name, parent_id, position)
