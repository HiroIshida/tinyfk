import numpy as np
from . import _tinyfk


# higher layer wrap
class RobotModel(object):

    from ._inverse_kinematics import solve_inverse_kinematics
    from ._inverse_kinematics import solve_multi_endeffector_inverse_kinematics

    def __init__(self, urdfpath):
        with open(urdfpath, 'r') as reader:
            xml_text = reader.read()
        self._xml_text = xml_text # solely for pickling & unpickling
        self._robot = _tinyfk.RobotModel(xml_text)

    def set_joint_angles(self, joint_ids, joint_angles_, with_base=False):
        if with_base:
            joint_angles, base_pose = joint_angles_[:-3], joint_angles_[-3:]
            self._robot.set_joint_angles(joint_ids, joint_angles)
            self._robot.set_base_pose(base_pose)
        else:
            self._robot.set_joint_angles(joint_ids, joint_angles_)

    def solve_forward_kinematics(self, joint_angles_sequence, elink_ids, joint_ids,
                                 with_rot=False, with_base=False, with_jacobian=False, use_cache=False):
        """
        if use_cache is False, before solving FK, internal caches in the tinyfk side will be
        cleared. If True, pre-exisiting cache will be took advantaged.
        Setting use_cache=True is potentially dangeroud feature for developers who
        understand the caching mechanism of the tinyfk side. 
        """
        return self._robot.solve_forward_kinematics(
            joint_angles_sequence, elink_ids, joint_ids,
            with_rot, with_base, with_jacobian, use_cache)

    def get_joint_ids(self, joint_names):
        return self._robot.get_joint_ids(joint_names)

    def get_link_ids(self, link_names):
        return self._robot.get_link_ids(link_names)

    def add_new_link(self, link_name, parent_id, position):
        return self._robot.add_new_link(link_name, parent_id, position)

    def clear_cache(self):
        self._robot.clear_cache()

    # for pickling and unpickling
    # https://stackoverflow.com/questions/1939058/simple-example-of-use-of-setstate-and-getstate
    def __getstate__(self): # pickling
        return {'_xml_text': self._xml_text}

    def __setstate__(self, d): # unpickling
        self._xml_text = d['_xml_text']
        self._robot = _tinyfk.RobotModel(self._xml_text)
