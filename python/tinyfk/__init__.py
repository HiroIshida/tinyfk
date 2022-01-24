import sys
import os
import numpy as np
from . import _tinyfk

_cache_dir = os.path.expanduser('~/.tinyfk')
if not os.path.exists(_cache_dir):
    os.makedirs(_cache_dir)
    if sys.version_info[0] >= 3:
        from urllib.request import urlretrieve
    else:
        from urllib import urlretrieve
    addr_pr2 = 'https://raw.githubusercontent.com/HiroIshida/tinyfk/master/data/pr2.urdf'
    addr_fetch = 'https://raw.githubusercontent.com/HiroIshida/tinyfk/master/data/fetch.urdf'
    addr_testdata = 'https://raw.githubusercontent.com/HiroIshida/tinyfk/master/test/test_data.json'
    urlretrieve(addr_pr2, os.path.join(_cache_dir, 'pr2.urdf'))
    urlretrieve(addr_fetch, os.path.join(_cache_dir, 'fetch.urdf'))
    urlretrieve(addr_testdata, os.path.join(_cache_dir, 'test_data.json')) # used only in the unit test

def pr2_urdfpath():
    return os.path.join(_cache_dir, 'pr2.urdf')

def fetch_urdfpath():
    return os.path.join(_cache_dir, 'fetch.urdf')

def _test_data_urdfpath():
    return os.path.join(_cache_dir, 'test_data.json')

# higher layer wrap
class RobotModel(object):

    from ._inverse_kinematics import solve_inverse_kinematics
    from ._inverse_kinematics import solve_multi_endeffector_inverse_kinematics

    def __init__(self, urdfpath=None, xml_text=None):
        assert (urdfpath is None) ^ (xml_text is None)
        if not xml_text:
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
        if not isinstance(joint_angles_sequence, np.ndarray):
            joint_angles_sequence = np.array(joint_angles_sequence)
            if joint_angles_sequence.ndim == 1:
                joint_angles_sequence = np.expand_dims(joint_angles_sequence, axis=0)
        n_seq, n_dof = joint_angles_sequence.shape
        assert n_dof == len(joint_ids) + (3 if with_base else 0)

        return self._robot.solve_forward_kinematics(
            joint_angles_sequence, elink_ids, joint_ids,
            with_rot, with_base, with_jacobian, use_cache)

    def get_joint_ids(self, joint_names):
        return self._robot.get_joint_ids(joint_names)

    def get_link_ids(self, link_names):
        return self._robot.get_link_ids(link_names)

    def get_joint_limits(self, joint_ids):
        ret = self._robot.get_joint_limits(joint_ids)
        limits = []
        for lower, upper in ret:
            if lower==0.0 and upper==0.0:
                # NOTE: if no limit is set, the value is set to 0.0 in urdfdom
                # TODO: I assume that if joint limit is not set, both lower and upper
                # are not set. Is this assumption correct?
                limits.append([None, None])
            else:
                limits.append([lower, upper])
        return limits

    def add_new_link(self, link_name, parent_id, position, rotation=None):
        if rotation is None:
            rotation = [0, 0, 0]
        return self._robot.add_new_link(link_name, parent_id, position, rotation)

    def clear_cache(self):
        self._robot.clear_cache()

    # for pickling and unpickling
    # https://stackoverflow.com/questions/1939058/simple-example-of-use-of-setstate-and-getstate
    def __getstate__(self): # pickling
        return {'_xml_text': self._xml_text}

    def __setstate__(self, d): # unpickling
        self._xml_text = d['_xml_text']
        self._robot = _tinyfk.RobotModel(self._xml_text)
