import os
import sys

import numpy as np

from . import _tinyfk

_cache_dir = os.path.expanduser("~/.tinyfk")
if not os.path.exists(_cache_dir):
    os.makedirs(_cache_dir)
    if sys.version_info[0] >= 3:
        from urllib.request import urlretrieve
    else:
        from urllib import urlretrieve
    addr_pr2 = "https://raw.githubusercontent.com/HiroIshida/tinyfk/master/data/pr2.urdf"
    addr_fetch = "https://raw.githubusercontent.com/HiroIshida/tinyfk/master/data/fetch.urdf"
    urlretrieve(addr_pr2, os.path.join(_cache_dir, "pr2.urdf"))
    urlretrieve(addr_fetch, os.path.join(_cache_dir, "fetch.urdf"))


def pr2_urdfpath():
    return os.path.join(_cache_dir, "pr2.urdf")


def fetch_urdfpath():
    return os.path.join(_cache_dir, "fetch.urdf")


# higher layer wrap
class RobotModel(object):
    def __init__(self, urdfpath=None, xml_text=None):
        assert (urdfpath is None) ^ (xml_text is None)
        if not xml_text:
            with open(urdfpath, "r") as reader:
                xml_text = reader.read()
        self._xml_text = xml_text  # solely for pickling & unpickling
        self._robot = _tinyfk.RobotModel(xml_text)

    @property
    def root_link_name(self) -> str:
        return self._robot.get_root_link_name()

    def set_joint_angles(self, joint_ids, q, with_3dof_base=False, with_6dof_base=False):
        if with_3dof_base:
            assert not with_6dof_base
            assert len(q) == len(joint_ids) + 3
            joint_angles, base_xytheta = q[:-3], q[-3:]
            base_pose = np.array([base_xytheta[0], base_xytheta[1], 0.0, 0.0, 0.0, base_xytheta[2]])
            self._robot.set_joint_angles(joint_ids, joint_angles)
            self._robot.set_base_pose(base_pose)
        elif with_6dof_base:
            assert not with_3dof_base
            assert len(q) == len(joint_ids) + 6
            joint_angles, base_pose = q[:-6], q[-6:]
            self._robot.set_joint_angles(joint_ids, joint_angles)
            self._robot.set_base_pose(base_pose)
        else:
            self._robot.set_joint_angles(joint_ids, q)

    def _modify_input_with_3dof_base(self, n_joint, joint_angles_sequence):
        n_seq, n_dof = joint_angles_sequence.shape
        joint_angles_sequence_modified = np.zeros((n_seq, n_dof + 3))
        joint_angles_sequence_modified[:, : n_joint + 2] = joint_angles_sequence[:, : n_joint + 2]
        joint_angles_sequence_modified[:, -1] = joint_angles_sequence[:, n_joint + 2]
        return joint_angles_sequence_modified

    def solve_forward_kinematics(
        self,
        joint_angles_sequence,
        elink_ids,
        joint_ids,
        with_rot=False,
        with_3dof_base=False,
        with_6dof_base=False,
        with_jacobian=False,
        use_cache=False,
    ):
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

        n_joint = len(joint_ids)
        if with_3dof_base:
            assert not with_6dof_base
            assert n_dof == n_joint + 3
        elif with_6dof_base:
            assert n_dof == n_joint + 6
        else:
            assert n_dof == n_joint

        if with_3dof_base:
            joint_angles_sequence = self._modify_input_with_3dof_base(
                n_joint, joint_angles_sequence
            )

        with_base = with_3dof_base or with_6dof_base
        P, J = self._robot.solve_forward_kinematics(
            joint_angles_sequence,
            elink_ids,
            joint_ids,
            with_rot,
            with_base,
            with_jacobian,
            use_cache,
        )
        if with_3dof_base:
            extrac_indices = np.hstack(
                (np.arange(n_joint), np.array([n_joint, n_joint + 1, n_joint + 5]))
            )
            J = J[:, extrac_indices]
        return P, J

    def get_joint_names(self):
        return self._robot.get_joint_names()

    def get_joint_ids(self, joint_names):
        return self._robot.get_joint_ids(joint_names)

    def get_link_ids(self, link_names):
        return self._robot.get_link_ids(link_names)

    def get_joint_limits(self, joint_ids):
        ret = self._robot.get_joint_limits(joint_ids)
        limits = []
        for lower, upper in ret:
            if lower == 0.0 and upper == 0.0:
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

    def compute_inter_link_sqdists(
        self,
        angle_vectors,
        link_id_pairs,
        joint_ids,
        with_3dof_base=False,
        with_6dof_base=False,
        with_jacobian=False,
        use_cache=False,
    ):
        if with_3dof_base:
            angle_vectors = self._modify_input_with_3dof_base(len(joint_ids), angle_vectors)

        with_base = with_3dof_base or with_6dof_base
        link_ids1, link_ids2 = zip(*link_id_pairs)
        V, J = self._robot.compute_inter_link_squared_dists(
            angle_vectors,
            list(link_ids1),
            list(link_ids2),
            joint_ids,
            with_base,
            with_jacobian,
            use_cache,
        )

        if with_3dof_base:
            n_joint = len(joint_ids)
            extrac_indices = np.hstack(
                (np.arange(n_joint), np.array([n_joint, n_joint + 1, n_joint + 5]))
            )
            J = J[:, extrac_indices]
        return V, J

    def clear_cache(self):
        self._robot.clear_cache()

    # for pickling and unpickling
    # https://stackoverflow.com/questions/1939058/simple-example-of-use-of-setstate-and-getstate
    def __getstate__(self):  # pickling
        return {"_xml_text": self._xml_text}

    def __setstate__(self, d):  # unpickling
        self._xml_text = d["_xml_text"]
        self._robot = _tinyfk.RobotModel(self._xml_text)
