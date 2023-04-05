import copy
import pickle

import tinyfk


def test_pickle():
    urdf_model_path = tinyfk.pr2_urdfpath()
    fksolver = tinyfk.RobotModel(urdf_model_path)
    pickle.dumps(fksolver)
    copy.copy(fksolver)


def test_deepcopy():
    urdf_model_path = tinyfk.pr2_urdfpath()
    fksolver = tinyfk.RobotModel(urdf_model_path)
    copy.deepcopy(fksolver)
