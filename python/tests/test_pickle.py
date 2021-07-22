import copy
import pickle
import unittest
import numpy as np

try:
    import tinyfk
except:
    import _tinyfk as tinyfk

urdf_model_path = tinyfk.pr2_urdfpath()
fksolver = tinyfk.RobotModel(urdf_model_path)
p = pickle.dumps(fksolver)
fksolver2 = copy.copy(fksolver)
