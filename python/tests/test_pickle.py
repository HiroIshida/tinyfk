import os
import copy
import pickle
import unittest
import numpy as np

try:
    import tinyfk
except:
    import _tinyfk as tinyfk

here_full_filepath = os.path.join(os.getcwd(), __file__)
here_full_dirpath = os.path.dirname(here_full_filepath)
project_base_path = os.path.join(here_full_dirpath, "..", "..")
urdf_model_path = os.path.join(project_base_path, "data", "pr2.urdf")

fksolver = tinyfk.RobotModel(urdf_model_path)
p = pickle.dumps(fksolver)
fksolver2 = copy.copy(fksolver)
