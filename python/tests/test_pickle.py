import copy
import pickle

import tinyfk

urdf_model_path = tinyfk.pr2_urdfpath()
fksolver = tinyfk.RobotModel(urdf_model_path)
p = pickle.dumps(fksolver)
fksolver2 = copy.copy(fksolver)
