import os
import json
import unittest
import numpy as np
import tinyfk

here_full_filepath = os.path.join(os.getcwd(), __file__)
here_full_dirpath = os.path.dirname(here_full_filepath)
project_base_path = os.path.join(here_full_dirpath, "..", "..")

urdf_model_path = os.path.join(project_base_path, "data", "fetch.urdf")
test_data_path = os.path.join(project_base_path, "test", "test_data.json")

with open(test_data_path, 'r') as f:
    test_data = json.load(f)

def test_fksovler():
    fksolver = tinyfk.RobotModel(urdf_model_path)
