## tinyfk [![CI](https://github.com/HiroIshida/tinyfk/workflows/CI/badge.svg)](https://github.com/HiroIshida/tinyfk/actions/workflows/main.yml) [![PyPI version](https://badge.fury.io/py/tinyfk.svg)](https://pypi.org/project/tinyfk/)
A tiny fast forward-kinematics solver written in c++ and its python wrapper

Installation by downloading wheel from PyPI (only linux):
```bash
pip install tinyfk
```

or, building locally from source (for developer):
```bash
sudo apt-get install libeigen3-dev
git clone https://github.com/HiroIshida/tinyfk.git
cd tinyfk
git submodule update --init
pip install . 
```

### Usage
```python
import numpy as np

import tinyfk
from tinyfk import BaseType, KinematicModel, RotationType

urdf_model_path = tinyfk.pr2_urdfpath()
kin = KinematicModel(urdf_model_path)
joint_names = [
    "r_shoulder_pan_joint",
    "r_shoulder_lift_joint",
    "r_upper_arm_roll_joint",
    "r_elbow_flex_joint",
    "r_forearm_roll_joint",
    "r_wrist_flex_joint",
    "r_wrist_roll_joint",
]

joint_ids = kin.get_joint_ids(joint_names)
end_link_id = kin.get_link_ids(["r_gripper_tool_frame"])[0]

# first 7 elements are for joints and the last 3 elements are for x, y, yaw of base.
q = np.array([0.564, 0.35, -0.74, -0.7, -0.7, -0.17, -0.63, 0.1, 0.2, 0.3])

poses, jacobians = kin.solve_fk(
    q,
    end_link_id,
    joint_ids,
    rot_type=RotationType.RPY,
    base_type=BaseType.PLANER,
    with_jacobian=True,
)
```
Also, simple inverse-kinematics demo is available in `python/example/ik.py`.

### For debugging
For debugging or developing, it's better using cmake directly rather than using `pip`. In this case, please use
```bash
sudo apt-get install libeigen3-dev
git clone https://github.com/HiroIshida/tinyfk.git
cd tinyfk
git submodule update --init
mkdir build
cd build
cmake --DCMAKE_BUILD_TYPE=Debug --DINSTALL_VIA_PIP=OFF ..
make -j4
# make install # please read the CMakeLists.txt before doing this
```
Note that when you build the python wrapper using cmake (not using pip), the package name for the wrapper will be `_tinyfk`. So, to make it compatible with the pip-installed one, please insert `import _tinyfk as tinyfk` in the beginning of the python script.
