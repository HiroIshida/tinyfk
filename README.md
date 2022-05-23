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
The following example includes both inverse-kinematics and forward kinematics. It first solves inverse kinematics to get an angle vector to achieve desired end effector pose. Then put the solved angle vector to forward kinematics and check that it actually achieves desired end effector pose. 
```python
import numpy as np
import tinyfk

urdf_model_path = tinyfk.pr2_urdfpath()
kin_solver = tinyfk.RobotModel(urdf_model_path)
rarm_joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]

rarm_joint_ids = kin_solver.get_joint_ids(rarm_joint_names)
end_link_id = kin_solver.get_link_ids(["r_gripper_tool_frame"])[0]

angle_vector_init = np.array([0.564, 0.35, -0.74, -0.7, -0.7, -0.17, -0.63])

# if you set 3 dim vector instead, just rpy componenent is not considered 
# and point-ik will be solved
desired_xyzrpy = [0.7, -0.5, 0.6, 0.0, 0.0, 0.0]

av_sol = kin_solver.solve_inverse_kinematics(
        desired_xyzrpy, 
        angle_vector_init,
        end_link_id,
        rarm_joint_ids,
        with_base=False)

# When with_rot = True, fk will be solved about rotation not only about the position.
# If you need only position-fk, set it to False, makes computation about 2x faster
# Set with_jacobian=False if you don't need jacobian. This leads jac=None, and it's 2x faster.
xyzrpy, jac = kin_solver.solve_forward_kinematics(
        [av_sol], [end_link_id], rarm_joint_ids, with_rot=True, with_jacobian=False)
print(np.linalg.norm(xyzrpy - desired_xyzrpy))
```
The output is
```
9.50047504061831e-05
```

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
