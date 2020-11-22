## tinyfk
A tiny fast forward-kinematics solver written in c++ and its python wrapper

### Installation for python
By using pip (currently supports only linux),
```bash
pip install tinyfk --user
```
or you can also, install by locally building:
```bash
git clone https://github.com/HiroIshida/tinyfk.git
cd tinyfk
git clone https://github.com/pybind/pybind11.git
pip install . --user 
```

For debugging or developing please use: 
```bash
git clone https://github.com/HiroIshida/tinyfk.git
cd tinyfk
git clone https://github.com/pybind/pybind11.git
mkdir build
cd build
cmake --DCMAKE_BUILD_TYPE=Debug --DINSTALL_VIA_PIP=OFF ..
make -j4
# make install # please read the CMakeLists.txt before doing this
```

### Usage 
The `solve_forward_kinematics` method compute pose `P` and (geometric) jacobian `J` given joing angle's sequence `av_seq`. Before running the following example, please run `cd data; ./download.sh` to obtain the urdf data of fetch.
```python
import tinyfk
fksolver = tinyfk.RobotModel("data/fetch_description/fetch.urdf")
ef_links = fksolver.get_link_ids(["gripper_link", "forearm_roll_link"])
joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
rarm_jids = fksolver.get_joint_ids(joint_names)

av = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1] 
av_seq = [av] * 20 # sequence of joint angles (just copied here for simplicity)

# returining poses and jacobian 
rot_also = True
base_also = False
P, J = fksolver.solve_forward_kinematics(av_seq, ef_links, rarm_jids, 
        rot_also, base_also)
```


