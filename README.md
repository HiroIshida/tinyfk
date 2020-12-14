## tinyfk ![CI](https://github.com/HiroIshida/tinyfk/workflows/CI/badge.svg)
A tiny fast forward-kinematics solver written in c++ and its python wrapper

### requirement
Building this software requires that `libtinyxml` and `eigen3` is already installed. If not yet, please install them by:
```
sudo apt-get install libeigen3-dev libtinyxml-dev
```

### python wrapper installation
Installation by downloading wheel from PyPI (only linux):
```bash
pip install tinyfk
```
or, building locally:
```bash
# maybe you need to export:
# export PKG_CONFIG_PATH="$PKG_CONFIG_PATH:/usr/lib/x86_64-linux-gnu/pkgconfig"
git clone https://github.com/HiroIshida/tinyfk.git
cd tinyfk
git submodule update --init --depth=1
pip install . 
```

### For debugging
For debugging or developing, it's better using cmake directly rather than using `pip`. In this case, please use
```bash
git clone https://github.com/HiroIshida/tinyfk.git
cd tinyfk
git submodule update --init --depth=1
mkdir build
cd build
cmake --DCMAKE_BUILD_TYPE=Debug --DINSTALL_VIA_PIP=OFF ..
make -j4
# make install # please read the CMakeLists.txt before doing this
```
Note that when you build the python wrapper using cmake (not using pip), the package name for the wrapper will be `_tinyfk`. So, to make it compatible with the pip-installed one, please insert `import _tinyfk as tinyfk` in the beginning of the python script.

### Usage 
See `test_fksolver()` in `python/tests/test_tinyfk.py` ([here](/python/tests/test_tinyfk.py)).
