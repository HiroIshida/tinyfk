import sys

# see 
# https://github.com/scikit-build/scikit-build-sample-projects/blob/master/projects/hello-pybind11/setup.py
try:
    from skbuild import setup
except ImportError:
    raise Exception

setup(
    name="tinyfk",
    version="0.6.0.dev7",
    description="a fast kinematics solver",
    author='Hirokazu Ishida',
    license="MIT",
    packages=["tinyfk"],
    package_dir={'': 'python'},
    cmake_install_dir='python/tinyfk/'
    )
