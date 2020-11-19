import sys

# see 
# https://github.com/scikit-build/scikit-build-sample-projects/blob/master/projects/hello-pybind11/setup.py
try:
    from skbuild import setup
except ImportError:
    raise Exception

setup(
    name="pytinyfk",
    version="1.2.3",
    description="a minimal example package (with pybind11)",
    author='Hirokazu Ishida',
    license="MIT",
    packages=["pytinyfk"],
    package_dir={'': 'python'},
    cmake_install_dir='python/pytinyfk/'
    )
