#!/usr/bin/env python

# python dependencies

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['my package'],
     package_dir={'': 'src'},
     install_requires=[‘required_python_package1’, 'required_python_package2’]
)