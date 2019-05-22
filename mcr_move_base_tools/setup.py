#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_move_base_tools'],
    package_dir={'mcr_move_base_tools': 'ros/src/mcr_move_base_tools'}
)

setup(**d)
