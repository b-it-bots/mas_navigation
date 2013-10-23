#!/usr/bin/env python

import distutils.core
import catkin_pkg.python_setup

d = catkin_pkg.python_setup.generate_distutils_setup(
   packages=['mcr_relative_movements'],
   package_dir={'mcr_relative_movements': 'ros/scripts'}
)

distutils.core.setup(**d)
