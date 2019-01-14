#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['platforms_server', 'path_planner', 'vision'],
    package_dir={'': 'include'},
)

setup(**d)
