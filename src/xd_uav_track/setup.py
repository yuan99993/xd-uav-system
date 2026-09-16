#!/usr/bin/env python3
"""Catkin Python package declaration for optional ReID frontends."""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup(**generate_distutils_setup(
    packages=["xd_uav_track", "xd_uav_track.reid"],
    package_dir={"": "python"},
))
