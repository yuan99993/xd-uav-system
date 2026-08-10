#!/usr/bin/env python3

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=[
        "xd_uav_sead",
        "xd_uav_sead.airspace",
        "xd_uav_sead.comms",
        "xd_uav_sead.drone",
        "xd_uav_sead.formation",
        "xd_uav_sead.planning",
        "xd_uav_sead.strike",
    ],
    package_dir={"": "src"},
)

setup(**setup_args)
