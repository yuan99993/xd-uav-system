#!/usr/bin/env python3

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup


setup_args = generate_distutils_setup(
    packages=[
        "xd_uav_task_allocate",
        "xd_uav_task_allocate.core",
        "xd_uav_task_allocate.ros",
    ],
    package_dir={"": "src"},
)

setup(**setup_args)
