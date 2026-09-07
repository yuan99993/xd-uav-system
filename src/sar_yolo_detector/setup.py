#!/usr/bin/env python3
"""Catkin-aware Python package setup."""

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=[
        "sar_yolo_detector",
        "sar_yolo_detector.pixeagle",
        "sar_yolo_detector.pixeagle.backends",
    ],
    package_dir={"": "python"},
)

setup(**setup_args)
