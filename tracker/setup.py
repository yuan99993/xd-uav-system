#!/usr/bin/env python3
"""
setup.py — Catkin-aware Python package setup for tracker.
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tracker'],
    package_dir={'': 'src'},
)

setup(**d)
