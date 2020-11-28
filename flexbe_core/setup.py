#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['flexbe_core', 'flexbe_core.core', 'flexbe_core.proxy'],
    package_dir={'': 'src'}
)

setup(**d)
