#! /usr/bin/env python2.7

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
  packages=['audio_trigger_node'],
  package_dir={'': 'src'},
)

setup(**setup_args)
