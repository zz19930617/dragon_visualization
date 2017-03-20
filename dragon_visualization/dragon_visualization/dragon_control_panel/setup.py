#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['dragaon_control_panel', 'dragon_control_panel.data_plot'],
    package_dir={'': 'src'},
    scripts=['scripts/dragon_control_panel']
)

setup(**d)
