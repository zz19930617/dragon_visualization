#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_plot_entire', 'rqt_plot_entire.rqt_plot_lf'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_plot_entire']
)

setup(**d)
