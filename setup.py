#!/usr/bin/env python
# -*- coding: utf-8 -*-
# setup.py
"""Install script for this package."""

import os
from setuptools import setup, find_packages
from _version import Version

# Utility function to read the README file.
# Used for the long_description.  It's nice, because now 1) we have a top level
# README file and 2) it's easier to type in the README file than to put a raw
# string in below ...
#def read(fname):
#    return open(os.path.join(os.path.dirname(__file__), fname)).read()

VERSION = str(Version(os.popen("git describe --tags --dirty --always").read()[1:-1]))

if os.path.exists("VERSION"):
    STORED = open("VERSION", "r").read()

    if ".dev" in VERSION:
        _len = len(VERSION[:VERSION.index(".dev")+4])
    else:
        _len = len(VERSION)

    if STORED[:_len] == VERSION[:_len] and ".dev" in VERSION:
        # Obtain dev number
        VERSION = VERSION + str(int(STORED[_len:]) + 1)
    else:
        if ".dev" in VERSION:
            VERSION = VERSION + "0"
else:
    if ".dev" in VERSION:
        VERSION = VERSION + "0"


with open("VERSION", "w") as file:
    file.write(VERSION)


setup(
    name = "ng_trajectory",
    version = VERSION,
    author = "Jaroslav Klapálek",
    author_email = "klapajar@fel.cvut.cz",
    description = ("A trajectory generation script using Nevergrad."),
    license = "GPLv3",
    keywords = "trajectory Nevergrad",
    #url = "http://packages.python.org/an_example_pypi_project",
    packages=find_packages(),
    #long_description=read('README'),
    classifiers=[
        "Development Status :: 4 - Beta",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Topic :: Scientific/Engineering",
        "Typing :: Typed",
    ],
    install_requires=['nevergrad==0.3.0', "scipy>=0.18.0", "numpy>=1.12.0"],
    python_requires='>=3.6',
    extras_require={
        "plot_support": "matplotlib"
    },
    scripts=['bin/ng_run'],
)
