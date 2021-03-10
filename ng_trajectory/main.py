#!/usr/bin/env python3.6
# main.py
"""Main functions for ng_trajectory.
"""
######################
# Imports & Globals
######################

import numpy, json

#from ng_trajectory.configuration import configurationLoad
#from ng_trajectory.configuration import CONFIGURATION
CONFIGURATION = {}
from ng_trajectory.optimizers.matryoshka import init, optimize


######################
# Functions
######################

def configurationLoad(filename: str) -> bool:
    """Loads a configuration stored in 'filename'.

    Arguments:
    filename -- name of the file along with its path, str

    Returns:
    success -- True when loaded, otherwise False
    """
    global CONFIGURATION

    try:
        with open(filename, 'r') as configuration_file:
            conf = json.load(configuration_file)
            CONFIGURATION = {**CONFIGURATION, **conf}
    except:
        return False

    print (CONFIGURATION)


def dataLoad(filename: str) -> numpy.ndarray:
    """Loads data from stored file.

    Arguments:
    filename -- name of the file, str

    Returns:
    points -- an array of loaded points, nx2 numpy.ndarray
    """
    return numpy.load(filename)


def execute():
    """Execute GA according to the configuration."""
    global CONFIGURATION

    START_POINTS = dataLoad(CONFIGURATION.get("start_points"))
    VALID_POINTS = dataLoad(CONFIGURATION.get("valid_points"))

    init(VALID_POINTS, START_POINTS, **{**CONFIGURATION, **CONFIGURATION["cascade"][0]})
