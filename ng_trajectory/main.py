#!/usr/bin/env python3.6
# main.py
"""Main functions for ng_trajectory.
"""
######################
# Imports & Globals
######################

import sys, numpy, json

#from ng_trajectory.configuration import configurationLoad
#from ng_trajectory.configuration import CONFIGURATION
CONFIGURATION = {}

import ng_trajectory.optimizers as optimizers


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

            if conf.get("_version", 1) < 2:
                print ("Unsupported version of the configuration file.", file=sys.stderr)
            else:
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

    # Load data about the track
    START_POINTS = dataLoad(CONFIGURATION.get("start_points"))
    VALID_POINTS = dataLoad(CONFIGURATION.get("valid_points"))

    # Create loops
    for _loop in CONFIGURATION.get("loops"):
        # Initial solution
        fitness = 10000000
        candidate = numpy.asarray([ [0.5, 0.5] for _i in range(START_POINTS.shape[0])])
        result = START_POINTS

        # Run cascade
        for _alg in CONFIGURATION.get("cascade"):
            opt = optimizers.__getattribute__(_alg.get("algorithm"))

            opt.init(VALID_POINTS, result, **{**CONFIGURATION, **_alg})
            opt.optimize()
