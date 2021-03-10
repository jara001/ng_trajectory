#!/usr/bin/env python3.6
# configuration.py
"""Working with configuration for ng_trajectory.
"""
######################
# Imports & Globals
######################

import sys, json

# Global variables
CONFIGURATION = {}


######################
# Configuration import
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

    return True
