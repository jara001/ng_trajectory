#!/usr/bin/env python3.6
# main.py
"""Manual criterion, fitness value is specified by the user.
"""
######################
# Imports & Globals
######################

import sys
import numpy

from threading import Lock


# Global variables
INPUT_LOCK = Lock()


######################
# Functions
######################

def init(**kwargs):
    """Initialize criterion."""
    return None


def compute(
        points: numpy.ndarray,
        overlap: int = None,
        penalty: float = 100.0,
        **overflown) -> float:
    """Compute the speed profile using overlap.

    Arguments:
    points -- points of a trajectory with curvature, nx3 numpy.ndarray
    overlap -- size of trajectory overlap, int, default None/0 (disabled)
    penalty -- penalty value applied to the incorrect solutions,
               float, default 100.0
    **overflown -- arguments not caught by previous parts

    Returns:
    t -- fitness value
    """
    print(";".join(["%s,%s" % (x, y) for x, y in points[:, :2]]))

    with INPUT_LOCK:
        while True:
            try:
                val = input("Specify fitness value (%f): ")
                float(val)
            except ValueError:
                print ("Unable to parse the value.", file = sys.stderr)

            break

    return float(val)
