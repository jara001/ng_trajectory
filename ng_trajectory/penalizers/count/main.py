#!/usr/bin/env python3.6
# main.py
"""Penalize the incorrect solution by number of incorrectly placed points.
"""
######################
# Imports & Globals
######################

import numpy

from ng_trajectory.segmentators.utils import gridCompute


# Parameters
#from ng_trajectory.parameter import *
#P = ParameterList()
#P.createAdd("int_size", 400, int, "Number of points in the interpolation.", "")


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize penalizer."""
    pass


def penalize(points: numpy.ndarray, valid_points: numpy.ndarray, grid: float, penalty: float = 100, **overflown) -> float:
    """Get a penalty for the candidate solution based on number of incorrectly placed points.

    Arguments:
    points -- points to be checked, nx(>=2) numpy.ndarray
    valid_points -- valid area of the track, mx2 numpy.ndarray
    grid -- when set, use this value as a grid size, otherwise it is computed, float
    penalty -- constant used for increasing the penalty criterion, float, default 100
    **overflown -- arguments not caught by previous parts

    Returns:
    rpenalty -- value of the penalty, 0 means no penalty, float
    """

    # Use the grid or compute it
    _grid = grid if grid else gridCompute(points)

    invalid = 0

    for _p in points:
        if not numpy.any(numpy.all(numpy.abs( numpy.subtract(valid_points, _p[:2]) ) < _grid, axis = 1)):
            invalid += 1

    return invalid * penalty
