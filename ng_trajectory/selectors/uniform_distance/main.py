#!/usr/bin/env python3.6
# main.py
"""Select points from a path uniformly (and equidistantly).

Note: It is just selector branched from `Curvature2`.
"""
######################
# Imports & Globals
######################

import numpy, sys

# Cubic spline interpolation
from ng_trajectory.interpolators import cubic_spline

# Functions for equidistant resampling and selection
from ng_trajectory.selectors.curvature2.main import resolutionEstimate, factorCompute, pathPointDistanceAvg, pathLength


# Global variables
INTERPOLATOR = cubic_spline


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("sampling_distance", 1.0, float, "[m] Distance of super-sampling before the interpolation, skipped when 0.", "")
P.createAdd("distance", 0, float, "[m] Distance between the individual points, ignored when 0, used when requesting negative number of points.", "init")


######################
# Utilities
######################

def trajectoryResample(points, remain):
    """Resample path described by points.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int

    Returns:
    rpoints -- list of points,
               mx2 numpy.ndarray when remain < 0
               remainx2 numpy.ndarray otherwise
    """
    
    # Resample if requested
    if P.getValue("sampling_distance") != 0.0:
        points = INTERPOLATOR.interpolate(points[:, :2], resolutionEstimate(points, P.getValue("sampling_distance")))


    # Select points equidistantly
    if remain < 0:
        return INTERPOLATOR.interpolate(points[:, :2], resolutionEstimate(points, P.getValue("distance")))
    # Select 'remain' points
    else:
        return INTERPOLATOR.interpolate(points[:, :2], remain)


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize selector."""

    # Update parameters
    P.update(kwargs)


def select(points: numpy.ndarray, remain: int, **overflown) -> numpy.ndarray:
    """Select points from the path uniformly.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int
    **overflown -- arguments not caught by previous parts

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray

    Note: When 'remain' is negative, number of points is selected so in average
    the distance between points is 'distance'.

    Note: An Exception is raised when 'distance' <= 0 and 'remain' < 0.
    """

    if remain < 0 and P.getValue("distance") <= 0:
        # Raise an exception, as we cannot proceed without further information.
        raise ValueError("Negative selection requires set 'distance' parameter for 'uniform_distance' selector.")

    return trajectoryResample(points, remain)
