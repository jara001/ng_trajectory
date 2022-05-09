#!/usr/bin/env python3.6
# main.py
"""Select points from a path uniformly with respect to time.
"""
######################
# Imports & Globals
######################

import numpy

# Cubic spline interpolation
from ng_trajectory.interpolators import cubic_spline
from ng_trajectory.interpolators import utils

# Uniform distance selector for resampling
from ng_trajectory.selectors.uniform_distance.main import P as P_select
from ng_trajectory.selectors.uniform_distance.main import trajectoryResample

# Profile criterion
from ng_trajectory.criterions.profile.main import P as P_profile
from ng_trajectory.criterions.profile import profiler

# Typing
from typing import List


# Global variables
INTERPOLATOR = cubic_spline


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
for _, param in P_profile.iterate():
    P.add(param)
for _, param in P_select.iterate():
    P.add(param)


######################
# Utilities
######################

def timeSample(resampled_points: numpy.ndarray, time_vector: List[float], remain: int) -> numpy.ndarray:
    """Sample the trajectory equidistantly using time.

    Arguments:
    resampled_points -- list of resampled points, nx(>=2) numpy.ndarray
    time_vector -- list of time frames for every point, n-list of floats
    remain -- number of points in the result, int

    Returns:
    equidistant_points -- list of time-equidistantly spaced points, remainx(>=2) numpy.ndarray
    """
    return numpy.asarray(
        [
            resampled_points[
                numpy.abs(
                    numpy.subtract(
                        time_vector,
                        time
                    )
                ).argmin(),
            :]
            for time in numpy.linspace(0.0, time_vector[-1], remain, endpoint = False)
        ]
    )


######################
# Functions
######################

def init(rotate: float = 0,
        **kwargs) -> None:
    """Initialize selector."""

    P.updateAll(kwargs)

    profiler.parametersSet(**{param.name: param.value for _, param in P.iterate()})


def select(points: numpy.ndarray, remain: int, **overflown) -> numpy.ndarray:
    """Select points from the path uniformly with respect to time.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int
    **overflown -- arguments not caught by previous parts

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray

    Note: When 'remain' is negative the function raises an Exception.
    """

    # Resample the trajectory (even with the super sampling!)
    resampled_trajectory = trajectoryResample(points, -1)

    # Compute the profile
    _, _, _t = profiler.profileCompute(resampled_trajectory, P.getValue("overlap"))

    # Sample the trajectory equidistantly (time)
    equidistant_trajectory = timeSample(resampled_trajectory, _t, remain)

    return numpy.asarray(
        [
            utils.trajectoryClosest(points, resampled_point)
            for resampled_point in equidistant_trajectory
        ]
    )
