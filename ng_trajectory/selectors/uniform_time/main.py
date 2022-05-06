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

    Note: When 'remain' is negative the functions raises an Exception.
    """

    # Resample the trajectory (even with the super sampling!)
    resampled_trajectory = trajectoryResample(points, 1000)

    # Compute the profile
    _, _, _t = profiler.profileCompute(resampled_trajectory, P.getValue("overlap"))

    # Create a time-linspace
    space = numpy.linspace(0.0, _t[-1], remain, endpoint = False)

    # Sample the trajectory
    result = []

    for point in space:
        index = 0
        dist = 1000000

        for _i, p in enumerate(resampled_trajectory):
            _dist = abs(_t[_i] - point)
            if _dist < dist:
                index = _i
                dist = _dist

        resampled_point = resampled_trajectory[index, :2]

        pindex = 0
        pdist = 1000000

        for _i, p in enumerate(points):
            _dist = utils.pointDistance(resampled_point, p)
            if _dist < pdist:
                pindex = _i
                pdist = _dist

        result.append(points[pindex, :])


    return numpy.asarray(result)
