#!/usr/bin/env python3.6
# main.py
"""Select points from a path uniformly with respect to time.

Following algorithms are used:
- 'profile' criterion for computing the time,
- 'cubic_spline' interpolator for smoothing the input,
- 'uniform_distance' selector for resampling the input.
"""
######################
# Imports & Globals
######################

import sys
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

# ParameterList
from ng_trajectory.parameter import ParameterList

# Log
from ng_trajectory.log import print0

# Typing
from typing import (
    Any,
    Dict,
    List,
    Optional,
)


# Global variables
INTERPOLATOR = cubic_spline


# Parameters
P = ParameterList()
P.createAdd("rotate", 0, float, "Parameter for rotating the subset selection. 0 is not rotated. <0, 1)", "init")
for _, param in P_profile.iterate():
    P.add(param)
for _, param in P_select.iterate():
    P.add(param)


######################
# Utilities
######################

def timeSample(
        resampled_points: numpy.ndarray,
        time_vector: List[float],
        remain: int) -> numpy.ndarray:
    """Sample the trajectory equidistantly using time.

    Arguments:
    resampled_points -- list of resampled points, nx(>=2) numpy.ndarray
    time_vector -- list of time frames for every point, n-list of floats
    remain -- number of points in the result, int

    Returns:
    equidistant_points -- list of time-equidistantly spaced points,
                          remainx(>=2) numpy.ndarray
    """
    rotated_points = utils.trajectoryRotate(
        resampled_points,
        numpy.abs(
            numpy.subtract(
                time_vector,
                time_vector[-1] / remain
            )
        ).argmin(),
        P.getValue("rotate")
    )

    return numpy.asarray(
        [
            rotated_points[
                numpy.abs(
                    numpy.subtract(
                        time_vector,
                        time
                    )
                ).argmin(),
                :
            ]
            for time in numpy.linspace(
                0.0, time_vector[-1], remain, endpoint = False
            )
        ]
    )


######################
# Functions
######################

def init(**kwargs) -> Optional[Dict[str, Any]]:
    """Initialize selector."""
    P.updateAll(kwargs)

    profiler.parametersSet(
        **{param.name: param.value for _, param in P.iterate()}
    )


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
    if remain < 0:
        # Raise an exception, as we cannot guess number of points.
        raise ValueError(
            "Negative selection is not supported by 'uniform_time' selector."
        )

    if P.getValue("distance") <= 0:
        # Raise an exception, as we cannot proceed without further information.
        raise ValueError(
            "Selector 'uniform_time' requires 'distance' parameter "
            "to be set '>0.0'."
        )

    if P.getValue("overlap") <= 0:
        # This is not a hard error, but it affects the result a lot.
        print0(
            "Warning: Consider setting 'overlap' parameter, as otherwise, "
            "the initial conditions affect the results.",
            file = sys.stderr
        )

    # Resample the trajectory (even with the super sampling!)
    resampled_trajectory = trajectoryResample(points, -1)

    # Compute the profile
    _, _, _t = profiler.profileCompute(
        resampled_trajectory,
        P.getValue("overlap")
    )

    # Sample the trajectory equidistantly (time)
    equidistant_trajectory = timeSample(resampled_trajectory, _t, remain)

    return numpy.asarray(
        [
            utils.trajectoryClosest(points, resampled_point)
            for resampled_point in equidistant_trajectory
        ]
    )
