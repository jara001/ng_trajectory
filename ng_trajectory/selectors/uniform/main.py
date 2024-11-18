#!/usr/bin/env python3.6
# main.py
"""Select points from a path uniformly.

Note: It is just selector from `trajectoryReduce`.
"""
######################
# Imports & Globals
######################

import numpy
import sys

# Support for rotating the trajectory
from ng_trajectory.interpolators.utils import trajectoryRotate

from ng_trajectory.log import print0

from ng_trajectory.parameter import ParameterList

from typing import (
    Any,
    Dict,
    Optional,
)

# Global variables
ROTATE = 0


# Parameters
P = ParameterList()
P.createAdd("rotate", 0, float, "Parameter for rotating the subset selection. 0 is not rotated. <0, 1)", "init")


######################
# Functions
######################

def init(
        rotate: float = 0,
        **kwargs) -> Optional[Dict[str, Any]]:
    """Initialize selector.

    Arguments:
    rotate -- parameter for rotating the subset selection,
              <0, 1) float, default 0
    """
    global ROTATE

    if 0 <= rotate < 1:
        ROTATE = rotate
    else:
        print0(
            "Expected 'rotate' to be 0<=rotate<1, but it is %f. Omitting."
            % rotate,
            file=sys.stderr
        )


def trajectoryReduce(points: numpy.ndarray, remain: int) -> numpy.ndarray:
    """Select 'remain' points from 'points' equally.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray
    """
    global ROTATE

    if ROTATE == 0:
        return points[
            numpy.linspace(
                0, len(points) - 1, remain, dtype = numpy.int, endpoint = False
            ), :
        ]

    ls = numpy.linspace(
        0, len(points) - 1, remain, dtype = numpy.int, endpoint = False
    )

    return trajectoryRotate(points, ls[1], ROTATE)[ls, :]

    # Old rotate (without utils)
    ls2 = numpy.linspace(
        (ls[1] - ls[0]) * ROTATE,
        len(points) - 1 + (ls[1] - ls[0]) * ROTATE,
        remain,
        dtype = numpy.int,
        endpoint = False
    )

    # Wrap up the array
    ls2[ls2 >= len(points)] -= len(points)

    return points[ls2, :]

    # Older rotate (absolute)
    # Create linspace
    ls = numpy.linspace(
        (len(points) - 1) * ROTATE,
        (len(points) - 1) * (1 + ROTATE),
        remain,
        dtype = numpy.int,
        endpoint = False
    )

    # Wrap up the array
    ls[ls >= (len(points) - 1)] -= (len(points) - 1)

    return points[ls, :]


def select(points: numpy.ndarray, remain: int, **overflown) -> numpy.ndarray:
    """Select points from the path uniformly.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int
    **overflown -- arguments not caught by previous parts

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray

    Note: When 'remain' is negative the functions raises an Exception.
    """
    if remain < 0:
        # Raise an exception, as we cannot guess number of points.
        raise ValueError(
            "Negative selection is not supported by 'uniform' selector."
        )

    return trajectoryReduce(points, remain)
