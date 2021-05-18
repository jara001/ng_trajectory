#!/usr/bin/env python3.6
# main.py
"""Select points from a path uniformly.

Note: It is just selector from `trajectoryReduce`.
"""
######################
# Imports & Globals
######################

import numpy, sys


# Global variables
ROTATE = 0


######################
# Functions
######################

def init(rotate: float = 0,
        **kwargs) -> None:
    """Initialize selector.

    Arguments:
    rotate -- parameter for rotating the subset selection, <0, 1) float, default 0
    """
    global ROTATE

    if 0 <= rotate < 1:
        ROTATE = rotate
    else:
        print ("Expected 'rotate' to be 0<=rotate<1, but it is %f. Omitting." % rotate, file=sys.stderr)


def trajectoryReduce(points: numpy.ndarray, remain: int) -> numpy.ndarray:
    """Selects 'remain' points from 'points' equally.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray
    """
    global ROTATE

    if ROTATE == 0:
        return points[numpy.linspace(0, len(points)-1, remain, dtype=numpy.int, endpoint=False), :]

    # Create linspace
    ls = numpy.linspace(
            (len(points)-1) * ROTATE,
            (len(points)-1) * (1+ROTATE),
            remain,
            dtype = numpy.int,
            endpoint = False
        )

    # Wrap up the array
    ls[ls>=(len(points)-1)] -= (len(points) - 1)

    return points[ls, :]


def select(points: numpy.ndarray, remain: int, **overflown) -> numpy.ndarray:
    """Select points from the path uniformly.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int
    **overflown -- arguments not caught by previous parts

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray
    """
    return trajectoryReduce(points, remain)
