#!/usr/bin/env python3.6
# main.py
"""Select points from a path uniformly.

Note: It is just selector from `trajectoryReduce`.
"""
######################
# Imports & Globals
######################

import numpy


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize selector."""
    pass


def trajectoryReduce(points: numpy.ndarray, remain: int) -> numpy.ndarray:
    """Selects 'remain' points from 'points' equally.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray
    """
    return points[numpy.linspace(0, len(points)-1, remain, dtype=numpy.int, endpoint=False), :]


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
