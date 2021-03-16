#!/usr/bin/env python3.6
# utils.py3
"""Various utilities for interpolators.
"""
######################
# Imports & Globals
######################

import math, numpy


######################
# Utilities (Point)
######################

def pointDistance(a: list, b: list) -> float:
    """Computes distance between two points in 2D.

    Arguments:
    a -- first point, n-list of numbers
    b -- second point, n-list of numbers

    Returns:
    d -- distance between points, float

    Note: Taken from 'profile_trajectory2.py', ported to Py3.6.
    Differences:
        We are using n-list instead od geometry_msgs.msg/Point.
        Number of coordinates is dynamic.
    """
    return math.sqrt(sum(
            [ pow(b[i] - a[i], 2) for i in range( min(len(a), len(b)) ) ]
        ))


######################
# Utilities (Trajectory)
######################

def trajectorySort(points: numpy.ndarray) -> numpy.ndarray:
    """Sorts a trajectory (array of points) to be "in order".

    Arguments:
    points -- list of points, nx2 numpy.ndarray

    Returns:
    spoints -- sorted points, nx2 numpy.ndarray

    Note: This is taken from 'center_trajectory.py'.
    Note: This version is NOT making spoints[0]==spoints[-1]!
    Note: Taken from 'profile_trajectory2.py', ported to Py3.6.
    Differences:
        In here, we are working with numpy.ndarray!
        We are starting to sort from index 0.
    """

    _points = points.tolist()

    sorted_points = []
    sorted_points.append(_points.pop(0))

    while len(_points) > 0:
        min_dist = 100000
        point = None

        for p in _points:
            dist = pointDistance(p, sorted_points[-1])

            if dist < min_dist:
                min_dist = dist
                point = p

        sorted_points.append(point)
        _points.remove(point)

    return numpy.asarray(sorted_points)


def trajectoryReduce(points: numpy.ndarray, remain: int) -> numpy.ndarray:
    """Selects 'remain' points from 'points' equally.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray
    """
    return points[numpy.linspace(0, len(points)-1, remain, dtype=numpy.int, endpoint=False), :]
