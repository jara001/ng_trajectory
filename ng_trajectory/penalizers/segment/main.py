#!/usr/bin/env python3.6
# main.py
"""Penalize the incorrect solution by distance to the segments.
"""
######################
# Imports & Globals
######################

import numpy

import ng_trajectory.plot as ngplot

from ng_trajectory.interpolators.utils import pointDistance, trajectoryClosest, trajectoryClosestIndex
from ng_trajectory.segmentators.utils import *

from typing import List


# Global variables
DEBUG = False


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("debug", False, bool, "Whether debug plot is ought to be shown.", "Init.")


######################
# Functions
######################

def init(start_points: numpy.ndarray, **kwargs) -> None:
    """Initialize penalizer.

    Arguments:
    start_points -- initial line on the track, should be a centerline, nx2 numpy.ndarray
    """
    global DEBUG


    # Update parameters
    P.updateAll(kwargs)


    DEBUG = P.getValue("debug")


def penalize(points: numpy.ndarray, candidate: List[numpy.ndarray], valid_points: numpy.ndarray, grid: float, penalty: float = 100, **overflown) -> float:
    """Get a penalty for the candidate solution based on number of incorrectly placed points.

    Arguments:
    points -- points to be checked, nx(>=2) numpy.ndarray
    candidate -- raw candidate (non-interpolated points), m-list of 1x2 numpy.ndarray
    valid_points -- valid area of the track, px2 numpy.ndarray
    grid -- when set, use this value as a grid size, otherwise it is computed, float
    penalty -- constant used for increasing the penalty criterion, float, default 100
    **overflown -- arguments not caught by previous parts

    Returns:
    rpenalty -- value of the penalty, 0 means no penalty, float
    """

    # Use the grid or compute it
    _grid = grid if grid else gridCompute(points)

    _dists = []

    _invalid_ids = []

    for _ip, _p in enumerate(points):
        if not numpy.any(numpy.all(numpy.abs( numpy.subtract(valid_points, _p[:2]) ) < _grid, axis = 1)):
            _invalid_ids.append(_ip)

            _closest = trajectoryClosest(valid_points, _p)

            _dists.append(
                pointDistance(
                    _closest,
                    _p
                )
            )

            if DEBUG:
                ngplot.pointsPlot(numpy.vstack((_closest[:2], _p[:2])))


    _edges = []

    for _invalid_id in _invalid_ids:
        _a = (_invalid_id - 1) % len(points)
        _b = (_invalid_id + 1) % len(points)

        if _a not in _invalid_ids:
            _edges.append(
                trajectoryClosest(valid_points, points[_a])
            )

        if _b not in _invalid_ids:
            _edges.append(
                trajectoryClosest(valid_points, points[_b])
            )


    if DEBUG:
        ngplot.pointsScatter(numpy.asarray(_edges), color="green", marker="o")

    return penalty * max([0] + _dists)
