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
MAP = None
MAP_ORIGIN = None
MAP_GRID = None


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("debug", False, bool, "Whether debug plot is ought to be shown.", "Init.")


######################
# Functions
######################

def init(start_points: numpy.ndarray, map: numpy.ndarray, map_origin: numpy.ndarray, map_grid: float, map_last: numpy.ndarray, **kwargs) -> None:
    """Initialize penalizer.

    Arguments:
    start_points -- initial line on the track, should be a centerline, nx2 numpy.ndarray
    """
    global DEBUG, MAP, MAP_ORIGIN, MAP_GRID, BORDERLINES

    # Save the grid map for later usage
    MAP = map_last.copy()
    MAP_ORIGIN = map_origin
    MAP_GRID = map_grid


    # Update parameters
    P.updateAll(kwargs)


    # Debug is used for showing extra content
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
        # Check whether the point is invalid (i.e., there is not a single valid point next to it).
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


    if DEBUG:
        _edges = []
        _edges_area = []
        _is_border = []

        # Every invalid point is processed...
        for _invalid_id in _invalid_ids:
            # ... neighbours are observed...
            _a = (_invalid_id - 1) % len(points)
            _b = (_invalid_id + 1) % len(points)

            # ... and used only when they are valid to show the 'valid area edge'.
            if _a not in _invalid_ids:
                _close_index = trajectoryClosestIndex(valid_points, points[_a])
                _close_point = valid_points[_close_index, :]

                _edges.append(
                    _close_point
                )

                _distances = numpy.subtract(valid_points[:, :2], _close_point[:2])

                for _area_index in numpy.argwhere(
                    numpy.hypot(_distances[:, 0], _distances[:, 1]) <= numpy.hypot(_grid[0], _grid[1])
                ):
                    if _area_index[0] == _close_index:
                        continue

                    _edges_area.append(
                        valid_points[_area_index[0], :2]
                    )

                    _is_border.append(borderCheck(pointToMap(valid_points[_area_index[0], :2])))


            if _b not in _invalid_ids:
                _close_index = trajectoryClosestIndex(valid_points, points[_b])
                _close_point = valid_points[_close_index, :]

                _edges.append(
                    _close_point
                )

                _distances = numpy.subtract(valid_points[:, :2], _close_point[:2])

                for _area_index in numpy.argwhere(
                    numpy.hypot(_distances[:, 0], _distances[:, 1]) <= numpy.hypot(_grid[0], _grid[1])
                ):
                    if _area_index[0] == _close_index:
                        continue

                    _edges_area.append(
                        valid_points[_area_index[0], :2]
                    )

                    _is_border.append(borderCheck(pointToMap(valid_points[_area_index[0], :2])))


    if DEBUG:
        ngplot.pointsScatter(numpy.asarray(_edges), color="green", marker="o")
        ngplot.pointsScatter(numpy.asarray(_edges_area), color=[ ("red" if _border else "yellow") for _border in _is_border ], marker="o")

    return penalty * max([0] + _dists)
