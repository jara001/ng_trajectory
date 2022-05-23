#!/usr/bin/env python3.6
# main.py
"""Penalize the incorrect solution by distance to the segments.
"""
######################
# Imports & Globals
######################

import numpy

import ng_trajectory.plot as ngplot

from ng_trajectory.interpolators.utils import pointDistance, trajectoryClosest, trajectoryClosestIndex, trajectoryFarthest
from ng_trajectory.segmentators.utils import *

from typing import List


# Global variables
CENTERLINE = None
DEBUG = False
MAP = None
MAP_ORIGIN = None
MAP_GRID = None


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("debug", False, bool, "Whether debug plot is ought to be shown.", "Init.")


######################
# Utilities
######################

def arraySlice(points: numpy.ndarray, index1: int, index2: int) -> numpy.ndarray:
    """Obtain a slice of an array of points.

    Arguments:
    points -- points array to be sliced, nx(>=2) numpy.ndarray
    index1 -- start index of the slice, int
    index2 -- end index of the slice, int

    Returns:
    slice -- slice of the input array, mx(>=2) numpy.ndarray

    Note: In constrast to standard slice operator (:) we allow wrapping around 0.
    """
    if index2 > index1:
        return points[index1:index2, :]
    else:
        return numpy.vstack(
            (points[index1:], points[:index2])
        )


######################
# Functions
######################

def init(start_points: numpy.ndarray, map: numpy.ndarray, map_origin: numpy.ndarray, map_grid: float, map_last: numpy.ndarray, **kwargs) -> None:
    """Initialize penalizer.

    Arguments:
    start_points -- initial line on the track, should be a centerline, nx2 numpy.ndarray
    """
    global DEBUG, MAP, MAP_ORIGIN, MAP_GRID, CENTERLINE

    # Save the grid map for later usage
    MAP = map_last.copy()
    MAP_ORIGIN = map_origin
    MAP_GRID = map_grid


    # Update parameters
    P.updateAll(kwargs)


    # Debug is used for showing extra content
    DEBUG = P.getValue("debug")


    if CENTERLINE is None:
        CENTERLINE = start_points
        print ("Penalizer: Updating the centerline.")


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


    # Map candidate onto the centerline
    _candidate_centerline_mapping = [
        trajectoryClosestIndex(
            CENTERLINE,
            _candidate
        ) for _candidate in candidate
    ]


    # Map candidate onto the points
    _candidate_points_mapping = [
        trajectoryClosestIndex(
            points,
            _candidate
        ) for _candidate in candidate
    ]

    _dists = []

    _invalids = []

    # 1. Find invalid points
    for _ip, _p in enumerate(points):

        # Check whether the point is invalid (i.e., there is not a single valid point next to it).
        if not numpy.any(numpy.all(numpy.abs( numpy.subtract(valid_points, _p[:2]) ) < _grid, axis = 1)):

            _invalids.append(_ip)

            _closest = trajectoryClosest(valid_points, _p)

            _dists.append(
                pointDistance(
                    _closest,
                    _p
                )
            )

            if DEBUG:
                ngplot.pointsPlot(numpy.vstack((_closest[:2], _p[:2])))


    # 2. Find edges of the track area
    _edge_pairs = []

    for _id in _invalids:
        for _i in [-1, 1]:
            if (_id + _i) % len(points) not in _invalids:
                _space_id = (len([ _cpm for _cpm in _candidate_points_mapping if _cpm <= _id ]) - 1) % len(candidate)

                _edge_pairs.append((_id, (_id + _i) % len(points), _space_id))


    # 3. Find closest valid point on each edge
    _edges = []
    _discovered = []
    _center_indices = []

    for out, inside, space_id in _edge_pairs:
        # a. Compute center point on the edge
        _center_point = (points[out] + points[inside]) / 2

        # b. Find closest valid point
        _close_index = trajectoryClosestIndex(valid_points, _center_point)
        _close_point = valid_points[_close_index, :]


        # c. Find closest border
        while not borderCheck(pointToMap(_close_point)):

            _distances = numpy.subtract(valid_points[:, :2], _close_point[:2])

            _temp_close_point = None

            for _area_index in sorted(numpy.argwhere(
                numpy.hypot(_distances[:, 0], _distances[:, 1]) <= numpy.hypot(_grid[0], _grid[1])
            ), key = lambda x: pointDistance(valid_points[x[0], :2], points[out])):
                if _area_index[0] == _close_index:
                    continue

                _is_border_point = borderCheck(pointToMap(valid_points[_area_index[0], :2]))

                if _temp_close_point is None or (not _temp_close_point[0] and _is_border_point):
                    _temp_close_point = (_is_border_point, valid_points[_area_index[0], :2])

                _discovered.append((
                    valid_points[_area_index[0], :2],
                    _is_border_point
                ))


            _close_point = _temp_close_point[1]


        _edges.append(
            _close_point
        )

        # ~d. Find the farthest centerline point~
        #if DEBUG:
        #   ngplot.pointsPlot(numpy.asarray([_close_point[:2], trajectoryFarthest(arraySlice(CENTERLINE, _candidate_centerline_mapping[space_id], _candidate_centerline_mapping[(space_id+1)%len(candidate)]), points[out])]))

        # d. Actually, find the distance (and also direction) to corresponding centerline point
        _id1 = _candidate_points_mapping[space_id]
        _id2 = _candidate_points_mapping[(space_id+1)%len(candidate)]

        space_length = (len(points) - _id1 + _id2 if _id2 < _id1 else _id2 - _id1)

        # Procentual index of the invalid point
        relative_index = (out - _id1) / space_length

        # And now get the centerline point
        _id1 = _candidate_centerline_mapping[space_id]
        _id2 = _candidate_centerline_mapping[(space_id+1)%len(candidate)]

        space_center_length = (len(CENTERLINE) - _id1 + _id2 if _id2 < _id1 else _id2 - _id1)

        center_index = int(space_center_length * relative_index) + _id1
        _center_indices.append(center_index)

        if DEBUG:
            ngplot.pointsPlot(numpy.asarray([_close_point[:2], CENTERLINE[center_index, :2]]))


    # 4. Merge the edges etc. to make pairs for linking
    _merged = []

    _i = 0
    while _i < len(_edges):
        # Border A, border B, center index A
        _merged.append((_edges[_i], _edges[_i + 1], _center_indices[_i]))
        _i += 2


    if DEBUG:
        if len(_discovered) > 0:
            ngplot.pointsScatter(numpy.asarray([point for point, border in _discovered]), color=[ ("red" if border else "yellow") for point, border in _discovered ], marker="o")
        ngplot.pointsScatter(numpy.asarray(_edges), color="green", marker="o")

    return penalty * max([0] + _dists)
