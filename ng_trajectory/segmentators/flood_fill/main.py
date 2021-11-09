#!/usr/bin/env python3.6
# main.py
"""Segmentate track using flood fill algorithm.
"""
######################
# Imports & Globals
######################

import numpy

from ng_trajectory.segmentators.utils import *

from typing import List


# Global variables
MAP = None
MAP_ORIGIN = None
MAP_GRID = None


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("hold_map", False, bool, "When true, the map is created only once.", "init")
P.createAdd("range_limit", 0, float, "Maximum distance from the center of the segment. 0 disables this.", "")


######################
# Functions
######################

def init(track: numpy.ndarray, **kwargs) -> None:
    """Initialize segmentator by creating map."""
    global MAP, MAP_ORIGIN, MAP_GRID

    # Update parameters
    P.updateAll(kwargs)

    if MAP is None or not P.getValue("hold_map"):
        MAP, MAP_ORIGIN, MAP_GRID = mapCreate(track)


def segmentate(points: numpy.ndarray, group_centers: numpy.ndarray, **overflown) -> List[numpy.ndarray]:
    """Divide 'points' into groups using flood fill algorithm.

    Arguments:
    points -- points to be divided into groups, nx2 numpy.ndarray
    group_centers -- center points of to-be-created groups, mx2 numpy.ndarray
    range_limit -- maximum distance to the center, float, default 0 (disabled)
    **overflown -- arguments not caught by previous parts

    Returns:
    groups -- list of grouped points, m-list of x2 numpy.ndarrays
    """
    global MAP, MAP_ORIGIN, MAP_GRID

    # Update parameters
    P.updateAll(overflown, reset = False)

    _groups = [ [] for _i in range(len(group_centers)) ]

    _map = MAP.copy()
    _map[_map == 100] = 255

    for _i, _c in enumerate(pointsToMap(group_centers)):
        _map[tuple(_c)] = _i

    queue = pointsToMap(group_centers).tolist()

    while len(queue) > 0:
        cell = queue.pop(0)

        for _a in [-1, 0, 1]:
            for _b in [-1, 0, 1]:
                if _a == 0 and _b == 0:
                    continue

                # Try does catch larger values but not negative
                if cell[0] + _a < 0 or cell[1] + _b < 0:
                    continue

                try:
                    _cell = _map[cell[0] + _a, cell[1] + _b]
                except:
                    continue

                if _cell == 255:
                    _map[cell[0] + _a, cell[1] + _b] = _map[tuple(cell)]
                    queue.append((cell[0] + _a, cell[1] + _b))

    for p in points:
        _i = _map[tuple(pointToMap(p))]

        if _i != 255:
            _groups[ _i ].append( p )


    groups = [ numpy.asarray( g ) for g in _groups ]

    if P.getValue("range_limit") <= 0:
        return groups

    else:
        return [
            x[numpy.sqrt( numpy.sum( numpy.power( numpy.subtract(x[:, :2], group_centers[ix][:2]), 2), axis = 1 ) ) < P.getValue("range_limit")]
            for ix, x in enumerate(groups)
        ]
