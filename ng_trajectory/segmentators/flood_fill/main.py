#!/usr/bin/env python3.6
# main.py
"""Segmentate track using flood fill algorithm.
"""
######################
# Imports & Globals
######################

import numpy, sys

from ng_trajectory.segmentators.utils import *

# PointDistance
from ng_trajectory.interpolators.utils import pointDistance

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
P.createAdd("reserve_width", False, bool, "When true, the segments are reserved a path towards both walls.", "")


######################
# Utilities
######################

def segmentDistance(p: List[float], a: List[float], b: List[float]) -> float:
    """Computes the distance of a point from a segment.

    Arguments:
    p -- point that is being inspected, (>=2)-list like structure of floats
    a -- one endpoint of the segment, (>=2)-list like structure of floats
    b -- other endpoint of the segment, (>=2)-list like structure of floats

    Returns:
    d -- distance from the segment, float

    Source:
    https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    https://www.kite.com/python/answers/how-to-get-the-angle-between-two-vectors-in-python
    ... and edited

    Note: Throws an ValueError exception when an unexpected situation occurs.
    """
    line_length = pointDistance(a, b)

    distance_to_seg = abs((b[0] - a[0]) * (a[1] - p[1]) - (a[0] - p[0]) * (b[1] - a[1])) / \
            numpy.sqrt(
                numpy.power(
                    b[0] - a[0],
                    2
                ) +
                numpy.power(
                    b[1] - a[1],
                    2
                )
            )

    unit_ap = [p[0] - a[0], p[1] - a[1]]
    unit_ab = [b[0] - a[0], b[1] - a[1]]

    unit_ap = unit_ap / numpy.linalg.norm(unit_ap)
    unit_ab = unit_ab / numpy.linalg.norm(unit_ab)
    dot_ap = numpy.dot(unit_ap, unit_ab)
    # Fix overflowing the float
    dot_ap = min(1.0, max(-1.0, dot_ap))
    angle_ap = numpy.degrees(numpy.arccos(dot_ap))

    unit_bp = [p[0] - b[0], p[1] - b[1]]
    unit_ba = [a[0] - b[0], a[1] - b[1]]

    unit_bp = unit_bp / numpy.linalg.norm(unit_bp)
    unit_ba = unit_ba / numpy.linalg.norm(unit_ba)
    dot_bp = numpy.dot(unit_bp, unit_ba)
    dot_bp = min(1.0, max(-1.0, dot_bp))
    angle_bp = numpy.degrees(numpy.arccos(dot_bp))

    if 0.0 <= angle_ap <= 90.0 and 0.0 <= angle_bp <= 90.0:
        return distance_to_seg

    elif angle_ap > 90.0:
        return pointDistance(a, p)

    elif angle_bp > 90.0:
        return pointDistance(b, p)

    print ("segmentDistance: Unexpected situation.")
    print ("segmentDistance: point = %s, a = %s, b = %s" % (p, a, b))
    print ("segmentDistance: angle_ap = %f, angle_bp = %f" % (angle_ap, angle_bp))
    print ("segmentDistance: d_to_seg = %f, d_ap = %f, d_bp = %f" % (distance_to_seg, pointDistance(a, p), pointDistance(b, p)))
    print ("segmentDistance: V_ap = %s, V_ab = %s, V_ap*V_ab = %s" % (unit_ap, unit_ab, numpy.dot(unit_ap, unit_ab)))
    raise ValueError("Unexpected situation at 'segmentDistance'. Read the output for values of the variables.")


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

    if not P.getValue("reserve_width"):
        _map = MAP.copy()
        _map[_map == 100] = 255

        for _i, _c in enumerate(pointsToMap(group_centers)):
            _map[tuple(_c)] = _i

    else: # if reserve_width
        print ("Computing reserved zones...")

        # Use enlarged map (required for walls)
        _map = numpy.zeros((MAP.shape[0] + 2, MAP.shape[1] + 2), dtype=numpy.uint8)
        _map[1:-1, 1:-1] = MAP.copy()
        _map[_map == 100] = 255


        # Detect walls and color them
        color = 200
        # Find an occurence of wall
        walls = numpy.where(_map == 0)

        # Color them
        while len(walls[0]) > 0:
            queue = [(walls[0][0], walls[1][0])]

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

                        if _cell == 0:
                            _map[cell[0] + _a, cell[1] + _b] = color
                            queue.append((cell[0] + _a, cell[1] + _b))

            color = color + 1
            walls = numpy.where(_map == 0)

        # TODO: Ensure that we have only two walls. Otherwise merge them.
        print ("\tDetected walls: %d" % (color - 200))


        for _i, _c in enumerate(pointsToMap(group_centers)):
            print ("\tSegment %d/%d:" % (_i, len(group_centers)))
            _map[tuple(_c)] = _i

            # Create "links" to the nearest of both walls

            # Find closest points
            for _wall_index in range(2):
                distance = 100000
                closest = None

                walls = numpy.where(_map == (200 + _wall_index))

                for _wx, _wy in zip(walls[0], walls[1]):
                    _distance = numpy.sqrt(
                        numpy.power(
                            _wx - _c[0],
                            2
                        ) +
                        numpy.power(
                            _wy - _c[1],
                            2
                        )
                    )

                    if _distance < distance:
                        distance = _distance
                        closest = (_wx, _wy)

                sys.stdout.write("\t\tWall %i... %03.2f%%" % (_wall_index, 0.0))
                #print ("Closest to this:", closest, distance)

                # Create link to the wall; color all points that are in proximity of the line
                valids = numpy.where(_map == 255)
                valids_length = len(valids[0])

                for _vi, (_vx, _vy) in enumerate(zip(valids[0], valids[1])):
                    _distance = segmentDistance((_vx, _vy), _c, closest)

                    if _distance < 2:
                        _map[_vx, _vy] = 100 + _i #+ _wall_index

                    if _vi % 1000 == 0:
                        sys.stdout.write("\r\t\tWall %i... %03.2f%%" % (_wall_index, 100.0 * _vi / valids_length))

                sys.stdout.write("\r\t\tWall %i... %03.2f%%\n" % (_wall_index, 100.0))


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

                # Color if its empty or reserved for this group
                if _cell == 255 or (_cell == 100 + _map[tuple(cell)]):
                    _map[cell[0] + _a, cell[1] + _b] = _map[tuple(cell)]
                    queue.append((cell[0] + _a, cell[1] + _b))

    for p in points:
        _i = _map[tuple(pointToMap(p))]

        # Group only taken points
        if _i != 255 and _i < 100:
            _groups[ _i ].append( p )

    # TODO: Investigate whether 'if len(g) > 1' is required here.
    groups = [ numpy.asarray( g ) for g in _groups ]

    if P.getValue("range_limit") <= 0:
        return groups

    else:
        return [
            x[numpy.sqrt( numpy.sum( numpy.power( numpy.subtract(x[:, :2], group_centers[ix][:2]), 2), axis = 1 ) ) < P.getValue("range_limit")]
            for ix, x in enumerate(groups)
        ]
