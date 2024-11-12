#!/usr/bin/env python3.6
# main.py
"""Segmentate track using flood fill algorithm.

This segmentator splits the track into segments by flood
fill algorithm from the centers.
"""
######################
# Imports & Globals
######################

import numpy
import sys

# PointDistance
from ng_trajectory.interpolators.utils import pointDistance
from ng_trajectory.log import print0, printv
from ng_trajectory.parameter import ParameterList
from ng_trajectory.segmentators.utils import (
    mapCreate,
    pointToMap,
    pointsToMap
)

# Parallel execution of flood fill
from concurrent import futures

from typing import (
    Any,
    Dict,
    List,
    Optional,
)


# Global variables
MAP = None
MAP_ORIGIN = None
MAP_GRID = None


# Parameters
P = ParameterList()
P.createAdd("hold_map", False, bool, "When true, the map is created only once.", "init")
P.createAdd("range_limit", 0, float, "Maximum distance from the center of the segment. 0 disables this.", "")
P.createAdd("reserve_width", False, bool, "When true, the segments are reserved a path towards both walls.", "")
P.createAdd("reserve_selected", [], list, "IDs of segments that should use the reservation method, when empty, use all.", "")
P.createAdd("reserve_distance", 2.0, float, "Distance from the line segment that is reserved to the segment.", "")
P.createAdd("plot_flood", False, bool, "Whether the flooded areas should be plotted.", "")
P.createAdd("parallel_flood", 0, int, "Number of threads used for flood fill (0 = sequential execution).", "")


######################
# Utilities
######################

def segmentDistance(p: List[float], a: List[float], b: List[float]) -> float:
    """Compute the distance of a point from a segment.

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
    distance_to_seg = (
        abs(
            (b[0] - a[0]) * (a[1] - p[1]) - (a[0] - p[0]) * (b[1] - a[1])
        )
        / numpy.sqrt(
            numpy.power(
                b[0] - a[0],
                2
            )
            + numpy.power(
                b[1] - a[1],
                2
            )
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

    print0("segmentDistance: Unexpected situation.")
    print0("segmentDistance: point = %s, a = %s, b = %s" % (p, a, b))
    print0(
        "segmentDistance: angle_ap = %f, angle_bp = %f"
        % (angle_ap, angle_bp)
    )
    print0(
        "segmentDistance: d_to_seg = %f, d_ap = %f, d_bp = %f"
        % (distance_to_seg, pointDistance(a, p), pointDistance(b, p))
    )
    print0(
        "segmentDistance: V_ap = %s, V_ab = %s, V_ap*V_ab = %s"
        % (unit_ap, unit_ab, numpy.dot(unit_ap, unit_ab))
    )
    raise ValueError(
        "Unexpected situation at 'segmentDistance'. "
        "Read the output for values of the variables."
    )


######################
# Functions
######################

def init(track: numpy.ndarray, **kwargs) -> Optional[Dict[str, Any]]:
    """Initialize segmentator by creating map."""
    global MAP, MAP_ORIGIN, MAP_GRID

    # Update parameters
    P.updateAll(kwargs)

    if MAP is None or not P.getValue("hold_map"):
        MAP, MAP_ORIGIN, MAP_GRID = mapCreate(track)


def segmentate(
        points: numpy.ndarray,
        group_centers: numpy.ndarray,
        **overflown) -> List[numpy.ndarray]:
    """Divide 'points' into groups using flood fill algorithm.

    Arguments:
    points -- points to be divided into groups, nx2 numpy.ndarray
    group_centers -- center points of to-be-created groups, mx2 numpy.ndarray
    range_limit -- maximum distance to the center, float, default 0 (disabled)
    **overflown -- arguments not caught by previous parts

    Returns:
    groups -- list of grouped points, m-list of x2 numpy.ndarrays
    """
    global MAP, MAP_ORIGIN, MAP_GRID, MAP_LAST

    # Update parameters
    P.updateAll(overflown, reset = False)

    _groups = [[] for _i in range(len(group_centers))]

    if not P.getValue("reserve_width"):
        _map = MAP.copy()
        _map[_map == 100] = 255

        for _i, (_cx, _cy) in enumerate(pointsToMap(group_centers)):
            _map[_cx, _cy] = _i

    else:  # if reserve_width
        printv("Computing reserved zones...")

        # Use enlarged map (required for walls)
        _map = numpy.zeros(
            (MAP.shape[0] + 2, MAP.shape[1] + 2),
            dtype=numpy.uint8
        )
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
                cell_x, cell_y = queue.pop(0)

                for _a, _b in [(-1, -1), (-1, 0), (-1, 1),
                               (+0, -1),          (+0, 1),   # noqa: E241
                               (+1, -1), (+1, 0), (+1, 1)]:

                    # Try does catch larger values but not negative
                    if cell_x + _a < 0 or cell_y + _b < 0:
                        continue

                    try:
                        _cell = _map[cell_x + _a, cell_y + _b]
                    except IndexError:
                        continue

                    if _cell == 0:
                        _map[cell_x + _a, cell_y + _b] = color
                        queue.append((cell_x + _a, cell_y + _b))

            color = color + 1
            walls = numpy.where(_map == 0)

        # TODO: Ensure that we have only two walls. Otherwise merge them.
        printv("\tDetected walls: %d" % (color - 200))


        for _i, _c in enumerate(pointsToMap(group_centers)):
            printv("\tSegment %d/%d:" % (_i, len(group_centers)))
            _map[_c[0], _c[1]] = _i

            if (
                len(P.getValue("reserve_selected")) > 0
                and _i not in P.getValue("reserve_selected")
            ):
                continue

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
                        )
                        + numpy.power(
                            _wy - _c[1],
                            2
                        )
                    )

                    if _distance < distance:
                        distance = _distance
                        closest = (_wx, _wy)

                sys.stdout.write(
                    "\t\tWall %i... %03.2f%%"
                    % (_wall_index, 0.0)
                )
                # print ("Closest to this:", closest, distance)

                # Create link to the wall; color all points
                # that are in proximity of the line
                valids = numpy.where(_map == 255)
                valids_length = len(valids[0])

                for _vi, (_vx, _vy) in enumerate(zip(valids[0], valids[1])):
                    _distance = segmentDistance((_vx, _vy), _c, closest)

                    if _distance < P.getValue("reserve_distance"):
                        _map[_vx, _vy] = 100 + _i  # + _wall_index

                    if _vi % 1000 == 0:
                        sys.stdout.write(
                            "\r\t\tWall %i... %03.2f%%"
                            % (_wall_index, 100.0 * _vi / valids_length)
                        )

                sys.stdout.write(
                    "\r\t\tWall %i... %03.2f%%\n"
                    % (_wall_index, 100.0)
                )


    queue = pointsToMap(group_centers).tolist()

    # if create_borderlines:
    #     borderlines_map = { i: {} for i in range(len(group_centers)) }

    if P.getValue("parallel_flood") < 1:
        while len(queue) > 0:
            cell_x, cell_y = queue.pop(0)

            for _a, _b in [(-1, -1), (-1, 0), (-1, 1),
                           (+0, -1),          (+0, 1),   # noqa: E241
                           (+1, -1), (+1, 0), (+1, 1)]:

                # Try does catch larger values but not negative
                if cell_x + _a < 0 or cell_y + _b < 0:
                    continue

                try:
                    _cell = _map[cell_x + _a, cell_y + _b]
                except IndexError:
                    continue

                # Color if its empty or reserved for this group
                if _cell == 255 or (_cell == 100 + _map[cell_x, cell_y]):
                    _map[cell_x + _a, cell_y + _b] = _map[cell_x, cell_y]
                    queue.append((cell_x + _a, cell_y + _b))
                # Save some steps by continuing sooner when
                # borderlines are not created
                # elif not create_borderlines:
                #     continue
                # Store it if its another segment
                # elif _cell < 100 and _cell != _map[tuple(cell)]:
                #  Otherwise we also get "neighbours to itself".
                #     borderlines_map[
                #         _map[tuple(cell)]
                #     ][(cell[0] + _a, cell[1] + _b)] = _cell
                # ... also in case that we are using reservations
                # elif _cell < 200 and _cell != _map[tuple(cell)]:
                #     borderlines_map[
                #         _map[tuple(cell)]
                #     ][(cell[0] + _a, cell[1] + _b)] = _cell - 100

        # Save last map
        MAP_LAST = _map

    else:
        # Save last map
        MAP_LAST = _map

        with futures.ProcessPoolExecutor(
            max_workers = P.getValue("parallel_flood")
        ) as executor:
            queues = queue
            colors = [
                _map[queues[i][0], queues[i][1]]
                for i in range(len(group_centers))
            ]
            _run = True

            while _run:
                _run = False
                for i, new_queue in enumerate(
                    executor.map(expand_fill, queues)
                ):

                    if len(new_queue) == 0:
                        continue

                    _run = True
                    _color = colors[i]
                    queues[i] = []

                    for cell_x, cell_y in new_queue:
                        if MAP_LAST[cell_x, cell_y] == 255:
                            MAP_LAST[cell_x, cell_y] = _color
                            queues[i].append((cell_x, cell_y))

    # if create_borderlines:
    #     borderlines_real = {
    #         i: {
    #             j: [] for j in range(len(group_centers))
    #         } for i in range(len(group_centers))
    #     }

    for p in points:
        _px, _py = pointToMap(p)
        _i = MAP_LAST[_px, _py]

        # Group only taken points
        if _i != 255 and _i < 100:
            _groups[_i].append(p)

            # if create_borderlines:
            #    # Also add it to borderlines if on edge
            #    # Note: This does only "own" borderlines.
            #    #if tuple(pointToMap(p)) in borderlines_map[_i]:
            #    #    borderlines_real[_i][
            #    #        borderlines_map[_i][tuple(pointToMap(p))]
            #    #    ].append(p)
            #
            #    # Create all combinations of borderlines in real coordinates
            #    for i in range(len(group_centers)):
            #        if tuple(pointToMap(p)) in borderlines_map[i]:
            #            borderlines_real[i][borderlines_map[i][tuple(pointToMap(p))]].append(p)


    # Finally process the borderlines
    # if create_borderlines:
    #     borderlines = {
    #         i: {
    #             j: numpy.asarray(borderlines_real[i][j])
    #             for j in range(len(group_centers))
    #             if len(borderlines_real[i][j]) > 0
    #         }
    #         for i in range(len(group_centers))
    #     }


    # TODO: Investigate whether 'if len(g) > 1' is required here.
    # Note: 'len(g) > 0' is required when there is a group with zero elements.
    # This can happen when the selected group center is inside of a wall.
    # Note: However, this condition was moved to the end of the function,
    # as removing the group here caused mismatch in group indices.
    # ...
    # Instead, we ensure in the beginning that the center is
    # inside of reachable area by moving it there.
    # FIXME: Is it really the correct way to do it?
    # FIXME: Implement any solution and remove the condition
    #        from sections below.
    groups = [numpy.asarray(g) for g in _groups]

    if P.getValue("plot_flood"):
        import ng_trajectory.plot as ngplot
        ngplot.figureCreate()
        ngplot.axisEqual()

        for g in range(len(groups)):
            if len(groups[g]) > 0:
                ngplot.pointsScatter(groups[g])

        ngplot.figureShow()

        ngplot.figureCreate()

    if P.getValue("range_limit") <= 0:
        return [g for g in groups if len(g) > 0]
        # return groups if not create_borderlines else (groups, borderlines)

    else:
        return [
            x[
                numpy.sqrt(
                    numpy.sum(
                        numpy.power(
                            numpy.subtract(x[:, :2], group_centers[ix][:2]),
                            2
                        ),
                        axis = 1
                    )
                ) < P.getValue("range_limit")
            ]
            for ix, x in enumerate(groups) if len(x) > 0
        ]


def expand_fill(queue):
    """Perform a step of flood fill algorithm.

    This does not do any modification of the map,
    only keeps track of points that should be changed.
    Coloring is done by synchronization process.
    """
    global MAP_LAST

    n_queue = []

    if len(queue) == 2 and type(queue[0]) == int:
        queue = [queue]

    while len(queue) > 0:
        cell_x, cell_y = queue.pop(0)

        for _a, _b in [(-1, -1), (-1, 0), (-1, 1),
                       (+0, -1),          (+0, 1),   # noqa: E241
                       (+1, -1), (+1, 0), (+1, 1)]:

            # Try does catch larger values but not negative
            if cell_x + _a < 0 or cell_y + _b < 0:
                continue

            try:
                _cell = MAP_LAST[cell_x + _a, cell_y + _b]
            except IndexError:
                continue

            # Color if its empty or reserved for this group
            if _cell == 255 or (_cell == 100 + MAP_LAST[cell_x, cell_y]):
                # _map[cell_x + _a, cell_y + _b] = MAP_LAST[cell_x, cell_y]
                n_queue.append((cell_x + _a, cell_y + _b))

    return n_queue
