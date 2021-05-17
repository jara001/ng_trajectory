#!/usr/bin/env python3.6
# transform.py
"""Braghin's transform and transform utilities.
"""
######################
# Imports & Globals
######################

import numpy, math

# Typing support for other types
from typing import List, Tuple, Callable

# Utils functions
from ng_trajectory.interpolators.utils import trajectoryReduce, trajectorySort, pointDistance
from .interpolate import pointsInterpolate


######################
# Utilities (Perpendiculars)
######################

def trajectoryClosest(point: numpy.ndarray, ipoints: numpy.ndarray) -> int:
    """Find an index of the point from a path that is the closest.

    Arguments:
    point -- point where the tangent should be situated, 1x2 numpy.ndarray
    ipoints -- interpolated path, nx(>=2) numpy.ndarray

    Returns:
    index -- index of the closest point
    """

    # https://stackoverflow.com/questions/25823608/find-matching-rows-in-2-dimensional-numpy-array
    indices = numpy.where((ipoints[:, 0] == point[0]) & (ipoints[:, 1] == point[1]))[0]

    # Find closest point when an exact match is not found
    if len(indices) < 1:
        return int(
            numpy.argmin(
                numpy.sum(
                    numpy.power(
                        numpy.abs(
                            numpy.subtract(
                                ipoints, point
                            )
                        ), 2
                    ), axis = 1
                )
            )
        )
    else:
        return int(indices[0])


def trajectoryPerpendicular(point: numpy.ndarray, ipoints: numpy.ndarray, dipoints: numpy.ndarray) -> Callable[[float], float]:
    """Find a perpendicular line at specified point on the interpolated path.

    Arguments:
    point -- point where the perpendicular line should be situated, 1x2 numpy.ndarray
    ipoints -- interpolated path, nx(>=2) numpy.ndarray
    dipoints -- first order derviation of the interpolated path, nx(>=2) numpy.ndarray

    Returns:
    line -- function for the line, 1-float/float lambda

    Note:
        a * x + b * y + c = 0; y = k * x + q
        dipoints[_, 0] = dx/dt
        dipoints[_, 1] = dy/dt
        k = dy / dx = ^^.

        Normal vector (not directional)
        a = dy
        b = -dx

        Moved line
        a * (x - px) + b * (y - py) + c = 0
        dy * (x - px) - dx * (y - py) + c = 0
        dy * (x - px) + dx * py + c = dx * y
        k * (x - px) + py + c / dx = y

    TODO: Make them the same length.
    """

    index = trajectoryClosest(point, ipoints)

    try:
        # https://www.mytutor.co.uk/answers/7280/A-Level/Maths/How-do-you-find-the-gradient-of-a-parametric-equation-at-a-certain-point/
        # http://www.szscb.cz/wp-content/uploads/2016/11/vy_32_inovace_ma4-ja-13.pdf
        magnitude = - dipoints[index, 0] / dipoints[index, 1]
    except ZeroDivisionError:
        # Horizontala
        sys.stderr.write("trajectoryPerpendicular(%s at index %d) is horizontal\n" % (str(point.tolist()), index))
        magnitude = - dipoints[index, 0] / 0.0000001

    return lambda x: magnitude * ( x - point[0] ) + point[1]


def lineEndpointBorderObtain(line: Callable[[float], float], center: numpy.ndarray, endpoint_distance: float, endpoint_accuracy: float, positive_dir: bool = True, track: numpy.ndarray = None) -> numpy.ndarray:
    """Obtain a furthest endpoint in the valid area.

    Arguments:
    line -- function for the line, 1-float/float lambda
    center -- reference point of the line, 1x2 numpy.ndarray
    endpoint_distance -- starting distance from the center, float
    endpoint_accuracy -- accuracy of the center-endpoint distance, float
    positive_dir -- which endpoint to select, True/False of the direction, bool
    track -- valid area for the line, nx2 numpy.ndarray
                    when not set, global VALID_POINTS is used

    Returns:
    endpoint -- coordinates (x, y) of the endpoint, 1x2 numpy.ndarray

    Note: You will get one of the endpoints. Currently, it is random.
    """
    global VALID_POINTS

    _valid_points = track if track is not None else VALID_POINTS

    # Obtain dfactor
    dfactor = 1 / pointDistance([0, line(0)], [1, line(1)])
    dist = 0# dfactor * endpoint_distance * (1 if positive_dir else -1)
    step = dfactor * endpoint_distance * (1 if positive_dir else -1)
    center2 = center
    point = numpy.zeros((1, 2))

    while abs(step) > (endpoint_accuracy * dfactor):
        point = numpy.asarray([[center[0] + dist + step, line(center[0] + dist + step)]])

        if not numpy.any(numpy.all(numpy.abs( numpy.subtract(_valid_points, point) ) < [ math.sqrt(2)*endpoint_accuracy, math.sqrt(2)*endpoint_accuracy ], axis = 1)):
            step /= 2
        else:
            dist += step

    return point



######################
# Braghin's cuts
######################

def create(track: numpy.ndarray, group_centerline: numpy.ndarray, group_centers: numpy.ndarray) -> List[numpy.ndarray]:
    """Create Braghin's transformation.

    Arguments:
    track -- valid area of the track, nx2 numpy.ndarray
    group_centerline -- line where the group centers lie, px2 numpy.ndarray
    group_centers -- points selected from the line, mx2 numpy.ndarray

    Returns:
    cuts -- endpoints of cuts on the track, m-list of 2x2 numpy.ndarray

    TODO: Interpolate only once and use the reduction as a direct index.
    """

    i, i1, i2 = pointsInterpolate(trajectoryReduce(group_centerline, int(len(group_centerline)/3)), 440)

    cuts = []

    for point in group_centers:

        point = point[:2]

        p = trajectoryPerpendicular(point, i, i1)

        point1 = lineEndpointBorderObtain(p, point, 0.2, 0.02, True, track)
        point2 = lineEndpointBorderObtain(p, point, 0.2, 0.02, False, track)

        perpendicular = numpy.vstack((point1, point2))

        cuts.append(perpendicular)

    return cuts


def transform(points: numpy.ndarray, cuts: List[Tuple[numpy.ndarray]]) -> numpy.ndarray:
    """Transform points from 1D transformed to real coordinates.

    Arguments:
    points -- points to transform (from transformed coordinates to real coordinates),
        nx1 numpy.ndarray
    cuts -- limit border points of cuts, n-list of 2x2 numpy.ndarray

    Returns:
    tpoints -- transformed points, nx2 numpy.ndarray
    """

    return numpy.asarray(
        [
            ( cuts[i][1] - cuts[i][0] ) * points[i] + cuts[i][0]
            for i in range(len(points))
        ]
    )

