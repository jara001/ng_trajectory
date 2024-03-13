#!/usr/bin/env python3.6
# utils.py3
"""Various utilities for interpolators.

Used mostly for the interpolators, but can be used
by other algorithms.
"""
######################
# Imports & Globals
######################

import math
import numpy

from ng_trajectory.log import print0


######################
# Utilities (Point)
######################

def pointDistance(a: list, b: list) -> float:
    """Compute distance between two points in 2D.

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
    return math.sqrt(sum([
        pow(b[i] - a[i], 2) for i in range(min(len(a), len(b)))
    ]))


def pointsDistance(points: numpy.ndarray) -> numpy.ndarray:
    """Compute distances points in array of points in 2D.

    Arguments:
    points -- set of points, nx(>=2) numpy.ndarray

    Returns:
    d -- set of distances, nx1 numpy.ndarray

    Note: Taken from 'length' criterion.
    """
    return numpy.sqrt(
        numpy.sum(
            numpy.power(
                numpy.subtract(
                    numpy.roll(points[:, :2], 1, axis = 0),
                    points[:, :2]
                ),
                2
            ),
            axis = 1
        )
    )


######################
# Utilities (Trajectory)
######################

def trajectorySort(
        points: numpy.ndarray,
        verify_sort: bool = False) -> numpy.ndarray:
    """Sort a trajectory (array of points) to be "in order".

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    verify_sort -- when True, the trajectory is checked for incorrect
                   sorting and outliers are moved to their appropriate places

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


    # Verify the sorting
    # Sometimes, when multiple points with same distance are present,
    # the result is not correct. This checks for points outside the
    # expected sqrt(2)*_grid distance and tries to move them to better
    # positions.
    spoints = numpy.asarray(sorted_points)

    if verify_sort:
        # Obtain grid size
        _grid = numpy.min(
            [
                numpy.abs(
                    numpy.min(
                        numpy.subtract(spoints[1:], spoints[:-1])
                    )
                ) for u in [
                    numpy.unique(spoints[:, d])
                    for d in range(spoints.shape[1])
                ]
            ]
        )

        # FIXME: We are currently deleting outliers, one by one.
        # TODO: Remake this with reconnection.
        while True:

            # Get distances between consecutive points
            _dists = pointsDistance(spoints)

            # Find outliers
            _outliers = _dists[_dists > numpy.sqrt(2) * _grid]


            # In case that there is only one, there is probably
            # some larger problem.
            # TODO: Investigate whether this can happen.
            # FIXME: Raise an Exception?
            if len(_outliers) == 1:
                print0("trajectorySort: Only one large jump "
                       "in the trajectory found.")
                print0("trajectorySort: points = %s" % spoints.tolist())
                print0("trajectorySort: dists = %s" % _dists.tolist())
                print0("trajectorySort: Continuing without "
                       "dealing with outliers.")
                break

            # Continue only if outliers found
            elif len(_outliers) > 0:
                # Outlier indices
                _oi = numpy.argwhere(_dists > numpy.sqrt(2) * _grid)

                # Find group sizes
                _groups = (
                    [
                        (_oi[_i] + 1, _oi[_i + 1], _oi[_i + 1] - _oi[_i])
                        for _i in range(len(_oi) - 1)
                    ]
                    + [
                        (_oi[-1] + 1, _oi[0], _oi[0] + len(spoints) - _oi[-1])
                    ]
                )  # start id, end id (both inclusive), size of the group

                # Sort the groups in order to find the largest group
                _groups = sorted(
                    _groups,
                    key = lambda x: x[-1],
                    reverse = True
                )

                # TODO: Do a reconnection here.
                # Delete outlier
                spoints = numpy.delete(
                    spoints,
                    (_groups[0][1]) % len(spoints),
                    axis = 0
                )

            else:
                break

    return spoints


def trajectoryReduce(
        points: numpy.ndarray,
        remain: int) -> numpy.ndarray:
    """Select 'remain' points from 'points' equally.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray
    """
    return points[
        numpy.linspace(
            0, len(points) - 1, remain, dtype = numpy.int, endpoint = False
        ), :
    ]


def trajectoryClosest(
        points: numpy.ndarray,
        reference: numpy.ndarray,
        *,
        from_left: bool = False) -> numpy.ndarray:
    """Find the closest point on the trajectory to the 'reference'.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray
    reference -- point closest to the trajectory, 1x(>=2) numpy.ndarray

    Returns:
    closest -- point on the trajectory closest to the reference,
               1x(>=2) numpy.ndarray
    """
    return points[
        trajectoryClosestIndex(points, reference, from_left = from_left), :
    ]


def trajectoryClosestIndex(
        points: numpy.ndarray,
        reference: numpy.ndarray,
        *,
        from_left: bool = False) -> int:
    """Find the index of the closest point on the trajectory to the 'reference'.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray
    reference -- point closest to the trajectory, 1x(>=2) numpy.ndarray

    Returns:
    index -- index of the point on the trajectory closest to the reference, int
    """
    _distances = numpy.subtract(points[:, :2], reference[:2])

    index = numpy.hypot(_distances[:, 0], _distances[:, 1]).argmin()

    if not from_left:
        return index

    else:
        """Obtain the closest point from the beginning of the path.

        We use law of cosines to do this.
        - B: closest point on the path
        - R: reference point


                            R
                           / \_
                       d1 |    \_ d2
                         /a      \
         -----A---------B---------C-----
                            ds

        d2^2 = d1^2 + ds^2 - 2 * d1 * ds * cos(a)

        Rewritten as:
        cos(a) = (d1^2 - d2^2 + ds^2) / (2 * d1 * ds)

        If cos(a) > 0 then the angle 'a' is in (-90°, 90°), meaning
        that R is on the side closer to C.

        Note: Isn't it easier to use just dA and dC?
        """  # noqa: W605
        d1 = numpy.hypot(
            _distances[index, 0],
            _distances[index, 1]
        )

        if d1 == 0.0:
            return index

        d2 = numpy.hypot(
            _distances[(index + 1) % len(points), 0],
            _distances[(index + 1) % len(points), 1]
        )
        ds = pointDistance(
            points[index, :2], points[(index + 1) % len(points), :2]
        )

        return (
            index if (d1**2 - d2**2 + ds**2) / (2 * d1 * ds) > 0 else index - 1
        )


def trajectoryFarthest(
        points: numpy.ndarray,
        reference: numpy.ndarray) -> numpy.ndarray:
    """Find the farthest point on the trajectory to the 'reference'.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray
    reference -- point farthest to the trajectory, 1x(>=2) numpy.ndarray

    Returns:
    farthest -- point on the trajectory farthest to the reference,
                1x(>=2) numpy.ndarray
    """
    return points[trajectoryFarthestIndex(points, reference), :]


def trajectoryFarthestIndex(
        points: numpy.ndarray,
        reference: numpy.ndarray) -> int:
    """Find the index of the farthest point on the trajectory to the 'reference'.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray
    reference -- point farthest to the trajectory, 1x(>=2) numpy.ndarray

    Returns:
    index -- index of the point on the trajectory farthest to the reference,
             int
    """
    _distances = numpy.subtract(points[:, :2], reference[:2])

    return numpy.hypot(_distances[:, 0], _distances[:, 1]).argmax()


def trajectoryRotate(
        points: numpy.ndarray,
        next_point_index: int,
        rotation: float = 0.0) -> numpy.ndarray:
    """Rotate the closed trajectory, moving the first point along it.

    Basically, we have the trajectory 'points', point A (index 0) and
    point B (index 'next_point_index'). This function moves point A
    along the trajectory towards point B.

    Rotation 0.0 keeps A on its position, leaving the trajectory intact.
    In theory, rotation 1.0 would take point A and rotate the trajectory
    in such a way, that A=B.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray
    next_point_index -- index of the second point in the trajectory, int
    rotation -- factor to rotate the trajectory, 0.0<=rotation<1.0, float

    Returns:
    rotated_points -- rotated list of points, nx(>=2) numpy.ndarray
    """
    return numpy.roll(
        points,
        -int(next_point_index * rotation),
        axis = 0
    )
