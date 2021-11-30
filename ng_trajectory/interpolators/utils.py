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


def pointsDistance(points: numpy.ndarray) -> numpy.ndarray:
    """Computes distances points in array of points in 2D.

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
                            numpy.roll(points[:, :2], 1, axis=0),
                            points[:, :2]
                        ),
                    2),
                axis=1)
            )


######################
# Utilities (Trajectory)
######################

def trajectorySort(points: numpy.ndarray, verify_sort: bool = False) -> numpy.ndarray:
    """Sorts a trajectory (array of points) to be "in order".

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
                        numpy.min( numpy.subtract(spoints[1:], spoints[:-1]) )
                    ) for u in
                            [
                                numpy.unique( spoints[:, d] ) for d in range(spoints.shape[1])
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


            # In case that there is only one, there is probably some larger problem.
            # TODO: Investigate whether this can happen.
            # FIXME: Raise an Exception?
            if len(_outliers) == 1:
                print ("trajectorySort: Only one large jump in the trajectory found.")
                print ("trajectorySort: points = %s" % spoints.tolist())
                print ("trajectorySort: dists = %s" % _dists.tolist())
                print ("trajectorySort: Continuing without dealing with outliers.")
                break

            # Continue only if outliers found
            elif len(_outliers) > 0:
                # Outlier indices
                _oi = numpy.argwhere(_dists > numpy.sqrt(2) * _grid)

                # Find group sizes
                _groups = (
                    [ (_oi[_i]+1, _oi[_i+1], _oi[_i+1] - _oi[_i]) for _i in range(len(_oi) - 1) ] + [ (_oi[-1]+1, _oi[0], _oi[0] + len(spoints) - _oi[-1]) ]
                ) # start id, end id (both inclusive), size of the group

                # Sort the groups in order to find the largest group
                _groups = sorted(_groups, key = lambda x: x[-1], reverse = True)

                # TODO: Do a reconnection here.
                # Delete outlier
                spoints = numpy.delete(spoints, ( _groups[0][1] ) % len(spoints), axis = 0)

            else:
                break

    return spoints


def trajectoryReduce(points: numpy.ndarray, remain: int) -> numpy.ndarray:
    """Selects 'remain' points from 'points' equally.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray
    """
    return points[numpy.linspace(0, len(points)-1, remain, dtype=numpy.int, endpoint=False), :]
