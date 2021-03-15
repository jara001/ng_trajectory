#!/usr/bin/env python3.6
# main.py
"""Segmentate track using euclidean distance.
"""
######################
# Imports & Globals
######################

import numpy

from ng_trajectory.interpolators.utils import pointDistance

from typing import List


######################
# Functions
######################

def init(track: numpy.ndarray, **kwargs) -> None:
    """Initialize segmentator."""
    pass


def segmentate(points: numpy.ndarray, group_centers: numpy.ndarray, range_limit: float = 0, **overflown) -> List[numpy.ndarray]:
    """Divide 'points' into groups by their distance to 'group_centers'.

    Arguments:
    points -- points to be divided into groups, nx2 numpy.ndarray
    group_centers -- center points of to-be-created groups, mx2 numpy.ndarray
    range_limit -- maximum distance to the center, float, default 0 (disabled)
    **overflown -- arguments not caught by previous parts

    Returns:
    groups -- list of grouped points, m-list of x2 numpy.ndarrays
    """
 
    _groups = [ [] for _i in range(len(group_centers)) ]

    for p in points:
        distance = 100000
        index = 0

        for _i, _c in enumerate(group_centers):
            _dist = pointDistance(p, _c)

            if ( _dist < distance ):
                distance = _dist
                index = _i


        _groups[index].append( p )


    groups = [ numpy.asarray( g ) for g in _groups ]

    if range_limit <= 0:
        return groups

    else:
        return [
            x[numpy.sqrt( numpy.sum( numpy.power( numpy.subtract(x[:, :2], group_centers[ix][:2]), 2), axis = 1 ) ) < range_limit]
            for ix, x in enumerate(groups)
        ]
