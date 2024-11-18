#!/usr/bin/env python3.6
# main.py
"""Imitate selector by returning a fixed set of points.

This is used just for simple testing / enforcing a candidate.
"""
######################
# Imports & Globals
######################

import numpy

from ng_trajectory.parameter import ParameterList

from typing import (
    Any,
    Dict,
    List,
    Optional,
)


# Global variables
POINTS = 0


# Parameters
P = ParameterList()
P.createAdd("points", "", list, "Points to be returned upon calling 'select', list of points", "init")


######################
# Functions
######################

def init(
        points: List[List[float]] = "",
        **kwargs) -> Optional[Dict[str, Any]]:
    """Initialize selector.

    Arguments:
    points -- points to be returned upon calling select,
              list of nx2 points, default ""
    """
    global POINTS

    POINTS = numpy.asarray(points)


def select(points: numpy.ndarray, remain: int, **overflown) -> numpy.ndarray:
    """Return fixed points.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int
    **overflown -- arguments not caught by previous parts

    Returns:
    rpoints -- list of points, mx2 numpy.ndarray

    Note: Remain is ignored.
    """
    return POINTS[:, :2]
