#!/usr/bin/env python3.6
# main.py
"""Select points from a path uniformly with respect to time.
"""
######################
# Imports & Globals
######################

import numpy

# Profile criterion
from ng_trajectory.criterions.profile.main import P as P_profile
from ng_trajectory.criterions.profile import profiler


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
for _, param in P_profile.iterate():
    P.add(param)


######################
# Functions
######################

def init(rotate: float = 0,
        **kwargs) -> None:
    """Initialize selector."""

    P.updateAll(**kwargs)

    profiler.parametersSet(**{param.name: param.value for _, param in P.iterate()})


def select(points: numpy.ndarray, remain: int, **overflown) -> numpy.ndarray:
    """Select points from the path uniformly with respect to time.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int
    **overflown -- arguments not caught by previous parts

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray

    Note: When 'remain' is negative the functions raises an Exception.
    """
    pass
