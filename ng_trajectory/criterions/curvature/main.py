#!/usr/bin/env python3.6
# main.py
"""Compute criterion using curvature of the path.
"""
######################
# Imports & Globals
######################

import numpy


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize criterion."""
    pass


def compute(points: numpy.ndarray, **overflown) -> float:
    """Compute curvature criterion.

    Arguments:
    points -- points of a trajectory with curvature, nx3 numpy.ndarray
    **overflown -- arguments not caught by previous parts

    Returns:
    k -- curvature criterion of the trajectory, [m^-2], float
         minimization criterion
    """
    return numpy.sum(
        numpy.power(points[:, -1], 2)
    )
