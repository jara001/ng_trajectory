#!/usr/bin/env python3.6
# main.py
"""Compute criterion using the length of the path.
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
    """Compute length of the trajectory.

    Arguments:
    points -- points of a trajectory with curvature, nx3 numpy.ndarray
    **overflown -- arguments not caught by previous parts

    Returns:
    l -- length of the trajectory, [m], float
         minimization criterion
    """
    return float(
        numpy.sum(
            numpy.sqrt(
                numpy.sum(
                    numpy.power(
                        numpy.subtract(
                            numpy.roll(points[:, :2], 1, axis=0),
                            points[:, :2]
                        ),
                    2),
                axis=1)
            )
        )
    )
