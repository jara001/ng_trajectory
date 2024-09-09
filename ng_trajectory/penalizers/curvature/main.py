#!/usr/bin/env python3.6
# main.py
"""Penalize the incorrect solution by curvature.

This penalizer finds all points that exceed the maximum
admitted curvature / are outside of the track.
"""
######################
# Imports & Globals
######################

import numpy

from ng_trajectory.parameter import ParameterList
from ng_trajectory.penalizers.utils import eInvalidPoints


# Global variables
INVALID_POINTS = []


# Parameters
P = ParameterList()
P.createAdd("k_max", 1.5, float, "Maximum allowed curvature in abs [m^-1]", "")


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize penalizer."""
    # Update parameters
    P.updateAll(kwargs)


def penalize(
        points: numpy.ndarray,
        valid_points: numpy.ndarray,
        grid: float,
        penalty: float = 100,
        **overflown) -> float:
    """Get a penalty for the candidate solution.

    Penalty is based on the number of incorrectly placed points
    and path curvature.

    Arguments:
    points -- points to be checked, nx(>=2) numpy.ndarray
    valid_points -- valid area of the track, mx2 numpy.ndarray
    grid -- when set, use this value as a grid size, otherwise it is computed,
            float
    penalty -- constant used for increasing the penalty criterion,
               float, default 100
    **overflown -- arguments not caught by previous parts

    Returns:
    rpenalty -- value of the penalty, 0 means no penalty, float
    """
    global INVALID_POINTS

    # Update parameters
    P.updateAll(overflown, reset = False)

    _k_max = P.getValue("k_max")


    invalid = 0
    INVALID_POINTS.clear()

    for _, _p in eInvalidPoints(points):
        INVALID_POINTS.append(_p)
        invalid += 1

    if invalid == 0:
        invalid = numpy.add(
            numpy.sum(
                points[points[:, 2] > _k_max, 2]
            ),
            -numpy.sum(
                points[points[:, 2] < -_k_max, 2]
            )
        ) / 100

        INVALID_POINTS += points[
            (points[:, 2] > _k_max) | (points[:, 2] < -_k_max), :
        ].tolist()
        # print (points[(points[:, 2] > _k_max) | (points[:, 2] < -_k_max), 2])

    # print(points[(points[:, 2] > _k_max) | (points[:, 2] < -_k_max), 2])

    return invalid * penalty * 10
