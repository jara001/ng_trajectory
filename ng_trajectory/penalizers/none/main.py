#!/usr/bin/env python3.6
# main.py
"""Dummy penalizer.

Passes any candidate.
"""
######################
# Imports & Globals
######################

import numpy


# Global variables
INVALID_POINTS = []


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize penalizer."""
    pass


def penalize(
        points: numpy.ndarray,
        valid_points: numpy.ndarray,
        grid: float,
        penalty: float = 100,
        **overflown) -> float:
    """Get a penalty for the candidate solution.

    Penalty is based on the number of incorrectly placed points.

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

    INVALID_POINTS.clear()

    return 0.0
