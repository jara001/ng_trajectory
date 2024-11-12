#!/usr/bin/env python3.6
# main.py
"""Compute criterion using curvature of the path.

Optimize using sum( (k_i)^2 ).
"""
######################
# Imports & Globals
######################

import numpy

from typing import (
    Any,
    Dict,
    Optional,
)


######################
# Functions
######################

def init(**kwargs) -> Optional[Dict[str, Any]]:
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
