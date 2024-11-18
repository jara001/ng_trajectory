#!/usr/bin/env python3.6
# main.py
"""Randomly select points from a path based on its curvature.

Suggested and designed by Ondra Benedikt.
"""
######################
# Imports & Globals
######################

import numpy as np

from scipy.interpolate import interp1d

from ng_trajectory.parameter import ParameterList
from ng_trajectory.selectors.curvature import curve_fitting as cf

from typing import (
    Any,
    Dict,
    Optional,
)


# Parameters
P = ParameterList()
P.createAdd("interpolation_size", 100, int, "Number of points used for interpolation.", "")


######################
# Functions
######################

def init(**kwargs) -> Optional[Dict[str, Any]]:
    """Initialize selector.

    Arguments:
    **kwargs -- overflown arguments
    """
    pass


def select(
        points: np.ndarray,
        remain: int,
        **overflown) -> np.ndarray:
    """Select points from the path uniformly based on the curvature.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int
    **overflown -- arguments not caught by previous parts

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray
    """
    if remain < 0:
        # Raise an exception, as we cannot guess number of points.
        raise ValueError(
            "Negative selection is not supported by "
            "'curvature_sample' selector."
        )


    # Update parameters
    P.updateAll(overflown)


    # Repair points array
    # Sometimes the last point is the same as the first,
    # and we do not want that.
    if (points[0] == points[-1]).all():
        points = points[0:-1]


    # Interpolate points
    n_interpolation_points = int(P.getValue("interpolation_size"))
    alpha = cf.get_linspace(n_interpolation_points)

    ipoints = cf.interpolate_points(points, n_interpolation_points, 4)

    distance = np.cumsum(
        np.sqrt(
            np.sum(
                np.diff(ipoints, axis = 0)**2, axis = 1
            )
        )
    )
    distance = np.insert(distance, 0, 0) / distance[-1]


    # Compute curvature and derivatives
    K = cf.get_curvature(ipoints, n_interpolation_points)


    # Normalized curvature
    fact = np.trapz(np.abs(K), x=alpha)
    # divide by the number of samples - each can be at most 1
    vals = np.abs(K) / fact / n_interpolation_points
    # print (sum(vals))
    vals[-1] = 1 - sum(vals[:-1])  # normalize

    rnd_choice = np.random.choice(
        alpha, size = remain, replace = False, p = vals
    )
    rnd_choice = np.sort(rnd_choice)

    interpolator = interp1d(distance, ipoints, kind = 'quadratic', axis = 0)
    peaks_on_track = interpolator(rnd_choice)

    # TODO
    return np.asarray(peaks_on_track)
