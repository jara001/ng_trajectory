#!/usr/bin/env python3.6
# main.py
"""Interpolate points of the path by cubic spline.

Using scipy.interpolate.CubicSpline. Curvature is computed
using K = (x' * y'' - y' * x'') / ( x'**2 + y'**2 )**(3/2).
"""
######################
# Imports & Globals
######################

import numpy

from scipy.interpolate import CubicSpline

from ng_trajectory.parameter import ParameterList

from typing import (
    Any,
    Dict,
    Optional,
)


# Parameters
P = ParameterList()
P.createAdd("int_size", 400, int, "Number of points in the interpolation.", "")
P.createAdd("closed_loop", True, bool, "When set, interpolation creates a closed loop.", "init")


######################
# Functions
######################

def init(**kwargs) -> Optional[Dict[str, Any]]:
    """Initialize interpolator."""
    P.updateAll(kwargs)


def interpolate(
        points: numpy.ndarray,
        int_size: int = 400,  # TODO: THIS CANNOT BE CHANGED?
        **overflown) -> numpy.ndarray:
    """Interpolate points using cubic spline.

    Arguments:
    points -- points to interpolate, nx2 numpy.ndarray
    int_size -- number of points in the interpolation, int, default 400
    **overflown -- arguments not caught by previous parts

    Returns:
    ipoints -- coordinates (x, y) and curvature of interpolated points,
               int_sizex3 numpy.ndarray

    Note: It is expected that the input trajectory is continuous (end-start).

    Source:
    center_trajectory.py:interpolate_points() by David Kopecky
    https://stackoverflow.com/questions/52014197/how-to-interpolate-a-2d-curve-in-python
    profile_trajectory.py:interpolate_points()
    profile_trajectory2.py:interpolate_points()

    Note: Ported to Py3.6.
    Differences:
        Purely numpy / scipy.
        First and last point are not the same.
        Reduced default int_size from 440 to 400.
    """
    if P.getValue("closed_loop"):
        _points = numpy.vstack((points, points[0, :]))
        _bc_type = "periodic"
    else:
        _points = points.copy()
        _bc_type = "natural"

    x, y = _points.T

    distance = numpy.cumsum(
        numpy.sqrt(
            numpy.sum(
                numpy.diff(_points, axis=0)**2,
                axis = 1
            )
        )
    )
    distance = numpy.insert(distance, 0, 0) / distance[-1]

    alpha = numpy.linspace(0, 1, int_size, endpoint = False)

    # CubicSpline allows forcing a 2nd order differentiability on the spline
    # start/end
    ipol = CubicSpline(distance, _points, axis=0, bc_type=_bc_type)(alpha)

    # 2nd order derivative
    # https://en.wikipedia.org/wiki/Curvature#Curvature_of_a_graph
    # curvature = sqrt(x''**2 + y''**2)
    # pokus #2
    # https://www.math24.net/curvature-radius/
    # K = (x' * y'' - y' * x'') / ( x'**2 + y'**2 )**(3/2)
    ipol2 = CubicSpline(distance, _points, axis=0, bc_type=_bc_type)(alpha, 2)
    ipol1 = CubicSpline(distance, _points, axis=0, bc_type=_bc_type)(alpha, 1)

    return numpy.hstack((
        ipol,
        (
            numpy.divide(
                numpy.subtract(
                    numpy.multiply(ipol1[:, 0], ipol2[:, 1]),
                    numpy.multiply(ipol1[:, 1], ipol2[:, 0])
                ),
                numpy.sqrt(
                    numpy.power(
                        numpy.add(
                            numpy.power(ipol1[:, 0], 2),
                            numpy.power(ipol1[:, 1], 2)
                        ),
                        3
                    )
                )
            )
        )[:, numpy.newaxis]
    ))
