#!/usr/bin/env python3.6
# interpolate.py
"""Interpolator for Braghin.

NOTE!: THIS SHOULD NOT BE USED AS A TRAJECTORY INTERPOLATOR.
Use .interpolators instead.
"""
######################
# Imports & Globals
######################

import numpy

# Interpolation for Braghin
from scipy.interpolate import CubicSpline

from typing import Tuple


######################
# Utilities (Trajectory)
######################

def pointsInterpolate(points: numpy.ndarray, int_size: int = 440) -> Tuple[numpy.ndarray, numpy.ndarray, numpy.ndarray]:
    """Interpolate points using cubic spline.

    Arguments:
    points -- points to interpolate, nx2 numpy.ndarray
    int_size -- number of points in the interpolation, int, default 440

    Returns:
    ipoints -- coordinates (x, y) of the interpolated path, int_sizex2 numpy.ndarray
    dipoints -- coordinates (x, y) of the first order derviation of the path,
                int_sizex2 numpy.ndarray
    d2ipoints -- coordinates (x, y) of the second order derviation of the path,
                int_sizex2 numpy.ndarray

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
    """

    _points = numpy.vstack((points, points[0, :]))

    # Throw away additional information
    if _points.shape[1] > 2:
        _points = _points[:, :2]

    x, y = _points.T
    i = numpy.arange(len(_points))

    distance = numpy.cumsum( numpy.sqrt( numpy.sum( numpy.diff(_points, axis=0)**2, axis=1 ) ) )
    distance = numpy.insert(distance, 0, 0) / distance[-1]

    alpha = numpy.linspace(0, 1, int_size, endpoint = True) # True? False? (bylo False)

    # CubicSpline allows forcing 2nd order differentiability on spline start/end
    ipol = CubicSpline(distance, _points, axis=0, bc_type="periodic")(alpha)
    ipol1 = CubicSpline(distance, _points, axis=0, bc_type="periodic")(alpha, 1)
    ipol2 = CubicSpline(distance, _points, axis=0, bc_type="periodic")(alpha, 2)

    return (ipol, ipol1, ipol2)
