#!/usr/bin/env python3.6
# interpolate.py
"""Interpolator for Matryoshka.

NOTE!: THIS SHOULD NOT BE USED AS A TRAJECTORY INTERPOLATOR.
Use .interpolators instead.
"""
######################
# Imports & Globals
######################

import numpy

# Interpolation for Matryoshka
from scipy.interpolate import CubicSpline


######################
# Utilities (Trajectory)
######################

def trajectoryInterpolate(points: numpy.ndarray, int_size: int = 440) -> numpy.ndarray:
    """Interpolate points using cubic spline.

    Arguments:
    points -- points to interpolate, nx2 numpy.ndarray
    int_size -- number of points in the interpolation, int, default 440

    Returns:
    ipoints -- coordinates (x, y) and curvature of interpolated points, int_sizex3 numpy.ndarray

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

    NOTE!: THIS IS USED BY MATRYOSHKA AND SHOULD NOT BE USED AS A TRAJECTORY INTERPOLATOR.
    """

    #if ( points[0, :] != points[-1, :] ).all():
    _points = numpy.vstack((points, points[0, :]))
    #else:
    #    # Shapely's LinearRings have first and last point equal
    #    _points = points

    x, y = _points.T
    i = numpy.arange(len(_points))

    distance = numpy.cumsum( numpy.sqrt( numpy.sum( numpy.diff(_points, axis=0)**2, axis=1 ) ) )
    distance = numpy.insert(distance, 0, 0) / distance[-1]

    alpha = numpy.linspace(0, 1, int_size, endpoint = False)

    # CubicSpline allows forcing 2nd order differentiability on spline start/end
    ipol = CubicSpline(distance, _points, axis=0, bc_type="periodic")(alpha)

    # 2nd order derivative
    # https://en.wikipedia.org/wiki/Curvature#Curvature_of_a_graph
    # curvature = sqrt(x''**2 + y''**2)
    # pokus #2
    # https://www.math24.net/curvature-radius/
    # K = (x' * y'' - y' * x'') / ( x'**2 + y'**2 )**(3/2)
    ipol2 = CubicSpline(distance, _points, axis=0, bc_type="periodic")(alpha, 2)
    ipol1 = CubicSpline(distance, _points, axis=0, bc_type="periodic")(alpha, 1)

    return numpy.hstack((ipol, (
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
                                3)
                            )
                        )
                    )[:, numpy.newaxis]
            ))
