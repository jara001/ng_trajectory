#!/usr/bin/env python3.6
# main.py
"""Select points from a path uniformly (and equidistantly).

Note: It is just selector branched from `Curvature2`.
"""
######################
# Imports & Globals
######################

import numpy, sys

# Cubic spline interpolation
from ng_trajectory.interpolators import cubic_spline

# Functions for equidistant resampling and selection
from ng_trajectory.selectors.curvature2.main import resolutionEstimate, factorCompute, pathPointDistanceAvg, pathLength

# Support for rotating the trajectory
from ng_trajectory.interpolators.utils import trajectoryRotate, trajectoryClosestIndex

# Resampling for rotation required GCD
import fractions


# Global variables
INTERPOLATOR = cubic_spline


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("sampling_distance", 1.0, float, "[m] Distance of super-sampling before the interpolation, skipped when 0.", "init")
P.createAdd("distance", 0, float, "[m] Distance between the individual points, ignored when 0, used when requesting negative number of points.", "init")
P.createAdd("rotate", 0, float, "Parameter for rotating the input path. 0 is not rotated. <0, 1)", "init")
P.createAdd("fixed_points", [], list, "Points to be used in the selection upon calling 'select'.", "init")


######################
# Utilities
######################

def fractions_gcd(a, b):
    """Computes gcd of two Fractions.

    Arguments:
    a -- first fraction, Fraction
    b -- second fraction, Fraction

    Returns:
    gcd -- gcd of 'a' and 'b', Fraction

    Note:
    This is a replacement for 'fractions.gcd' as it
    is removed in Python>=3.9.

    Source:
    https://github.com/python/cpython/blob/3.6/Lib/fractions.py
    """

    while b:
        a, b = b, a % b

    return a


def trajectoryResample(points, remain):
    """Resample path described by points.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int

    Returns:
    rpoints -- list of points,
               mx2 numpy.ndarray when remain < 0
               remainx2 numpy.ndarray otherwise
    """

    # Throw away repeated point
    if points[0, :1] == points[-1, :1]:
        points = points[:-1, :]


    # Rotate to get to the first fixed point
    if len(P.getValue("fixed_points")) > 0:
        points = numpy.roll(
            points,
            -trajectoryClosestIndex(points, P.getValue("fixed_points")[0]),
            axis = 0
        )

    
    # Resample if requested
    if P.getValue("sampling_distance") != 0.0:
        points = INTERPOLATOR.interpolate(points[:, :2], resolutionEstimate(points, P.getValue("sampling_distance")))


    # Select points equidistantly
    if remain < 0:
        rpoints = INTERPOLATOR.interpolate(points[:, :2], resolutionEstimate(points, P.getValue("distance")))
    # Select 'remain' points
    else:
        rpoints = INTERPOLATOR.interpolate(points[:, :2], remain)


    # Return when no rotation
    if P.getValue("rotate") == 0.0:
        return rpoints


    ## Precise rotation using fractions
    # Note: The precision is set to centimeters for 'remain'.
    # 1) Current distance between individual points
    if remain < 0:
        f_dist = fractions.Fraction(str(P.getValue("distance")))
    else:
        f_dist = fractions.Fraction("%.2f" % (pathLength(points) / remain))

    # 2) Rotation + distance to the first rotated point
    f_rot = fractions.Fraction(str(P.getValue("rotate")))
    f_rot_dist = f_dist * f_rot

    # 3) Greatest common divisor
    # This tells us how much we have to increase the number of path points
    # in order to rotate by shifting the indices
    gcd = fractions_gcd(f_dist, f_rot_dist)

    # 4) Interpolate the path by the gcd factor
    factor = int(f_dist / gcd)
    fpoints = INTERPOLATOR.interpolate(points[:, :2], factor * len(rpoints))

    # 5) Compute index shift
    shift = int(
        f_rot / fractions_gcd(fractions.Fraction(1), f_rot)
    )

    # 6) Return rotated path
    return numpy.roll(
        fpoints,
        -shift,
        axis = 0
    )[numpy.linspace(0, len(fpoints), len(rpoints), endpoint = False, dtype = numpy.int), :]


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize selector."""

    # Check value for rotate
    if "rotate" in kwargs and not (0 <= kwargs.get("rotate") < 1):
        print ("Expected 'rotate' to be 0<=rotate<1, but it is %f. Omitting." % kwargs.get("rotate"), file=sys.stderr)
        del kwargs["rotate"]

    # Check fixed points
    if "fixed_points" in kwargs and len(kwargs.get("fixed_points")) > 1:
        print ("Passing multiple fixed points is not currently supported. Taking only the first one.", file = sys.stderr)
        kwargs["fixed_points"] = [kwargs.get("fixed_points")[0]]

    # Update parameters
    P.updateAll(kwargs)


def select(points: numpy.ndarray, remain: int, **overflown) -> numpy.ndarray:
    """Select points from the path uniformly.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int
    **overflown -- arguments not caught by previous parts

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray

    Note: When 'remain' is negative, number of points is selected so in average
    the distance between points is 'distance'.

    Note: An Exception is raised when 'distance' <= 0 and 'remain' < 0.
    """

    if remain < 0 and P.getValue("distance") <= 0:
        # Raise an exception, as we cannot proceed without further information.
        raise ValueError("Negative selection requires set 'distance' parameter for 'uniform_distance' selector.")

    return trajectoryResample(points, remain)
