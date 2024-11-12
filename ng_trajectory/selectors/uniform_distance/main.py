#!/usr/bin/env python3.6
# main.py
"""Select points from a path uniformly (and equidistantly).

Note: It is just selector branched from `Curvature2`.
"""
######################
# Imports & Globals
######################

import numpy
import sys

# Cubic spline interpolation
from ng_trajectory.interpolators import cubic_spline

# Functions for equidistant resampling and selection
from ng_trajectory.selectors.curvature2.main import (
    resolutionEstimate,
    pathLength
)

# Support for rotating the trajectory
from ng_trajectory.interpolators.utils import (
    trajectoryClosestIndex
)

from ng_trajectory.parameter import ParameterList

from ng_trajectory.log import print0

# Resampling for rotation required GCD
import fractions

from typing import (
    Any,
    Dict,
    Optional,
)


# Global variables
INTERPOLATOR = cubic_spline


# Parameters
P = ParameterList()
P.createAdd("sampling_distance", 1.0, float, "[m] Distance of super-sampling before the interpolation, skipped when 0.", "init")
P.createAdd("distance", 0, float, "[m] Distance between the individual points, ignored when 0, used when requesting negative number of points.", "init")
P.createAdd("rotate", 0, float, "Parameter for rotating the input path. 0 is not rotated. <0, 1)", "init")
P.createAdd("fixed_points", [], list, "Points to be used in the selection upon calling 'select'.", "init")


######################
# Utilities
######################

def fractions_gcd(a, b):
    """Compute gcd of two Fractions.

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


    # Keep fixed points local
    raw_fixed_points = P.getValue("fixed_points").copy()

    # Result
    rpoints = []
    # Intermediate results
    fixed_points = []
    upoints = []

    # Other values
    if type(P.getValue("rotate")) is not list:
        rotate = [
            P.getValue("rotate")
            for _ in range(max(1, len(P.getValue("fixed_points"))))
        ]
    else:
        rotate = P.getValue("rotate").copy()

    # Loop at least once (for sure) then with respect to the fixed points.
    while True:

        # Rotate to get to the first fixed point
        _points = numpy.roll(
            points,
            (
                -trajectoryClosestIndex(points, raw_fixed_points.pop(0))
                if len(raw_fixed_points) > 0
                else 0
            ),
            axis = 0
        )


        # Resample if requested
        if P.getValue("sampling_distance") != 0.0:
            _points = INTERPOLATOR.interpolate(
                _points[:, :2],
                resolutionEstimate(_points, P.getValue("sampling_distance"))
            )


        # Select points equidistantly
        if remain < 0:
            _rpoints = INTERPOLATOR.interpolate(
                _points[:, :2],
                resolutionEstimate(_points, P.getValue("distance"))
            )
        # Select 'remain' points
        else:
            _rpoints = INTERPOLATOR.interpolate(_points[:, :2], remain)


        # Rotate when required
        if rotate[0] > 0.0:

            # # Precise rotation using fractions # #
            # Note: The precision is set to centimeters for 'remain'.
            # 1) Current distance between individual points
            if remain < 0:
                f_dist = fractions.Fraction(str(P.getValue("distance")))
            else:
                f_dist = fractions.Fraction(
                    "%.2f" % (pathLength(_points) / remain)
                )

            # 2) Rotation + distance to the first rotated point
            f_rot = fractions.Fraction(str(rotate.pop(0)))
            f_rot_dist = f_dist * f_rot

            # 3) Greatest common divisor
            # This tells us how much we have to increase the number
            # of path points in order to rotate by shifting the indices
            gcd = fractions_gcd(f_dist, f_rot_dist)

            # 4) Interpolate the path by the gcd factor
            factor = int(f_dist / gcd)
            _fpoints = INTERPOLATOR.interpolate(
                _points[:, :2],
                factor * len(_rpoints)
            )

            # 5) Compute index shift
            shift = int(
                f_rot / fractions_gcd(fractions.Fraction(1), f_rot)
            )

            # 6) Return rotated path
            _upoints = numpy.roll(
                _fpoints,
                -shift,
                axis = 0
            )

            _rpoints = _upoints[
                numpy.linspace(
                    0,
                    len(_fpoints),
                    len(_rpoints),
                    endpoint = False,
                    dtype = numpy.int
                ), :
            ]

            fixed_points.append(_fpoints[0])
            upoints.append(_upoints)

        else:
            rotate.pop(0)

            # Create fpoints with a set factor to allow concatenating
            _fpoints = INTERPOLATOR.interpolate(
                # TODO: Set this automatically. When the fixed points are
                #       too close to each other, they match the same point
                #       in this array, leading to "eating" them. On the other
                #       hand, with larger number, it is possible to do this,
                #       but the number of segments grows.
                # PR @jara001: This was increased from 100 to 1000.
                _points[:, :2], 1000 * len(_rpoints)
            )

            fixed_points.append(_fpoints[0])
            upoints.append(_fpoints)


        rpoints.append(_rpoints)

        if len(raw_fixed_points) <= 0:
            break

    if len(rpoints) == 1:
        return rpoints[0]

    else:
        # Build up the new path waypoints
        result = None

        for _i in range(len(rpoints)):

            # 1) Take the fixed point from the next segment
            _p = fixed_points[(_i + 1) % len(rpoints)]

            # 2) Find it in current path (rotated, full)
            _cpi = trajectoryClosestIndex(upoints[_i], _p, from_left = True)

            # 3) Loop through the selection to find all points
            #    that are more to the left
            _max_i = -1

            while (
                _max_i + 1 < len(rpoints[_i])
                and trajectoryClosestIndex(
                    upoints[_i], rpoints[_i][_max_i + 1, :], from_left = True
                ) < _cpi
            ):
                _max_i += 1

            # 4) Append them to the result
            if _max_i >= 0:
                if result is None:
                    result = rpoints[_i][:_max_i + 1, :]
                else:
                    result = numpy.vstack(
                        (result, rpoints[_i][:_max_i + 1, :])
                    )

        return result


######################
# Functions
######################

def init(**kwargs) -> Optional[Dict[str, Any]]:
    """Initialize selector."""
    # Check value for rotate
    if "rotate" in kwargs:

        if (
            type(kwargs.get("rotate")) is not list
            and not (0 <= kwargs.get("rotate") < 1)
        ):
            print0(
                "Expected 'rotate' to be 0<=rotate<1, but it is %f. Omitting."
                % kwargs.get("rotate"),
                file=sys.stderr
            )
            del kwargs["rotate"]

        elif (
            type(kwargs.get("rotate")) is list
            and len(kwargs.get("rotate")) != len(
                kwargs.get("fixed_points", [])
            )
        ):
            print0(
                "Expected 'rotate' length to match number of fixed points "
                "(%d), but it is %d. Using only %f."
                % (
                    len(kwargs.get("fixed_points", [])),
                    len(kwargs.get("rotate")),
                    kwargs.get("rotate")[0]
                )
            )
            kwargs["rotate"] = kwargs["rotate"][0]


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
        raise ValueError(
            "Negative selection requires set 'distance' parameter "
            "for 'uniform_distance' selector."
        )

    rpoints = trajectoryResample(points, remain)

    # !Force number of points
    if remain > 0 and len(rpoints) != remain:
        return trajectoryResample(points, remain - 1)

    return rpoints
