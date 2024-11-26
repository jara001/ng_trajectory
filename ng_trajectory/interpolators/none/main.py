#!/usr/bin/env python3.6
# main.py
"""Forward the points instead of interpolating them.

Uses Menger curvature to obtain curvature.
"""
######################
# Imports & Globals
######################

import numpy

from ng_trajectory.abc.interpolators import InterpolatorABC

from ng_trajectory.parameter import ParameterList

from typing import (
    Any,
    Dict,
    Optional,
)


# Parameters
P = ParameterList()
P.createAdd(
    "closed_loop", True, bool,
    "When set, interpolation creates a closed loop.", "init"
)


######################
# Functions
######################

class NoneInterpolator(InterpolatorABC):

    def init(self, **kwargs) -> Optional[Dict[str, Any]]:
        """Initialize interpolator."""
        P.updateAll(kwargs)


    def interpolate(
            self,
            points: numpy.ndarray,
            int_size: int = 400,
            **overflown) -> numpy.ndarray:
        """Forward points without any interpolation.

        Arguments:
        points -- points to forward, nx2 numpy.ndarray
        **overflown -- arguments not caught by previous parts

        Returns:
        ipoints -- coordinates (x, y) and curvature estimation of the points,
                   nx3 numpy.ndarray
        """
        curvatures = numpy.zeros((len(points), 1))

        # Inspired by:
        # https://github.com/MaciejPMarciniak/curvature/blob/master/src/curvature.py
        """
        This is used there. How does it work?

        @staticmethod
        def _get_twice_triangle_area(a: NDArray, b: NDArray, c: NDArray) -> float:
            "Calculates the doubled triangle area from a set of 2D points.

            Args:
                a: 2D point
                b: 2D point
                c: 2D point

            Returns:
                Doubled triangle area.
            "
            if np.all(a == b) or np.all(b == c) or np.all(c == a):
                sys.exit("CURVATURE:\nAt least two points are at the same position")

            twice_triangle_area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

            if twice_triangle_area == 0:
                warnings.warn(f"Collinear consecutive points found: \na: {a}\t b: {b}\t c: {c}")

            return twice_triangle_area
        """  # noqa: E501
        def menger_curvature(a, b, c) -> float:
            def area(a, b, c) -> float:
                """Use Heron's formula to obtain triangle area."""
                _a = numpy.linalg.norm(b - c)
                _b = numpy.linalg.norm(c - a)
                _c = numpy.linalg.norm(a - b)

                s = (_a + _b + _c) / 2

                return numpy.sqrt(
                    s * (s - _a) * (s - _b) * (s - _c)
                )

            return (
                4 * area(a, b, c)
                / (
                    numpy.linalg.norm(a - b)
                    * numpy.linalg.norm(b - c)
                    * numpy.linalg.norm(c - a)
                )
            )


        for i in range(1, len(points) - 1):
            curvatures[i] = menger_curvature(
                points[i - 1, :],
                points[i, :],
                points[i + 1, :]
            )

        if P.getValue("closed_loop"):
            curvatures[0] = menger_curvature(
                points[-1, :],
                points[0, :],
                points[1, :]
            )

            curvatures[-1] = menger_curvature(
                points[-2, :],
                points[-1, :],
                points[0, :]
            )

        return numpy.hstack((points, curvatures))
