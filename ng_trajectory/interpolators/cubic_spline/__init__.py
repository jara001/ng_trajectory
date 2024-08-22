#!/usr/bin/env python3.6
"""Cubic spline interpolator.

This interpolator connects the racing line waypoints using
cubic spline. Therefore, the resulting path is differentiable
two times, and its curvature is continuous and "smooth". The
curvature is computed as follows:

	K = (x' * y'' - y' * x'') / ( x'**2 + y'**2 )**(3/2)

Source: https://www.math24.net/curvature-radius/

Interpolation is done by CubicSpline from scipy.interpolate.

Note: It is expected that the input points describe a continuous
path (end-start).
"""  # noqa: W191
from .main import init, interpolate  # noqa: F401
