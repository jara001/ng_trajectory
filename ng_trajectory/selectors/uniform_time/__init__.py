#!/usr/bin/env python3.6
"""Uniform time selector.

This selector uniformly samples the input path, so
that the points are equidistantly spaced in time.

Following algorithms are used:
- 'profile' criterion for computing the time,
- 'cubic_spline' interpolator for smoothing the input,
- 'uniform_distance' selector for resampling the input.
"""
from .main import init, select  # noqa: F401
