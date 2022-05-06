#!/usr/bin/env python3.6
"""Uniform time selector.

This selector uniformly samples the input path, so
that the points are equidistantly spaced in time.

For evaluating the line, the 'profile' criterion
is used.
"""
from .main import init, select
