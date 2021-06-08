#!/usr/bin/env python3.6
"""Matryoshka transformation for track optimization (2D).

This optimizers segmentates the track into 2D sequence
of segments. Each waypoint of the racing line is situated
into one of these segments. Interpolating them in order
yields a path.

In order to efficiently move inside the segments,
a homeomorphism transformation is created (Matryoshka).

For the optimization itself, Nevergrad is used.
"""
from .main import init, optimize
