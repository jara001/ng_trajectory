#!/usr/bin/env python3.6
"""Braghin's transformation method for track optimization (1D).

This optimizer is an implementation of an approach described
in [1]. The track is characterized by "cuts", a 1D lines placed
on the track, where on each cut there is a single path waypoint.
Since we are aware of their order, we can just simply interpolate
these points to receive a path.

The optimization itself is done using Nevergrad.

[1]: F. Braghin, F. Cheli, S. Melzi, and E. Sabbioni. 2008.
     Race driver model. Computers & Structures 86, 13 (July
     2008), 1503–1516.
"""
from .main import init, optimize
