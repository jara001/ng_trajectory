#!/usr/bin/env python3.6
"""Segment penalizer.

This penalizer detects all misplaced points. Each point is associated
with two segments based on its location.

The penalty is calculated as a distance between invalid point
and the points of the two segments.
"""
from .main import init, penalize
