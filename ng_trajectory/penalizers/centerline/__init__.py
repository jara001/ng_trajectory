#!/usr/bin/env python3.6
"""Centerline penalizer.

This penalizer detects all misplaced points. Each point is associated
with a section of the track centerline based upon its location.

The penalty is calculated as maximum distance between invalid point
and points of the borderline.

Final penalty is the minimum of all of these distances.

Note: Initialization of this is done only once; we expect that the
centerline is present there (as it usually is).
"""
from .main import init, penalize
