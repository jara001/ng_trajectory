#!/usr/bin/env python3.6
"""Borderlines penalizer.

Borderlines are sets of points on the borders between to adjacent
segments. We have borderlines for each segment and for each neighbour
(usually resulting into n * 2 arrays).

This penalizer detects all misplaced points. Each point is associated
with a borderline based upon its location -- e.g., points in between
selected points of segments #5 and #6 belong to borderline 5-6.

The penalty is calculated as maximum distance between invalid point
and points of the borderline.

Final penalty is the minimum of all of these distances.
"""
from .main import init, penalize, INVALID_POINTS
