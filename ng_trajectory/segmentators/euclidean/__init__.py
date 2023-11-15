#!/usr/bin/env python3.6
"""Track segmentator based on euclidean distance from the center.

This segmentator splits the track into segments based on the distance
of the individual track parts from the group centers.

Note: Even though this is fast, it can missalign points (e.g., when
they are behind a close wall).
"""
from .main import init, segmentate  # noqa: F401
