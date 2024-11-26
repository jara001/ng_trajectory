#!/usr/bin/env python3.6
"""Segment penalizer.

This penalizer detects all misplaced points.

The penalty is calculated as a distance between invalid point
and valid points.
"""
from .main import SegmentPenalizer as Penalizer  # noqa: F401
