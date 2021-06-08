#!/usr/bin/env python3.6
"""Track segmentator based on the flood fill.

This segmentator splits the track into segments by flood
fill algorithm from the centers.
"""
from .main import init, segmentate
