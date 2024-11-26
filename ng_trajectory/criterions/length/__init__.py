#!/usr/bin/env python3.6
"""Length criterion for fitness evaluation.

This criterion computes fitness value from length of the path.
We calculate real segment-based length, i.e., sum of all sub-
segment parts of the path.
"""
from .main import LengthCriterion as Criterion  # noqa: F401
