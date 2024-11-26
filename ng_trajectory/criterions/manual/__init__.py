#!/usr/bin/env python3.6
"""Manual criterion for fitness evaluation.

Upon calling 'compute()' the criterion prints out current candidate
and waits for the user to specify the fitness value.

Note: This is usable only when a single worker is used.
"""
from .main import ManualCriterion as Criterion  # noqa: F401
