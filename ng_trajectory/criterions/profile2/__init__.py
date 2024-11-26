#!/usr/bin/env python3.6
"""Profile criterion for fitness evaluation.

This criterion computes fitness value by simulating the vehicle
over the input path. There are various parameters to be set. But
mostly, we focus on a simple vehicle model and simple environment
interaction by friction coefficient, air density, etc.

Note: The parameters shown below are not synced with the algorithm
itself. Therefore, pay attention to any updates.

This is a version developed by Tomas Nagy in his Master thesis.

This tool was used to identify possible overtaking zones on
a given track when the opponent's racing line is known. Moreover,
we assume that the opponent cannot perform a blocking move (F1TENTH
vehicles do not have a rear-facing sensor).
"""
from .main import Profile2Criterion as Criterion  # noqa: F401

from .main import OVERTAKING_POINTS  # noqa: F401
