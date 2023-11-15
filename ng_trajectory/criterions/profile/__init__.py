#!/usr/bin/env python3.6
"""Profile criterion for fitness evaluation.

This criterion computes fitness value by simulating the vehicle
over the input path. There are various parameters to be set. But
mostly, we focus on a simple vehicle model and simple environment
interaction by friction coefficient, air density, etc.

Note: The parameters shown below are not synced with the algorithm
itself. Therefore, pay attention to any updates.
"""
from .main import init, compute  # noqa: F401

from .main import OVERTAKING_POINTS  # noqa: F401
