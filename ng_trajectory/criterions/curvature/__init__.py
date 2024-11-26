#!/usr/bin/env python3.6
"""Curvature criterion for fitness evaluation.

This criterion computes fitness value from curvature of the path.
Since we expect that the input data already contain curvature,
the fitness itself is computed as:

	sum( (k_i)^2 )
"""  # noqa: W191
from .main import CurvatureCriterion as Criterion  # noqa: F401
