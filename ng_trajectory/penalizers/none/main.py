#!/usr/bin/env python3.6
# main.py
"""Dummy penalizer.

Passes any candidate.
"""
######################
# Imports & Globals
######################

import numpy

from ng_trajectory.abc.penalizers import PenalizerABC

from typing import (
    Any,
    Dict,
    Optional,
)


######################
# Functions
######################

class NonePenalizer(PenalizerABC):

    def init(self, **kwargs) -> Optional[Dict[str, Any]]:
        """Initialize penalizer."""
        pass


    def penalize(
            self,
            points: numpy.ndarray,
            valid_points: numpy.ndarray,
            grid: float,
            penalty: float = 100,
            **overflown) -> float:
        """Get a penalty for the candidate solution.

        Penalty is based on the number of incorrectly placed points.

        Arguments:
        points -- points to be checked, nx(>=2) numpy.ndarray
        valid_points -- valid area of the track, mx2 numpy.ndarray
        grid -- when set, use this value as a grid size, otherwise it is computed,
                float
        penalty -- constant used for increasing the penalty criterion,
                   float, default 100
        **overflown -- arguments not caught by previous parts

        Returns:
        rpenalty -- value of the penalty, 0 means no penalty, float
        """
        self.INVALID_POINTS.clear()

        return 0.0
