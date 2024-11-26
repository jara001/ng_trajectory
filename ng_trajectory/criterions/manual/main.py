#!/usr/bin/env python3.6
# main.py
"""Manual criterion, fitness value is specified by the user.
"""
######################
# Imports & Globals
######################

import sys
import numpy

from ng_trajectory.abc.criterions import CriterionABC

from threading import Lock

from typing import (
    Any,
    Dict,
    Optional,
)


# Global variables
INPUT_LOCK = Lock()


######################
# Functions
######################

class ManualCriterion(CriterionABC):

    def init(self, **kwargs) -> Optional[Dict[str, Any]]:
        """Initialize criterion."""
        return None


    def compute(
            self,
            points: numpy.ndarray,
            overlap: int = None,
            penalty: float = 100.0,
            **overflown) -> float:
        """Compute the speed profile using overlap.

        Arguments:
        points -- points of a trajectory with curvature, nx3 numpy.ndarray
        overlap -- size of trajectory overlap, int, default None/0 (disabled)
        penalty -- penalty value applied to the incorrect solutions,
                   float, default 100.0
        **overflown -- arguments not caught by previous parts

        Returns:
        t -- fitness value
        """
        print(";".join(["%s,%s" % (x, y) for x, y in points[:, :2]]))

        with INPUT_LOCK:
            while True:
                try:
                    val = input("Specify fitness value (%f): ")
                    float(val)
                    break

                except ValueError:
                    print ("Unable to parse the value.", file = sys.stderr)

        return float(val)
