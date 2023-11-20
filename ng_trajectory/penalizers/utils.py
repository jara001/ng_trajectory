#!/usr/bin/env python3.6
# utils.py3
"""Various utilities for penalizers.

Created to shrink the code inside the penalizers.
"""
######################
# Imports & Globals
######################

import numpy

from typing import List, Tuple

from ng_trajectory.parameter import ParameterList
from ng_trajectory.segmentators.utils import (
    pointToMap,
    validCheck,
)

# Parameters
P = ParameterList()
P.createAdd("method", "max", str, "Optimization method for final penalty -- min / max / sum / avg.", "Init.")
P.createAdd("huber_loss", False, bool, "Whether to use Huber loss for computing the fitness.", "Init.")
P.createAdd("huber_delta", 1.0, float, "(Requires 'huber_loss'). Delta used for computing the fitness.", "Init.")


######################
# Optimization methods
######################

METHODS = {
    "min": {
        "function": lambda old, new: min(old, new),
        "initial": 1000,
        "after": lambda result, invalid_count: result,
    },
    "max": {
        "function": lambda old, new: max(old, new),
        "initial": 0,
        "after": lambda result, invalid_count: result,
    },
    "sum": {
        "function": lambda old, new: old + new,
        "initial": 0,
        "after": lambda result, invalid_count: result,
    },
    "avg": {
        "function": lambda old, new: old + new,
        "initial": 0,
        "after": lambda result, invalid_count: (
            result / invalid_count if invalid_count > 0 else result
        ),
    },
}

METHOD = METHODS["max"]["function"]
INITIAL = METHODS["max"]["initial"]
AFTER = METHODS["max"]["after"]
HUBER_LOSS = False
HUBER_DELTA = 0.0


######################
# Utilities
######################

def eInvalidPoints(points: numpy.ndarray) -> List[Tuple[int, List[float]]]:
    """Iterate over all invalid points with enumeration.

    Arguments:
    points -- points to check, nx2 numpy.ndarray

    Return:
    yield (_i, _p) -- invalid points with their indices,
                      m-list of (int, [float])
    """
    for _i, _p in enumerate(points):
        try:
            # Depending on the context (i.e., how often something happens)
            # it might be better to just avoid 'pointInBounds' and catch
            # the exception.
            # If there are many points outside the map it should be better
            # to call the function instead of try-except.
            # if not pointInBounds(_p) or not validCheck(pointToMap(_p)):
            if not validCheck(pointToMap(_p)):
                yield _i, _p
        except IndexError:
            # Index is outside of the map.
            yield _i, _p
        except OverflowError:
            # OverflowError: Python int too large to convert to C long
            # Happens when we try to convert negative values to uint64.
            yield _i, _p
