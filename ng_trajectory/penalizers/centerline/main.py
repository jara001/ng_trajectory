#!/usr/bin/env python3.6
# main.py
"""Penalize the incorrect solution by distance to the centerline.
"""
######################
# Imports & Globals
######################

import numpy

from ng_trajectory.segmentators.utils import *

from typing import List, Dict


# Global variables
INVALID_POINTS = []
CENTERLINE = None


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("method", "min", str, "Optimization method for final penalty -- min / max / sum / avg.", "Init.")
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
        "after": lambda result, invalid_count: result / invalid_count if invalid_count > 0 else result,
    },
}

METHOD = METHODS["min"]["function"]
INITIAL = METHODS["min"]["initial"]
AFTER = METHODS["min"]["after"]
HUBER_LOSS = False
HUBER_DELTA = 0.0


######################
# Utilities
######################

# TODO: Create function for obtaining the centerline.


######################
# Functions
######################

def init(start_points: numpy.ndarray, **kwargs) -> None:
    """Initialize penalizer.

    Arguments:
    start_points -- initial line on the track, should be a centerline, nx2 numpy.ndarray
    """
    global CENTERLINE, METHOD, INITIAL, AFTER, HUBER_LOSS, HUBER_DELTA


    # Update parameters
    P.updateAll(kwargs)


    # Update method
    if P.getValue("method") in METHODS:
        METHOD = METHODS[P.getValue("method")]["function"]
        INITIAL = METHODS[P.getValue("method")]["initial"]
        AFTER = METHODS[P.getValue("method")]["after"]


    HUBER_LOSS = P.getValue("huber_loss")
    HUBER_DELTA = P.getValue("huber_delta")


    if CENTERLINE is None:
        CENTERLINE = start_points
        print ("Penalizer: Updating the centerline.")


def penalize(points: numpy.ndarray, candidate: List[numpy.ndarray], valid_points: numpy.ndarray, grid: float, penalty: float = 100, **overflown) -> float:
    """Get a penalty for the candidate solution based on number of incorrectly placed points.

    Arguments:
    points -- points to be checked, nx(>=2) numpy.ndarray
    candidate -- raw candidate (non-interpolated points), m-list of 1x2 numpy.ndarray
    valid_points -- valid area of the track, px2 numpy.ndarray
    grid -- when set, use this value as a grid size, otherwise it is computed, float
    penalty -- constant used for increasing the penalty criterion, float, default 100
    **overflown -- arguments not caught by previous parts

    Returns:
    rpenalty -- value of the penalty, 0 means no penalty, float

    Note: This is mostly the same as 'Borderlines'.
    """
    global CENTERLINE, INVALID_POINTS

    # Use the grid or compute it
    _grid = grid if grid else gridCompute(points)

    # Mapping between the candidate points and their interpolation
    _points_line_mapping = [
        numpy.argmin(
            numpy.sqrt(
                numpy.sum(
                    numpy.power(
                        numpy.subtract(
                            CENTERLINE[:, :2],
                            candidate[i]
                        ),
                        2
                    ),
                    axis = 1
                )
            )
        ) for i in range(len(candidate))
    ]

    # Check if all interpolated points are valid
    # Note: This is required for low number of groups.
    invalid = INITIAL
    any_invalid = False

    invalid_points = 0
    INVALID_POINTS.clear()

    for _ip, _p in enumerate(points):
        if not numpy.any(numpy.all(numpy.abs( numpy.subtract(valid_points, _p[:2]) ) < _grid, axis = 1)):

            invalid_points += 1

            # Store invalid point
            INVALID_POINTS.append(_p)

            # Note: Trying borderlines here, it works the same, just the meaning of 'invalid' is different.
            # Note: We used to have '<' here, however that failed with invalid index 0.
            _segment_id = len([ _plm for _plm in _points_line_mapping if _plm <= _ip ]) - 1

            # We need to wrap around when reaching over end of the line
            if _points_line_mapping[_segment_id] > _points_line_mapping[(_segment_id+1)%len(_points_line_mapping)]:
                _invalid = numpy.max(
                    numpy.sqrt(
                        numpy.sum(
                            numpy.power(
                                numpy.subtract(
                                    numpy.vstack((
                                        CENTERLINE[_points_line_mapping[_segment_id]:, :2],
                                        CENTERLINE[0:_points_line_mapping[(_segment_id+1)%len(_points_line_mapping)]+1, :2]
                                    )),
                                    _p[:2]
                                ),
                                2
                            ),
                            axis = 1
                        )
                    )
                )
            else:
                _invalid = numpy.max(
                    numpy.sqrt(
                        numpy.sum(
                            numpy.power(
                                numpy.subtract(
                                    CENTERLINE[_points_line_mapping[_segment_id]:_points_line_mapping[(_segment_id+1)%len(_points_line_mapping)]+1, :2],
                                    _p[:2]
                                ),
                                2
                            ),
                            axis = 1
                        )
                    )
                )


            if HUBER_LOSS:
                # '_invalid' is always positive
                if _invalid <= HUBER_DELTA:
                    _invalid = 0.5 * pow(_invalid, 2)
                else:
                    _invalid = HUBER_DELTA * (_invalid - 0.5 * HUBER_DELTA)


            invalid = METHOD(invalid, _invalid)


    invalid = AFTER(invalid, invalid_points)


    return invalid * penalty if invalid != INITIAL else 0
