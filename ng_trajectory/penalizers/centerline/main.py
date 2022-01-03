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
CENTERLINE = None


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("method", "min", str, "Optimization method for final penalty -- min / max.", "Init.")


######################
# Optimization methods
######################

METHODS = {
    "min": {
        "function": lambda x, y: min(x, y),
        "initial": 1000,
    },
    "max": {
        "function": lambda x, y: max(x, y),
        "initial": 0,
    }
}

METHOD = METHODS["min"]["function"]
INITIAL = METHODS["min"]["initial"]


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
    global CENTERLINE, METHOD, INITIAL


    # Update parameters
    P.updateAll(kwargs)


    # Update method
    if P.getValue("method") in METHODS:
        METHOD = METHODS[P.getValue("method")]["function"]
        INITIAL = METHODS[P.getValue("method")]["initial"]


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
    global CENTERLINE

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

    for _ip, _p in enumerate(points):
        if not numpy.any(numpy.all(numpy.abs( numpy.subtract(valid_points, _p[:2]) ) < _grid, axis = 1)):

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

            invalid = METHOD(invalid, _invalid)


    return invalid * penalty if invalid != INITIAL else 0


