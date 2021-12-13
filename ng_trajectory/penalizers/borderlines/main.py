#!/usr/bin/env python3.6
# main.py
"""Penalize the incorrect solution by distance to appropriate borderline.
"""
######################
# Imports & Globals
######################

import numpy

from ng_trajectory.segmentators.utils import gridCompute

from typing import List, Dict


# Parameters
#from ng_trajectory.parameter import *
#P = ParameterList()
#P.createAdd("int_size", 400, int, "Number of points in the interpolation.", "")


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize penalizer."""

    # Add important note
    print ("Warning: Currently, 'borderlines' penalizer works only with when:\n" +
           "\tOptimizer is set to Matryoshka.\n" +
           "\tOptimizer's 'use_borderlines' is set to True.\n" +
           "\tSegmentator is set to Flood Fill.")


def penalize(points: numpy.ndarray, candidate: List[numpy.ndarray], valid_points: numpy.ndarray, borderlines: Dict[int, Dict[int, numpy.ndarray]], grid: float, penalty: float = 100, **overflown) -> float:
    """Get a penalty for the candidate solution based on number of incorrectly placed points.

    Arguments:
    points -- points to be checked, nx(>=2) numpy.ndarray
    candidate -- raw candidate (non-interpolated points), m-list of 1x2 numpy.ndarray
    valid_points -- valid area of the track, px2 numpy.ndarray
    borderlines -- borderlines of all segments, m-dict of dicts of x2 numpy.ndarrays
    grid -- when set, use this value as a grid size, otherwise it is computed, float
    penalty -- constant used for increasing the penalty criterion, float, default 100
    **overflown -- arguments not caught by previous parts

    Returns:
    rpenalty -- value of the penalty, 0 means no penalty, float
    """

    # Use the grid or compute it
    _grid = grid if grid else gridCompute(points)

    # Mapping between the candidate points and their interpolation
    _points_line_mapping = [
        numpy.argmin(
            numpy.sqrt(
                numpy.sum(
                    numpy.power(
                        numpy.subtract(
                            points[:, :2], #START_POINTS[:, :2], #_points[:, :2],
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
    invalid = 1000
    any_invalid = False

    for _ip, _p in enumerate(points):
        if not numpy.any(numpy.all(numpy.abs( numpy.subtract(valid_points, _p[:2]) ) < _grid, axis = 1)):

            # Note: Trying borderlines here, it works the same, just the meaning of 'invalid' is different.
            # Note: We used to have '<' here, however that failed with invalid index 0.
            _segment_id = len([ _plm for _plm in _points_line_mapping if _plm <= _ip ]) - 1

            _invalid = numpy.max(
                numpy.sqrt(
                    numpy.sum(
                        numpy.power(
                            numpy.subtract(
                                borderlines[_segment_id][(_segment_id + 1) % len(candidate)],
                                _p[:2]
                            ),
                            2
                        ),
                        axis = 1
                    )
                )
            )

            invalid = min(invalid, _invalid)


    return invalid * penalty if invalid != 1000 else 0
