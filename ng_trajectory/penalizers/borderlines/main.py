#!/usr/bin/env python3.6
# main.py
"""Penalize the incorrect solution by distance to appropriate borderline.

Borderlines are sets of points on the borders between two adjacent
segments. We have borderlines for each segment and for each neighbour
(usually resulting into n * 2 arrays).
"""
######################
# Imports & Globals
######################

import numpy

from ng_trajectory.abc.penalizers import PenalizerABC

from ng_trajectory.segmentators.utils import (
    gridCompute,
    pointToMap
)

from typing import (
    Any,
    Dict,
    List,
    Optional,
)


# Global variables
BORDERLINES = None


######################
# Utilities
######################

def borderlinesCreate(
        valid_points: numpy.ndarray,
        group_centers: numpy.ndarray) -> Dict[int, Dict[int, numpy.ndarray]]:
    """Create borderlines for later penalty computation.

    valid_points -- valid area of the track, nx2 numpy.ndarray
    group_centers -- center points of created groups, mx2 numpy.ndarray
    """
    global MAP, MAP_ORIGIN, MAP_GRID

    # Create empty borderlines map
    borderlines_map = {i: {} for i in range(len(group_centers))}

    # Go through every point on the map (only those that belong to any segment)
    _valids = numpy.where(MAP < 100)

    for cell in zip(_valids[0], _valids[1]):

        for _a in [-1, 0, 1]:
            for _b in [-1, 0, 1]:
                if _a == 0 and _b == 0:
                    continue

                # Try does catch larger values but not negative
                if cell[0] + _a < 0 or cell[1] + _b < 0:
                    continue

                try:
                    _cell = MAP[cell[0] + _a, cell[1] + _b]
                except IndexError:
                    continue

                # Store it if its another segment
                # Without this we also get "neighbours to itself".
                if _cell < 100 and _cell != MAP[tuple(cell)]:
                    borderlines_map[
                        MAP[tuple(cell)]
                    ][(cell[0] + _a, cell[1] + _b)] = _cell
                # ... also in case that we are using reservations
                # elif _cell < 200 and _cell != MAP[tuple(cell)]:
                #     borderlines_map[
                #         MAP[tuple(cell)]
                #     ][(cell[0] + _a, cell[1] + _b)] = _cell - 100


    borderlines_real = {
        i: {
            j: []
            for j in range(len(group_centers))
        }
        for i in range(len(group_centers))
    }


    for p in valid_points:
        _i = MAP[tuple(pointToMap(p))]

        # Group only taken points
        if _i != 255 and _i < 100:

            # Create all combinations of borderlines in real coordinates
            for i in range(len(group_centers)):
                if tuple(pointToMap(p)) in borderlines_map[i]:
                    borderlines_real[i][
                        borderlines_map[i][tuple(pointToMap(p))]
                    ].append(p)

    # Finally process the borderlines
    return {
        i: {
            j: numpy.asarray(borderlines_real[i][j])
            for j in range(len(group_centers))
            if len(borderlines_real[i][j]) > 0
        }
        for i in range(len(group_centers))
    }


######################
# Functions
######################

class BorderlinesPenalizer(PenalizerABC):

    def init(
            self,
            valid_points: numpy.ndarray,
            map: numpy.ndarray,
            map_origin: numpy.ndarray,
            map_grid: float,
            map_last: numpy.ndarray,
            group_centers: numpy.ndarray,
            **kwargs) -> Optional[Dict[str, Any]]:
        """Initialize penalizer.

        Arguments:
        valid_points -- valid area of the track, nx2 numpy.ndarray
        map -- map of the environment, mxp numpy.ndarray
        map_origin -- origin of the map, 1x2 numpy.ndarray
        map_grid -- size of the map grid, float
        map_last -- last map of the environment, mxp numpy.ndarray
        group_centers -- centers of the created groups, qx2 numpy.ndarray

        Note: All three mandatory arguments can be received from the segmentators,
        if they use discrete map.
        """
        global MAP, MAP_ORIGIN, MAP_GRID, BORDERLINES

        MAP = map_last.copy()
        MAP_ORIGIN = map_origin
        MAP_GRID = map_grid

        BORDERLINES = borderlinesCreate(valid_points, group_centers)


    def penalize(
            self,
            points: numpy.ndarray,
            candidate: List[numpy.ndarray],
            valid_points: numpy.ndarray,
            grid: float,
            penalty: float = 100,
            **overflown) -> float:
        """Get a penalty for the candidate solution.

        Penalty is based on the number of incorrectly placed points.

        Arguments:
        points -- points to be checked, nx(>=2) numpy.ndarray
        candidate -- raw candidate (non-interpolated points),
                     m-list of 1x2 numpy.ndarray
        valid_points -- valid area of the track, px2 numpy.ndarray
        grid -- when set, use this value as a grid size, otherwise it is computed,
                float
        penalty -- constant used for increasing the penalty criterion,
                   float, default 100
        **overflown -- arguments not caught by previous parts

        Returns:
        rpenalty -- value of the penalty, 0 means no penalty, float
        """
        global BORDERLINES

        # Use the grid or compute it
        _grid = grid if grid else gridCompute(points)

        # Mapping between the candidate points and their interpolation
        _points_line_mapping = [
            numpy.argmin(
                numpy.sqrt(
                    numpy.sum(
                        numpy.power(
                            numpy.subtract(
                                points[:, :2],
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
        self.INVALID_POINTS.clear()

        for _ip, _p in enumerate(points):
            if not numpy.any(
                numpy.all(
                    numpy.abs(numpy.subtract(valid_points, _p[:2])) < _grid,
                    axis = 1
                )
            ):

                # Store invalid point
                self.INVALID_POINTS.append(_p)

                # Note: Trying borderlines here, it works the same,
                #       just the meaning of 'invalid' is different.
                # Note: We used to have '<' here, however that failed
                #       with invalid index 0.
                _segment_id = len(
                    [_plm for _plm in _points_line_mapping if _plm <= _ip]
                ) - 1

                _invalid = numpy.max(
                    numpy.sqrt(
                        numpy.sum(
                            numpy.power(
                                numpy.subtract(
                                    BORDERLINES[_segment_id][
                                        (_segment_id + 1) % len(candidate)
                                    ],
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
