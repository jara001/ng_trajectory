#!/usr/bin/env python3.6
# main.py
"""Interface for Braghin's transformation."""
######################
# Imports & Globals
######################

from ng_trajectory.interpolators.utils import *
import ng_trajectory.plot as ngplot

from ng_trajectory.segmentators.utils import gridCompute

from . import transform

import nevergrad

import sys

# Parallel computing of genetic algorithm
from concurrent import futures

# Thread lock for log file
from threading import Lock

# Typing
from typing import Tuple, Callable, Dict, TextIO, List


# Global variables
OPTIMIZER = None
CUTS = None
VALID_POINTS = None
CRITERION = None
CRITERION_ARGS = None
INTERPOLATOR = None
INTERPOLATOR_ARGS = None
SEGMENTATOR = None
SEGMENTATOR_ARGS = None
SELECTOR = None
SELECTOR_ARGS = None
LOGFILE = None
VERBOSITY = 3
FILELOCK = Lock()
HOLDMAP = None
GRID = None
PENALTY = None


######################
# Functions
######################

def init(points: numpy.ndarray, group_centers: numpy.ndarray, group_centerline: numpy.ndarray, \
        budget: int = 100,
        groups: int = 8,
        workers: int = 4,
        penalty: float = 100,
        criterion: Callable[[numpy.ndarray], float] = lambda x: 0,
        criterion_args: Dict[str, any] = {},
        interpolator: Callable[[numpy.ndarray], numpy.ndarray] = lambda x: x,
        interpolator_args: Dict[str, any] = {},
        segmentator: Callable[[numpy.ndarray, numpy.ndarray], numpy.ndarray] = lambda x, y: [ x for i in y ],
        segmentator_args: Dict[str, any] = {},
        selector: Callable[[numpy.ndarray, int], numpy.ndarray] = lambda x, y: [ x for i in range(y) ],
        selector_args: Dict[str, any] = {},
        logfile: TextIO = sys.stdout,
        logging_verbosity: int = 2,
        hold_transform: bool = False,
        plot: bool = False,
        plot_cuts: bool = True,
        plot_reduced_line: bool = False,
        endpoint_distance: float = 0.2,
        endpoint_accuracy: float = 0.02,
        line_reduction: float = 3,
        grid: List[float] = [],
        **kwargs):
    """Initialize variables for Braghin's transformation.

    Arguments:
    points -- valid area of the track, nx2 numpy.ndarray
    group_centers -- points for centers of individual groups, mx2 numpy.ndarray
    group_centerline -- line where the group centers lie, px2 numpy.ndarray
    budget -- number of generations of genetic algorithm, int, default 100
    groups -- number of groups to segmentate the track into, int, default 8
    workers -- number of threads for GA, int, default 4
    penalty -- constant used for increasing the penalty criterion, float, default 100
    criterion -- function to evaluate current criterion, callable (mx2 numpy.ndarray -> float),
                 default 'static 0'
    criterion_args -- arguments for the criterion function, dict, default {}
    interpolator -- function to interpolate points, callable (mx2 numpy.ndarray -> qx2 numpy.ndarray),
                 default 'return the same'
    interpolator_args -- arguments for the interpolation function, dict, default {}
    segmentator -- function to segmentate points, callable (nx2 numpy.ndarray -> m-list of rx2 numpy.ndarray),
                   default 'each segment span over the whole track'
    segmentator_args -- arguments for the segmentation function, dict, default {}
    selector -- function to select points as group centers,
                callable (nx2 numpy.ndarray + m -> m-list of rx2 numpy.ndarray),
                default 'first m points are selected'
    selector_args -- arguments for the selector function, dict, default {}
    logfile -- file descriptor for logging, TextIO, default sys.stdout
    logging_verbosity -- index for verbosity of logger, int, default 2
    hold_transform -- whether the transformation should be created only once, bool, default False
    plot -- whether a graphical representation should be created, bool, default False
    plot_cuts -- whether cuts should be plotted if plot is enabled, bool, default True
    plot_reduced_line -- whether reduced line should be plotted if plot is enabled, bool, default False
    endpoint_distance -- starting distance from the center for transformation, float, default 0.2
    endpoint_accuracy -- accuracy of the center-endpoint distance for transformation, float, default 0.02
    line_reduction -- factor by which the number of line points is lowered before internal interpolation, float, default 3
    grid -- size of the grid used for the points discretization, 2-float List, computed by default
    **kwargs -- arguments not caught by previous parts
    """
    global OPTIMIZER, CUTS, VALID_POINTS, LOGFILE, VERBOSITY, GRID, PENALTY
    global CRITERION, CRITERION_ARGS, INTERPOLATOR, INTERPOLATOR_ARGS, SEGMENTATOR, SEGMENTATOR_ARGS, SELECTOR, SELECTOR_ARGS

    # Local to global variables
    CRITERION = criterion
    CRITERION_ARGS = criterion_args
    INTERPOLATOR = interpolator
    INTERPOLATOR_ARGS = interpolator_args
    SEGMENTATOR = segmentator
    SEGMENTATOR_ARGS = segmentator_args
    SELECTOR = selector
    SELECTOR_ARGS = selector_args
    LOGFILE = logfile
    VERBOSITY = logging_verbosity
    _holdtransform = hold_transform
    PENALTY = penalty


    VALID_POINTS = points

    if CUTS is None or not _holdtransform:

        # Transform construction
        group_centers_ = SELECTOR(**{**{"points": group_centerline, "remain": groups}, **SELECTOR_ARGS})
        CUTS = transform.create(points, group_centerline, group_centers_, endpoint_distance, endpoint_accuracy, line_reduction)

        if plot:
            if plot_cuts:
                for cut in CUTS:
                    ngplot.pointsPlot(cut, color="indigo")

                    # New center point
                    ngplot.pointsScatter(
                        (numpy.divide(cut[1, :] - cut[0, :], 2) + cut[0, :])[:, numpy.newaxis].T
                    )

            if plot_reduced_line:
                i, i1, i2 = transform.pointsInterpolate(transform.trajectoryReduce(group_centerline, int(len(group_centerline)/line_reduction)), len(group_centerline))
                ngplot.pointsPlot(numpy.asarray(i))

        print ("Braghin's transformation constructed.")

        if GRID is None:
            if len(grid) == 2:
                GRID = grid
            else:
                _GRID = gridCompute(points)
                GRID = [ _GRID, _GRID ]


    # Optimizer definition
    instrum = nevergrad.Instrumentation(nevergrad.var.Array(groups, 1).bounded(0, 1))
    OPTIMIZER = nevergrad.optimizers.DoubleFastGADiscreteOnePlusOne(instrumentation = instrum, budget = budget, num_workers = workers)


def optimize() -> Tuple[float, numpy.ndarray, numpy.ndarray, numpy.ndarray]:
    """Run genetic algorithm via Nevergrad.

    Returns:
    final -- best value of the criterion, float
    points -- points in the best solution in real coordinates, nx2 numpy.ndarray
    tcpoints -- points in the best solution in transformed coordinates, nx2 numpy.ndarray
    trajectory -- trajectory of the best solution in real coordinates, mx2 numpy.ndarray
    """
    global OPTIMIZER, CUTS, LOGFILE, FILELOCK, VERBOSITY, INTERPOLATOR, INTERPOLATOR_ARGS

    with futures.ProcessPoolExecutor(max_workers=OPTIMIZER.num_workers) as executor:
        recommendation = OPTIMIZER.minimize(_opt, executor=executor, batch_mode=False)

    points = transform.transform(recommendation.args[0], CUTS)

    final = _opt(numpy.asarray(recommendation.args[0]))

    with FILELOCK:
        if VERBOSITY > 0:
            print ("solution:%s" % str(numpy.asarray(points).tolist()), file=LOGFILE)
            print ("final:%f" % final, file=LOGFILE)

    return final, numpy.asarray(points), numpy.asarray(recommendation.args[0]), INTERPOLATOR(**{**{"points": numpy.asarray(points)}, **INTERPOLATOR_ARGS})


def _opt(points: numpy.ndarray) -> float:
    """Interpolate points, verify feasibility and calculate criterion.

    Function to be optimized.

    Arguments:
    points -- selected points to interpolate, nx2 numpy.ndarray

    Returns:
    _c -- criterion value, float

    Note: It receives selected points from the groups (see callback_centerline).

    Note: The result of this function is 'criterion' when possible, otherwise
    number of invalid points (multiplied by some value) is returned.

    Note: This function is called after all necessary data is received.
    """
    global VALID_POINTS, CRITERION, CRITERION_ARGS, INTERPOLATOR, INTERPOLATOR_ARGS, CUTS, LOGFILE, FILELOCK, VERBOSITY, GRID, PENALTY

    # Transform points
    points = transform.transform(points, CUTS)

    # Interpolate received points
    # It is expected that they are unique and sorted.
    _points = INTERPOLATOR(**{**{"points": numpy.asarray(points)}, **INTERPOLATOR_ARGS})

    # Check if all interpolated points are valid
    # Note: This is required for low number of groups.
    invalid = 0

    for _p in _points:
        if not numpy.any(numpy.all(numpy.abs( numpy.subtract(VALID_POINTS, _p[:2]) ) < GRID, axis = 1)):
            invalid += 1

    if ( invalid > 0 ):
        with FILELOCK:
            if VERBOSITY > 2:
                print ("pointsA:%s" % str(points), file=LOGFILE)
                print ("pointsT:%s" % str(_points.tolist()), file=LOGFILE)
            if VERBOSITY > 1:
                print ("invalid:%f" % invalid, file=LOGFILE)
            LOGFILE.flush()
        return PENALTY * invalid

    _c = CRITERION(**{**{'points': _points}, **CRITERION_ARGS})
    with FILELOCK:
        if VERBOSITY > 2:
            print ("pointsA:%s" % str(points), file=LOGFILE)
            print ("pointsT:%s" % str(_points.tolist()), file=LOGFILE)
        if VERBOSITY > 1:
            print ("correct:%f" % _c, file=LOGFILE)
        LOGFILE.flush()

    return _c
