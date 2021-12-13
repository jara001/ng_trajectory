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

import sys, os

# Parallel computing of genetic algorithm
from concurrent import futures

# Thread lock for log file
from threading import Lock

# Typing
from typing import Tuple, Callable, Dict, TextIO, List, types


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
PENALIZER = None
PENALIZER_INIT = None
PENALIZER_ARGS = None
LOGFILE = None
VERBOSITY = 3
FILELOCK = Lock()
HOLDMAP = None
GRID = None
PENALTY = None
FIGURE = None


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("budget", 100, int, "Budget parameter for the genetic algorithm.", "init (general)")
P.createAdd("groups", 8, int, "Number of groups to segmentate the track into.", "init (general)")
P.createAdd("workers", "os.cpu_count()", int, "Number threads for the genetic algorithm.", "init (general)")
P.createAdd("penalty", 100, float, "Constant used for increasing the penalty criterion.", "init (general)")
P.createAdd("criterion", None, types.ModuleType, "Module to evaluate current criterion.", "init (general)")
P.createAdd("criterion_args", {}, dict, "Arguments for the criterion function.", "init (general)")
P.createAdd("interpolator", None, types.ModuleType, "Module to interpolate points.", "init (general)")
P.createAdd("interpolator_args", {}, dict, "Arguments for the interpolator function.", "init (general)")
P.createAdd("segmentator", None, types.ModuleType, "Module to segmentate track.", "init (general)")
P.createAdd("segmentator_args", {}, dict, "Arguments for the segmentator function.", "init (general)")
P.createAdd("selector", None, types.ModuleType, "Module to select path points as segment centers.", "init (general)")
P.createAdd("selector_args", {}, dict, "Arguments for the selector function.", "init (general)")
P.createAdd("penalizer", None, types.ModuleType, "Module to evaluate penalty criterion.", "init (general)")
P.createAdd("penalizer_init", {}, dict, "Arguments for the init part of the penalizer function.", "init (general)")
P.createAdd("penalizer_args", {}, dict, "Arguments for the penalizer function.", "init (general)")
P.createAdd("logging_verbosity", 2, int, "Index for verbosity of the logger.", "init (general)")
P.createAdd("hold_transform", False, bool, "Whether the transformation should be created only once.", "init (Braghin)")
P.createAdd("plot", False, bool, "Whether a graphical representation should be created.", "init (viz.)")
P.createAdd("plot_cuts", True, bool, "Whether cuts should be plotted if plot is enabled.", "init (viz.)")
P.createAdd("plot_reduced_line", False, bool, "Whether reduced line should be plotted if plot is enabled.", "init (viz.)")
P.createAdd("endpoint_distance", 0.2, float, "Starting distance from the center for creating transformation.", "init (Braghin)")
P.createAdd("endpoint_accuracy", 0.02, float, "Accuracy of the center-endpoint distance for transformation.", "init (Braghin)")
P.createAdd("line_reduction", 3, float, "Factor by which the number of line points is lowered before internal interpolation.", "init (Braghin)")
P.createAdd("grid", "computed by default", list, "X-size and y-size of the grid used for points discretization.", "init (Braghin)")


######################
# Functions
######################

def init(points: numpy.ndarray, group_centers: numpy.ndarray, group_centerline: numpy.ndarray, \
        budget: int = 100,
        groups: int = 8,
        workers: int = os.cpu_count(),
        penalty: float = 100,
        criterion: types.ModuleType = None,
        criterion_args: Dict[str, any] = {},
        interpolator: types.ModuleType = None,
        interpolator_args: Dict[str, any] = {},
        segmentator: types.ModuleType = None,
        segmentator_args: Dict[str, any] = {},
        selector: types.ModuleType = None,
        selector_args: Dict[str, any] = {},
        penalizer: types.ModuleType = None,
        penalizer_init: Dict[str, any] = {},
        penalizer_args: Dict[str, any] = {},
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
        figure: ngplot.matplotlib.figure.Figure = None,
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
    criterion -- module to evaluate current criterion,
                 module with compute callable (mx2 numpy.ndarray -> float),
                 default None
    criterion_args -- arguments for the criterion function, dict, default {}
    interpolator -- module to interpolate points,
                    module with interpolate callable (mx2 numpy.ndarray -> qx2 numpy.ndarray),
                    default None
    interpolator_args -- arguments for the interpolation function, dict, default {}
    segmentator -- module to segmentate points,
                   module with segmentate callable (nx2 numpy.ndarray -> m-list of rx2 numpy.ndarray),
                   default None
    segmentator_args -- arguments for the segmentation function, dict, default {}
    selector -- module to select points as group centers,
                module with select callable (nx2 numpy.ndarray + m -> m-list of rx2 numpy.ndarray),
                default None
    selector_args -- arguments for the selector function, dict, default {}
    penalizer -- module to evaluate penalty criterion,
                 module with penalize callable (nx(>=2) numpy.ndarray + mx2 numpy.ndarray -> float),
                 default None
    penalizer_init -- arguments for the init part of the penalizer function, dict, default {}
    penalizer_args -- arguments for the penalizer function, dict, default {}
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
    figure -- target figure for plotting, matplotlib.figure.Figure, default None (get current)
    **kwargs -- arguments not caught by previous parts
    """
    global OPTIMIZER, CUTS, VALID_POINTS, LOGFILE, VERBOSITY, GRID, PENALTY, FIGURE
    global CRITERION, CRITERION_ARGS, INTERPOLATOR, INTERPOLATOR_ARGS, SEGMENTATOR, SEGMENTATOR_ARGS, SELECTOR, SELECTOR_ARGS, PENALIZER, PENALIZER_INIT, PENALIZER_ARGS

    # Local to global variables
    CRITERION = criterion
    CRITERION_ARGS = criterion_args
    INTERPOLATOR = interpolator
    INTERPOLATOR_ARGS = interpolator_args
    SEGMENTATOR = segmentator
    SEGMENTATOR_ARGS = segmentator_args
    SELECTOR = selector
    SELECTOR_ARGS = selector_args
    PENALIZER = penalizer
    PENALIZER_INIT = penalizer_init
    PENALIZER_ARGS = penalizer_args
    LOGFILE = logfile
    VERBOSITY = logging_verbosity
    _holdtransform = hold_transform
    PENALTY = penalty
    FIGURE = figure


    VALID_POINTS = points

    if CUTS is None or not _holdtransform:

        # Transform construction
        group_centers_ = SELECTOR.select(**{**{"points": group_centerline, "remain": groups}, **SELECTOR_ARGS})
        CUTS = transform.create(points, group_centerline, group_centers_, endpoint_distance, endpoint_accuracy, line_reduction)

        # Call init part of the penalizer
        # TODO: Check whether this works.
        PENALIZER.init(
            valid_points = VALID_POINTS,
            start_points = group_centerline,
            map = SEGMENTATOR.main.MAP,
            map_origin = SEGMENTATOR.main.MAP_ORIGIN,
            map_grid = SEGMENTATOR.main.MAP_GRID,
            map_last = SEGMENTATOR.main.MAP_LAST,
            group_centers = group_centers,
            **{**PENALIZER_INIT}
        )

        if plot:
            if plot_cuts:
                for cut in CUTS:
                    ngplot.pointsPlot(cut, figure=figure, color="indigo")

                    # New center point
                    ngplot.pointsScatter(
                        (numpy.divide(cut[1, :] - cut[0, :], 2) + cut[0, :])[:, numpy.newaxis].T,
                        figure=figure
                    )

            if plot_reduced_line:
                i, i1, i2 = transform.pointsInterpolate(transform.trajectoryReduce(group_centerline, int(len(group_centerline)/line_reduction)), len(group_centerline))
                ngplot.pointsPlot(numpy.asarray(i), figure=figure)

        print ("Braghin's transformation constructed.")

        if GRID is None:
            if len(grid) == 2:
                GRID = grid
            else:
                _GRID = gridCompute(points)
                GRID = [ _GRID, _GRID ]


    # Optimizer definition
    instrum = nevergrad.Instrumentation(nevergrad.var.Array(len(CUTS), 1).bounded(0, 1))
    OPTIMIZER = nevergrad.optimizers.DoubleFastGADiscreteOnePlusOne(instrumentation = instrum, budget = budget, num_workers = workers)


def optimize() -> Tuple[float, numpy.ndarray, numpy.ndarray, numpy.ndarray]:
    """Run genetic algorithm via Nevergrad.

    Returns:
    final -- best value of the criterion, float
    points -- points in the best solution in real coordinates, nx2 numpy.ndarray
    tcpoints -- points in the best solution in transformed coordinates, nx2 numpy.ndarray
    trajectory -- trajectory of the best solution in real coordinates, mx2 numpy.ndarray
    """
    global OPTIMIZER, CUTS, LOGFILE, FILELOCK, VERBOSITY, INTERPOLATOR, INTERPOLATOR_ARGS, FIGURE

    with futures.ProcessPoolExecutor(max_workers=OPTIMIZER.num_workers) as executor:
        recommendation = OPTIMIZER.minimize(_opt, executor=executor, batch_mode=False)

    points = transform.transform(recommendation.args[0], CUTS)

    final = _opt(numpy.asarray(recommendation.args[0]))


    ## Plot invalid points if available

    # Transform points
    points = transform.transform(recommendation.args[0], CUTS)

    # Interpolate received points
    # It is expected that they are unique and sorted.
    _points = INTERPOLATOR.interpolate(**{**{"points": numpy.asarray(points)}, **INTERPOLATOR_ARGS})

    # Check if all interpolated points are valid
    # Note: This is required for low number of groups.
    invalid = []

    for _p in _points:
        if not numpy.any(numpy.all(numpy.abs( numpy.subtract(VALID_POINTS, _p[:2]) ) < GRID, axis = 1)):
            invalid.append(_p)

    if len(invalid) > 0:
        ngplot.pointsScatter(numpy.asarray(invalid), FIGURE, color="red", marker="x")


    ##

    with FILELOCK:
        if VERBOSITY > 0:
            print ("solution:%s" % str(numpy.asarray(points).tolist()), file=LOGFILE)
            print ("final:%f" % final, file=LOGFILE)

    return final, numpy.asarray(points), numpy.asarray(recommendation.args[0]), INTERPOLATOR.interpolate(**{**{"points": numpy.asarray(points)}, **INTERPOLATOR_ARGS})


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
    global VALID_POINTS, CRITERION, CRITERION_ARGS, INTERPOLATOR, INTERPOLATOR_ARGS, PENALIZER, PENALIZER_ARGS
    global CUTS, LOGFILE, FILELOCK, VERBOSITY, GRID, PENALTY

    # Transform points
    points = transform.transform(points, CUTS)

    # Interpolate received points
    # It is expected that they are unique and sorted.
    _points = INTERPOLATOR.interpolate(**{**{"points": numpy.asarray(points)}, **INTERPOLATOR_ARGS})

    # Check if all interpolated points are valid and compute penalty
    # Note: This is required for low number of groups.
    penalty = PENALIZER.penalize(**{**{"points": _points, "valid_points": VALID_POINTS, "grid": GRID, "penalty": PENALTY, "candidate": points}, **PENALIZER_ARGS})

    if ( penalty != 0 ):
        with FILELOCK:
            if VERBOSITY > 2:
                print ("pointsA:%s" % str(points), file=LOGFILE)
                print ("pointsT:%s" % str(_points.tolist()), file=LOGFILE)
            if VERBOSITY > 1:
                print ("penalty:%f" % penalty, file=LOGFILE)
            LOGFILE.flush()
        return penalty

    _c = CRITERION.compute(**{**{'points': _points}, **CRITERION_ARGS})
    with FILELOCK:
        if VERBOSITY > 2:
            print ("pointsA:%s" % str(points), file=LOGFILE)
            print ("pointsT:%s" % str(_points.tolist()), file=LOGFILE)
        if VERBOSITY > 1:
            print ("correct:%f" % _c, file=LOGFILE)
        LOGFILE.flush()

    return _c
