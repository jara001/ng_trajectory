#!/usr/bin/env python3.6
# main.py
"""Interface for Matryoshka mapping."""
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
from typing import Tuple, Callable, Dict, TextIO, List


# Global variables
OPTIMIZER = None
MATRYOSHKA = None
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
PENALIZER_ARGS = None
LOGFILE = None
VERBOSITY = 3
FILELOCK = Lock()
HOLDMAP = None
GRID = None
PENALTY = None
FIGURE = None
PLOT = None
USE_BORDERLINES = None
BORDERLINES = None


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("budget", 100, int, "Budget parameter for the genetic algorithm.", "init (general)")
P.createAdd("groups", 8, int, "Number of groups to segmentate the track into.", "init (general)")
P.createAdd("workers", "os.cpu_count()", int, "Number threads for the genetic algorithm.", "init (general)")
P.createAdd("penalty", 100, float, "Constant used for increasing the penalty criterion.", "init (general)")
P.createAdd("criterion", "static 0", callable, "Function to evaluate current criterion.", "init (general)")
P.createAdd("criterion_args", {}, dict, "Arguments for the criterion function.", "init (general)")
P.createAdd("interpolator", "return the same", callable, "Function to interpolate points.", "init (general)")
P.createAdd("interpolator_args", {}, dict, "Arguments for the interpolator function.", "init (general)")
P.createAdd("segmentator", "each segment span over the whole track", callable, "Function to segmentate track.", "init (general)")
P.createAdd("segmentator_args", {}, dict, "Arguments for the segmentator function.", "init (general)")
P.createAdd("selector", "first m points are selected", callable, "Function to select path points as segment centers.", "init (general)")
P.createAdd("selector_args", {}, dict, "Arguments for the selector function.", "init (general)")
P.createAdd("penalizer", "0 is returned", callable, "Function to evaluate penalty criterion.", "init (general)")
P.createAdd("penalizer_args", {}, dict, "Arguments for the penalizer function.", "init (general)")
P.createAdd("logging_verbosity", 2, int, "Index for verbosity of the logger.", "init (general)")
P.createAdd("hold_matryoshka", False, bool, "Whether the transformation should be created only once.", "init (Matryoshka)")
P.createAdd("plot", False, bool, "Whether a graphical representation should be created.", "init (viz.)")
P.createAdd("grid", "computed by default", list, "X-size and y-size of the grid used for points discretization.", "init (Matryoshka)")
P.createAdd("use_borderlines", False, bool, "Whether to use borderlines for calculating the penalty.", "init (Matryoshka)")


######################
# Functions
######################

def init(points: numpy.ndarray, group_centers: numpy.ndarray, group_centerline: numpy.ndarray, \
        budget: int = 100,
        layers: int = 5,
        groups: int = 8,
        workers: int = os.cpu_count(),
        penalty: float = 100,
        criterion: Callable[[numpy.ndarray], float] = lambda x: 0,
        criterion_args: Dict[str, any] = {},
        interpolator: Callable[[numpy.ndarray], numpy.ndarray] = lambda x: x,
        interpolator_args: Dict[str, any] = {},
        segmentator: Callable[[numpy.ndarray, numpy.ndarray], numpy.ndarray] = lambda x, y: [ x for i in y ],
        segmentator_args: Dict[str, any] = {},
        selector: Callable[[numpy.ndarray, int], numpy.ndarray] = lambda x, y: [ x for i in range(y) ],
        selector_args: Dict[str, any] = {},
        penalizer: Callable[[numpy.ndarray, numpy.ndarray], float] = lambda x, y: 0,
        penalizer_args: Dict[str, any] = {},
        logfile: TextIO = sys.stdout,
        logging_verbosity: int = 2,
        hold_matryoshka: bool = False,
        plot: bool = False,
        grid: List[float] = [],
        use_borderlines: bool = False,
        figure: ngplot.matplotlib.figure.Figure = None,
        **kwargs):
    """Initialize variables for Matryoshka transformation.

    Arguments:
    points -- valid area of the track, nx2 numpy.ndarray
    group_centers -- points for centers of individual groups, mx2 numpy.ndarray
    group_centerline -- line where the group centers lie, px2 numpy.ndarray
    budget -- number of generations of genetic algorithm, int, default 100
    layers -- number of layers for each Matryoshka, int, default 5
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
    penalizer -- function to evaluate penalty criterion,
                callable (nx(>=2) numpy.ndarray + mx2 numpy.ndarray -> float),
                default '0 is returned'
    penalizer_args -- arguments for the penalizer function, dict, default {}
    logfile -- file descriptor for logging, TextIO, default sys.stdout
    logging_verbosity -- index for verbosity of logger, int, default 2
    hold_matryoshka -- whether the Matryoshka should be created only once, bool, default False
    plot -- whether a graphical representation should be created, bool, default False
    grid -- size of the grid used for the points discretization, 2-float List, computed by default
    use_borderlines -- whether to use borderlines for calculating the penalty, bool, default False
    figure -- target figure for plotting, matplotlib.figure.Figure, default None (get current)
    **kwargs -- arguments not caught by previous parts
    """
    global OPTIMIZER, MATRYOSHKA, VALID_POINTS, LOGFILE, VERBOSITY, HOLDMAP, GRID, PENALTY, FIGURE, PLOT, USE_BORDERLINES, BORDERLINES
    global CRITERION, CRITERION_ARGS, INTERPOLATOR, INTERPOLATOR_ARGS, SEGMENTATOR, SEGMENTATOR_ARGS, SELECTOR, SELECTOR_ARGS, PENALIZER, PENALIZER_ARGS

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
    PENALIZER_ARGS = penalizer_args
    LOGFILE = logfile
    VERBOSITY = logging_verbosity
    _holdmatryoshka = hold_matryoshka
    PENALTY = penalty
    FIGURE = figure
    PLOT = plot
    USE_BORDERLINES = use_borderlines


    VALID_POINTS = points

    if MATRYOSHKA is None or not _holdmatryoshka:
        # Note: In version <=1.3.0 the group_centerline passed to the SELECTOR was sorted using
        #       ng_trajectory.interpolators.utils.trajectorySort, but it sometimes rotated the
        #       already sorted centerline; interestingly, the result was counterclockwise at all
        #       times (or at least very very often).
        group_centers = SELECTOR(**{**{"points": group_centerline, "remain": groups}, **SELECTOR_ARGS})

        if plot:
            ngplot.indicesPlot(group_centers)

        # Matryoshka construction
        _groups = SEGMENTATOR(points=points, group_centers=group_centers, create_borderlines=USE_BORDERLINES, **{**SEGMENTATOR_ARGS})

        if type(_groups) == tuple:
            BORDERLINES = _groups[1]
            _groups = _groups[0]
        elif USE_BORDERLINES:
            print ("Borderlines requested but not received from the segmentator. Switching this off.")
            USE_BORDERLINES = False

        grouplayers = transform.groupsBorderObtain(_groups)
        grouplayers = transform.groupsBorderBeautify(grouplayers, 400)

        if plot:
            ngplot.bordersPlot(grouplayers, figure)

        layers_center = transform.groupsCenterCompute(_groups)
        layers_count = [ layers for i in range(len(grouplayers)) ]


        MATRYOSHKA = [ transform.matryoshkaCreate(grouplayers[_i], layers_center[_i], layers_count[_i]) for _i in range(len(_groups)) ]

        print ("Matryoshka mapping constructed.")

        if GRID is None:
            if len(grid) == 2:
                GRID = grid
            else:
                _GRID = gridCompute(points)
                GRID = [ _GRID, _GRID ]


    # Optimizer definition
    instrum = nevergrad.Instrumentation(nevergrad.var.Array(len(MATRYOSHKA), 2).bounded(0, 1))
    OPTIMIZER = nevergrad.optimizers.DoubleFastGADiscreteOnePlusOne(instrumentation = instrum, budget = budget, num_workers = workers)


def optimize() -> Tuple[float, numpy.ndarray, numpy.ndarray, numpy.ndarray]:
    """Run genetic algorithm via Nevergrad.

    Returns:
    final -- best value of the criterion, float
    points -- points in the best solution in real coordinates, nx2 numpy.ndarray
    tcpoints -- points in the best solution in transformed coordinates, nx2 numpy.ndarray
    trajectory -- trajectory of the best solution in real coordinates, mx2 numpy.ndarray
    """
    global OPTIMIZER, MATRYOSHKA, LOGFILE, FILELOCK, VERBOSITY, INTERPOLATOR, INTERPOLATOR_ARGS, FIGURE, PLOT

    with futures.ProcessPoolExecutor(max_workers=OPTIMIZER.num_workers) as executor:
        recommendation = OPTIMIZER.minimize(_opt, executor=executor, batch_mode=False)

    points = [ transform.matryoshkaMap(MATRYOSHKA[i], [p])[0] for i, p in enumerate(numpy.asarray(recommendation.args[0])) ]

    final = _opt(numpy.asarray(recommendation.args[0]))


    ## Plot invalid points if available

    # Interpolate received points
    # It is expected that they are unique and sorted.
    _points = INTERPOLATOR(**{**{"points": numpy.asarray(points)}, **INTERPOLATOR_ARGS})

    # Check if all interpolated points are valid
    # Note: This is required for low number of groups.
    invalid = []

    for _p in _points:
        if not numpy.any(numpy.all(numpy.abs( numpy.subtract(VALID_POINTS, _p[:2]) ) < GRID, axis = 1)):
            invalid.append(_p)

    if PLOT and len(invalid) > 0:
        ngplot.pointsScatter(numpy.asarray(invalid), FIGURE, color="red", marker="x")


    ##

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
    global VALID_POINTS, CRITERION, CRITERION_ARGS, INTERPOLATOR, INTERPOLATOR_ARGS, PENALIZER, PENALIZER_ARGS
    global MATRYOSHKA, LOGFILE, FILELOCK, VERBOSITY, GRID, PENALTY, USE_BORDERLINES, BORDERLINES

    # Transform points
    points = [ transform.matryoshkaMap(MATRYOSHKA[i], [p])[0] for i, p in enumerate(points) ]

    # Interpolate received points
    # It is expected that they are unique and sorted.
    _points = INTERPOLATOR(**{**{"points": numpy.asarray(points)}, **INTERPOLATOR_ARGS})

    # Check the correctness of the points and compute penalty
    penalty = PENALIZER(**{**{"points": _points, "valid_points": VALID_POINTS, "grid": GRID, "penalty": PENALTY}, **PENALIZER_ARGS})

    if ( penalty != 0 ):
        with FILELOCK:
            if VERBOSITY > 2:
                print ("pointsA:%s" % str(points), file=LOGFILE)
                print ("pointsT:%s" % str(_points.tolist()), file=LOGFILE)
            if VERBOSITY > 1:
                print ("penalty:%f" % penalty, file=LOGFILE)
            LOGFILE.flush()
        return penalty

    _c = CRITERION(**{**{'points': _points}, **CRITERION_ARGS})
    with FILELOCK:
        if VERBOSITY > 2:
            print ("pointsA:%s" % str(points), file=LOGFILE)
            print ("pointsT:%s" % str(_points.tolist()), file=LOGFILE)
        if VERBOSITY > 1:
            print ("correct:%f" % _c, file=LOGFILE)
        LOGFILE.flush()

    return _c
