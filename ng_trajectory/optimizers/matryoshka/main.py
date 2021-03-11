#!/usr/bin/env python3.6
# main.py
"""Interface for Matryoshka mapping."""
######################
# Imports & Globals
######################

from ng_trajectory.utils import *

from . import transform

import nevergrad

import sys

# Parallel computing of genetic algorithm
from concurrent import futures

# Thread lock for log file
from threading import Lock

# Typing
from typing import Callable, Dict, TextIO


# Global variables
OPTIMIZER = None
MATRYOSHKA = None
VALID_POINTS = None
CRITERION = None
CRITERION_ARGS = None
LOGFILE = None
VERBOSITY = 3
FILELOCK = Lock()
HOLDMAP = None


######################
# Functions
######################

def init(points: numpy.ndarray, group_centers: numpy.ndarray, group_centerline: numpy.ndarray, \
        budget: int = 100,
        layers: int = 5,
        groups: int = 8,
        workers: int = 4,
        criterion: Callable[[numpy.ndarray], float] = lambda x: 0,
        criterion_args: Dict[str, any] = {},
        logfile: TextIO = sys.stdout,
        logging_verbosity: int = 2,
        hold_map: bool = False,
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
    criterion -- function to evaluate current criterion, callable (mx2 numpy.ndarray -> float),
                 default 'static 0'
    criterion_args -- arguments for the criterion function, dict, default {}
    logfile -- file descriptor for logging, TextIO, default sys.stdout
    logging_verbosity -- index for verbosity of logger, int, default 2
    hold_map -- whether the map should be created only once, bool, default False
    **kwargs -- arguments not caught by previous parts
    """
    global OPTIMIZER, MATRYOSHKA, VALID_POINTS, CRITERION, CRITERION_ARGS, LOGFILE, VERBOSITY, HOLDMAP

    # Local to global variables
    CRITERION = criterion
    CRITERION_ARGS = criterion_args
    LOGFILE = logfile
    VERBOSITY = logging_verbosity
    _HOLDMAP = hold_map

    if HOLDMAP is None or not _HOLDMAP:
        mapCreate(points)

    HOLDMAP = _HOLDMAP


    VALID_POINTS = points
    group_centers = trajectoryReduce(trajectorySort(group_centerline), groups)

    # Matryoshka construction
    groups = pointsToGroups(points, group_centers)
    grouplayers = groupsBorderObtain(groups)
    grouplayers = groupsBorderBeautify(grouplayers, 400)
    layers_center = groupsCenterCompute(groups)
    layers_count = [ layers for i in range(len(grouplayers)) ]

    
    MATRYOSHKA = [ transform.matryoshkaCreate(grouplayers[_i], layers_center[_i], layers_count[_i]) for _i in range(len(groups)) ]


    # Optimizer definition
    instrum = nevergrad.Instrumentation(nevergrad.var.Array(len(groups), 2).bounded(0, 1))
    OPTIMIZER = nevergrad.optimizers.DoubleFastGADiscreteOnePlusOne(instrumentation = instrum, budget = budget, num_workers = workers)


def optimize() -> Tuple[float, numpy.ndarray, numpy.ndarray, numpy.ndarray]:
    """Run genetic algorithm via Nevergrad.

    Returns:
    final -- best value of the criterion, float
    points -- points in the best solution in real coordinates, nx2 numpy.ndarray
    tcpoints -- points in the best solution in transformed coordinates, nx2 numpy.ndarray
    trajectory -- trajectory of the best solution in real coordinates, mx2 numpy.ndarray
    """
    global OPTIMIZER, MATRYOSHKA, LOGFILE, FILELOCK, VERBOSITY

    with futures.ProcessPoolExecutor(max_workers=OPTIMIZER.num_workers) as executor:
        recommendation = OPTIMIZER.minimize(_opt, executor=executor, batch_mode=False)

    points = [ transform.matryoshkaMap(MATRYOSHKA[i], [p])[0] for i, p in enumerate(numpy.asarray(recommendation.args[0])) ]

    final = _opt(numpy.asarray(recommendation.args[0]))

    with FILELOCK:
        if VERBOSITY > 0:
            print ("solution:%s" % str(numpy.asarray(points).tolist()), file=LOGFILE)
            print ("final:%f" % final, file=LOGFILE)

    return final, numpy.asarray(points), numpy.asarray(recommendation.args[0]), trajectoryInterpolate(numpy.asarray(points), 400)


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
    global VALID_POINTS, CRITERION, CRITERION_ARGS, MATRYOSHKA, LOGFILE, FILELOCK, VERBOSITY

    # Transform points
    points = [ transform.matryoshkaMap(MATRYOSHKA[i], [p])[0] for i, p in enumerate(points) ]

    # Interpolate received points
    # It is expected that they are unique and sorted.
    _points = trajectoryInterpolate(numpy.asarray(points), 400)

    # Check if all interpolated points are valid
    # Note: This is required for low number of groups.
    invalid = 0

    for _p in _points:
        if not numpy.any(numpy.all(numpy.abs( numpy.subtract(VALID_POINTS, _p[:2]) ) < [ 0.05, 0.05 ], axis = 1)):
            invalid += 1

    if ( invalid > 0 ):
        with FILELOCK:
            if VERBOSITY > 2:
                print ("pointsA:%s" % str(points), file=LOGFILE)
                print ("pointsT:%s" % str(_points.tolist()), file=LOGFILE)
            if VERBOSITY > 1:
                print ("invalid:%f" % invalid, file=LOGFILE)
            LOGFILE.flush()
        return 100 * invalid

    _c = CRITERION(**{**{'points': _points}, **CRITERION_ARGS})
    with FILELOCK:
        if VERBOSITY > 2:
            print ("pointsA:%s" % str(points), file=LOGFILE)
            print ("pointsT:%s" % str(_points.tolist()), file=LOGFILE)
        if VERBOSITY > 1:
            print ("correct:%f" % _c, file=LOGFILE)
        LOGFILE.flush()

    return _c
