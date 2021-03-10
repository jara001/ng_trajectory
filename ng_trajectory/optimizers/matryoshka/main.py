#!/usr/bin/env python3.6
# matryoshka.py
"""Entrypoint for ng_trajectory."""
######################
# Imports & Globals
######################

from ng_trajectory.utils import *

from . import transform

import nevergrad

# Parallel computing of genetic algorithm
from concurrent import futures

# Global variables
OPTIMIZER = None
MATRYOSHKA = None
VALID_POINTS = None
CRITERION = None
CRITERION_ARGS = None


######################
# Functions
######################

def init(points: numpy.ndarray, group_centers: numpy.ndarray, **kwargs):
    """Initialize variables for Matryoshka transformation."""
    global OPTIMIZER, MATRYOSHKA, VALID_POINTS, CRITERION, CRITERION_ARGS

    # Local variables
    _budget = kwargs.get("budget", 100)
    _layers = kwargs.get("layers", 5)
    _groups = kwargs.get("groups", 8)
    _workers = kwargs.get("workers", 4)
    CRITERION = globals()[kwargs.get("criterion")]
    CRITERION_ARGS = kwargs.get("criterion_args")


    mapCreate(points)
    VALID_POINTS = points

    group_centers = trajectoryReduce(trajectorySort(group_centers), _groups)

    # Matryoshka construction
    groups = pointsToGroups(points, group_centers)
    layers = groupsBorderObtain(groups)
    layers = groupsBorderBeautify(layers, 400)
    layers_center = groupsCenterCompute(groups)
    layers_count = [ 5 for i in range(len(layers)) ]

    
    MATRYOSHKA = [ transform.matryoshkaCreate(layers[_i], layers_center[_i], layers_count[_i]) for _i in range(len(groups)) ]


    # Optimizer definition
    instrum = nevergrad.Instrumentation(nevergrad.var.Array(len(groups), 2).bounded(0, 1))
    OPTIMIZER = nevergrad.optimizers.DoubleFastGADiscreteOnePlusOne(instrumentation = instrum, budget = _budget, num_workers = _workers)


def optimize() -> Tuple[float, numpy.ndarray, numpy.ndarray]:
    """Run genetic algorithm via Nevergrad.

    Arguments:
    groups -- groups of points from which the values are selected, n-list of mx2 numpy.ndarray
    budget -- number of generations of genetic algorithm, int, default 100
    marker_ns -- identification of published Markers, str, default "" (do not publish)
    criterion -- function to evaluate current criterion, callable (px2 numpy.ndarray -> float),
                 default 'profile_criterion' with 'overlap = 150'
    criterion_args -- arguments for the criterion function, dict, default {}
    **overflown -- arguments not caught by previous parts

    Returns:
    final_time -- lap time of the trajectory, float
    points -- points in the best solution in real coordinates, nx2 numpy.ndarray
    tcpoints -- points in the best solution in transformed coordinates, nx2 numpy.ndarray
    """
    global OPTIMIZER, MATRYOSHKA

    with futures.ProcessPoolExecutor(max_workers=OPTIMIZER.num_workers) as executor:
        recommendation = OPTIMIZER.minimize(_opt, executor=executor, batch_mode=False)

    points = [ transform.matryoshkaMap(MATRYOSHKA[i], [p]) for i, p in enumerate(numpy.asarray(recommendation.args[0])) ]

    final_time = _opt(numpy.asarray(recommendation.args[0]))

    return final_time, numpy.asarray(points), numpy.asarray(recommendation.args[0])


def _opt(points: numpy.ndarray) -> float:
    """Interpolate points, compute speed profile and return lap time.

    Function to be optimized.

    Arguments:
    points -- selected points to interpolate, nx2 numpy.ndarray

    Returns:
    time -- lap time, float

    Note: It receives selected points from the groups (see callback_centerline).

    Note: The result of this function is 'lap time' when possible, otherwise
    number of invalid points (multiplied by some value) is returned.

    Note: This function is called after all necessary data is received.
    """
    global VALID_POINTS, CRITERION, CRITERION_ARGS, MATRYOSHKA

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
        return 100 * invalid

    #_v, _a, _t = profileCompute(_points, 150)
    #
    #FILE.write("correct:%f\n" % float(_t[-1]))
    #return float(_t[-1])

    _c = CRITERION(**{**{'points': _points}, **CRITERION_ARGS})
    return _c
