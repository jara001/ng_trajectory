#!/usr/bin/env python3.6
# main.py
"""Optimization toolbox ng_trajectory

ng_trajectory is a toolbox for solving the optimal racing line
problem using various methods and approaches. The main idea
stands on using a genetic algorithm, although other approaches
may be used as well.

Currently we distinguish 5 groups of algorithms:
1) Selectors
   Selectors take the input path and select a subset of these
   points.
2) Segmentators
   Segmentators split the track into segments using the selected
   point subset.
3) Interpolators
   Interpolators are used for interpolating the selected point
   subset in order to get, e.g., curvature.
4) Optimizers
   Optimizers take the data from the previous three parts and
   use them to find the optimal racing line.
5) Criterions
   Criterions are used for obtaining the fitness value given
   the waypoints.
6) Penalizers
   Penalizers are used to decide whether the candidate is invalid,
   and in that case compute its penalty.

The whole application does run multiple times:
 - variating "variate" parameter,
 - repeating "loop" times,
 - optimization "cascade".

The configuration file is using "parent-nesting" parameter
handling. This means that the parameter defined on the top level
is visible in lower levels (i.e., instead of specifying
segmentator for each part of the cascade, it can be set on the
top level).

Minimal version of the configuration:
{
    "_version": 2,
    "loops": 1,
    "groups": 20,
    "interpolator": "cubic_spline",
    "segmentator": "flood_fill",
    "selector": "uniform",
    "cascade": [
        {
            "algorithm": "matryoshka",
            "budget": 10,
            "layers": 5,
            "criterion": "profile",
            "criterion_args": {
                "overlap": 100
            }
        }
    ],
    "start_points": "start_points.npy",
    "valid_points": "valid_points.npy",
    "logging_verbosity": 2
}
"""
######################
# Imports & Globals
######################

import sys, numpy, json, time

#from ng_trajectory.configuration import configurationLoad
#from ng_trajectory.configuration import CONFIGURATION
CONFIGURATION = {}

import ng_trajectory

import ng_trajectory.optimizers as optimizers
import ng_trajectory.criterions as criterions
import ng_trajectory.interpolators as interpolators
import ng_trajectory.segmentators as segmentators
import ng_trajectory.selectors as selectors
import ng_trajectory.penalizers as penalizers

import ng_trajectory.plot as plot

# Typing
from typing import Tuple, Dict
Solution = Tuple[float, numpy.ndarray, numpy.ndarray, numpy.ndarray]


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
# FIXME: Actually, the default values do not work here as it was not adapted for ParameterList.
P.createAdd("_version", None, int, "Version of the configuration.", "General")
P.createAdd("_comment", None, str, "Commentary of the configuration file.", "General")
P.createAdd("loops", None, int, "Number of repetitions.", "Optimization")
P.createAdd("groups", None, int, "Number of segments on the track.", "Optimization")
P.createAdd("variate", None, str, "Name of the field that contains multiple values. Its values are varied, run loop-cascade times.", "Optimization")
P.createAdd("logging_verbosity", 1, int, "Index to the verbosity of used logger.", "Utility")
P.createAdd("prefix", None, str, "Prefix of the output log file. When unset, use terminal.", "Utility")
P.createAdd("cascade", None, list, "List of dicts, that is performed loops-times. Req. 'algorithm': OPTIMIZER", "General")
P.createAdd("start_points", None, str, "Name of the file with initial solution (i.e., centerline).", "General")
P.createAdd("valid_points", None, str, "Name of the file with valid positions of the track.", "General")
P.createAdd("plot", None, bool, "When true, images are plotted.", "Utility")
P.createAdd("plot_args", None, list, "List of dicts with information for plotter. 1 el. is used prior to the optimization, 2nd after.", "Utility")
P.createAdd("silent_stub", False, bool, "When set, the application does not report that an algorithm for some part is missing.", "Utility")
P.createAdd("criterion", None, str, "Name of the function to evaluate current criterion.", "Optimization")
P.createAdd("criterion_init", {}, dict, "Arguments for the init part of the criterion function.", "Optimization")
P.createAdd("criterion_args", {}, dict, "Arguments for the criterion function.", "Optimization")
P.createAdd("interpolator", None, str, "Name of the function to interpolate points.", "Optimization")
P.createAdd("interpolator_init", {}, dict, "Arguments for the init part of the interpolator function.", "Optimization")
P.createAdd("interpolator_args", {}, dict, "Arguments for the interpolator function.", "Optimization")
P.createAdd("segmentator", None, str, "Name of the function to segmentate track.", "Optimization")
P.createAdd("segmentator_init", {}, dict, "Arguments for the init part of the segmentator function.", "Optimization")
P.createAdd("segmentator_args", {}, dict, "Arguments for the segmentator function.", "Optimization")
P.createAdd("selector", None, str, "Name of the function to select path points as segment centers.", "Optimization")
P.createAdd("selector_init", {}, dict, "Arguments for the init part of the selector function.", "Optimization")
P.createAdd("selector_args", {}, dict, "Arguments for the selector function.", "Optimization")
P.createAdd("penalizer", None, str, "Name of the function to evaluate penalty criterion.", "Optimization")
P.createAdd("penalizer_init", {}, dict, "Arguments for the init part of the penalizer function.", "Optimization")
P.createAdd("penalizer_args", {}, dict, "Arguments for the penalizer function.", "Optimization")


######################
# Stub
######################

class Stub:
    """Object used when optimizer/etc. is not given."""

    def __init__(self, name, silent_stub):
        self.name = name
        self.silent_stub = silent_stub

    def __getattr__(self, attr):
        """Return dummy lambda function."""
        return lambda *x, **y: print("Called '%s' on '%s' object, but it was not specified in the configuration." % (attr, self.name), file=sys.stderr) if not self.silent_stub else None


######################
# Decorators
######################

def loop(iterator):
    """Decorator for looping functions."""

    def wrapper(function, *args, **kwags):
        def looper(elements, *args, **kwargs):
            first = True
            output = []
            for i in iterator(elements):
                if first:
                    output = function(*args, **{**kwargs, **{"loop_i": i}})
                    first = False
                else:
                    output = function(*args, **{**kwargs, **{"loop_i": i, "loop_output": output}})
            return output
        return looper
    return wrapper


######################
# Functions
######################

def configurationLoad(filename: str) -> bool:
    """Loads a configuration stored in 'filename'.

    Arguments:
    filename -- name of the file along with its path, str

    Returns:
    success -- True when loaded, otherwise False
    """
    global CONFIGURATION

    try:
        with open(filename, 'r') as configuration_file:
            conf = json.load(configuration_file)

            if conf.get("_version", 1) < 2:
                print ("Unsupported version of the configuration file.", file=sys.stderr)
            else:
                CONFIGURATION = {**CONFIGURATION, **conf}
    except Exception as e:
        print (e)
        return False

    if conf.get("logging_verbosity", 1) > 1:
        print (CONFIGURATION)

    return True


def configurationAppend(conf: Dict[str, any]) -> bool:
    """Appends configuration to the global settings.

    Arguments:
    conf -- new configuration, dict

    Returns:
    success -- True when loaded, otherwise False
    """
    global CONFIGURATION

    CONFIGURATION = {**CONFIGURATION, **conf}

    if conf.get("logging_verbosity", 1) > 1:
        print (CONFIGURATION)

    return True


def dataLoad(filename: str) -> numpy.ndarray:
    """Loads data from stored file.

    Arguments:
    filename -- name of the file, str

    Returns:
    points -- an array of loaded points, nx2 numpy.ndarray
    """
    return numpy.load(filename)


######################
# Execute functions
######################

@loop(lambda x: enumerate(x))
def cascadeRun(track: numpy.ndarray, fileformat: str, notification: str, loop_i: int, \
    loop_output: Solution, **conf) \
    -> Solution:
    """Run steps of the GA cascade.

    Arguments:
    track -- points of the track valid area, nx2 numpy.ndarray
    fileformat -- name of the logging file, str
    notification -- notification about current progress, str
    loop_i -- loop index, int
    loop_output -- initial/previous output of the cascade, 4-tuple
    **conf -- GA configuration, dict

    Returns:
    fitness -- best value of the criterion, float
    rcandidate -- points in the best solution in real coordinates, nx2 numpy.ndarray
    tcandidate -- points in the best solution in transformed coordinates, nx2 numpy.ndarray
    result -- trajectory of the best solution in real coordinates, mx2 numpy.ndarray
    """

    # Cascade step timing
    step_time = time.time()

    # Get configuration for current step
    _alg = {**conf, **loop_i[1]}

    # Rename output from previous stages
    fitness, rcandidate, tcandidate, result = loop_output

    # Check for logging file
    # If exists, we continue to next round.
    if fileformat:
        try:
            with open(fileformat % (loop_i[0]+1) + "-%s.log" % _alg.get("algorithm"), "r") as f:
                _stored = { name: json.loads(value) for name, value in [ line.split(":") for line in f.readlines() if line[0] == "#" ]}

                if _stored["#fitness"] < fitness:
                    return _stored["#fitness"], \
                           numpy.asarray(_stored["#rcandidate"]), \
                           numpy.asarray(_stored["#tcandidate"]), \
                           numpy.asarray(_stored["#trajectory"])
                else:
                    return loop_output

        # File not found, therefore we want to run this.
        except FileNotFoundError:
            pass

        # One of the values is not found, meaning we probably have corrupted log.
        # So we just to the measurement again.
        except KeyError:
            pass

    # Open up logging file
    if fileformat:
        LOGFILE = open(fileformat % (loop_i[0]+1) + "-%s.log" % _alg.get("algorithm"), "w")
        print (_alg, file=LOGFILE)
        print ("Running %s version %s" % (ng_trajectory.__name__, ng_trajectory.__version__), file=LOGFILE)
        LOGFILE.flush()
    else:
        LOGFILE = sys.stdout


    ## Initialization
    # Get optimizers etc.
    obtain = lambda group, name: group.__getattribute__(_alg.get(name)) if hasattr(group, _alg.get(name, "")) else Stub(name, _alg.get("silent_stub", False))
    opt = obtain(optimizers, "algorithm")
    cri = obtain(criterions, "criterion")
    itp = obtain(interpolators, "interpolator")
    seg = obtain(segmentators, "segmentator")
    sel = obtain(selectors, "selector")
    # Set default penalizer. Not used in 'ParameterList' as it has no effect.
    if "penalizer" not in _alg:
        _alg = {**_alg, **{"penalizer": "count"}}
    pen = obtain(penalizers, "penalizer")

    # Show up current progress
    print (notification % (loop_i[0]+1) + " %s with %s criterion (penalized by %s), int. by %s" % (_alg.get("algorithm", ""), _alg.get("criterion", ""), _alg.get("penalizer", ""), _alg.get("interpolator", "")), file=LOGFILE)
    LOGFILE.flush()

    # Prepare plot
    if _alg.get("plot", False):
        fig = plot.figureCreate()

        # Append figure
        _alg = {**_alg, **{"figure": fig}}

        plot.plotDyn(_alg.get("plot_args", [])[0], **{**_alg, **{"track": track, "fitness": fitness, "rcandidate": rcandidate, "tcandidate": tcandidate, "result": result}})
        #plot.axisEqual()
        #plot.trackPlot(track)

    # Initialize parts
    sel.init(**{**_alg, **_alg.get("selector_init", {}), **{"logfile": LOGFILE}})
    itp.init(**{**_alg, **_alg.get("interpolator_init", {}), **{"logfile": LOGFILE}})
    seg.init(track, **{**_alg, **_alg.get("segmentator_init", {}), **{"logfile": LOGFILE}})
    cri.init(**{**_alg, **_alg.get("criterion_init", {}), **{"logfile": LOGFILE}})
    opt.init(track, rcandidate, result, **{**_alg, **{"criterion": cri}, **{"interpolator": itp}, **{"segmentator": seg}, **{"selector": sel}, **{"penalizer": pen}, **{"logfile": LOGFILE}})


    ## Optimization
    _fitness, _rcandidate, _tcandidate, _result = opt.optimize()


    ## Plot the solution
    if _alg.get("plot", False):
        #plot.pointsPlot(_result)
        #plot.pointsScatter(_rcandidate)
        if len(_alg.get("plot_args", [])) > 1:
            plot.plotDyn(_alg.get("plot_args", [])[-1], **{**_alg, **{"track": track, "fitness": _fitness, "rcandidate": _rcandidate, "tcandidate": _tcandidate, "result": _result}})
        if fileformat:
            plot.figureSave(fileformat % (loop_i[0]+1) + "-%s.png" % _alg.get("algorithm"))
        else:
            plot.figureShow()


    ## End parts
    if fileformat:
        # Show all results of optimize function (log only)
        print ("#fitness:%.14f" % _fitness, file=LOGFILE)
        print ("#rcandidate:%s" % _rcandidate.tolist(), file=LOGFILE)
        print ("#tcandidate:%s" % _tcandidate.tolist(), file=LOGFILE)
        print ("#trajectory:%s" % _result.tolist(), file=LOGFILE)
    # Show up time elapsed
    print ("time:%f" % (time.time() - step_time), file=LOGFILE)
    print ("==============", file=LOGFILE)

    # Close file if opened
    if fileformat:
        LOGFILE.close()

    # Store only better solution for next steps of the cascade
    if _fitness < fitness:
        return _fitness, _rcandidate, _tcandidate, _result
    else:
        return loop_output


@loop(lambda x: range(x))
def loopCascadeRun(track: numpy.ndarray, initline: numpy.ndarray, fileformat: str, notification: str, loop_i: int, loop_output: Solution = None, **conf) -> Solution:
    """Loop the whole GA cascade.

    Arguments:
    track -- points of the track valid area, nx2 numpy.ndarray
    initline -- points of the initial line for segmentation, mx2 numpy.ndarray
    fileformat -- name of the logging file, str
    notification -- notification about current progress, str
    loop_i -- loop index, int
    **conf -- GA configuration, dict
    """

    # Cascade timing
    cascade_time = time.time()

    # Initial solution
    fitness = 10000000
    result = initline
    rcandidate = initline
    tcandidate = numpy.asarray([ [0.5, 0.5] for _i in range(initline.shape[0])])

    # Update logging file
    if fileformat:
        _fileformat = fileformat % (loop_i+1) + "-%%0%dd" % len(str(len(conf.get("cascade"))))
    else:
        _fileformat = None

    # Update notification
    notification = notification % (loop_i+1) + " Running step %%d/%d" % len(conf.get("cascade"))


    ## Run cascade
    cascade_output = cascadeRun(
        elements=conf.get("cascade"),
        track=track,
        fileformat=_fileformat,
        notification=notification,
        **{**conf, "loop_output": (fitness, rcandidate, tcandidate, result)}
    )


    if fileformat:
        with open(fileformat % (loop_i+1) + ".log", "w") as logfile:
            print ("timeA:%f" % (time.time() - cascade_time), file=logfile)
    else:
        print ("timeA:%f" % (time.time() - cascade_time), file=sys.stdout)

    if loop_output is None or cascade_output[0] < loop_output[0]:
        return cascade_output
    else:
        return loop_output


@loop(lambda x: enumerate(x))
def variateRun(fileformat: str, notification: str, loop_i: Tuple[int, Tuple[str, int]], loop_output: Solution = None, **conf) -> Solution:
    """Run GA with variated number of an element.

    Arguments:
    fileformat -- name of the logging file, str
    notification -- notification about current progress, str
    loop_i -- loop index and (name of the variable, value), 2-tuple [int, Tuple[str, int]]
    **conf -- GA configuration, dict
    """

    # Group timing
    variate_time = time.time()

    # Local variables
    _i = loop_i[0]
    _param = loop_i[1][0]
    _value = loop_i[1][1]

    # Update logging file
    if fileformat:
        # Fill group count, and add format for number of loops
        fileformat = fileformat % (_value) + "-%%0%dd" % len(str(CONFIGURATION.get("loops")))

    # Update notification
    # Fill loop index, group count and prepare loops progress
    notification = notification % (_i+1, _value, _param) + " [%%d / %d]" % CONFIGURATION.get("loops")


    ## Loop cascade
    cascade_output = loopCascadeRun(
        elements=CONFIGURATION.get("loops"),
        fileformat=fileformat,
        notification=notification,
        **{**conf, **{_param: _value}}
    )


    print ("Variating %s %s finished in %fs." % (_param, _value, time.time() - variate_time))

    if loop_output is None or cascade_output[0] < loop_output[0]:
        return cascade_output
    else:
        return loop_output


def execute(START_POINTS: numpy.ndarray = None, VALID_POINTS: numpy.ndarray = None) -> Solution:
    """Execute GA according to the configuration.

    Arguments:
    START_POINTS -- points of the track valid area, nx2 numpy.ndarray
    VALID_POINTS -- points of the initial line for segmentation, mx2 numpy.ndarray

    Note: Currently, it is executed as follows:
        - groupsRun() for each group
            - loopCascadeRun() loop-times
                - cascadeRun() step-times
    """
    global CONFIGURATION

    print ("Starting %s version %s" % (ng_trajectory.__name__, ng_trajectory.__version__))

    # Overall time
    overall_time = time.time()

    # Load data about the track
    if START_POINTS is None:
        START_POINTS = dataLoad(CONFIGURATION.get("start_points"))
    if VALID_POINTS is None:
        VALID_POINTS = dataLoad(CONFIGURATION.get("valid_points"))


    # Logging file format
    if "prefix" in CONFIGURATION:
        fileformat = "%s" % str(CONFIGURATION.get("prefix"))
    else:
        fileformat = None


    # Notification about progress
    notification = ""


    # Identify and prepare variating variable
    if "variate" in CONFIGURATION and CONFIGURATION.get("variate") in CONFIGURATION:
        param = CONFIGURATION.get("variate")
        values = CONFIGURATION.get(param)

        # Force list
        if not isinstance(values, list):
            values = [ values ]

        # Convert to tuples
        tvalues = [ (param, value) for value in values ]

        # Add variate to the file format
        if fileformat:
            if all([ isinstance(_value, int) for _value in values ]):
                fileformat = fileformat + "-%%0%dd" % len(str(max(values)))
            else:
                fileformat = fileformat + "-%s"
        else:
            fileformat = None

        # ... and also to the notification
        notification = notification + "{%%d / %d (%%s %%s)}" % len(values)


        ## And variate the parameter
        solution = variateRun(
            elements=tvalues,
            track=VALID_POINTS,
            initline=START_POINTS,
            fileformat=fileformat,
            notification=notification,
            **CONFIGURATION
        )

    else:
        # Skip to the loop
        # Update logging file
        if fileformat:
            fileformat = fileformat + "-%%0%dd" % len(str(CONFIGURATION.get("loops")))

        # Update notification
        notification = notification + "[%%d / %d]" % CONFIGURATION.get("loops")

        ## Loop cascade
        solution = loopCascadeRun(
            elements=CONFIGURATION.get("loops"),
            track=VALID_POINTS,
            initline=START_POINTS,
            fileformat=fileformat,
            notification=notification,
            **CONFIGURATION
        )

    print ("Optimization finished in %fs." % (time.time() - overall_time))

    return solution
