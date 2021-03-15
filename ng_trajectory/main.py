#!/usr/bin/env python3.6
# main.py
"""Main functions for ng_trajectory.
"""
######################
# Imports & Globals
######################

import sys, numpy, json, time

#from ng_trajectory.configuration import configurationLoad
#from ng_trajectory.configuration import CONFIGURATION
CONFIGURATION = {}

import ng_trajectory.optimizers as optimizers
import ng_trajectory.criterions as criterions
import ng_trajectory.interpolators as interpolators
import ng_trajectory.segmentators as segmentators

import ng_trajectory.plot as plot

# Typing
from typing import Tuple
Solution = Tuple[float, numpy.ndarray, numpy.ndarray, numpy.ndarray]


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
    except:
        return False

    print (CONFIGURATION)


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

    # Open up logging file
    if fileformat:
        LOGFILE = open(fileformat % (loop_i[0]+1) + "-%s.log" % _alg.get("algorithm"), "w")
        print (_alg, file=LOGFILE)
        LOGFILE.flush()
    else:
        LOGFILE = sys.stdout


    ## Initialization
    # Get optimizers etc.
    opt = optimizers.__getattribute__(_alg.get("algorithm"))
    cri = criterions.__getattribute__(_alg.get("criterion"))
    itp = interpolators.__getattribute__(_alg.get("interpolator"))
    seg = segmentators.__getattribute__(_alg.get("segmentator"))

    # Show up current progress
    print (notification % (loop_i[0]+1) + " %s with %s criterion, int. by %s" % (_alg.get("algorithm"), _alg.get("criterion"), _alg.get("interpolator")), file=LOGFILE)
    LOGFILE.flush()

    # Prepare plot
    if _alg.get("plot", False):
        fig = plot.figureCreate()
        plot.plotDyn(_alg.get("plot_args", []), fig, **{**_alg, **{"track": track, "fitness": fitness, "rcandidate": rcandidate, "tcandidate": tcandidate, "result": result}})
        #plot.axisEqual()
        #plot.trackPlot(track)

    # Initialize parts
    itp.init(**{**_alg, **_alg.get("interpolator_init", {}), **{"logfile": LOGFILE}})
    seg.init(track, **{**_alg, **_alg.get("segmentator_init", {}), **{"logfile": LOGFILE}})
    cri.init(**{**_alg, **_alg.get("criterion_init", {}), **{"logfile": LOGFILE}})
    opt.init(track, rcandidate, result, **{**_alg, **{"criterion": cri.compute}, **{"interpolator": itp.interpolate}, **{"segmentator": seg.segmentate}, **{"logfile": LOGFILE}})


    ## Optimization
    _fitness, _rcandidate, _tcandidate, _result = opt.optimize()


    ## Plot the solution
    if _alg.get("plot", False):
        plot.pointsPlot(_result)
        plot.pointsScatter(_rcandidate)
        if fileformat:
            plot.figureSave(fileformat % (loop_i[0]+1) + "-%s.png" % _alg.get("algorithm"))
        else:
            plot.figureShow()


    ## End parts
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


    print ("Variating %s %d finished in %fs." % (_param, _value, time.time() - variate_time))

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
            fileformat = fileformat + "-%%0%dd" % len(str(max(values)))
        else:
            fileformat = None

        # ... and also to the notification
        notification = notification + "{%%d / %d (%%d %%s)}" % len(values)


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
