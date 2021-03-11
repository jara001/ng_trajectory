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
                else:
                    output = function(*args, **{**kwargs, **{"loop_i": i, "loop_output": output}})
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
def cascadeRun(track, fileformat, notification, loop_i, loop_output, **conf):
    """Run GA Cascade."""

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

    # Show up current progress
    print (notification % (loop_i[0]+1) + " %s with %s criterion" % (_alg.get("algorithm"), _alg.get("criterion")), file=LOGFILE)
    LOGFILE.flush()

    # Initialize parts
    cri.init(**{**_alg, **_alg.get("criterion_init"), **{"logfile": LOGFILE}})
    opt.init(track, result, rcandidate, **{**_alg, **{"criterion": cri.compute}, **{"logfile": LOGFILE}})


    ## Optimization
    _fitness, _rcandidate, _tcandidate, _result = opt.optimize()


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


def execute():
    """Execute GA according to the configuration."""
    global CONFIGURATION

    # Overall time
    overall_time = time.time()

    # Load data about the track
    START_POINTS = dataLoad(CONFIGURATION.get("start_points"))
    VALID_POINTS = dataLoad(CONFIGURATION.get("valid_points"))

    _groups = [ CONFIGURATION.get("groups") ] if isinstance(CONFIGURATION.get("groups"), int) else CONFIGURATION.get("groups")


    # Logging file format
    if "prefix" in CONFIGURATION:
        fileformat = "%s-%%0%dd" % (str(CONFIGURATION.get("prefix")), len(str(max(_groups))))
    else:
        fileformat = None


    # Notification about progress
    notification = "{%%d / %d (%%d groups)}" % len(_groups)


    # Create groups variation
    for _gi, _group in enumerate(_groups):
        groups_time = time.time()

        _configuration = {**CONFIGURATION, **{"groups": _group}}

        # Update logging file
        if fileformat:
            _fileformat = fileformat % (_group) + "-%%0%dd" % len(str(CONFIGURATION.get("loops")))

        # Update notification
        _notification = notification % (_gi+1, _group) + " [%%d / %d]" % CONFIGURATION.get("loops")


        # Create loops
        for _loop in range(CONFIGURATION.get("loops")):
            # Cascade timing
            cascade_time = time.time()

            # Initial solution
            fitness = 10000000
            result = START_POINTS
            rcandidate = START_POINTS
            tcandidate = numpy.asarray([ [0.5, 0.5] for _i in range(START_POINTS.shape[0])])

            # Update logging file
            if fileformat:
                __fileformat = _fileformat % (_loop+1) + "-%%0%dd" % len(str(len(CONFIGURATION.get("cascade"))))
            else:
                __fileformat = None

            # Update notification
            __notification = _notification % (_loop+1) + " Running step %%d/%d" % len(CONFIGURATION.get("cascade"))

            cascadeRun(
                elements=CONFIGURATION.get("cascade"),
                track=VALID_POINTS,
                loop_output=(fitness, rcandidate, tcandidate, result),
                **{**CONFIGURATION, "fileformat": __fileformat, "notification": __notification}
            )

            if "prefix" in CONFIGURATION:
                with open(
                        ("%s-%%0%dd.log" % (
                                str(CONFIGURATION.get("prefix", "ng")),
                                len(str(CONFIGURATION.get("loops")+1))
                            )) % (_loop+1), "w"
                    ) as logfile:
                    print ("timeA:%f" % (time.time() - cascade_time), file=logfile)
            else:
                print ("timeA:%f" % (time.time() - cascade_time), file=sys.stdout)

        print ("Group %d finished in %fs." % (_group, time.time() - groups_time))

    print ("Optimization finished in %fs." % (time.time() - overall_time))
