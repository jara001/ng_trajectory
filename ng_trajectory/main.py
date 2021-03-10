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


def execute():
    """Execute GA according to the configuration."""
    global CONFIGURATION

    # Overall time
    overall_time = time.time()

    # Load data about the track
    START_POINTS = dataLoad(CONFIGURATION.get("start_points"))
    VALID_POINTS = dataLoad(CONFIGURATION.get("valid_points"))

    # Create loops
    for _loop in range(CONFIGURATION.get("loops")):
        # Cascade timing
        cascade_time = time.time()

        # Logging file format
        if "prefix" in CONFIGURATION:
            fileformat = "%s-%%0%dd-%%0%dd-%%s.log" % (
                str(CONFIGURATION.get("prefix")),
                len(str(CONFIGURATION.get("loops")+1)),
                len(str(len(CONFIGURATION.get("cascade"))+1))
            )
        else:
            fileformat = None

        # Initial solution
        fitness = 10000000
        result = START_POINTS
        rcandidate = START_POINTS
        tcandidate = numpy.asarray([ [0.5, 0.5] for _i in range(START_POINTS.shape[0])])

        # Run cascade
        for _i, _alg in enumerate(CONFIGURATION.get("cascade")):
            # Cascade step timing
            step_time = time.time()

            if fileformat:
                LOGFILE = open(fileformat % (_loop+1, _i+1, _alg.get("algorithm")), "w")
                print ({**CONFIGURATION, **_alg}, file=LOGFILE)
                LOGFILE.flush()
            else:
                LOGFILE = sys.stdout

            opt = optimizers.__getattribute__(_alg.get("algorithm"))
            cri = criterions.__getattribute__(_alg.get("criterion"))

            # Initialize criterion
            cri.init(**{**CONFIGURATION, **_alg, **_alg.get("criterion_init"), **{"logfile": LOGFILE}})

            opt.init(VALID_POINTS, result, rcandidate, **{**CONFIGURATION, **_alg, **{"criterion": cri.compute}, **{"logfile": LOGFILE}})
            _fitness, _rcandidate, _tcandidate, _result = opt.optimize()

            # Store only better solution for next steps of the cascade
            if (_fitness < fitness):
                fitness, rcandidate, tcandidate, rcandidate = _fitness, _rcandidate, _tcandidate, _result

            print ("time:%f" % (time.time() - step_time), file=LOGFILE)
            print ("==============", file=LOGFILE)

            if fileformat:
                LOGFILE.close()

        if "prefix" in CONFIGURATION:
            with open(
                    ("%s-%%0%dd.log" % (
                            str(CONFIGURATION.get("prefix", "ng")),
                            len(str(CONFIGURATION.get("loops")+1))
                        )) % (_loop+1), "w"
                ) as logfile:
                print ("timeA:%f" % (time.time() - cascade_time), file=logfile)
        else:
            print ("timeA:%f" % (time.time() - cascade_time), file=logfile)

    print ("Optimization finished in %fs." % (time.time() - overall_time))
