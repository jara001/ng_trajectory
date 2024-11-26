#!/usr/bin/env python3.6
# main.py
"""# Optimization toolbox ng_trajectory

**ng_trajectory** is a toolbox for solving the optimal racing line
problem using various methods and approaches. The main idea
stands on using a genetic algorithm, although other approaches
may be used as well.

Currently we distinguish 6 groups of algorithms:
(1) **Selectors**
	Selectors take the input path and select a subset of these
    points.
(2) **Segmentators**
	Segmentators split the track into segments using the selected
    point subset.
(3) **Interpolators**
	Interpolators are used for interpolating the selected point
    subset in order to get, e.g., curvature.
(4) **Optimizers**
	Optimizers take the data from the previous three parts and
    use them to find the optimal racing line.
(5) **Criterions**
	Criterions are used for obtaining the fitness value given
    the waypoints.
(6) **Penalizers**
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

## Minimal version of the configuration:
```json
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
```
"""  # noqa: W191,D206,D400
######################
# Imports & Globals
######################

import sys
import numpy
import json
import time
from packaging import specifiers  # SpecifierSet

# from ng_trajectory.configuration import configurationLoad
# from ng_trajectory.configuration import CONFIGURATION

import ng_trajectory

import ng_trajectory.optimizers as optimizers
import ng_trajectory.criterions as criterions
import ng_trajectory.interpolators as interpolators
import ng_trajectory.segmentators as segmentators
import ng_trajectory.selectors as selectors
import ng_trajectory.penalizers as penalizers

from ng_trajectory.log import (
    verbositySet,
    logfileSet, logfileReset, logfileFlush,
    log,
    print0, printvv
)

import ng_trajectory.plot as plot

from ng_trajectory.parameter import ParameterList

# Typing
from typing import Tuple, Dict


# Global variables
CONFIGURATION = {}
Solution = Tuple[float, numpy.ndarray, numpy.ndarray, numpy.ndarray]
P = ParameterList()


# Parameters
# FIXME: Actually, the default values do not work here as it was not adapted for ParameterList.
P.createAdd("_version", None, int, "Version of the configuration.", "General")
P.createAdd("_comment", None, str, "Commentary of the configuration file.", "General")
P.createAdd("_ng_version", "", str, "Specifier for supported ng_trajectory versions with the configuration file.", "General")
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
        """Initialize Stub object.

        Arguments:
        name -- name of the object, str
        silent_stub -- when True, usage of this stub is not reported, bool
        """
        self.name = name
        self.silent_stub = silent_stub

    def __getattr__(self, attr):
        """Return dummy lambda function."""
        return lambda *x, **y: print(
            "Called '%s' on '%s' object, but it was not specified "
            "in the configuration."
            % (attr, self.name), file=sys.stderr
        ) if not self.silent_stub else None


######################
# Decorators
######################

def loop(iterator):
    """Decorate to create looping functions."""

    def wrapper(function, *args, **kwags):
        def looper(elements, *args, **kwargs):
            first = True
            output = []
            for i in iterator(elements):
                if first:
                    output = function(*args, **{**kwargs, **{"loop_i": i}})
                    first = False
                else:
                    output = function(
                        *args,
                        **{**kwargs, **{"loop_i": i, "loop_output": output}}
                    )
            return output
        return looper
    return wrapper


######################
# Functions
######################

def configurationLoad(filename: str) -> bool:
    """Load a configuration stored in 'filename'.

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
                print (
                    "Unsupported version of the configuration file.",
                    file=sys.stderr
                )
            else:
                CONFIGURATION = {**CONFIGURATION, **conf}

            spec = specifiers.SpecifierSet(
                CONFIGURATION.get("_ng_version", ""), prereleases = True
            )

            if ng_trajectory.__version__ not in spec:
                raise ValueError (
                    "Unsupported version of ng_trajectory == %s. "
                    "Configuration requires: %s"
                    % (ng_trajectory.__version__, spec)
                )
    except Exception as e:
        print (e)
        return False

    verbositySet(conf.get("logging_verbosity", 1))

    printvv (CONFIGURATION)

    return True


def configurationAppend(conf: Dict[str, any]) -> bool:
    """Append configuration to the global settings.

    Arguments:
    conf -- new configuration, dict

    Returns:
    success -- True when loaded, otherwise False
    """
    global CONFIGURATION

    CONFIGURATION = {**CONFIGURATION, **conf}

    verbositySet(CONFIGURATION.get("logging_verbosity", 1))

    printvv (CONFIGURATION)

    return True


def dictMerge(dict1: Dict[str, any], dict2: Dict[str, any]) -> Dict[str, any]:
    """Merge two dictionaries in a 'true merge'.

    Arguments:
    dict1, dict2 -- dictionaries to merge, dict[str, any]

    Returns:
    merged dictionary, dict[str, any]

    Note: This crawls through the dicts and performs
    a 'true merge'; merging values in the subdicts.

    Sources:
    https://stackoverflow.com/a/7205672
    """
    def mergedicts(dict1, dict2):
        for k in set(dict1.keys()).union(dict2.keys()):
            if k in dict1 and k in dict2:
                if isinstance(dict1[k], dict) and isinstance(dict2[k], dict):
                    yield (k, dict(mergedicts(dict1[k], dict2[k])))
                else:
                    # If one of the values is not a dict, you can't continue
                    # merging it. Value from second dict overrides one in first
                    # and we move on.
                    yield (k, dict2[k])
                    # Alternatively, replace this with exception raiser to
                    # alert you of value conflicts
            elif k in dict1:
                yield (k, dict1[k])
            else:
                yield (k, dict2[k])

    return dict(mergedicts(dict1, dict2))


def configurationMerge(conf: Dict[str, any]) -> bool:
    """Merge configuration with the global settings.

    Arguments:
    conf -- configuration to merge, dict

    Returns:
    success -- True when loaded, otherwise False
    """
    global CONFIGURATION

    CONFIGURATION = dictMerge(CONFIGURATION, conf)

    verbositySet(CONFIGURATION.get("logging_verbosity", 1))

    printvv (CONFIGURATION)

    return True


def dataLoad(filename: str) -> numpy.ndarray:
    """Load data from stored file.

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
def cascadeRun(
        track: numpy.ndarray,
        fileformat: str,
        notification: str,
        loop_i: int,
        loop_output: Solution,
        **conf) -> Solution:
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
    rcandidate -- points in the best solution in real coordinates,
                  nx2 numpy.ndarray
    tcandidate -- points in the best solution in transformed coordinates,
                  nx2 numpy.ndarray
    result -- trajectory of the best solution in real coordinates,
              mx2 numpy.ndarray
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
            with open(
                fileformat % (loop_i[0] + 1) + "-%s.log"
                % _alg.get("algorithm"),
                "r"
            ) as f:
                _stored = {
                    name: json.loads(value)
                    for name, value in [
                        line.split(":")
                        for line in f.readlines() if line[0] == "#"
                    ]
                }

                if _stored["#fitness"] < fitness:
                    return (
                        _stored["#fitness"],
                        numpy.asarray(_stored["#rcandidate"]),
                        numpy.asarray(_stored["#tcandidate"]),
                        numpy.asarray(_stored["#trajectory"])
                    )
                else:
                    return loop_output

        # File not found, therefore we want to run this.
        except FileNotFoundError:
            pass

        # One of the values is not found, meaning that
        # we probably have a corrupted log.
        # So we just to the measurement again.
        except KeyError:
            pass

    # Open up logging file
    if fileformat:
        LOGFILE = open(
            fileformat % (loop_i[0] + 1) + "-%s.log" % _alg.get("algorithm"),
            "w"
        )
        logfileSet(LOGFILE)

        log (_alg)
        log (
            "Running %s version %s"
            % (
                ng_trajectory.__name__,
                ng_trajectory.__version__
            )
        )
        logfileFlush()
    else:
        LOGFILE = sys.stdout


    # # Initialization # #
    # Get optimizers etc.
    obtain = lambda group, name: \
        group.__getattribute__(_alg.get(name)) \
        if hasattr(group, _alg.get(name, "")) \
        else Stub(name, _alg.get("silent_stub", False))
    opt = obtain(optimizers, "algorithm")
    cri = obtain(criterions, "criterion").Criterion()
    itp = obtain(interpolators, "interpolator")
    seg = obtain(segmentators, "segmentator")
    sel = obtain(selectors, "selector")
    # Set default penalizer. Not used in 'ParameterList' as it has no effect.
    if "penalizer" not in _alg:
        _alg = {**_alg, **{"penalizer": "count"}}
    pen = obtain(penalizers, "penalizer").Penalizer()

    # Show up current progress
    log (
        notification % (loop_i[0] + 1) + " "
        "%s with %s criterion (penalized by %s), int. by %s"
        % (
            _alg.get("algorithm", ""),
            _alg.get("criterion", ""),
            _alg.get("penalizer", ""),
            _alg.get("interpolator", "")
        )
    )
    logfileFlush()

    # Prepare plot
    if _alg.get("plot", False):
        fig = plot.figureCreate()

        # Append figure
        _alg = {**_alg, **{"figure": fig}}

        plot.plotDyn(
            _alg.get("plot_args", [])[0],
            **{
                **_alg,
                **{
                    "track": track,
                    "fitness": fitness,
                    "rcandidate": rcandidate,
                    "tcandidate": tcandidate,
                    "result": result
                }
            }
        )

    # Initialize parts
    _sel_dict = sel.init(
        **{
            **_alg,
            **_alg.get("selector_init", {})
        }
    )
    if _sel_dict is not None:
        _alg = dictMerge(_alg, _sel_dict)

    _itp_dict = itp.init(
        **{
            **_alg,
            **_alg.get("interpolator_init", {})
        }
    )
    if _itp_dict is not None:
        _alg = dictMerge(_alg, _itp_dict)

    _seg_dict = seg.init(
        track,
        **{
            **_alg,
            **_alg.get("segmentator_init", {})
        }
    )
    if _seg_dict is not None:
        _alg = dictMerge(_alg, _seg_dict)

    _cri_dict = cri.init(
        **{
            **_alg,
            **_alg.get("criterion_init", {})
        }
    )
    if _cri_dict is not None:
        _alg = dictMerge(_alg, _cri_dict)

    # Note: This passes the initial line (which is usually centerline).
    # TODO: Actually pass centerline.
    if not hasattr(cri, "CENTERLINE") or cri.CENTERLINE is None:
        cri.CENTERLINE = result.copy()

    _opt_dict = opt.init(
        track,
        rcandidate,
        result,
        **{
            **_alg,
            **{"criterion": cri},
            **{"interpolator": itp},
            **{"segmentator": seg},
            **{"selector": sel},
            **{"penalizer": pen}
        }
    )
    if _opt_dict is not None:
        _alg = dictMerge(_alg, _opt_dict)

    logfileFlush()

    # # Optimization # #
    _fitness, _rcandidate, _tcandidate, _result = opt.optimize()


    # # Plot the solution # #
    if _alg.get("plot", False):
        if len(_alg.get("plot_args", [])) > 1:
            plot.plotDyn(
                _alg.get("plot_args", [])[-1],
                **{
                    **_alg,
                    **{
                        "track": track,
                        "fitness": _fitness,
                        "rcandidate": _rcandidate,
                        "tcandidate": _tcandidate,
                        "result": _result
                    }
                }
            )
        if fileformat:
            plot.figureSave(
                fileformat % (loop_i[0] + 1) + "-%s.png"
                % _alg.get("algorithm")
            )
            plot.figureClose()
        else:
            plot.figureShow()


    # # End parts # #
    if fileformat:
        # Show all results of optimize function (log only)
        log ("#fitness:%.14f" % _fitness)
        log ("#rcandidate:%s" % _rcandidate.tolist())
        log ("#tcandidate:%s" % _tcandidate.tolist())
        log ("#trajectory:%s" % _result.tolist())
    # Show up time elapsed
    log ("time:%f" % (time.time() - step_time))
    log ("==============")

    # Close file if opened
    if fileformat:
        LOGFILE.close()
        logfileReset()

    # Store only better solution for next steps of the cascade
    if _fitness < fitness:
        return _fitness, _rcandidate, _tcandidate, _result
    else:
        return loop_output


@loop(lambda x: range(x))
def loopCascadeRun(
        track: numpy.ndarray,
        initline: numpy.ndarray,
        fileformat: str,
        notification: str,
        loop_i: int,
        loop_output: Solution = None,
        **conf) -> Solution:
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
    tcandidate = numpy.asarray([
        [0.5, 0.5] for _i in range(initline.shape[0])
    ])

    # Update logging file
    if fileformat:
        _fileformat = fileformat % (loop_i + 1) + (
            "-%%0%dd" % len(str(len(conf.get("cascade"))))
        )
    else:
        _fileformat = None

    # Update notification
    notification = notification % (loop_i + 1) + (
        " Running step %%d/%d" % len(conf.get("cascade"))
    )


    # # Run cascade # #
    cascade_output = cascadeRun(
        elements=conf.get("cascade"),
        track=track,
        fileformat=_fileformat,
        notification=notification,
        **{**conf, "loop_output": (fitness, rcandidate, tcandidate, result)}
    )

    # TODO: Do not do this when all cascade parts / loops were skipped.
    if fileformat:
        with open(fileformat % (loop_i + 1) + ".log", "w") as logfile:
            log ("timeA:%f" % (time.time() - cascade_time), logfile=logfile)
    else:
        log ("timeA:%f" % (time.time() - cascade_time))

    if loop_output is None or cascade_output[0] < loop_output[0]:
        return cascade_output
    else:
        return loop_output


@loop(lambda x: enumerate(x))
def variateRun(
        fileformat: str,
        notification: str,
        loop_i: Tuple[int, Tuple[str, int]],
        loop_output: Solution = None,
        **conf) -> Solution:
    """Run GA with variated number of an element.

    Arguments:
    fileformat -- name of the logging file, str
    notification -- notification about current progress, str
    loop_i -- loop index and (name of the variable, value),
              2-tuple [int, Tuple[str, int]]
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
        fileformat = fileformat % (_value) + (
            "-%%0%dd" % len(str(CONFIGURATION.get("loops")))
        )

    # Update notification
    # Fill loop index, group count and prepare loops progress
    notification = notification % (_i + 1, _value, _param) + (
        " [%%d / %d]" % CONFIGURATION.get("loops")
    )


    # # Loop cascade # #
    cascade_output = loopCascadeRun(
        elements=CONFIGURATION.get("loops"),
        fileformat=fileformat,
        notification=notification,
        **{**conf, **{_param: _value}}
    )


    print0(
        "Variating %s %s finished in %fs."
        % (_param, _value, time.time() - variate_time)
    )

    if loop_output is None or cascade_output[0] < loop_output[0]:
        return cascade_output
    else:
        return loop_output


def execute(
        START_POINTS: numpy.ndarray = None,
        VALID_POINTS: numpy.ndarray = None) -> Solution:
    """Execute GA according to the configuration.

    Arguments:
    START_POINTS -- points of the track valid area,
                    nx2 numpy.ndarray
    VALID_POINTS -- points of the initial line for segmentation,
                    mx2 numpy.ndarray

    Note: Currently, it is executed as follows:
        - groupsRun() for each group
            - loopCascadeRun() loop-times
                - cascadeRun() step-times
    """
    global CONFIGURATION

    print (
        "Starting %s version %s"
        % (ng_trajectory.__name__, ng_trajectory.__version__)
    )

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
    if CONFIGURATION.get("variate", None) in CONFIGURATION:
        param = CONFIGURATION.get("variate")
        values = CONFIGURATION.get(param)

        # Force list
        if not isinstance(values, list):
            values = [values]

        # Convert to tuples
        tvalues = [(param, value) for value in values]

        # Add variate to the file format
        if fileformat:
            if all([isinstance(_value, int) for _value in values]):
                fileformat = fileformat + "-%%0%dd" % len(str(max(values)))
            elif all([isinstance(_value, (int, float)) for _value in values]):
                _len_decimal = max([
                    len(str(float(_value)).split(".")[1])
                    for _value in values
                ])
                fileformat = fileformat + (
                    "-%%0%d.%df"
                    % (
                        len(str(int(max(values)))) + 1 + _len_decimal,
                        _len_decimal
                    )
                )
            else:
                fileformat = fileformat + "-%s"
        else:
            fileformat = None

        # ... and also to the notification
        notification = notification + "{%%d / %d (%%s %%s)}" % len(values)


        # # And variate the parameter # #
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
            fileformat = fileformat + (
                "-%%0%dd" % len(str(CONFIGURATION.get("loops")))
            )

        # Update notification
        notification = notification + "[%%d / %d]" % CONFIGURATION.get("loops")

        # # Loop cascade # #
        solution = loopCascadeRun(
            elements=CONFIGURATION.get("loops"),
            track=VALID_POINTS,
            initline=START_POINTS,
            fileformat=fileformat,
            notification=notification,
            **CONFIGURATION
        )

    print0("Optimization finished in %fs." % (time.time() - overall_time))

    return solution
