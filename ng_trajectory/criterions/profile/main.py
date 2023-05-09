#!/usr/bin/env python3.6
# main.py
"""Compute criterion using speed profile and lap time.
"""
######################
# Imports & Globals
######################

import sys, numpy

from . import profiler

from ng_trajectory.interpolators.utils import pointDistance


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("overlap", 0, int, "Size of the trajectory overlap. 0 disables this.", "")
P.createAdd("_mu", 0.2, float, "Friction coeficient", "init")
P.createAdd("_g", 9.81, float, "Gravity acceleration coeficient", "init")
P.createAdd("_m", 3.68, float, "Vehicle mass", "init")
P.createAdd("_ro", 1.2, float, "Air density", "init")
P.createAdd("_A", 0.3, float, "Frontal reference aerodynamic area", "init")
P.createAdd("_cl", 1, float, "Drag coeficient", "init")
P.createAdd("v_0", 0, float, "Initial speed [m.s^-1]", "init")
P.createAdd("v_lim", 4.5, float, "Maximum forward speed [m.s^-1]", "init")
P.createAdd("a_acc_max", 0.8, float, "Maximum longitudal acceleration [m.s^-2]", "init")
P.createAdd("a_break_max", 4.5, float, "Maximum longitudal decceleration [m.s^-2]", "init")
P.createAdd("reference", None, str, "Name of the file to load (x, y, t) reference path that cannot be close.", "init")
P.createAdd("reference_dist", 1.0, float, "Minimum allowed distance from the reference at given time [m].", "init")
P.createAdd("reference_rotate", 0, int, "Number of points to rotate the reference trajectory.", "init")


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize criterion."""
    global REFERENCE

    profiler.parametersSet(**kwargs)

    P.updateAll(kwargs)

    if P.getValue("reference") is not None:
        REFERENCE = numpy.load(P.getValue("reference"))
        REFERENCE = numpy.hstack((numpy.roll(REFERENCE[:, :2], -P.getValue("reference_rotate")), REFERENCE[:, 2:]))
        print ("Loaded reference with '%d' points." % len(REFERENCE), file = kwargs.get("logfile", sys.stdout))
    else:
        REFERENCE = None


def compute(points: numpy.ndarray, overlap: int = 0, penalty: float = 100.0, **overflown) -> float:
    """Compute the speed profile using overlap.

    Arguments:
    points -- points of a trajectory with curvature, nx3 numpy.ndarray
    overlap -- size of trajectory overlap, int, default 0 (disabled)
    penalty -- penalty value applied to the incorrect solutions, float, default 100.0
    **overflown -- arguments not caught by previous parts

    Returns:
    t -- time of reaching the last point of the trajectory, [s], float
         minimization criterion
    """
    global REFERENCE

    _, _, _t = profiler.profileCompute(points, overlap, lap_time = True)

    if REFERENCE is not None:
        _d = P.getValue("reference_dist")

        for rx, ry, rt in REFERENCE:

            # Closest index
            _ci = 0
            __t = 0

            while True:
                _ci = (abs(_t[:-1] + __t - rt)).argmin()

                if _ci == len(_t) - 2:
                    __t += _t[-1]
                else:
                    break

            if pointDistance([rx, ry], points[_ci, :]) < _d:
                return float(penalty)

    return float(_t[-1])

