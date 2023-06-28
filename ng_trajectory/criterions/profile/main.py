#!/usr/bin/env python3.6
# main.py
"""Compute criterion using speed profile and lap time.
"""
######################
# Imports & Globals
######################

import sys, numpy

from . import profiler

from ng_trajectory.interpolators.utils import pointDistance, trajectoryClosestIndex

import ng_trajectory.plot as ngplot

from multiprocessing import Queue


# Global variables
CENTERLINE = None
REFERENCE_PROGRESS = None
OVERTAKING_POINTS = Queue()


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
P.createAdd("_lf", 0.191, float, "Distance from center of mass to the front axle [m]", "init")
P.createAdd("_lr", 0.139, float, "Distance from center of mass to the rear axle [m]", "init")
P.createAdd("reference", None, str, "Name of the file to load (x, y, t) reference path that cannot be close.", "init")
P.createAdd("reference_dist", 1.0, float, "Minimum allowed distance from the reference at given time [m].", "init")
P.createAdd("reference_rotate", 0, int, "Number of points to rotate the reference trajectory.", "init")
P.createAdd("save_solution_csv", "$", str, "When non-empty, save final trajectory to this file as CSV. Use '$' to use log name instead.", "init")
P.createAdd("plot", False, bool, "Whether a graphical representation should be created.", "init (viz.)")
P.createAdd("plot_reference", False, bool, "Whether the reference trajectory should be plotted.", "init (viz.)")
P.createAdd("plot_reference_width", 0.4, float, "Linewidth of the reference trajectory. 0 = disabled", "init (viz.)")
P.createAdd("plot_solution", False, bool, "Whether the optimized solution should be plotted. (Using 'plot_reference_width'.)", "init (viz.)")
P.createAdd("plot_timelines", False, bool, "Whether the lines between points in the same time should be plotted.", "init (viz.)")
P.createAdd("plot_timelines_size", 1, float, "Size of the points of the timelines endpoints. 0 = disabled", "init (viz.)")
P.createAdd("plot_timelines_width", 0.6, float, "Linewidth of the timelines. 0 = disabled", "init (viz.)")


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize criterion."""
    global REFERENCE, CENTERLINE, OVERTAKING_POINTS

    profiler.parametersSet(**kwargs)

    P.updateAll(kwargs)

    OVERTAKING_POINTS = Queue()

    if P.getValue("save_solution_csv") == "":
        P.update("save_solution_csv", None)
    elif P.getValue("save_solution_csv") == "$":
        P.update("save_solution_csv", kwargs.get("logfile").name + ".csv")

    if P.getValue("reference") is not None:
        REFERENCE = numpy.load(P.getValue("reference"))
        REFERENCE = numpy.hstack((numpy.roll(REFERENCE[:, :2], -P.getValue("reference_rotate"), axis=0), REFERENCE[:, 2:]))
        print ("Loaded reference with '%d' points." % len(REFERENCE), file = kwargs.get("logfile", sys.stdout))
    else:
        REFERENCE = None


def compute(points: numpy.ndarray, overlap: int = None, penalty: float = 100.0, **overflown) -> float:
    """Compute the speed profile using overlap.

    Arguments:
    points -- points of a trajectory with curvature, nx3 numpy.ndarray
    overlap -- size of trajectory overlap, int, default None/0 (disabled)
    penalty -- penalty value applied to the incorrect solutions, float, default 100.0
    **overflown -- arguments not caught by previous parts

    Returns:
    t -- time of reaching the last point of the trajectory, [s], float
         minimization criterion
    """
    global REFERENCE, CENTERLINE, REFERENCE_PROGRESS, OVERTAKING_POINTS

    if overlap is None:
        overlap = P.getValue("overlap")

    _v, _a, _t = profiler.profileCompute(points, overlap, lap_time = True,
        save = P.getValue("save_solution_csv") if not overflown.get("optimization", True) and P.getValue("save_solution_csv") is not None else None
    )

    invalid_points = []
    closest_indices = []

    if REFERENCE is not None:
        _d = P.getValue("reference_dist")

        for rx, ry, rt in REFERENCE:

            # Closest index
            _ci = 0
            __t = 0

            while True:
                # Find closest point in time domain
                _ci = (abs(_t[:-1] + __t - rt)).argmin()

                # In case that we select the last point
                # Do it again for next repetition of the trajectory
                if _ci == len(_t) - 2:
                    __t += _t[-1]
                else:
                    break

            closest_indices.append(_ci)

            # If the points are too close to each other, return penalty
            if pointDistance([rx, ry], points[_ci, :]) < _d:
                if overflown.get("optimization", True):
                    return float(penalty)
                else:
                    invalid_points.append([rx, ry])

        # Visualization
        if not overflown.get("optimization", True) and P.getValue("plot"):

            # Last time sample
            ts = int(_t[-1]) - 1

            if P.getValue("plot_timelines"):
                for ts in range(int(_t[-1])):
                    _closest = numpy.abs(numpy.subtract(REFERENCE[:, 2], ts)).argmin()

                    if _closest >= len(REFERENCE) - 1:
                        ts = ts - 1
                        break

                    ngplot.pointsScatter(
                        REFERENCE[_closest, None, :2], # Trick to force 2D array.
                        s = P.getValue("plot_timelines_size"),
                        color = "black"
                    )

                    if ts % 4 == 0:
                        ngplot.labelText(REFERENCE[_closest, :2], ts,
                            verticalalignment = "top",
                            horizontalalignment = "left",
                            fontsize = 6
                        )

                    _closest_p = numpy.abs(numpy.subtract(_t[:-1], ts)).argmin()

                    ngplot.pointsScatter(
                        points[_closest_p, None, :2],
                        s = P.getValue("plot_timelines_size"),
                        color = "red"
                    )

                    if ts % 4 == 0:
                        ngplot.labelText(points[_closest_p, :2], ts,
                            verticalalignment = "bottom",
                            horizontalalignment = "right",
                            fontsize = 6,
                            color = "red",
                        )

                    ngplot.pointsPlot(
                        numpy.vstack((points[_closest_p , :2], REFERENCE[_closest , :2])),
                        color = "red",
                        linewidth = P.getValue("plot_timelines_width"),
                        linestyle = (
                            "--" if pointDistance(points[_closest_p , :2], REFERENCE[_closest , :2]) < 5.0 else " "
                        )
                    )

            if P.getValue("plot_reference"):
                # Plot only to the last time point
                # That is specified by ts, which can be altered by previous if.
                _closest = numpy.abs(numpy.subtract(REFERENCE[:, 2], ts)).argmin()
                _closest_p = numpy.abs(numpy.subtract(_t[:-1], ts)).argmin()

                ngplot.pointsPlot(REFERENCE[:_closest, :2], color="black", linewidth = P.getValue("plot_reference_width"))

                if P.getValue("plot_solution"):
                    ngplot.pointsPlot(points[:_closest_p, :2], color="orange", linewidth = P.getValue("plot_reference_width"))

            if len(invalid_points) > 0:
                ngplot.pointsScatter(numpy.asarray(invalid_points), color="blue", marker="x", s = 1)


    if len(invalid_points) > 0:
        return float(penalty)


    # Locate points where overtaking occurs
    # Centerline is used to obtain track progression.
    if REFERENCE is not None and CENTERLINE is not None:
        if REFERENCE_PROGRESS is None:
            REFERENCE_PROGRESS = [
                trajectoryClosestIndex(CENTERLINE, REFERENCE[_i, :2])
                for _i in range(len(REFERENCE))
            ]

        overtaken = False
        for _i, (rx, ry, _) in enumerate(REFERENCE):
            rd = REFERENCE_PROGRESS[_i]                    # nejblizsi i ve stejnem case
            pd = trajectoryClosestIndex(CENTERLINE, points[closest_indices[_i], :2])

            if rd > 50 and pd > rd and not overtaken:
                overtaken = True
                OVERTAKING_POINTS.put(points[closest_indices[_i], :2])
            #elif pd < rd and overtaken:
            #    overtaken = False


    return float(_t[-1])

