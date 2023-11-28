#!/usr/bin/env python3.6
# main.py
"""Compute criterion using speed profile and lap time.

[1] N. R. Kapania, J. Subosits, and J. Christian Gerdes, ‘A Sequential
Two-Step Algorithm for Fast Generation of Vehicle Racing Trajectories’,
Journal of Dynamic Systems, Measurement, and Control, vol. 138, no. 9,
p. 091005, Sep. 2016, doi: 10.1115/1.4033311.
"""
######################
# Imports & Globals
######################

import sys
import numpy

from . import profiler

from ng_trajectory.interpolators.utils import (
    pointDistance,
    trajectoryClosestIndex
)

from ng_trajectory.segmentators.utils import (
    pointsToMap,
    getMap
)

from ng_trajectory.log import (
    logfileName,
    log,
    print0
)

import ng_trajectory.plot as ngplot

from ng_trajectory.parameter import ParameterList

from multiprocessing import Queue

from itertools import chain  # Join generators


# Global variables
CENTERLINE = None
REFERENCE_PROGRESS = None
OVERTAKING_POINTS = Queue()


# Parameters
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
P.createAdd("reference_laptime", 0, float, "Lap time of the given reference. 0 = estimated from data", "init")
P.createAdd("save_solution_csv", "$", str, "When non-empty, save final trajectory to this file as CSV. Use '$' to use log name instead.", "init")
P.createAdd("plot", False, bool, "Whether a graphical representation should be created.", "init (viz.)")
P.createAdd("plot_reference", False, bool, "Whether the reference trajectory should be plotted.", "init (viz.)")
P.createAdd("plot_reference_width", 0.4, float, "Linewidth of the reference trajectory. 0 = disabled", "init (viz.)")
P.createAdd("plot_solution", False, bool, "Whether the optimized solution should be plotted. (Using 'plot_reference_width'.)", "init (viz.)")
P.createAdd("plot_timelines", False, bool, "Whether the lines between points in the same time should be plotted.", "init (viz.)")
P.createAdd("plot_timelines_size", 1, float, "Size of the points of the timelines endpoints. 0 = disabled", "init (viz.)")
P.createAdd("plot_timelines_width", 0.6, float, "Linewidth of the timelines. 0 = disabled", "init (viz.)")
P.createAdd("plot_overtaking", True, bool, "Whether to plot places where an overtaking occurs. (Has to be supported by optimizer.)", "init (viz.)")
P.createAdd("favor_overtaking", 0, float, "Penalty value to add to the lap time when overtaking does not occur.", "init")
P.createAdd("friction_map", None, str, "Name of the file to load (x, y, mu*100) with friction map.", "init")
P.createAdd("friction_map_inverse", False, bool, "When True, invert the values in the friction map.", "init")
P.createAdd("friction_map_expand", False, bool, "When True, values from the friction map are expanded over the whole map using flood fill.", "init")


######################
# Utilities
######################

def fmap_expand(friction_map, known_values):
    """Expand the known values over the whole friction map.

    Arguments:
    friction_map -- map of the environment with friction, numpy.ndarray
    known_values -- known values of the friction, used for expansion,
                    nx3 numpy.ndarray

    Returns:
    updated friction_map
    """
    cxy = pointsToMap(known_values[:, :2])
    help_map = numpy.ones_like(friction_map, dtype = bool)

    help_map[cxy[:, 0], cxy[:, 1]] = False


    queue = cxy.tolist()

    while len(queue) > 0:
        cell_x, cell_y = queue.pop(0)

        for _a, _b in [(-1, -1), (-1, 0), (-1, 1),
                       (+0, -1),          (+0, 1),   # noqa: E241
                       (+1, -1), (+1, 0), (+1, 1)]:

            # Try does catch larger values but not negative
            if cell_x + _a < 0 or cell_y + _b < 0:
                continue

            try:
                _cell = help_map[cell_x + _a, cell_y + _b]
                _cx = cell_x + _a
                _cy = cell_y + _b
            except IndexError:
                continue

            # Expand the value if not yet done
            if _cell:
                friction_map[_cx, _cy] = friction_map[cell_x, cell_y]
                help_map[_cx, _cy] = False

                queue.append((_cx, _cy))

    return friction_map


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize criterion."""
    global REFERENCE, CENTERLINE, OVERTAKING_POINTS

    profiler.parametersSet(**kwargs)

    P.updateAll(kwargs)

    # Recreating the Queue here makes the parent process use
    # different Queue than the ProcessPool.
    # OVERTAKING_POINTS = Queue()

    if P.getValue("save_solution_csv") == "":
        P.update("save_solution_csv", None)
    elif P.getValue("save_solution_csv") == "$":
        P.update(
            "save_solution_csv",
            logfileName() + ".csv"
        )

    if P.getValue("reference") is not None:
        REFERENCE = numpy.load(P.getValue("reference"))
        # REFERENCE = numpy.hstack((numpy.roll(REFERENCE[:, :2], -P.getValue("reference_rotate"), axis=0), REFERENCE[:, 2:]))
        # TODO: Lap time should be given, not estimated like this.
        lap_time = P.getValue("reference_laptime")

        if lap_time == 0.0:
            # Lap time estimate
            lap_time = REFERENCE[-1, 2] + numpy.mean([
                REFERENCE[-1, 2] - REFERENCE[-2, 2],
                REFERENCE[1, 2] - REFERENCE[0, 2]
            ])

        REFERENCE = numpy.roll(
            REFERENCE,
            -P.getValue("reference_rotate"),
            axis = 0
        )
        REFERENCE[:, 2] = REFERENCE[:, 2] - REFERENCE[0, 2]
        REFERENCE[REFERENCE[:, 2] < 0, 2] += lap_time
        log (
            "Loaded reference with '%d' points, lap time %fs."
            % (len(REFERENCE), lap_time)
        )
    else:
        REFERENCE = None

    if P.getValue("friction_map") is not None:
        fmap = numpy.load(P.getValue("friction_map"))

        if P.getValue("friction_map_inverse"):
            fmap[:, 2] = 255 - fmap[:, 2]

        FRICTION_MAP = getMap().copy()

        # Set default value to the _mu parameter.
        FRICTION_MAP[:] = profiler._mu * 100

        # Set all obtained values.
        cxy = pointsToMap(fmap[:, :2])
        FRICTION_MAP[cxy[:, 0], cxy[:, 1]] = fmap[:, 2]

        # Expand the fmap is required
        if P.getValue("friction_map_expand"):
            FRICTION_MAP = fmap_expand(FRICTION_MAP, fmap)

        profiler.FRICTION_MAP = FRICTION_MAP

        log (
            "Loaded friction map from '%s'."
            % P.getValue("friction_map")
        )

        uqs, cnt = numpy.unique(fmap[:, 2], return_counts = True)

        log (
            "\n".join([
                "\t%.2f: %3.2f%%" % (_u, _r)
                for _u, _r in zip(
                    uqs / 100.0,
                    (cnt / (float(sum(cnt)))) * 100.0
                )
            ])
        )


def compute(
        points: numpy.ndarray,
        overlap: int = None,
        penalty: float = 100.0,
        **overflown) -> float:
    """Compute the speed profile using overlap.

    Arguments:
    points -- points of a trajectory with curvature, nx3 numpy.ndarray
    overlap -- size of trajectory overlap, int, default None/0 (disabled)
    penalty -- penalty value applied to the incorrect solutions,
               float, default 100.0
    **overflown -- arguments not caught by previous parts

    Returns:
    t -- time of reaching the last point of the trajectory, [s], float
         minimization criterion
    """
    global REFERENCE, CENTERLINE, REFERENCE_PROGRESS, OVERTAKING_POINTS

    if overlap is None:
        overlap = P.getValue("overlap")

    profiler.CENTERLINE = CENTERLINE

    _v, _a, _t = profiler.profileCompute(
        points,
        overlap,
        lap_time = True,
        save = (
            P.getValue("save_solution_csv")
            if (
                not overflown.get("optimization", True)
                and P.getValue("save_solution_csv") is not None
            ) else None
        )
    )

    invalid_points = []
    closest_indices = []

    if REFERENCE is not None:
        _d = P.getValue("reference_dist")

        for rx, ry, rt in REFERENCE:

            # Closest index
            _ci = 0
            __t = 0

            # Selected last
            selected_last = False

            while True:
                # Find closest point in time domain
                _ci = (abs(_t[:-1] + __t - rt)).argmin()

                # In case that we select the last point
                # Do it again for next repetition of the trajectory
                if _ci == len(_t) - 2:
                    if selected_last:
                        # Extra condition for non-closed paths.
                        break
                    __t += _t[-1]
                    selected_last = True
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
                for ts in (
                    range(int(_t[-1])) if overlap > 0
                    else chain(range(int(_t[-1]) + 1), _t[-1])
                ):
                    _closest = numpy.abs(
                        numpy.subtract(REFERENCE[:, 2], ts)
                    ).argmin()

                    if _closest >= len(REFERENCE) - 1:
                        ts = ts - 1
                        break

                    ngplot.pointsScatter(
                        # Trick to force 2D array.
                        REFERENCE[_closest, None, :2],
                        s = P.getValue("plot_timelines_size"),
                        color = "black"
                    )

                    if ts % 4 == 0:
                        ngplot.labelText(
                            REFERENCE[_closest, :2],
                            ts,
                            verticalalignment = "top",
                            horizontalalignment = "left",
                            fontsize = 6
                        )

                    _closest_p = numpy.abs(
                        numpy.subtract(_t[:-1], ts)
                    ).argmin()

                    ngplot.pointsScatter(
                        points[_closest_p, None, :2],
                        s = P.getValue("plot_timelines_size"),
                        color = "red"
                    )

                    if ts % 4 == 0:
                        ngplot.labelText(
                            points[_closest_p, :2],
                            ts,
                            verticalalignment = "bottom",
                            horizontalalignment = "right",
                            fontsize = 6,
                            color = "red",
                        )

                    ngplot.pointsPlot(
                        numpy.vstack(
                            (points[_closest_p, :2], REFERENCE[_closest, :2])
                        ),
                        color = "red",
                        linewidth = P.getValue("plot_timelines_width"),
                        linestyle = (
                            "--" if pointDistance(
                                points[_closest_p, :2], REFERENCE[_closest, :2]
                            ) < 5.0 else " "
                        )
                    )

            if P.getValue("plot_reference"):
                # Plot only to the last time point
                # That is specified by ts, which can be altered by previous if.
                _closest = numpy.abs(
                    numpy.subtract(REFERENCE[:, 2], ts)
                ).argmin()
                _closest_p = numpy.abs(numpy.subtract(_t[:-1], ts)).argmin()

                ngplot.pointsPlot(
                    REFERENCE[:_closest, :2],
                    color = "black",
                    linewidth = P.getValue("plot_reference_width")
                )

                if P.getValue("plot_solution"):
                    ngplot.pointsPlot(
                        points[:_closest_p, :2],
                        color="orange",
                        linewidth = P.getValue("plot_reference_width")
                    )

            if len(invalid_points) > 0:
                ngplot.pointsScatter(
                    numpy.asarray(invalid_points),
                    color = "blue",
                    marker = "x",
                    s = 1
                )


    if len(invalid_points) > 0:
        return float(penalty)


    # Locate points where overtaking occurs
    # Centerline is used to obtain track progression.
    # Do not do this when not optimizing; just to avoid
    # having duplicate marker(s).
    if (P.getValue("plot_overtaking")
            and REFERENCE is not None
            and CENTERLINE is not None
            and overflown.get("optimization", True)):
        # It does not actually plot, just sends the data via Queue
        # to the parent process.
        # That said, plotting has to be handled by the optimizer.
        if REFERENCE_PROGRESS is None:
            REFERENCE_PROGRESS = [
                trajectoryClosestIndex(CENTERLINE, REFERENCE[_i, :2])
                for _i in range(len(REFERENCE))
            ]

        overtaken = False
        prev_rd = REFERENCE_PROGRESS[0]
        prev_pd = trajectoryClosestIndex(
            CENTERLINE,
            points[closest_indices[0], :2]
        )
        # reference_end = trajectoryClosestIndex(CENTERLINE, points[-1, :2])
        for _i, (rx, ry, _) in enumerate(REFERENCE):
            rd = REFERENCE_PROGRESS[_i]  # Closest reference in the same time
            pd = trajectoryClosestIndex(
                CENTERLINE,
                points[closest_indices[_i], :2]
            )

            # This sequence should ensure that only correct overtaking
            # points are selected and it should be comfortable with
            # rotating the centerline (i.e., having the 0 points somewhere
            # along the way).
            # if rd > reference_end:
            #     break

            if prev_rd > rd:
                rd += len(CENTERLINE)

            if prev_pd > pd:
                pd += len(CENTERLINE)

            # This should apply only for non-closed paths.
            if pd - prev_pd > 50:
                if overlap > 0:
                    print0 (
                        "WARNING: Detected jump in the trajectory "
                        "track progress."
                    )
                    print0 (
                        f"rx: {rx}\try: {ry}\trt: {_}\n"
                        f"rd: {rd}\tpd: {pd}\n"
                        f"prev_rd: {prev_rd}\tprev_pd: {prev_pd}"
                    )
                break

            # if rd > 50 and pd > rd and not overtaken:
            if pd > rd and not overtaken:
                overtaken = True
                OVERTAKING_POINTS.put(points[closest_indices[_i], :2])
            # elif pd < rd and overtaken:
            #     overtaken = False

            prev_rd = rd
            prev_pd = pd

        if not overtaken:
            _t[-1] += P.getValue("favor_overtaking")

    return float(_t[-1])
