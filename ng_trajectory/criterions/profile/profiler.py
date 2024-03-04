#!/usr/bin/env python3.6
# profiler.py
"""Compute criterion using speed profile and lap time."""
######################
# Imports & Globals
######################

import math
import numpy
import csv

from ng_trajectory.interpolators.utils import (
    pointsDistance,
    trajectoryClosestIndex,
)

from ng_trajectory.segmentators.utils import pointToMap

from typing import List, Tuple, TextIO


_mu = 0.2           # Friction coeficient
_g = 9.81           # Gravity acceleration coeficient
_m = 3.68           # Vehicle mass
_ro = 1.2           # Air density
_A = 0.3            # Frontal reference aerodynamic area
_cl = 1             # Drag coeficient
v_0 = 0             # Initial speed [m.s^-1]
v_lim = 4.5         # Maximum forward speed [m.s^-1]
a_acc_max = 0.8     # Maximum longitudal acceleration [m.s^-2]
a_break_max = 4.5   # Maximum longitudal decceleration [m.s^-2]
_lf = 0.191         # distance from the center of mass to the front axle [m]
_lr = 0.139         # distance from the center of mass to the rear axle [m]
CENTERLINE = None
FRICTION_MAP = None


######################
# Utilities
######################

def parametersSet(**kwargs) -> None:
    """Set parameters of the profiler."""
    for _parameter, _value in kwargs.items():
        globals()[_parameter] = _value


def listFlatten(ls: List[List[any]]) -> List[any]:
    """Flatten the list of lists into a single list.

    Arguments:
    l -- list to flatten, n-list of lists

    Returns:
    f -- flattened list, m-list

    Source:
    https://stackoverflow.com/questions/952914/how-to-make-a-flat-list-out-of-list-of-lists
    """
    return [item for sublist in ls for item in sublist]


######################
# Functions
######################

def Fz(v: float) -> float:
    """Compute force on z-axis.

    Arguments:
    v -- speed of the vehicle, [m.s^-1], float

    Returns:
    Fz -- force on z-axis, [N], float

    Source:
    speed_profile.py:Fz() by David Kopecky
    profile_trajectory.py:Fz()

    Note: Ported to Py3.6.
    """
    return _m * _g + 0.5 * (_ro * _A * _cl * v * v)


def Fy(v: float, k: float) -> float:
    """Compute force on y-axis (possibly lateral).

    Arguments:
    v -- speed of the vehicle, [m.s^-1], float
    k -- curvature of a turn, [m^-1], float

    Returns:
    Fy -- force on y-axis, [N], float

    Source:
    speed_profile.py:Fy() by David Kopecky
    profile_trajectory.py:Fy()

    Note: Ported to Py3.6.
    """
    return _m * v * v * k


def h(k: float, v: float, d: int, i: int = -1) -> float:
    """Compute maximum available longitudal acceleration.

    Arguments:
    k -- curvature of a turn, [m^-1], float
    v -- speed of the vehicle, [m.s^-1], float
    d -- direction in values {-1, 1}, int
    i -- index of the computation (for debugging), int, -1 by default

    Returns:
    a_max -- maximum longitudal acceleration, [m.s^-2], float

    Source:
    speed_profile.py:h() by David Kopecky
    profile_trajectory.py:h()

    Note: Ported to Py3.6.
    Differences:
        Removed comments.
    """
    fz_res = Fz(v)
    fy_res = Fy(v, k)

    if (_mu * _mu * fz_res * fz_res - fy_res * fy_res) <= 0:
        return 0
    # TODO: Umazat ten bod, ktery jde mimo.
    if d == -1:
        return -(math.sqrt(_mu * _mu * fz_res * fz_res - fy_res * fy_res) / _m)
    elif d == 1:
        return (math.sqrt(_mu * _mu * fz_res * fz_res - fy_res * fy_res) / _m)


def overlapCreate(points: numpy.ndarray, overlap: int):
    """Overlap the points array to make end-start transition smoother.

    Arguments:
    points -- points of a trajectory, nx3 numpy.ndarray
    overlap -- number of points to overlap on both sides of trajectory, int

    Returns:
    opoints -- overlapped trajectory, (n+2xoverlap)x3 numpy.ndarray

    Note: Overlap means, that the `overlap` points of trajectory are copied
          from the start to the end of the trajectory and vice versa. This
          helps to make the transition from the end to the start of
          the trajectory.
    """
    return numpy.vstack((points[-overlap:, :], points[:, :], points[:overlap]))


def overlapRemove(points: numpy.ndarray, overlap: int):
    """Remove the overlap in a points array.

    Arguments:
    points -- points of a trajectory, nx3 numpy.ndarray
    overlap -- number of overlapped points on both sides of trajectory, int

    Returns:
    opoints -- non-overlapped trajectory, (n-2xoverlap)x3 numpy.ndarray

    Note: Overlap means, that the `overlap` points of trajectory are copied
          from the start to the end of the trajectory and vice versa. This
          helps to make the transition from the end to the start of
          the trajectory.

    Note: This does the exact opposite of 'overlapCreate()'.
    """
    return points[overlap:-overlap, :]


def backward_pass(
        points: numpy.ndarray) -> Tuple[List[float], List[float], List[float]]:
    """Compute maximum and backward speed for a trajectory.

    Arguments:
    points -- points of a trajectory, nx3 numpy.ndarray

    Returns:
    v_bwd -- speed profile of the trajectory after backward step, [m.s^-1],
             list of floats
    v_max -- maximum speed profile (before backward step),
             [m.s^-1], list of floats
    cur -- curvature of the trajectory, [m^-1], list of floats

    Source:
    speed_profile.py:backward_pass() by David Kopecky
    profile_trajectory.py:backward_pass()

    Note: Ported to Py3.6.
    Differences:
        Removed comments.
        Moved a_break_max to arguments.
    """
    global FRICTION_MAP

    k = len(points)

    cur = numpy.zeros((len(points)))
    for i, p in enumerate(points):
        cur[i] = abs(p[2])

    v_bwd = numpy.zeros((len(points)))

    a = numpy.zeros((len(points)))
    v_max = numpy.zeros((len(points)))
    v_max_cr = numpy.zeros((len(points)))
    alim = numpy.zeros((len(points)))
    v_bwd[k % len(points)] = v_lim
    v_max[k % len(points)] = v_lim

    while k > 0:
        if FRICTION_MAP is not None:
            cxy = pointToMap(points[k - 1, :2])
            mu = FRICTION_MAP[cxy[0], cxy[1]] / 100.0
        else:
            mu = _mu

        v_max_cr[k - 1] = (
            math.sqrt(mu * _g / cur[k - 1]) if cur[k - 1] != 0.0
            else v_lim
        )
        v_max[k - 1] = min(v_max_cr[k - 1], v_lim)
        ds = math.sqrt(
            math.pow(points[k % len(points), 0] - points[k - 1, 0], 2)
            + math.pow(points[k % len(points), 1] - points[k - 1, 1], 2)
        )
        alim[k % len(points)] = (
            math.pow(v_max[k - 1], 2) - math.pow(v_bwd[k % len(points)], 2)
        ) / (2 * ds)
        a[k - 1] = -h(cur[k % len(points)], v_bwd[k % len(points)], -1, k)
        a[k - 1] = min(min(a[k - 1], a_break_max), alim[k % len(points)])

        try:
            v_bwd[k - 1] = math.sqrt(
                (v_bwd[k % len(points)] * v_bwd[k % len(points)])
                + (2 * a[k - 1] * ds)
            )
        except ValueError:
            # Math domain error, requested deceleration is too large
            v_bwd[k - 1] = 0.0

        k = k - 1

    return v_bwd, v_max, cur


def forward_pass(
        points: numpy.ndarray,
        v_bwd: List[float],
        v_max: List[float],
        cur: List[float]) -> Tuple[List[float], List[float], List[float]]:
    """Compute forward speed for a trajectory. Preceded by `backward_pass()`.

    Arguments:
    points -- points of a trajectory, nx3 numpy.ndarray
    v_bwd -- speed profile of the trajectory after backward step, [m.s^-1],
             list of floats
    v_max -- maximum speed profile (before backward step),
             [m.s^-1], list of floats
    cur -- curvature of the trajectory, [m^-1], list of floats

    Returns:
    v_fwd -- speed profile of the trajectory after forward step, [m.s^-1],
             list of floats
    a -- final acceleration profile of the trajectory, [m.s^-2], list of floats
    t -- time of reaching the points of the trajectory, [s], list of floats

    Source:
    speed_profile.py:forward_pass() by David Kopecky
    profile_trajectory.py:forward_pass()

    Note: Ported to Py3.6.
    Differences:
        Removed comments.
        Moved a_acc_max to arguments.
        Changed the order of arguments to match the output
        order of 'backward_pass()'.
    """
    k = 0
    t = numpy.zeros((len(points)))
    v_fwd = numpy.zeros((len(points)))
    a = numpy.zeros((len(points)))
    t[k] = 0
    v_fwd[k] = v_0

    while k < len(points):
        ds = math.sqrt(
            math.pow(points[k, 0] - points[(k + 1) % len(points), 0], 2)
            + math.pow(points[k, 1] - points[(k + 1) % len(points), 1], 2)
        )
        a_lim = (
            v_bwd[(k + 1) % len(points)] * v_bwd[(k + 1) % len(points)]
            - v_fwd[k] * v_fwd[k]
        ) / (2 * ds)
        a[k] = h(cur[k], v_fwd[k], 1)
        a[k] = min(min(a[k], a_acc_max), a_lim)
        v_fwd[(k + 1) % len(points)] = math.sqrt(
            v_fwd[k] * v_fwd[k] + 2 * a[k] * ds
        )

        if v_fwd[(k + 1) % len(points)] + v_fwd[k] > 0.0:
            dt = 2 * ds / (v_fwd[(k + 1) % len(points)] + v_fwd[k])
        else:
            # TODO: It would be nice to actually stay at t[k] all the time.
            # But now all we can do is to just add some random time.
            dt = 100

        t[(k + 1) % len(points)] = (
            t[k] + min(dt, 100)
        )

        k = k + 1

    return v_fwd, a, t


def saveState(
        filename: str,
        points: numpy.ndarray,
        t: numpy.ndarray,
        v: numpy.ndarray,
        a: numpy.ndarray) -> bool:
    """Save the computed profile into a CSV file.

    Arguments:
    filename -- name of the file to save state, str
    points -- points of a trajectory with curvature, nx3 numpy.ndarray
    v -- speed profile of the trajectory, [m.s^-1], nx1 numpy.ndarray
    a -- acceleration profile of the trajectory, [m.s^-2], nx1 numpy.ndarray
    t -- time of reaching the points of the trajectory,
         [s], nx1 or (n+1)x1 numpy.ndarray

    Returns:
    success -- True if saved, otherwise False

    Note: This is inspired by the 'ng_trajectory_ros' wrapper.

    TODO: Resolve this.
    FIXME: This is probably not correct. Below I put the information from Tomas:
        _beta = math.atan(_lr * k[_i])  # << k[_i] in the CoG
        "k_radpm": k[_i],  # << k[i] in the CoG
        "vx_mps": v[_i] * math.cos(_beta),  # << k[_i] and v[_i] in the CoG
        "vy_mps": v[_i] * math.sin(_beta),  # << k[_i] and v[_i] in the CoG
        "a_mps2": a[_i],  # << a[_i] in the center of gravity
        "omega_radps": v[_i] * math.cos(_beta) * k[_i], # << k[_i] and v[_i] in the CoG
        "delta_rad": np.atan((_lf + _lr) * k[_i])  # << k[_i] in the center of rear axle

        Purely rear axle should be:
            # _beta not required
            writer.writerow(
                {
                    "t_s": t[_i],
                    "d_m": d[_i],
                    "x_m": x[_i],
                    "y_m": y[_i],
                    "psi_rad": psi[_i],
                    "k_radpm": k[_i],
                    "vx_mps": v[_i],
                    "vy_mps": 0,
                    "a_mps2": a[_i],
                    "omega_radps": v[_i] * k[_i],
                    "delta_rad": math.atan((_lf + _lr) * k[_i])
                }
            )

        Purely CoG should be:
            _beta = math.asin(_lr * k[_i])

            writer.writerow(
                {
                    "t_s": t[_i],
                    "d_m": d[_i],
                    "x_m": x[_i],
                    "y_m": y[_i],
                    "psi_rad": psi[_i] - _beta,
                    "k_radpm": k[_i],
                    "vx_mps": v[_i] * math.cos(_beta),
                    "vy_mps": v[_i] * math.sin(_beta),
                    "a_mps2": a[_i],
                    "omega_radps": v[_i] * k[_i],
                    "delta_rad": math.atan((_lf + _lr)/_lr * math.tan(_beta))
                }
            )
    """  # noqa: E501
    global CENTERLINE

    x, y, k = points[:, :3].T
    t, v, a = t.flatten(), v.flatten(), a.flatten()

    # Line distance (d) !! NOT TRACK PROGRESS !!
    d = pointsDistance(points)
    _d = d[0]
    d[0] = 0.0
    d = numpy.cumsum(d)

    # Centerline distance
    if CENTERLINE is not None:
        cd = pointsDistance(CENTERLINE)
        cd[0] = 0.0
        cd = numpy.cumsum(cd)

    # TODO: Share the information about closed path / lap time.
    if len(t) > len(x):
        t[0] = t[-1]
        d[0] = d[-1] + _d

    # Direction (angle from the point towards the next one)
    direction = numpy.asarray([
        math.atan2(
            y[(_i + 1) % len(points)] - _y,
            x[(_i + 1) % len(points)] - _x
        ) for _i, (_x, _y) in enumerate(points[:, :2])
    ])


    with open(filename, "w", newline = "") as f:
        writer = csv.DictWriter(
            f,
            fieldnames = [
                "t_s", "d_m", "x_m", "y_m",
                "psi_rad", "k_radpm",
                "vx_mps", "vy_mps", "a_mps2",
                "omega_radps", "delta_rad"
            ] + ([] if CENTERLINE is None else ["s_m"])
        )

        writer.writeheader()

        for _i in range(len(points)):
            _beta = math.atan(_lr * k[_i])

            writer.writerow({
                **{
                    "t_s": t[_i],
                    "d_m": d[_i],
                    "x_m": x[_i],
                    "y_m": y[_i],
                    "psi_rad": direction[_i] - _beta,
                    "k_radpm": k[_i],
                    "vx_mps": v[_i] * math.cos(_beta),
                    "vy_mps": v[_i] * math.sin(_beta),
                    "a_mps2": a[_i],
                    "omega_radps": v[_i] * math.cos(_beta) * k[_i],
                    "delta_rad": math.atan((_lf + _lr) * k[_i])
                }, **{
                    [] if CENTERLINE is None
                    else "s_m": cd[
                        trajectoryClosestIndex(
                            CENTERLINE,
                            numpy.asarray([x[_i], y[_i]])
                        )
                    ]}
            })


def profileCompute(
        points: numpy.ndarray,
        overlap: int = 0,
        fd: TextIO = None,
        lap_time: bool = False,
        save: str = None) \
        -> Tuple[numpy.ndarray, numpy.ndarray, numpy.ndarray]:
    """Compute the speed profile using overlap.

    Arguments:
    points -- points of a trajectory with curvature, nx3 numpy.ndarray
    overlap -- size of trajectory overlap, int, default 0 (disabled)
    lap_time -- when True, time is appended with lap_time, bool, default False
    save -- when given, save the solution to this file, default None (disabled)

    Returns:
    v_fwd -- speed profile of the trajectory after forward step, [m.s^-1],
             nx1 numpy.ndarray
    a -- final acceleration profile of the trajectory,
         [m.s^-2], nx1 numpy.ndarray
    t -- time of reaching the points of the trajectory,
         [s], nx1 or (n+1)x1 numpy.ndarray
    """
    # Overlap points
    if overlap > 0:
        # Overlapping fails when there is not enough points.
        # So we limit the overlap here.
        if len(points) < overlap:
            overlap = len(points)
        _points = overlapCreate(points, overlap)
    else:
        _points = numpy.asarray(points)

    # Compute speed profile
    bwd, mx, cur = backward_pass(points = _points)
    v, a, t = forward_pass(_points, bwd, mx, cur)

    # Convert to numpy.ndarray
    bwd = bwd[:, numpy.newaxis]
    mx = mx[:, numpy.newaxis]
    cur = cur[:, numpy.newaxis]
    _v = v[:, numpy.newaxis]
    _a = a[:, numpy.newaxis]
    _t = t[:, numpy.newaxis]

    if lap_time:
        __t = _t[-overlap]

    # Remove overlap and convert to numpy.ndarray
    if overlap > 0:
        _v = overlapRemove(_v, overlap)
        _a = overlapRemove(_a, overlap)
        _t = overlapRemove(_t, overlap)

        # Fix time array
        if lap_time:
            _t = numpy.vstack((_t, __t))

        _t = numpy.subtract(_t, _t[0])

        bwd = overlapRemove(bwd, overlap)
        mx = overlapRemove(mx, overlap)
        cur = overlapRemove(cur, overlap)

        if fd:
            fd.write("Backward speed: %s\n" % str(listFlatten(bwd.tolist())))
            fd.write("Maximum speed: %s\n" % str(listFlatten(mx.tolist())))
            fd.write("Curvature: %s\n" % str(listFlatten(cur.tolist())))
            fd.write("Final speed: %s\n" % str(listFlatten(_v.tolist())))
            fd.flush()

        # Check for overlap consistency
        # e.g., when dynamic parameters are large, but overlap small
        #       it is not smooth enough on the path end
        """
        Small comment on the derivation.
        v[-1] -- initial speed
        v[0] -- end speed
        _ds -- distance between the points

        Since we know the speeds, the time to travel from A to B is given by:
        $
            t = _ds / ( (v[0] + v[-1]) / 2 )
        $

        Acceleration required to attain these speeds is:
        $
            a = (v[0] - v[-1]) / t
        $

        Substituing them into each other yields:
        $
            a = ( v[0]^2 - v[-1]^2 ) / 2s
        $

        And this acceleration has to be in the limits.
        """
        _ds = pointsDistance(points[[-1, 0], :])[0]
        _ac = (_v[0]**2 - _v[-1]**2) / (2 * _ds)
        try:
            assert -a_break_max <= _ac <= a_acc_max, \
                (
                    "Overlap is violating the acceleration limit "
                    "(-%f <= %f <= %f), so it is not long enough to make "
                    "the path velocity smooth."
                    % (a_break_max, _ac, a_acc_max)
                )
        except AssertionError:
            if (
                not math.isclose(_ac, a_acc_max)
                and not math.isclose(_ac, -a_break_max)
            ):
                raise
    elif lap_time:  # no overlap; use the last time values
        _t = numpy.vstack((_t, _t[-1]))
        _t[0] = 0

    if save is not None:
        saveState(save, points, _t, _v, _a)

    return _v, _a, _t
