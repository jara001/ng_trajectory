#!/usr/bin/env python3.6
# profiler.py
"""Compute criterion using speed profile and lap time.
"""
######################
# Imports & Globals
######################

import math, numpy

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


######################
# Utilities
######################

def parametersSet(**kwargs) -> None:
    """Set parameters of the profiler."""

    for _parameter, _value in kwargs.items():
        globals()[_parameter] = _value


######################
# Functions
######################

def Fz(v: float) -> float:
    """Computes force on z-axis.

    Arguments:
    v -- speed of the vehicle, [m.s^-1], float

    Returns:
    Fz -- force on z-axis, [N], float

    Source:
    speed_profile.py:Fz() by David Kopecky
    profile_trajectory.py:Fz()

    Note: Ported to Py3.6.
    """
    return _m*_g + 0.5*(_ro*_A*_cl*v*v)


def Fy(v: float, k: float) -> float:
    """Computes force on y-axis (possibly lateral).

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
    return _m*v*v*k


def h(k: float, v: float, d: int, i: int = -1) -> float:
    """Computes maximum available longitudal acceleration.

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
    fy_res = Fy(v,k)

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

    Note: Overlap means, that the `overlap` points of trajectory are copied from
    the start to the end of the trajectory and vice versa. This helps to make
    the transition from the end to the start of the trajectory.
    """
    return numpy.vstack((points[-overlap:, :], points[:, :], points[:overlap]))


def overlapRemove(points: numpy.ndarray, overlap: int):
    """Remove the overlap in a points array.

    Arguments:
    points -- points of a trajectory, nx3 numpy.ndarray
    overlap -- number of overlapped points on both sides of trajectory, int

    Returns:
    opoints -- non-overlapped trajectory, (n-2xoverlap)x3 numpy.ndarray

    Note: Overlap means, that the `overlap` points of trajectory are copied from
    the start to the end of the trajectory and vice versa. This helps to make
    the transition from the end to the start of the trajectory.

    Note: This does the exact opposite of 'overlapCreate()'.
    """
    return points[overlap:-overlap, :]


def backward_pass(points: numpy.ndarray) -> Tuple[List[float], List[float], List[float]]:
    """Computes maximum and backward speed for a trajectory.

    Arguments:
    points -- points of a trajectory, nx3 numpy.ndarray

    Returns:
    v_bwd -- speed profile of the trajectory after backward step, [m.s^-1],
             list of floats
    v_max -- maximum speed profile (before backward step), [m.s^-1], list of floats
    cur -- curvature of the trajectory, [m^-1], list of floats

    Source:
    speed_profile.py:backward_pass() by David Kopecky
    profile_trajectory.py:backward_pass()

    Note: Ported to Py3.6.
    Differences:
        Removed comments.
        Moved a_break_max to arguments.
    """

    k = len(points)
    k_ax = numpy.linspace(0, k, k)

    cur = numpy.zeros((len(points)))
    for i, p in enumerate(points):
        cur[i] = (abs(p[2]) if p[2] != 0 else 0.001)

    v_bwd = numpy.zeros((len(points)))

    a = numpy.zeros((len(points)))
    v_max = numpy.zeros((len(points)))
    v_max_cr = numpy.zeros((len(points)))
    alim = numpy.zeros((len(points)))
    v_bwd[k%len(points)] = v_lim
    v_max[k%len(points)] = v_lim

    while k > 0:
        v_max_cr[k-1] = math.sqrt(_mu*_g/cur[k-1])
        v_max[k-1] = min(v_max_cr[k-1],v_lim)
        ds = math.sqrt(math.pow(points[k%len(points), 0] - points[k-1, 0],2)+math.pow(points[k%len(points), 1] - points[k-1, 1],2))
        alim[k%len(points)] = (math.pow(v_max[k-1],2)-math.pow(v_bwd[k%len(points)],2))/(2*ds)
        a[k-1] = -h(cur[k%len(points)],v_bwd[k%len(points)],-1, k)
        a[k-1] = min(min(a[k-1],a_break_max),alim[k%len(points)])
        v_bwd[k-1] = math.sqrt((v_bwd[k%len(points)] * v_bwd[k%len(points)]) + (2 * a[k-1] * ds))
        k = k - 1

    return v_bwd, v_max, cur


def forward_pass(points: numpy.ndarray, v_bwd: List[float], v_max: List[float], cur: List[float]) \
    -> Tuple[List[float], List[float], List[float]]:
    """Computes forward speed for a trajectory. Preceded by `backward_pass()`.

    Arguments:
    points -- points of a trajectory, nx3 numpy.ndarray
    v_bwd -- speed profile of the trajectory after backward step, [m.s^-1],
             list of floats
    v_max -- maximum speed profile (before backward step), [m.s^-1], list of floats
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
        Changed the order of arguments to match the output order of 'backward_pass()'.
    """

    k = 0
    k_ax = numpy.linspace(0,len(points), len(points))
    t = numpy.zeros((len(points)))
    v_fwd = numpy.zeros((len(points)))
    a = numpy.zeros((len(points)))
    t[k] = 0
    v_fwd[k] = v_0

    while k < len(points):
        ds = math.sqrt(math.pow(points[k, 0] - points[(k + 1)%len(points), 0], 2) + math.pow(points[k, 1] - points[(k + 1)%len(points), 1], 2))
        a_lim = (v_bwd[(k+1)%len(points)]*v_bwd[(k+1)%len(points)] - v_fwd[k]*v_fwd[k])/(2*ds)
        a[k] = h(cur[k],v_fwd[k],1)
        a[k] = min(min(a[k], a_acc_max),a_lim)
        v_fwd[(k+1)%len(points)] = math.sqrt(v_fwd[k]*v_fwd[k] + 2*a[k]*ds)
        t[(k+1)%len(points)] = t[k] + 2*ds/(v_fwd[(k+1)%len(points)] + v_fwd[k])
        k = k + 1

    return v_fwd, a, t


def profileCompute(points: numpy.ndarray, overlap: int = 0, fd: TextIO = None) -> Tuple[numpy.ndarray, numpy.ndarray, numpy.ndarray]:
    """Compute the speed profile using overlap.

    Arguments:
    points -- points of a trajectory with curvature, nx3 numpy.ndarray
    overlap -- size of trajectory overlap, int, default 0 (disabled)

    Returns:
    v_fwd -- speed profile of the trajectory after forward step, [m.s^-1],
             nx1 numpy.ndarray
    a -- final acceleration profile of the trajectory, [m.s^-2], nx1 numpy.ndarray
    t -- time of reaching the points of the trajectory, [s], nx1 numpy.ndarray
    """

    # Overlap points
    if overlap > 0:
        _points = overlapCreate(points, overlap)
    else:
        _points = numpy.asarray(points)

    # Compute speed profile
    bwd, mx, cur = backward_pass(points = _points)
    v, a, t = forward_pass(_points,
                bwd, mx, cur)

    # Convert to numpy.ndarray
    bwd = bwd[:, numpy.newaxis]
    mx = mx[:, numpy.newaxis]
    cur = cur[:, numpy.newaxis]
    _v = v[:, numpy.newaxis]
    _a = a[:, numpy.newaxis]
    _t = t[:, numpy.newaxis]

    # Remove overlap and convert to numpy.ndarray
    if overlap > 0:
        _v = overlapRemove(_v, overlap)
        _a = overlapRemove(_a, overlap)
        _t = overlapRemove(_t, overlap)

        # Fix time array
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

    return _v, _a, _t

