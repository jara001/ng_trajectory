#!/usr/bin/env python3.6
# main.py
"""Compute criterion using speed profile and lap time.

[1] R. N. Jazar, Advanced Vehicle Dynamics. Cham: Springer International
Publishing, 2019. doi: 10.1007/978-3-030-13062-6.
"""
######################
# Imports & Globals
######################

import numpy
import math

from ng_trajectory.abc.criterions import CriterionABC

from typing import (
    Any,
    Dict,
    Optional,
)


# Parameters
from ng_trajectory.parameter import ParameterList
P = ParameterList()
P.createAdd("overlap", 0, int, "Size of the trajectory overlap. 0 disables this.", "")

P.createAdd("v_0", 0, float, "Initial speed [m.s^-1]", "init")
P.createAdd("v_lim", 4.5, float, "Maximum forward speed [m.s^-1]", "init")
P.createAdd("a_acc_max", 0.8, float, "Maximum longitudal acceleration [m.s^-2]", "init")
P.createAdd("a_break_max", 4.5, float, "Maximum longitudal decceleration [m.s^-2]", "init")

P.createAdd("g", 9.81, float, "Gravity acceleration coeficient [m.s^-2]", "vehicle")
P.createAdd("m", 3.68, float, "Vehicle mass [kg]", "vehicle")
P.createAdd("l_f", 0.16, float, "Distance between center of mass and front axle [m]", "vehicle")
P.createAdd("l_r", 0.16, float, "Distance between center of mass and rear axle [m]", "vehicle")
P.createAdd("h", 0.08, float, "Height of the center of mass [m]", "vehicle")
P.createAdd("I_z", 0.051558333, float, "Moment of interia of the vehicle around z-axis [kg.m^2]", "vehicle")

P.createAdd("C_sf", 7.5, float, "Longitudinal slip coefficient of the front tire [-]", "tire")
P.createAdd("C_sr", 7.5, float, "Longitudinal slip coefficient of the rear tire [-]", "tire")
P.createAdd("C_sa", 0.5, float, "Longitudinal drop factor [-]", "tire")
P.createAdd("C_af", 8.5, float, "Cornering stiffness of the front tire [-]", "tire")
P.createAdd("C_ar", 8.5, float, "Cornering stiffness of the rear tire [-]", "tire")
P.createAdd("C_as", 0.5, float, "Lateral drop factor [-]", "tire")

P.createAdd("s_s", 0.1, float, "Saturation slip ratio [-]", "tire")
P.createAdd("alpha_s", 0.0873, float, "Saturation sideslip angle [rad]", "tire")

P.createAdd("cl", 0.3, float, "Air drag coefficient [-]", "aerodynamic")
P.createAdd("ro", 1.249512, float, "Air density [N.m^-2]", "aerodynamic")
P.createAdd("A", 0.3, float, "Effective flow surface [m^2]", "aerodynamic")


######################
# Functions
######################

addOverlap = lambda points, overlap: numpy.vstack((points[-overlap:, :], points[:, :], points[:overlap]))
removeOverlap = lambda points, overlap: points[overlap:-overlap]

"""
Constrain a value to be lb <= value <= ub.
"""
C = lambda lb, value, ub: max(min(value, ub), lb)

"""
Saturation function constrains a value to defined limit.
"""
S = lambda value, limit: max(min(value, limit), -limit)


######################
# Equations
######################

r"""
Forces on 'z-axis', i.e. oriented down to the surface. The original
version assumes load transfer:

$$
F_{zf} = m * g * l_r / l - m * (a_x - omega * v_y) * h / l,
F_{zr} = m * g * l_f / l + m * (a_x - omega * v_y) * h / l,
$$

where first term is the static load on the axle and second term
is load transfer (caused by longitudinal and lateral acceleration).

Note: We try to assume no transfer load caused by longitudinal
acceleration.
"""
F_zf = lambda omega, v_y: P.getValue("m") * P.getValue("g") * P.getValue("l_r") / (P.getValue("l_f") + P.getValue("l_r")) + P.getValue("m") * omega * v_y * P.getValue("h") / (P.getValue("l_f") + P.getValue("l_r"))
F_zr = lambda omega, v_y: P.getValue("m") * P.getValue("g") * P.getValue("l_f") / (P.getValue("l_f") + P.getValue("l_r")) - P.getValue("m") * omega * v_y * P.getValue("h") / (P.getValue("l_f") + P.getValue("l_r"))


r"""
Tire forces based on elliptic combined tire forces:

$$
F_{xf} = F_{zf} * C_{sf} * S(s_f - s_s) * \sqrt{1 - C_{sa} * (S(\alpha_f - \alpha_s) / \alpha_s)^2}
F_{xr} = F_{zr} * C_{sr} * S(s_r - s_s) * \sqrt{1 - C_{sa} * (S(\alpha_r - \alpha_s) / \alpha_s)^2}
F_{yf} = - F_{zf} * C_{af} * S(\alpha_f - \alpha_s) * \sqrt{1 - C_{as} * (S(s_f - s_s) / s_s)^2}
F_{yr} = - F_{zr} * C_{ar} * S(\alpha_r - \alpha_s) * \sqrt{1 - C_{as} * (S(s_r - s_s) / s_s)^2}
$$

which basically alters ratios between F_x / F_y and F_z according
to the slip (s) and slip angle (alpha).

Function S(value - limit) constrains the value to <-limit, limit>.

Note: We assume that slips (s) are maximal, i.e. s_f = s_r = s_s and
that slip angle is zero, i.e., alpha_f = alpha_r = 0.

Note: These assumptions (with current implementation) disables reverse
driving.
"""
F_xf = lambda omega, v_y: F_zf(omega, v_y) * P.getValue("C_sf") * P.getValue("s_s")
F_xr = lambda omega, v_y: F_zr(omega, v_y) * P.getValue("C_sr") * P.getValue("s_s")
F_yf = lambda omega, v_y: -F_zf(omega, v_y) * P.getValue("C_af") * 0
F_yr = lambda omega, v_y: -F_zr(omega, v_y) * P.getValue("C_ar") * 0


r"""
Bicycle model is desribed by following set of equations
(without control part):

$$
F_x = F_{xf} * cos(delta) + F_{xr} - F_{yf} * sin(delta)
F_y = F_{yf} * cos(delta) + F_{yr} + F_{xf} * sin(delta)
M_z = l_f * F_{yf} * cos(delta) + l_f * F_{xf} * sin(delta) - l_r * F_{yr}
$$
"""
F_x = lambda delta, omega, v_y: F_xf(omega, v_y) * math.cos(delta) + F_xr(omega, v_y) - F_yf(omega, v_y) * math.sin(delta)
F_y = lambda delta, omega, v_y: F_yf(omega, v_y) * math.cos(delta) + F_yr(omega, v_y) + F_xf(omega, v_y) * math.sin(delta)
M_z = lambda delta, omega, v_y: P.getValue("l_f") * F_yf(omega, v_y) * math.cos(delta) + P.getValue("l_f") * F_xf(omega, v_y) * math.sin(delta) - P.getValue("l_r") * F_yr(omega, v_y)


r"""
Aerodynamic forces acting on the car are simplified to:

$$
F_a = 0.5 * ro * cl * A * (v_x)^2
$$
"""
F_a = lambda v_x: 0.5 * P.getValue("ro") * P.getValue("cl") * P.getValue("A") * v_x * v_x


r"""
Equations of motion are (without the control part):

$$
\dot{v}_x = ( 1 / m ) * (F_x - F_a) + omega * v_y
\dot{v}_y = ( 1 / m ) * F_y - omega * v_x
\dot{omega} = M_z / I_z
$$
"""
dv_x = lambda delta, omega, v_x, v_y: (1 / P.getValue("m")) * (F_x(delta, omega, v_y) - F_a(v_x)) + omega * v_y
dv_y = lambda delta, omega, v_x, v_y: (1 / P.getValue("m")) * F_y(delta, omega, v_y) - omega * v_x
domega = lambda delta, omega, v_y: M_z(delta, omega, v_y) / P.getValue("I_z")


######################
# Model functions
######################

def forward_pass(points: numpy.ndarray):
    """Do the forward pass and obtain velocity for each point.

    Arguments:
    points -- points of a trajectory with curvature, nx3 numpy.ndarray

    Returns:
    v_fwd -- velocities in the points after forward pass, nx1 numpy.ndarray
    v_max -- maximum permissible velocities in the points, nx1 numpy.ndarray
    """
    v_fwd = numpy.zeros(len(points))
    v_max = numpy.zeros(len(points))

    beta = 0
    _ca = 0.5 * P.getValue("ro") * P.getValue("cl") * P.getValue("A")
    v_fwd[0] = min(
        P.getValue("v_0"),
        math.sqrt(
            math.sqrt(
                ((min(P.getValue("C_sf"), P.getValue("C_sr")) * P.getValue("s_s"))**2 * P.getValue("m")**2 * P.getValue("g")**2) / (_ca**2 - 2 * _ca * P.getValue("m") * abs(points[0, 2]) * math.tan(beta) + P.getValue("m")**2 * points[0, 2]**2 * math.tan(beta)**2 + P.getValue("m")**2 * points[0, 2]**2)
            )
        )
    )

    for i in range(len(points)):
        if i == len(points) - 1:
            break

        _x, _y, _k = points[i, :]
        _x1, _y1, _k1 = points[(i + 1) % len(points), :]


        # Distance between points
        ds = numpy.hypot(
            _x1 - _x,
            _y1 - _y
        )


        if _k1 != 0:
            # Compute maximum permissible steady-state vehicle speed
            # Given zero longitudinal force
            # Edit: Actually not. It is without Fa only.
            # v_fwd[i + 1] = math.sqrt(
            #     math.sqrt(
            #         ((min(P.getValue("C_sf"), P.getValue("C_sr")) * P.getValue("s_s"))**2 * P.getValue("g")**2 * (1/_k1)**2)/(1 + math.tan(beta)**2)
            #     )
            # )
            # With aerodynamics:
            _ca = 0.5 * P.getValue("ro") * P.getValue("cl") * P.getValue("A")
            v_fwd[i + 1] = math.sqrt(
                math.sqrt(
                    ((min(P.getValue("C_sf"), P.getValue("C_sr")) * P.getValue("s_s"))**2 * P.getValue("m")**2 * P.getValue("g")**2) / (_ca**2 - 2 * _ca * P.getValue("m") * _k1 * math.tan(beta) + P.getValue("m")**2 * _k1**2 * math.tan(beta)**2 + P.getValue("m")**2 * _k1**2)
                )
            )


            # Constrain it with the maximum allowed speed
            v_fwd[i + 1] = min(v_fwd[i + 1], P.getValue("v_lim"))
            v_max[i + 1] = v_fwd[i + 1]


            # Compute acceleration target, i.e., how much we have to accelerate
            # in order to match maximum speed on the next point.
            a_tar = (v_fwd[i + 1]**2 - v_fwd[i]**2) / (2 * ds)


            # Compute maximum acceleration with respect to Kamm's circle.
            # As we are calculating v_x, I assume that we want mu_x;
            # that should be the maximum friction C_s * s_s
            _a_lim = (min(P.getValue("C_sf"), P.getValue("C_sr")) * P.getValue("s_s"))**2 * P.getValue("g")**2 - (v_fwd[i]**4 * _k**2)

            if _a_lim < 0:
                # FIXME: Možná tady je mínus?
                a_lim = math.sqrt(-_a_lim)  # *-1
            else:
                a_lim = math.sqrt(_a_lim)


        else:
            # Acceleration target is to match maximum speed
            a_tar = (P.getValue("v_lim")**2 - v_fwd[i]**2) / (2 * ds)


            # Kamm's circle with zero turning. (Does this even make sense?)
            a_lim = math.sqrt(
                (min(P.getValue("C_sf"), P.getValue("C_sr")) * P.getValue("s_s"))**2 * P.getValue("g")**2
            )


        # Compute acceleration (use all constraints together)
        # a = S(a_tar, S(a_lim, P.getValue("a_acc_max")))
        a = min(min(a_tar, a_lim), P.getValue("a_acc_max"))
        # print(v_fwd[i], v_fwd[i+1], v_max[i+1], a_tar, a_lim, a)

        # Modify velocity of the next point
        v_fwd[i + 1] = math.sqrt(v_fwd[i]**2 + 2 * a * ds)


    return v_fwd, v_max



def backward_pass(points: numpy.ndarray, v_fwd: numpy.ndarray):
    """Do the backward pass and obtain states for each point.

    Arguments:
    points -- points of a trajectory with curvature, nx3 numpy.ndarray
    v_fwd -- velocities of the points, nx1 numpy.ndarray

    Returns:
    v -- velocities in the points, nx1 numpy.ndarray
    a -- accelerations in the points, nx1 numpy.ndarray
    t -- time required to reach target point, nx1 numpy.ndarray
    """
    v = numpy.zeros(len(points))
    a = numpy.zeros(len(points))
    dt = numpy.zeros(len(points))

    v[-1] = v_fwd[-1]

    # beta = 0
    mu = (min(P.getValue("C_sf"), P.getValue("C_sr")) * P.getValue("s_s"))

    for i in reversed(range(len(points))):
        if i == 0:
            break
        _x, _y, _k = points[i, :]
        _x1, _y1, _k1 = points[(i - 1) % len(points), :]


        # Distance between points
        ds = numpy.hypot(
            _x1 - _x,
            _y1 - _y
        )


        if _k1 != 0:
            # Compute acceleration target, i.e., how much we have to accelerate
            # in order to match maximum speed on the next point.
            a_tar = (v_fwd[i - 1]**2 - v[i]**2) / (2 * ds)


            # Compute maximum acceleration with respect to Kamm's circle.
            _a_lim = mu**2 * P.getValue("g")**2 - (v[i]**4 * _k**2)

            if _a_lim < 0:
                # FIXME: Možná tady je mínus?
                a_lim = math.sqrt(-_a_lim)  # *-1
            else:
                a_lim = math.sqrt(_a_lim)


        else:
            # Acceleration target is to match maximum speed
            a_tar = (P.getValue("v_lim")**2 - v[i]**2) / (2 * ds)


            # Kamm's circle with zero turning. (Does this even make sense?)
            a_lim = math.sqrt(
                mu**2 * P.getValue("g")**2
            )


        # Compute acceleration (use all constraints together)
        # a = S(a_tar, S(a_lim, P.getValue("a_break_max")))
        a[i - 1] = min(min(a_tar, a_lim), P.getValue("a_break_max"))


        # Modify velocity of the next point
        try:
            v[i - 1] = math.sqrt(v[i]**2 + 2 * a[i - 1] * ds)
        except ValueError:
            print (v[i], v[i - 1], a, ds, _x, _y, _k, _x1, _y1, _k1)
            # raise
            v[i - 1] = 0


        # Compute time required to drive over ds
        dt[i] = ds / ((v[i] + v[i - 1]) / 2)


    return v, a, numpy.cumsum(dt)


######################
# Profile compute
######################

def computeProfile(points):
    """Compute the speed profile using Jazar model.

    Arguments:
    points -- points of a trajectory with curvature, nx3 numpy.ndarray

    Returns:
    v -- speed profile of the trajectory, [m.s^-1], nx1 numpy.ndarray
    a -- final acceleration profile of the trajectory, [m.s^-2],
         nx1 numpy.ndarray
    t -- time of reaching the points of the trajectory, [s], nx1 numpy.ndarray
    """
    # Add overlap if requested
    if P.getValue("overlap") > 0:
        _points = addOverlap(points, P.getValue("overlap"))
    else:
        _points = numpy.asarray(points)

    v_fwd, v_max = forward_pass(_points)
    v, a, t = backward_pass(_points, v_fwd)


    # If used, remove the overlap
    if P.getValue("overlap") > 0:
        points = removeOverlap(points, P.getValue("overlap"))
        v = removeOverlap(v, P.getValue("overlap"))
        a = removeOverlap(a, P.getValue("overlap"))
        t = removeOverlap(t, P.getValue("overlap"))
        t = numpy.subtract(t, t[0])

    return v, a, t


######################
# General functions
######################

class JazarCriterion(CriterionABC):

    def init(self, **kwargs) -> Optional[Dict[str, Any]]:
        """Initialize criterion."""
        P.updateAll(kwargs, reset = False)


    def compute(self, points: numpy.ndarray, **overflown) -> float:
        """Compute the speed profile using overlap.

        Arguments:
        points -- points of a trajectory with curvature, nx3 numpy.ndarray
        **overflown -- arguments not caught by previous parts

        Returns:
        t -- time of reaching the last point of the trajectory, [s], float
             minimization criterion
        """
        P.updateAll(overflown, reset = False)

        _, _, t = computeProfile(points)

        return float(t[-1])
