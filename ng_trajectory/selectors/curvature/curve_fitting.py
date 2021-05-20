#!/usr/bin/env python3.6
# curve_fitting.py
"""Curve fitting library for curvature-based point selection.

Implemented by Ondra Benedikt.
"""
######################
# Imports & Globals
######################

import numpy as np
from scipy.interpolate import CubicSpline
from enum import Enum
from scipy.interpolate import interp1d


######################
# InterpMethod (enum)
######################

class InterpMethod(Enum):
    I1D_LIN = 1 # interp 1D linear
    I1D_QUA = 2 # interp 1D quadratic
    SPL_CUB = 3 # cubic spline


######################
# Functions
######################

def get_linspace(n_points: int):
    """Create a linspace alpha \in [0,1) with n_points."""
    return np.linspace(0, 1, n_points, endpoint = False)


def interpolate_points(points: np.ndarray, interp_size: int = 400, downsample: int = 3, method: InterpMethod = InterpMethod.SPL_CUB) -> np.ndarray:
    """ Interpolate points in 2D as a parametric curve c(alpha), alpha \in [0,1)

    Arguments:
    points -- points to interpolate, nx2 np.ndarray
    interp_size -- number of points used for the interpolation, int, default 400
    downsample -- every [downsample] point will be taken
    method -- which interpolation method to use, InterpMethod, default SPL_CUB

    Returns:
    ipoints -- interpolated points
    """

    if downsample and downsample > 1:  # TODO: this might not be robust
        points = points[::downsample,:]
        points = np.vstack((points, points[0, :]))

    alpha = get_linspace(interp_size)

    # Linear length along the line:
    distance = np.cumsum( np.sqrt(np.sum( np.diff(points, axis=0)**2, axis=1 ) ) )
    distance = np.insert(distance, 0, 0)/distance[-1]

    if method == InterpMethod.SPL_CUB:
        interpolator = CubicSpline(distance, points, axis=0, bc_type="periodic")
    elif method == InterpMethod.I1D_LIN:
        interpolator = interp1d(distance, points, kind='linear', axis=0)
    elif method == InterpMethod.I1D_QUA:
        interpolator = interp1d(distance, points, kind='quadratic', axis=0)
    else:
        raise Exception("Interpolation method {} is not supported.".format(method))

    return  interpolator(alpha)


def smoothen(points: np.ndarray, kernel_size: int) -> np.ndarray:
    """Smoothen the points by a convolution with kernel of size kernel_size (averaging).

    Arguments:
    points -- points to smoothen
    kernel_size -- size of the averaging window

    Returns:
    smoothened_points
    """
    kernel = np.ones(kernel_size) / kernel_size
    return np.convolve(points, kernel, mode='same')


def get_derivatives(ipoints: np.ndarray, interp_size: int = 400) -> np.ndarray:
    """ Compute derivative of the interpolated points using numpy.gradient

    Arguments:
    ipoints -- interpolated points, nx2 np.ndarray
    interp_size -- number of points used for the interpolation needed to calculate delta, int, default 400

    Returns:
    derivatives -- derivative of the ipoints

    """
    alpha = get_linspace(interp_size)
    delta = alpha[1] - alpha[0]

    return np.gradient(ipoints, delta)[0]


def get_curvature(ipoints: np.ndarray, interp_size: int = 400):
    """ Calculate a curvature K of interpolated points.

    K = (x' * y'' - y' * x'') / ( x'**2 + y'**2 )**(3/2)
    """
    # First derivatives
    d1 = get_derivatives(ipoints, interp_size)
    dx1 = d1[:,0]
    dy1 = d1[:,1]

    # Second derivatives
    d2 = get_derivatives(d1, interp_size)
    dx2 = d2[:,0]
    dy2 = d2[:,1]

    # Calculate K
    return  np.divide(
                np.subtract(
                    np.multiply(dx1, dy2),
                    np.multiply(dy1, dx2)
                ),
                np.sqrt(
                    np.power(
                        np.add(
                            np.power(dx1, 2),
                            np.power(dy1, 2)
                        ),
                    3)
                )
            )
