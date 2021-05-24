#!/usr/bin/env python3.6
# main.py
"""Select points from a path based on its curvature.

Suggested and designed by Ondra Benedikt.
"""
######################
# Imports & Globals
######################

import numpy as np
from matplotlib import pyplot as plt
from scipy.interpolate import interp1d
from scipy.signal import find_peaks
from . import curve_fitting as cf


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize selector.

    Arguments:
    **kwargs -- overflown arguments
    """
    pass


def select(points: np.ndarray, remain: int, track_name: str = "unknown", plot: bool = False, **overflown) -> np.ndarray:
    """Select points from the path uniformly.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int
    track_name -- name of the track, str, default "unknown"
    plot -- when True, images are generated, default False
    **overflown -- arguments not caught by previous parts

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray

    Note: Since in this version we are not able to set the number of points, raise an exception for positive numbers.
    """

    if remain > 0:
        raise ValueError("Exact number of points cannot be handled by 'curvature' selector.")


    # Interpolate points
    n_interpolation_points = 100
    alpha = cf.get_linspace(n_interpolation_points)
    delta = alpha[1] - alpha[0]

    ipoints = cf.interpolate_points(points, n_interpolation_points, 4)

    distance = np.cumsum( np.sqrt(np.sum( np.diff(ipoints, axis=0)**2, axis=1 )) )
    distance = np.insert(distance, 0, 0)/distance[-1]


    # Compute curvature and derivatives
    K = cf.get_curvature(ipoints, n_interpolation_points)

    ipoints_derivatives = cf.get_derivatives(ipoints, n_interpolation_points)
    dx = ipoints_derivatives[:,0]
    dy = ipoints_derivatives[:,1]

    ipoints_derivatives2 = cf.get_derivatives(ipoints_derivatives, n_interpolation_points)
    dx2 = ipoints_derivatives2[:,0]
    dy2 = ipoints_derivatives2[:,1]


    # Visualize peaks
    all_peaks = []

    for arr, lbl in zip([dx2, dy2, K], ["dx2", "dy2", "K"]):
        arr_s = cf.smoothen(np.abs(arr), 3)
        arr_s = arr_s / max(arr_s)  # normalize

        peaks, _ = find_peaks(arr_s, height=0.2)

        bases = cf.find_peaks_bases(arr_s, peaks)
        peaks = np.sort(np.concatenate((peaks, bases), axis=0))

        all_peaks.append(peaks)

        if plot:
            plt.figure(figsize=(15,5))
            plt.plot(arr_s, color="red")

            plt.plot(peaks, arr_s[peaks], "x", color="black")
            plt.savefig("peaks_" + lbl + "_" + track_name + ".pdf")
            plt.show()


    # Visualize turns on the track
    if plot:
        fig, axs = plt.subplots(3,2, figsize=(15,20))

        for i in range(3):
            axs[i,0].title.set_text('Track')
            axs[i,0].scatter(ipoints[:,0], ipoints[:,1], c=range(len(ipoints)), cmap="hsv")


    # Show peaks of the diff gradient
    peaks_on_track = []

    interpolator =  interp1d(distance, ipoints, kind='quadratic', axis=0)

    i = 0
    for arr, p, lbl in zip([dx2,dy2,K], all_peaks, ["|dx2|", "|dy2|", "|K|"]):
        lsp = np.linspace(0,1, len(arr))
        arr_s = cf.smoothen(np.abs(arr), 3)

        peaks_on_track.append(interpolator([alpha[p_] for p_ in p]))

        if plot:
            axs[i,1].title.set_text(lbl)
            axs[i,1].plot(np.linspace(0,1,len(arr)), np.abs(arr), label=lbl, color="gray")
            axs[i,1].plot(np.linspace(0,1,len(arr)), np.abs(arr_s), color="red")

            axs[i,1].scatter(np.linspace(0,1,len(arr)), np.abs(arr), label=lbl, c=range(len(dx)), cmap="hsv")

            # Draw the turns onto the track
            axs[i,0].scatter(peaks_on_track[i][:,0], peaks_on_track[i][:,1], marker="x", color="black", s=200)
            i += 1

    if plot:
        plt.savefig("turns_identification_" + track_name + ".pdf")
        plt.show()


    # Select method for final points
    method = abs(remain)

    if 0 < method < len(peaks_on_track):
        return peaks_on_track[method]
    else:
        raise ValueError("Unknown method number selected by number of groups.")
