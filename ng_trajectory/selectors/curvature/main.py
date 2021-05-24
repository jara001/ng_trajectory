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

    if plot:
        figP, axs = plt.subplots(3, 1, figsize=(15,15))

    i = 0
    for arr, lbl in zip([dx2, dy2, K], ["dx2", "dy2", "K"]):
        #arr_s = np.abs(arr) #cf.smoothen(np.abs(arr), 3)
        #arr_s = arr_s / max(arr_s)  # normalize
        arr_s = arr / max(np.abs(arr))

        # Detect peaks separately on both sides
        peaks, _ = find_peaks(arr_s, height = 0.2)#, distance = 5)
        peaks2, _ = find_peaks(-arr_s, height = 0.2)#, distance = 5)
        peaks = np.unique(np.sort(np.concatenate((peaks, peaks2), axis=0)))

        # Detect parts without points
        threshold = 12
        filling = []
        for j in range(len(peaks)-1):
            if peaks[j+1] - peaks[j] > 12:
                filling += list(
                    np.linspace(
                        peaks[j],
                        peaks[j+1],
                        int((peaks[j+1] - peaks[j]) / 12) + 1,
                        endpoint = False,
                        dtype = np.int
                    )
                )[1:]

        # Detect over 0 switch and add a point there
        threshold = int(threshold / 2) # How far the points has to be from filling to be added
        switching = []
        for j in range(len(peaks)-1):
            if np.sign(arr_s[peaks[j+1]]) != np.sign(arr_s[peaks[j]]):
                ## y-wise in-fill

                # Target value
                target = arr_s[peaks[j]] + ((arr_s[peaks[j+1]] - arr_s[peaks[j]]) / 2)

                # Find closest point on the line (y-wise) between the points
                _error = 10000
                _index = 0
                for _p in range(peaks[j]+1, peaks[j+1]):
                    if (target - arr_s[_p])**2 < _error:
                        _error = (target - arr_s[_p])**2
                        _index = _p

                # Add the point only if too far from filling
                for _p in filling:
                    if abs(_p - _index) <= threshold:
                        break
                else:
                    switching += [_index]

        _peaks = np.unique(np.sort(np.concatenate((peaks, filling, switching), axis=0)).astype(np.int))

        all_peaks.append(_peaks)

        if plot:
            axs[i].title.set_text(lbl)
            axs[i].plot(arr_s, color="red")
            axs[i].plot(_peaks, arr_s[_peaks], "x", color="black")

        i += 1

    if plot:
        figP.savefig("peaks_" + track_name + ".pdf")
        figP.show()


    # Visualize turns on the track
    if plot:
        figP, axs = plt.subplots(3,2, figsize=(15,20))

        for i in range(3):
            axs[i,0].title.set_text('Track')
            axs[i,0].scatter(ipoints[:,0], ipoints[:,1], c=range(len(ipoints)), cmap="hsv")


    # Show peaks of the diff gradient
    peaks_on_track = []

    interpolator =  interp1d(distance, ipoints, kind='quadratic', axis=0)

    i = 0
    for arr, p, lbl in zip([dx2,dy2,K], all_peaks, ["|dx2|", "|dy2|", "|K|"]):
        lsp = np.linspace(0,1, len(arr))
        #arr_s = np.abs(arr) #cf.smoothen(np.abs(arr), 3)
        arr_s = arr

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
        figP.savefig("turns_identification_" + track_name + ".pdf")
        figP.show()


    # Select method for final points
    method = abs(remain)

    if 0 < method <= len(peaks_on_track):
        return peaks_on_track[method-1]
    else:
        raise ValueError("Unknown method number selected by number of groups.")
