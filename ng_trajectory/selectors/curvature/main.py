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

from typing import Dict


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("track_name", "unknown", str, "Name of the track.", "")
P.createAdd("plot", False, bool, "Whether the images are generated.", "")
P.createAdd("interpolation_factor", 24.0, float, "Factor to reduce number of points prior to the interpolation.", "")
P.createAdd("peaks_height", 0.0, float, "Minimum absolute height of peaks.", "")
P.createAdd("peaks_merge", 0, int, "Width of the area used for peaks merging.", "")
P.createAdd("peaks_filling", 1000000, int, "Width of the area for filling the points.", "")


######################
# Utility lambdas
######################

# Taken from the profiler
addOverlap = lambda points, overlap: np.vstack((points[-overlap:, :], points[:, :], points[:overlap]))
removeOverlap = lambda points, overlap: points[overlap:-overlap]


######################
# Utility functions
######################

def mergePeaks(peaks_indices: np.ndarray, other: Dict[str, np.ndarray], threshold: int) -> np.ndarray:
    """Merges close peaks.

    Arguments:
    peaks_indices -- indices of detected peaks, nx1 numpy.ndarray
    other -- other information about peaks, dict of numpy.ndarrays
    threshold -- surrounding area to gather, int

    Returns:
    merged_peaks -- indices of merged peaks, nx1 numpy.ndarray

    Note: For this function 'peak_heights' is required in 'other'.
    """

    merged_peaks = []

    _indices = peaks_indices.tolist()
    _peaks = sorted(zip(_indices, other.get("peak_heights").tolist()), key = lambda x: x[1], reverse = True)

    while len(_peaks) > 0:
        _peak, _ = _peaks.pop(0)

        if _peak not in _indices:
            continue

        _indices.remove(_peak)

        nearby = set.intersection(
            set(_indices),
            set([ _peak + _i - int(threshold / 2) for _i in range(threshold) ])
        )

        merged_peaks.append(int((_peak + sum(nearby)) / (1 + len(nearby))))

        for _p in nearby:
            _indices.remove(_p)

    return np.asarray(merged_peaks, dtype = np.int)


######################
# Functions
######################

def init(**kwargs) -> None:
    """Initialize selector.

    Arguments:
    **kwargs -- overflown arguments
    """
    pass


def select(
        points: np.ndarray,
        remain: int,
    **overflown) -> np.ndarray:
    """Select points from the path uniformly.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int
    **overflown -- arguments not caught by previous parts

    Returns:
    rpoints -- list of points, remainx2 numpy.ndarray

    Note: Since in this version we are not able to set the number of points, raise an exception for positive numbers.
    FIXME: It fails horribly when the number of peaks is low (like 1 or 2).
    """

    if remain > 0:
        raise ValueError("Exact number of points cannot be handled by 'curvature' selector.")

    # Update parameters
    P.updateAll(overflown)

    # Repair points array
    # Sometimes the last point is the same as the first, and we do not want that.
    if (points[0] == points[-1]).all():
        points = points[0:-1]


    # Interpolate points
    n_interpolation_points = int(len(points)/P.getValue("interpolation_factor"))
    overlap_size = int(n_interpolation_points/2)
    alpha = cf.get_linspace(n_interpolation_points)
    delta = alpha[1] - alpha[0]

    ipoints = cf.interpolate_points(points, n_interpolation_points, 4)

    distance = np.cumsum( np.sqrt(np.sum( np.diff(ipoints, axis=0)**2, axis=1 )) )
    distance = np.insert(distance, 0, 0)/distance[-1]

    ipoints = addOverlap(ipoints, overlap_size)


    # Compute curvature and derivatives
    K = cf.get_curvature(ipoints, n_interpolation_points+2*overlap_size)

    ipoints_derivatives = cf.get_derivatives(ipoints, n_interpolation_points+2*overlap_size)
    dx = ipoints_derivatives[:,0]
    dy = ipoints_derivatives[:,1]

    ipoints_derivatives2 = cf.get_derivatives(ipoints_derivatives, n_interpolation_points+2*overlap_size)
    dx2 = ipoints_derivatives2[:,0]
    dy2 = ipoints_derivatives2[:,1]


    # Visualize peaks
    all_peaks = []

    if P.getValue("plot"):
        figP, axs = plt.subplots(3, 1, figsize=(15,15))

    i = 0
    for arr, lbl in zip([dx2, dy2, K], ["dx2", "dy2", "K"]):
        #arr_s = np.abs(arr) #cf.smoothen(np.abs(arr), 3)
        #arr_s = arr_s / max(arr_s)  # normalize
        arr_s = arr / max(np.abs(arr))

        # Detect peaks separately on both sides
        peaks, adds = find_peaks(arr_s, height = P.getValue("peaks_height"))#, distance = 5)
        peaks2, adds2 = find_peaks(-arr_s, height = P.getValue("peaks_height"))#, distance = 5)

        # Merge close peaks manually
        mpeaks = mergePeaks(peaks, adds, P.getValue("peaks_merge"))
        mpeaks2 = mergePeaks(peaks2, adds2, P.getValue("peaks_merge"))

        peaks = np.unique(np.sort(np.concatenate((mpeaks, mpeaks2), axis=0)))

        # Detect parts without points
        filling = []
        for j in range(len(peaks)-1):
            if peaks[j+1] - peaks[j] > P.getValue("peaks_filling"):
                filling += list(
                    np.linspace(
                        peaks[j],
                        peaks[j+1],
                        int((peaks[j+1] - peaks[j]) / P.getValue("peaks_filling")) + 1,
                        endpoint = False,
                        dtype = np.int
                    )
                )[1:]

        # Detect over 0 switch and add a point there
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
                    #if abs(_p - _index) <= int(peaks_filling/2):
                    # Instead, check that between the peaks there is not filling
                    if _p in range(peaks[j] + 1, peaks[j + 1]):
                        break
                else:
                    switching += [_index]

        # Convert peaks to non-overlapped version
        peaks = peaks[overlap_size <= peaks]
        peaks = peaks[peaks < (len(ipoints) - overlap_size)]
        peaks = peaks - overlap_size

        filling = np.asarray(filling)
        filling = filling[overlap_size <= filling]
        filling = filling[filling < (len(ipoints) - overlap_size)]
        filling = filling - overlap_size

        switching = np.asarray(switching)
        switching = switching[overlap_size <= switching]
        switching = switching[switching < (len(ipoints) - overlap_size)]
        switching = switching - overlap_size

        _peaks = np.unique(np.sort(np.concatenate((peaks, filling, switching), axis=0)).astype(np.int))

        all_peaks.append(_peaks)

        if P.getValue("plot"):
            arr_s = arr_s[overlap_size:-overlap_size]
            axs[i].title.set_text(lbl)
            axs[i].plot(arr_s, color="red")
            #axs[i].plot(_peaks, arr_s[_peaks], "x", color="black")

            original_peaks = _peaks[np.isin(_peaks, peaks)]
            #print ("1", original_peaks, peaks)
            axs[i].plot(original_peaks, arr_s[original_peaks], "x", color="black")

            new_peaks = _peaks[np.isin(_peaks, filling)]
            #print ("2", new_peaks, filling)
            axs[i].plot(new_peaks, arr_s[new_peaks], "x", color="green")

            new_peaks2 = _peaks[np.isin(_peaks, switching)]
            #print ("3", new_peaks2, switching)
            axs[i].plot(new_peaks2, arr_s[new_peaks2], "x", color="blue")

        i += 1

    if P.getValue("plot"):
        figP.savefig("peaks_" + P.getValue("track_name") + ".pdf")
        figP.show()


    # Remove overlaps
    ipoints = removeOverlap(ipoints, overlap_size)

    K = removeOverlap(K, overlap_size)
    dx = removeOverlap(dx, overlap_size)
    dy = removeOverlap(dy, overlap_size)
    dx2 = removeOverlap(dx2, overlap_size)
    dy2 = removeOverlap(dy2, overlap_size)

    distance = np.cumsum( np.sqrt(np.sum( np.diff(ipoints, axis=0)**2, axis=1 )) )
    distance = np.insert(distance, 0, 0)/distance[-1]


    # Visualize turns on the track
    if P.getValue("plot"):
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

        if P.getValue("plot"):
            axs[i,1].title.set_text(lbl)
            axs[i,1].plot(np.linspace(0,1,len(arr)), np.abs(arr), label=lbl, color="gray")
            axs[i,1].plot(np.linspace(0,1,len(arr)), np.abs(arr_s), color="red")

            axs[i,1].scatter(np.linspace(0,1,len(arr)), np.abs(arr), label=lbl, c=range(len(dx)), cmap="hsv")

            # Draw the turns onto the track
            axs[i,0].scatter(peaks_on_track[i][:,0], peaks_on_track[i][:,1], marker="x", color="black", s=200)
            i += 1

    if P.getValue("plot"):
        figP.savefig("turns_identification_" + P.getValue("track_name") + ".pdf")
        figP.show()


    # Select method for final points
    method = abs(remain)

    if 0 < method <= len(peaks_on_track):
        return peaks_on_track[method-1]
    else:
        raise ValueError("Unknown method number selected by number of groups.")
