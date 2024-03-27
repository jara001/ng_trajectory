#!/usr/bin/env python3.6
# main.py
"""Select points from a path based on its curvature.

Based upon the previous work by Ondra Benedikt.
"""
######################
# Imports & Globals
######################

# Math
import numpy

# Peak detector
from scipy.signal import find_peaks

# Cubic spline interpolator
from ng_trajectory.interpolators import cubic_spline

# Path length for estimating the path resolution
from ng_trajectory.criterions.length import compute as pathLength

# Plot support
import ng_trajectory.plot as ngplot

# Typing
from typing import List


# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("track_name", "unknown", str, "Name of the track.", "")
P.createAdd("plot", False, bool, "Whether the images are generated.", "")
P.createAdd("show_plot", True, bool, "Whether the generated images are shown.", "")
P.createAdd("point_distance", 0.1, float, "[m] Distance between consecutive points of the path, skipped when 0.", "")
P.createAdd("sampling_distance", 1.0, float, "[m] Distance of super-sampling before the interpolation, skipped when 0.", "")
P.createAdd("peaks_height", 1.0, float, "[m^-1] Minimum absolute height of peaks.", "")
P.createAdd("peaks_distance", 16, int, "Minimum distance between two identified peaks.", "")
P.createAdd("peaks_bounds", 8, int, "Distance to the turn boundaries (created pseudo-peaks), skipped when 0.", "")
P.createAdd("peaks_filling", 10.0, float, "[m] Maximum distance between two consecutive peaks in the final array.", "")
P.createAdd("peaks_merge", 0, int, "Maximum distance between two subsequent peaks to be merged.", "")


######################
# Utility lambdas
######################

# Taken from the profiler
addOverlap = lambda points, overlap: numpy.vstack((points[-overlap:, :], points[:, :], points[:overlap]))
removeOverlap = lambda points, overlap: points[overlap:-overlap]

# Overlaps2 are used for arrays of indices where we want to modify the content
# in order to still point to the same path element.
addOverlap2 = lambda indices, overlap: numpy.hstack((indices[-1], indices + overlap, indices[0] + 2 * overlap))
removeOverlap2 = lambda indices, overlap: \
        indices[(overlap <= indices) & (indices < 2 * overlap)] - overlap \
        if isinstance(indices, numpy.ndarray) \
        else [ index - overlap for index in indices if overlap <= index < 2 * overlap ]


######################
# Utilities
######################

def pathPrepare(points: numpy.ndarray) -> numpy.ndarray:
    """Prepare the path for further processing.

    This means that we want to make sure that first element does not match the last,
    and that there is no additional information stored.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray

    Returns:
    p_points -- list of prepared points, mx2 numpy.ndarray
    """
    return points[1:, :2] if (points[0, :2] == points[-1, :2]).all() else points[:, :2]


def pathPointDistanceCompute(points: numpy.ndarray, index_1: int, index_2: int) -> numpy.ndarray:
    """Compute the distance between consecutive points of a path limited by indices.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray
    index_1 -- index of the first point, int
    index_2 -- index of the seconds point, int

    Returns:
    distances -- distances between consecutive points, [m], (index_2-index_1)x1 numpy.ndarray

    Note: Adapted from Length criterion.
    """
    return numpy.sqrt(
        numpy.sum(
            numpy.power(
                numpy.subtract(
                    points[index_1+1:index_2+1, :2],
                    points[index_1:index_2, :2]
                ),
            2),
        axis=1)
    )


def pathPointDistance(points: numpy.ndarray, index_1: int, index_2: int) -> float:
    """Compute the distance between two points of a path given by indices.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray
    index_1 -- index of the first point, int
    index_2 -- index of the seconds point, int

    Returns:
    distance -- distance between the points, [m], float

    Note: Adapted from Length criterion.
    """
    return float(
        numpy.sum(
            pathPointDistanceCompute(points, index_1, index_2)
        )
    )


def pathPointDistanceAvg(points: numpy.ndarray) -> float:
    """Get average distance between consecutive points in an array.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray

    Returns:
    avg_dist -- average distance between points, [m], float
    """
    return pathLength(points) / len(points)


def factorCompute(points: numpy.ndarray, resolution: float) -> float:
    """Compute a factor for modifying the number of line points to get a resolution.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray
    resolution -- required distance between consecutive points in meters, float

    Returns:
    factor -- factor to modify the line to get set resolution, float
    """
    return pathPointDistanceAvg(points) / resolution


def resolutionEstimate(points: numpy.ndarray, resolution: float) -> int:
    """Estimate the number of points of a line to receive set resolution.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray
    resolution -- required distance between consecutive points in meters, float

    Returns:
    points_length -- estimated number of points, int
    """
    return int(len(points) * factorCompute(points, resolution))


def peaksFill(points: numpy.ndarray, peaks: List[int], max_distance: float) -> List[int]:
    """Compute indices of filling points to constrain the distance between peaks.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray
    peaks -- list of peak indices, m-list of ints / mx1 numpy.ndarray
    max_distance -- maximum distance between two consecutive peaks, [m], float

    Returns:
    filling -- list of additional indices, p-list of ints
    """

    # Use overlap to make it simpler
    _points = addOverlap(points, len(points))
    _peaks = addOverlap2(peaks, len(points))

    filling = []

    for i in range(len(peaks) - 1):
        _distance = pathPointDistance(_points, _peaks[i], _peaks[i+1])

        if _distance > max_distance:
            filling += list(
                numpy.linspace(
                    _peaks[i],
                    _peaks[i+1],
                    int(_distance / max_distance) + 1,
                    endpoint = False,
                    dtype = int
                )
            )[1:]

    return removeOverlap2(filling, len(points))


def peaksMerge(points: numpy.ndarray, peaks: List[int], min_distance: float) -> List[int]:
    """Merge too close peaks based on their mutual distance.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray
    peaks -- list of peak indices, m-list of ints / mx1 numpy.ndarray
    min_distance -- minimum required distance between peaks, [m], float

    Returns:
    merged_peaks -- list of merged peak indices, p-list of ints
    """

    # Use overlap here as well
    _points = addOverlap(points, len(points))
    _peaks = addOverlap2(peaks, len(points))

    # Precompute the distances
    _distances = pathPointDistanceCompute(_points, min(_peaks), max(_peaks))
    _offset = min(_peaks) # Offset the distances to lower the computation

    # Merge the peaks
    while True:
        _peak_distances = [
            sum(
                [ _distances[_i - _offset] for _i in range(peak1, peak2) ]
            ) for peak1, peak2 in zip(_peaks[0:-1], _peaks[1:])
        ]

        _closest_distance = min(_peak_distances)

        # Finish if the peaks are far away
        if _closest_distance > min_distance:
            break

        # Find the closest peaks
        _closest_i = _peak_distances.index(_closest_distance)

        # Find point in-between of them
        # We take the subset of distances and find the closest cumulative sum closest to the average
        _average_i = numpy.argmin(
                numpy.abs(
                    numpy.subtract(
                        numpy.cumsum( # Cumulative sum
                            _distances[_peaks[_closest_i] - _offset:_peaks[_closest_i+1] - _offset] # Subset
                        ),
                        _closest_distance / 2
                    )
                )
            )

        # Replace one of the peaks with the average and delete the other
        _peaks[_closest_i] = _peaks[_closest_i] + _average_i
        _peaks = numpy.delete(_peaks, _closest_i + 1)


    return removeOverlap2(_peaks, len(points))


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
        points: numpy.ndarray,
        remain: int,
    **overflown) -> numpy.ndarray:
    """Select points from the path by its curvature.

    Arguments:
    points -- list of points, nx2 numpy.ndarray
    remain -- number of points in the result, int
    **overflown -- arguments not caught by previous parts

    Returns:
    rpoints -- list of points, mx2 numpy.ndarray

    Note: Similarly to 'Curvature' selector, 'remain' does not set the number of points.
    """

    # Update parameters
    P.updateAll(overflown, reset = False)

    final_peaks = None

    # Prepare the path
    #points = pathPrepare(points)
    points = points[1:, :] if (points[0, :2] == points[-1, :2]).all() else points

    print (pathLength(points), len(points), pathLength(points) / len(points))

    # Step 1
    # Interpolate the original line to get smoother one
    # At first, we interpolate the subset...
    if P.getValue("sampling_distance") != 0.0:
        points = cubic_spline.interpolate(points[:, :2], resolutionEstimate(points, P.getValue("sampling_distance")))

        print (pathLength(points), len(points), pathLength(points) / len(points))


    # ... and then we increase the number of points
    if P.getValue("point_distance") != 0.0:
        points = cubic_spline.interpolate(points[:, :2], resolutionEstimate(points, P.getValue("point_distance")))

        print (pathLength(points), len(points), pathLength(points) / len(points))


    # Step 2
    # Detect peaks in the "signal"
    arr_s = numpy.abs(points[:, 2])

    peaks, adds = find_peaks(arr_s, height = P.getValue("peaks_height"), distance = P.getValue("peaks_distance"))

    # Find them also in the inverted signal
    peaks2, adds2 = find_peaks(-arr_s, height = P.getValue("peaks_height"), distance = P.getValue("peaks_distance"))


    # Step 3
    # Unite both arrays of peaks
    peaks = numpy.unique(
        numpy.sort(
            numpy.concatenate((peaks, peaks2), axis=0)
        )
    )


    # Step 4
    # Create boundaries of turns
    peaksN = numpy.unique(
        numpy.sort(
            numpy.concatenate(
                ([max(0, i - P.getValue("peaks_bounds")) for i in peaks], [min(i + P.getValue("peaks_bounds"), len(arr_s)-1) for i in peaks]),
                axis = 0
            )
        )
    )
    final_peaks = peaksN


    # Step 5
    # Fill additional points to ensure maximum distance between two consecutive points
    filling = peaksFill(points, peaksN, P.getValue("peaks_filling"))

    # Join the filling
    if len(filling) > 0:
        final_peaks = numpy.unique(
            numpy.sort(
                numpy.concatenate(
                    (final_peaks, filling),
                    axis = 0
                )
            )
        )


    # Step 6
    # Merge too close peaks
    merged = peaksMerge(points, final_peaks, P.getValue("peaks_merge"))


    # Optional
    # Plot the track with curvature if requested
    if P.getValue("plot"):
        fig, ax = ngplot.pyplot.subplots(1, 2)

        ngplot.axisEqual(fig)

        ax[0].plot(points[:, 0], points[:, 1])
        ax[0].scatter(points[merged, 0], points[merged, 1], marker="o", color="orange")
        ax[0].scatter(points[peaks, 0], points[peaks, 1], marker="x", color="black")
        ax[0].scatter(points[peaksN, 0], points[peaksN, 1], marker="x", color="green")
        ax[0].scatter(points[filling, 0], points[filling, 1], marker="x", color="blue")

        ax[1].plot(points[:, 2])
        ax[1].scatter(merged, points[merged, 2], marker="o", color="orange")
        ax[1].scatter(peaks, points[peaks, 2], marker="x", color="black")
        ax[1].scatter(peaksN, points[peaksN, 2], marker="x", color="green")
        ax[1].scatter(filling, points[filling, 2], marker="x", color="blue")

        fig.savefig("curvature2_" + P.getValue("track_name") + ".pdf")

        if P.getValue("show_plot"):
            fig.show()
        else:
            ngplot.pyplot.close(fig)


    return points[merged, :]
