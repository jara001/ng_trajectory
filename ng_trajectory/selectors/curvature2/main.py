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

from ng_trajectory.interpolators import cubic_spline

from ng_trajectory.criterions.length import compute as pathLength

import ng_trajectory.plot as ngplot

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


def pathPointDistanceAvg(points: numpy.ndarray) -> float:
    """Get average distance between consecutive points in an array.

    Arguments:
    points -- list of points, nx(>=2) numpy.ndarray

    Returns:
    avg_dist -- average distance between points, float
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
    rpoints -- list of points, remainx2 numpy.ndarray

    Note: Similarly to 'Curvature' selector, 'remain' does not set the number of points.
    """

    # Update parameters
    P.updateAll(overflown)

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
                ([i - P.getValue("peaks_bounds") for i in peaks], [i + P.getValue("peaks_bounds") for i in peaks]),
                axis = 0
            )
        )
    )


    # Optional
    # Plot the track with curvature if requested
    if P.getValue("plot"):
        fig, ax = ngplot.pyplot.subplots(1, 2)

        ngplot.axisEqual(fig)

        ax[0].plot(points[:, 0], points[:, 1])
        ax[0].scatter(points[peaks, 0], points[peaks, 1], marker="x", color="black")
        ax[0].scatter(points[peaksN, 0], points[peaksN, 1], marker="x", color="green")

        ax[1].plot(points[:, 2])
        ax[1].scatter(peaks, points[peaks, 2], marker="x", color="black")
        ax[1].scatter(peaksN, points[peaksN, 2], marker="x", color="green")

        fig.savefig("curvature2_" + P.getValue("track_name") + ".pdf")

        if P.getValue("show_plot"):
            fig.show()
        else:
            ngplot.pyplot.close(fig)


    return points[peaksN, :]
