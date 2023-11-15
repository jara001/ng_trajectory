#!/usr/bin/env python3.6
"""Selector that utilizes the curvature of the path.

In contrast to 'Curvature' selector, this one is using
metric distances not indices.

Prior to the algorithm, the path can be resampled twice.
- First resampling (using 'sampling_distance') gets rid of
the wavy curvature induced by too many points.
- Second resampling (using 'point_distance') sets the distance
between subsequent path points.

The algorithm works as follows:
(i) Path is resampled if required / requested.
(ii) Peaks are detected on the original and inverted absolute
path curvature.
 - Peaks are curvatures above 'peaks_height'.
 - Only one peak can be detected within 'peaks_distance'.
(iii) Both arrays of peaks are merged.
(iv) Turn boundaries are selected by moving the peaks in the
positive and negative 'peaks_bounds'. These are used instead of
peaks now on.
(v) Detected peaks are accompanied by filled "dummy" peaks that
are added to empty spaces to ensure maximum distance between
two consecutive peaks 'peaks_filling'.
(vi) Too close peaks 'peaks_merge' are merged together (averaged
to the closest path point).

Note: Parameter 'remain' of the select function is
completely ignored.
"""
from .main import init, select  # noqa: F401
