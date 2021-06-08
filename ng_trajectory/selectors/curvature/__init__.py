#!/usr/bin/env python3.6
"""Points selector based on the path's shape.

This selector obtains a subset of path points in order
to segment the track more intelligently. Points are situated
in turns, with some filling on the straight sections.

The algorithm works as follows:
(i) positive and negative peaks on the curvature are found
    and populated by cuts,
(ii) close cuts are merged to avoid redundancy,
(iii) long cut-less sections of the track are artificially
      filled with equidistant cuts,
(iv) sections of the track between two consecutive cuts where
     the sign of the curvature changes are filled
     with additional cuts, and
(v) close cuts are filtered once again.

The current version segments the track automatically,
given several parameters.

Note: The number of segments is determined differently:
 - -1 is selection based on dx
 - -2 is selection based on dy
 - -3 is selection based on curvature
"""
from .main import init, select
