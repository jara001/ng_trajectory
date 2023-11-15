#!/usr/bin/env python3.6
# __init__.py
"""Initialize script for 'ng_trajectory' package."""
######################
# Imports & Globals
######################

# Computation engine (math), generally faster?
import math                                                     # noqa: F401

# Computation engine (numpy)
import numpy                                                    # noqa: F401

# Nevergrad
import nevergrad                                                # noqa: F401

# Time measurement
import time                                                     # noqa: F401

# Custom printing
import sys                                                      # noqa: F401

# Parallel computing of genetic algorithm
from concurrent import futures                                  # noqa: F401

# JSON importing
import json                                                     # noqa: F401

# Thread-safe writing to the file
from threading import Lock                                      # noqa: F401

# Computation engine (scipy) / interpolation
from scipy.interpolate import CubicSpline, bisplrep, bisplev    # noqa: F401

# Support for type hints
from typing import List, Tuple, Callable, Dict, TextIO          # noqa: F401

# Version of the package
from .version import __version__                                # noqa: F401


# Optional packages
ROS_AVAILABLE = None
PLOT_AVAILABLE = None
SHAPELY_AVAILABLE = None


try:
    import rospy                                                # noqa: F401
    import rospkg                                               # noqa: F401

    ROS_AVAILABLE = True

except ImportError:
    print ("ROS support is not available.")
    ROS_AVAILABLE = False


try:
    import matplotlib
    from matplotlib import pyplot                               # noqa: F401

    PLOT_AVAILABLE = True

except ImportError:
    print ("Matplotlib is not available.")
    # Mimic matplotlib for Typing
    matplotlib = type(
        "matplotlib",
        (object, ),
        {
            "figure":
                type(
                    "figure",
                    (object, ),
                    {
                        "Figure": 1
                    }
                ),
            "collections":
                type(
                    "collection",
                    (object, ),
                    {
                        "PathCollection": 1
                    }
                ),
            "lines":
                type(
                    "lines",
                    (object, ),
                    {
                        "Line2D": type("line2d", (object, ), {})
                    }
                ),
        }
    )
    PLOT_AVAILABLE = False


try:
    import shapely                                              # noqa: F401

    from shapely.geometry import Polygon                        # noqa: F401
    from shapely.geometry.polygon import LinearRing             # noqa: F401

    SHAPELY_AVAILABLE = True

except ImportError:
    print ("Shapely is not available.")
    SHAPELY_AVAILABLE = False


######################
# ROS dependencies
######################

if ROS_AVAILABLE:
    # Message types
    # ColorRGBA
    from std_msgs.msg import ColorRGBA

    # Header
    from std_msgs.msg import Header

    # Marker
    from visualization_msgs.msg import Marker, MarkerArray

    # Point, Vector3, Pose
    from geometry_msgs.msg import Point, Vector3, Pose

    # Vehicle trajectory
    from plan_msgs.msg import Trajectory

    # Path, GridCells, OccupancyGrid
    from nav_msgs.msg import Path, GridCells, OccupancyGrid

else:
    # Mimic messages
    ColorRGBA = lambda *x, **y: None
    Header = lambda *x, **y: None
    Marker = lambda *x, **y: None
    MarkerArray = lambda *x, **y: None
    Point = lambda *x, **y: None
    Vector3 = lambda *x, **y: None
    Pose = lambda *x, **y: None
    Trajectory = lambda *x, **y: None
    Path = lambda *x, **y: None
    GridCells = lambda *x, **y: None
    OccupancyGrid = lambda *x, **y: None


######################
# Package
######################

# from . import utils
from .main import (  # noqa: E402,F401
    execute,
    configurationLoad,
    configurationMerge
)


######################
# Configuration
######################

# from .configuration import configurationLoad
