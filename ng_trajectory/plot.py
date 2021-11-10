#!/usr/bin/env python3.6
# plot.py
"""Plot functions for ng_trajectory.
"""
######################
# Imports & Globals
######################

import numpy, sys


try:
    from ng_trajectory import matplotlib, pyplot
except:
    pass

from ng_trajectory import PLOT_AVAILABLE

from typing import List, Dict


######################
# Decorators
######################

def plot_only(f):
    """Decorator for disabling plotting functions."""
    global PLOT_AVAILABLE

    def block(*args, **kwargs):
        return lambda *x, **y: None

    def let_pass(*args, **kwargs):
        return f(*args, **kwargs)

    return let_pass if PLOT_AVAILABLE else block


######################
# Figures
######################

@plot_only
def figureCreate() -> matplotlib.figure.Figure:
    """Create a figure with export preset.

    Returns:
    figure -- created Figure
    """
    figure = pyplot.figure(dpi=300)
    figure.add_subplot(111)
    return figure


@plot_only
def axisEqual(figure: matplotlib.figure.Figure = None) -> List[float]:
    """Equal axis on current / selected figure.

    Returns:
    xmin, xmax, ymin, ymax -- the axis limits
    """
    if figure is None:
        figure = pyplot.gcf()

    return figure.axes[0].axis("equal")


@plot_only
def figureSave(filename: str, figure: matplotlib.figure.Figure = None) -> None:
    """Save figure into a file."""
    if figure is None:
        figure = pyplot.gcf()

    figure.savefig(filename, bbox_inches="tight", dpi=300, pad_inches=0)


@plot_only
def figureShow() -> None:
    """Show figure to the user."""
    pyplot.show()


######################
# Type plot functions
######################

def trackPlot(track: numpy.ndarray, figure: matplotlib.figure.Figure = None) -> None:
    """Plot a track to selected figure.

    Arguments:
    track -- points of the track valid area, nx2 numpy.ndarray
    figure -- figure to plot to, matplotlib.figure.Figure, default 'current figure'
    """
    pointsScatter(track, figure, s=1, color="gray")


def bordersPlot(borders: List[numpy.ndarray], colored: bool = True, figure: matplotlib.figure.Figure = None) -> None:
    """Plot borders of the segments to a selected figure.

    Arguments:
    borders -- border points of the groups, n-list of x2 numpy.ndarray
    colored -- when True, plot is done colored, otherwise single color, bool, default True
    figure -- figure to plot to, matplotlib.figure.Figure, default 'current figure'
    """

    if colored:
        #groupsScatter(borders, figure, s=1)
        groupsPlot(borders, figure, linewidth=0.6, linestyle="dotted")
    else:
        #groupsScatter(borders, figure, s=1, color="gray")
        groupsPlot(borders, figure, linewidth=0.6, linestyle="dotted", color="gray")


def indicesPlot(points: numpy.ndarray, figure: matplotlib.figure.Figure = None) -> None:
    """Show an index of each element on its location.

    Arguments:
    points -- locations where to show the indices, nx2 numpy.ndarray
    figure -- figure to plot to, matplotlib.figure.Figure, default 'current figure'
    """

    for i, point in enumerate(points):
        labelText(point, i,
            verticalalignment = "center",
            horizontalalignment = "center",
            fontsize = 18
        )


######################
# General functions
######################

@plot_only
def pointsScatter(points: numpy.ndarray, figure: matplotlib.figure.Figure = None, **kwargs) -> matplotlib.collections.PathCollection:
    """Scatter points.

    Arguments:
    points -- points to be scattered, nx(>=2) numpy.ndarray
    **kwargs -- keyword arguments to be passed to scatter

    Returns:
    paths -- instance of PathCollection with scattered data
    """
    if figure is None:
        figure = pyplot.gcf()

    return figure.axes[0].scatter(points[:, 0], points[:, 1], **kwargs)


@plot_only
def pointsPlot(points: numpy.ndarray, figure: matplotlib.figure.Figure = None, **kwargs) -> List[matplotlib.lines.Line2D]:
    """Plot points.

    Arguments:
    points -- points to be plotted, nx(>=2) numpy.ndarray
    **kwargs -- keyword arguments to be passed to plot

    Returns:
    lines -- list of Line2D objects representing the plotted data
    """
    if figure is None:
        figure = pyplot.gcf()

    return figure.axes[0].plot(points[:, 0], points[:, 1], **kwargs)


@plot_only
def groupsScatter(groups: numpy.ndarray, figure: matplotlib.figure.Figure = None, **kwargs) -> None:
    """Scatter points inside groups.

    Arguments:
    groups -- list of grouped points, m-list of x2 numpy.ndarrays
    **kwargs -- keyword arguments to be passed to scatter
    """

    for _g in groups:
        pointsScatter(_g, figure, **kwargs)


@plot_only
def groupsPlot(groups: numpy.ndarray, figure: matplotlib.figure.Figure = None, **kwargs) -> None:
    """Plot points inside groups.

    Arguments:
    groups -- list of grouped points, m-list of x2 numpy.ndarrays
    **kwargs -- keyword arguments to be passed to plot
    """

    for _g in groups:
        pointsPlot(_g, figure, **kwargs)


@plot_only
def grouplayersScatter(grouplayers: numpy.ndarray, figure: matplotlib.figure.Figure = None, **kwargs) -> None:
    """Scatter points from layers of all groups.

    Arguments:
    grouplayers -- points in all layers, n-list of (layer_count)-lists of x2 numpy.ndarray
    **kwargs -- keyword arguments to be passed to scatter
    """

    for _g in grouplayers:
        groupsScatter(_g, figure, **kwargs)


@plot_only
def grouplayersPlot(grouplayers: numpy.ndarray, figure: matplotlib.figure.Figure = None, **kwargs) -> None:
    """Plot points from layers of all groups.

    Arguments:
    grouplayers -- points in all layers, n-list of (layer_count)-lists of x2 numpy.ndarray
    **kwargs -- keyword arguments to be passed to plot
    """

    for _g in grouplayers:
        groupsPlot(_g, figure, **kwargs)


@plot_only
def labelText(point: numpy.ndarray, s: str, figure: matplotlib.figure.Figure = None, **kwargs) -> None:
    """Show a label on a point.

    Arguments:
    point -- location to show the text, 1x2 numpy.ndarray
    s -- text to show, str
    **kwargs -- keyword arguments to be passed to text
    """

    if figure is None:
        figure = pyplot.gcf()

    figure.axes[0].text(point[0], point[1], s = s, **kwargs)


######################
# pyplot gateway
######################

def _pyplot(*args, function: str, figure: matplotlib.figure.Figure = None, **kwargs) -> any:
    """Call directly a function of matplotlib pyplot.

    Arguments:
    function -- name of the function, str
    *args -- positional arguments to be passed to the function
    **kwargs -- keyword arguments to be passed to the function

    Returns:
    The returned object of the function.
    """

    if function not in dir(pyplot):
        print ("Unknown function '%s' of pyplot." % function, file = sys.stderr)
        return

    return pyplot.__getattribute__(function)(*args, **kwargs)


def _figure(*args, function: str, figure: matplotlib.figure.Figure = None, **kwargs) -> any:
    """Call directly a function of matplotlib pyplot's figure.

    Arguments:
    function -- name of the function, str
    figure -- figure to plot to, matplotlib.figure.Figure, default 'current figure'
    *args -- positional arguments to be passed to the function
    **kwargs -- keyword arguments to be passed to the function

    Returns:
    The returned object of the function.
    """

    if function not in dir(pyplot):
        print ("Unknown function '%s' of pyplot figure." % function, file = sys.stderr)
        return

    if figure is None:
        figure = pyplot.gcf()

    return figure.axes[0].__getattribute__(function)(*args, **kwargs)


######################
# Dynamic plotting
######################

def plotDyn(args: Dict[str, Dict[str, any]], figure: matplotlib.figure.Figure = None, **kwargs) -> None:
    """Dynamically create figure according to the configuration.

    Arguments:
    args -- dynamic plot arguments
    figure -- figure to plot to, matplotlib.figure.Figure, default 'current figure'
    **kwargs -- arguments not caught by previous parts
    """

    if figure is None:
        figure = pyplot.gcf()

    VERBOSITY = kwargs.get("logging_verbosity", 1)

    for function, fargs in args.items():
        if VERBOSITY > 1:
            print (function, fargs)

        if "-" in function:
            function = function[:function.index("-")]

        if function in globals():
            if VERBOSITY > 1:
                print (function)

            pargs = []

            # Obtain args for the function
            # 1: As a key-value in dict
            if "_args" in fargs:
                _fargs = fargs.get("_args")
            # 2: Or as a list when there is nothing else
            elif isinstance(fargs, list):
                _fargs = fargs
            else:
                _fargs = []

            for a in _fargs:
                if a[0] == "@":
                    a = a[1:]
                    if a not in kwargs:
                        if VERBOSITY > 0:
                            print ("Key '%s' is not available." % a, file=sys.stderr)
                    else:
                        pargs.append(kwargs.get(a))
                else:
                    pargs.append(a)

            if isinstance(fargs, dict):
                globals()[function](*pargs, **{**dict([ (f, i) for f, i in fargs.items() if f[0] != "_" ]), **{"figure": figure}})
            else:
                globals()[function](*pargs, **{"figure": figure})
