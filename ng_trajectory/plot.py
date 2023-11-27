#!/usr/bin/env python3.6
# plot.py
r"""## Plot functions for ng_trajectory

From the user side (i.e., configuration file), only dynamic plotting is available.
However, all plotting (package-wise) should be controlled by variable `plot` that
is set to False by default.

Dynamic plotting is defined using a custom key in the JSON. Following example
resembles what is a "standard" and most used configuration:
```json
{
	"plot": true,
	"plot_args": [
		{
			"_figure": {
				"function": "axis",
				"_args": [ "equal" ]
			},
			"trackPlot": [ "@track" ]
		},
		{
			"pointsPlot": {
				"_args": [ "@result" ]
			},
			"pointsScatter": {
				"_args": [ "@rcandidate" ]
			}
		}
	]
}
```

Note: This creates a figure with equal axis, underlying track, optimized control
points of the trajectory and their interpolation.

The list in `plot_args` contains two dictionaries. The first one is executed
before the optimization (and even before initialization of algorithms), whereas
the second one is executed after optimization finishes.

Commands in the `plot_args` are executed in order, key-wise. Basic syntax is
```json
{
	"func": [ "arg1", "arg2" ]
}
```

which sends all arguments to the function, or
```json
{
	"func": {
		"_args": [ "arg1", "arg2" ],
		"kw_arg": 4
	}
}
```

which calls `func` with the arguments stored in `_args`, appended with other
arguments as kwargs.

Note: Keys starting with `_` are treated differently; and are removed from kwargs.

Since it is not possible to have a dictionary with repeating keys, you can use
a meta character `-`. Dash, and everything after it is discarded during dynamic
plotting.

To pass a variable to the function, write its name prefixed with `@`. In case
that the variable is not available, an exception is raised.


### Available functions

Following functions are available for plotting, however only the first three
are usually used:

- trackPlot
- pointsScatter
- pointsPlot
- bordersPlot
- indicesPlot
- groupsScatter
- groupsPlot
- grouplayersScatter
- grouplayersPlot
- labelText


### Available variables

Following variables are available for plotting (by setting the value to `@` + name
of the variable):

- track -- All valid points of the track.
- fitness -- Fitness value of the best solution.
- rcandidate -- Control points of the best solution.
- tcandidate -- Control points of the best solution in Matryoshka space.
- result -- Optimized trajectory (interpolation of rcandidate).
- figure -- Currently used figure for plotting.
- \+ any variable defined in the current loop from the configuration file.


### Matplotlib wrapper

In addition, it is possible to call literally any function related to `pyplot` or
`figure` from the matplotlib. To do this, call function `_pyplot`/`_figure`
with argument `function` with the name of the required function.

For example, to make the axis equal, one can use this:
```json
{
	"_figure": {
		"function": "axis",
		"_args": [ "equal" ]
	}
}
```
"""  # noqa: D400,E501,W191
######################
# Imports & Globals
######################

import numpy
import sys


try:
    from ng_trajectory import matplotlib, pyplot
    import matplotlib._pylab_helpers
except Exception:
    pass

from ng_trajectory import PLOT_AVAILABLE  # noqa: F401

from typing import List, Dict


# Global variables
CANVAS = None
CANVAS_OLD = None


######################
# Decorators
######################

def plot_only(f):
    """Decorate to disable plotting functions."""
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
    global CANVAS, CANVAS_OLD

    figure = pyplot.figure(dpi=300)
    figure.add_subplot(111)

    try:
        # Hold onto the PhotoImage object (canvas) in the memory to avoid gc related errors:
        # Exception ignored in: <bound method Image.__del__ of <tkinter.PhotoImage object at 0x7f77550ab240>>
        # Traceback (most recent call last):
        #   File "/usr/lib/python3.6/tkinter/__init__.py", line 3519, in __del__
        #     self.tk.call('image', 'delete', self.name)
        # RuntimeError: main thread is not in main loop
        CANVAS_OLD = CANVAS

        # This is not very investigated, but basically any thread can run garbage collection
        # of any variable, even tkinter of another thread.
        # https://stackoverflow.com/questions/44781806/tkinter-objects-being-garbage-collected-from-the-wrong-thread
        # https://bugs.python.org/issue39093

        # Invoking garbage collection here (or during 'figureClose') does not work.
        # I assume that some part of the canvas is still kept in the memory, that said
        # calling 'delete' on tk image does not actually delete it right away.

        # Should be same as
        #    matplotlib._pylab_helpers.Gcf.get_fig_manager(1).canvas._tkphoto
        # or matplotlib._pylab_helpers.Gcf.get_all_fig_managers()[0].canvas._tkphoto
        CANVAS = matplotlib._pylab_helpers.Gcf.get_active().canvas._tkphoto
    except Exception:
        pass

    return figure


@plot_only
def axisEqual(figure: matplotlib.figure.Figure = None) -> List[float]:
    """Equal axis on current / selected figure.

    Arguments:
    figure -- figure to plot to,
              matplotlib.figure.Figure, default 'current figure'

    Returns:
    xmin, xmax, ymin, ymax -- the axis limits
    """
    if figure is None:
        figure = pyplot.gcf()

    return figure.axes[0].axis("equal")


@plot_only
def figureSave(
        filename: str,
        figure: matplotlib.figure.Figure = None,
        *,
        bbox_inches = "tight",
        dpi = 300,
        pad_inches = 0,
        **kwargs) -> None:
    """Save figure into a file.

    Arguments:
    filename -- path to the file where to save the image, str
    figure -- figure to plot to,
              matplotlib.figure.Figure, default 'current figure'

    Keyword-only arguments (same as matplotlib.pyplot.savefig):
    bbox_inches -- boundary box in inches, str or float, defaults to 'tight'
    dpi -- the image resolution, float > 0, defaults to 300
    pad_inches -- padding around the image with tight bbox,
                  float, defaults to 0
    """
    if figure is None:
        figure = pyplot.gcf()

    figure.savefig(
        filename,
        bbox_inches=bbox_inches,
        dpi=dpi,
        pad_inches=pad_inches,
        **kwargs
    )


@plot_only
def figureShow() -> None:
    """Show figure to the user."""
    pyplot.show()


@plot_only
def figureClose(figure: matplotlib.figure.Figure = None) -> None:
    """Close figure.

    Arguments:
    figure -- figure to plot to,
              matplotlib.figure.Figure, default 'current figure'
    """
    pyplot.close(figure)


######################
# Type plot functions
######################

def trackPlot(
        track: numpy.ndarray,
        figure: matplotlib.figure.Figure = None,
        *,
        s = 1,
        color = "gainsboro",
        **kwargs) -> None:
    """Plot a track to selected figure.

    Arguments:
    track -- points of the track valid area, nx2 numpy.ndarray
    figure -- figure to plot to,
              matplotlib.figure.Figure, default 'current figure'

    Keyword-only arguments (same as matplotlib.pyplot.scatter):
    s --  marker size in points**2,
          scalar or array-like with shape (n, ), defaults to 1
    color -- marker color for the track,
             color or sequence, defaults to 'gainsboro'
    """
    pointsScatter(track, figure, s=s, color=color, **kwargs)


def bordersPlot(
        borders: List[numpy.ndarray],
        colored: bool = True,
        figure: matplotlib.figure.Figure = None,
        *,
        linewidth = 0.6,
        linestyle = "dotted",
        color = "gray",
        **kwargs) -> None:
    """Plot borders of the segments to a selected figure.

    Arguments:
    borders -- border points of the groups, n-list of x2 numpy.ndarray
    colored -- when True, plot is done colored, otherwise single color,
               bool, default True
    figure -- figure to plot to,
              matplotlib.figure.Figure, default 'current figure'

    Keyword-only arguments (same as matplotlib.pyplot.plot):
    linewidth -- float, defaults to 0.6
    linestyle -- str, defaults to 'dotted'
    color -- color or sequence, defaults to 'gray'
    """
    groupsPlot(
        borders,
        figure,
        linewidth = linewidth,
        linestyle = linestyle,
        color = color if colored else None,
        **kwargs
    )


def indicesPlot(
        points: numpy.ndarray,
        figure: matplotlib.figure.Figure = None,
        *,
        verticalalignment = "center",
        horizontalalignment = "center",
        fontsize = 18,
        **kwargs) -> None:
    """Show an index of each element on its location.

    Arguments:
    points -- locations where to show the indices, nx2 numpy.ndarray
    figure -- figure to plot to,
              matplotlib.figure.Figure, default 'current figure'

    Keyword-only arguments (same as matplotlib.text.Text):
    verticalalignment -- str, defaults to 'center'
    horizontalalignment -- str, defaults to 'center'
    fontsize -- float, defaults to 18
    """
    for i, point in enumerate(points):
        labelText(
            point,
            i,
            verticalalignment = verticalalignment,
            horizontalalignment = horizontalalignment,
            fontsize = fontsize,
            **kwargs
        )


######################
# General functions
######################

@plot_only
def pointsScatter(
        points: numpy.ndarray,
        figure: matplotlib.figure.Figure = None,
        **kwargs) -> matplotlib.collections.PathCollection:
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
def pointsPlot(
        points: numpy.ndarray,
        figure: matplotlib.figure.Figure = None,
        **kwargs) -> List[matplotlib.lines.Line2D]:
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
def groupsScatter(
        groups: numpy.ndarray,
        figure: matplotlib.figure.Figure = None,
        **kwargs) -> None:
    """Scatter points inside groups.

    Arguments:
    groups -- list of grouped points, m-list of x2 numpy.ndarrays
    **kwargs -- keyword arguments to be passed to scatter
    """
    for _g in groups:
        pointsScatter(_g, figure, **kwargs)


@plot_only
def groupsPlot(
        groups: numpy.ndarray,
        figure: matplotlib.figure.Figure = None,
        **kwargs) -> None:
    """Plot points inside groups.

    Arguments:
    groups -- list of grouped points, m-list of x2 numpy.ndarrays
    **kwargs -- keyword arguments to be passed to plot
    """
    for _g in groups:
        pointsPlot(_g, figure, **kwargs)


@plot_only
def grouplayersScatter(
        grouplayers: numpy.ndarray,
        figure: matplotlib.figure.Figure = None,
        **kwargs) -> None:
    """Scatter points from layers of all groups.

    Arguments:
    grouplayers -- points in all layers,
                   n-list of (layer_count)-lists of x2 numpy.ndarray
    **kwargs -- keyword arguments to be passed to scatter
    """
    for _g in grouplayers:
        groupsScatter(_g, figure, **kwargs)


@plot_only
def grouplayersPlot(
        grouplayers: numpy.ndarray,
        figure: matplotlib.figure.Figure = None,
        **kwargs) -> None:
    """Plot points from layers of all groups.

    Arguments:
    grouplayers -- points in all layers,
                   n-list of (layer_count)-lists of x2 numpy.ndarray
    **kwargs -- keyword arguments to be passed to plot
    """
    for _g in grouplayers:
        groupsPlot(_g, figure, **kwargs)


@plot_only
def labelText(
        point: numpy.ndarray,
        s: str,
        figure: matplotlib.figure.Figure = None,
        **kwargs) -> None:
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

@plot_only
def _pyplot(
        *args,
        function: str,
        figure: matplotlib.figure.Figure = None,
        **kwargs) -> any:
    """Call directly a function of matplotlib pyplot.

    Arguments:
    function -- name of the function, str
    *args -- positional arguments to be passed to the function
    **kwargs -- keyword arguments to be passed to the function

    Returns:
    The returned object of the function.
    """
    if function not in dir(pyplot):
        print (
            "Unknown function '%s' of pyplot." % function, file = sys.stderr
        )
        return

    return pyplot.__getattribute__(function)(*args, **kwargs)


@plot_only
def _figure(
        *args,
        function: str,
        figure: matplotlib.figure.Figure = None,
        **kwargs) -> any:
    """Call directly a function of matplotlib pyplot's figure.

    Arguments:
    function -- name of the function, str
    figure -- figure to plot to,
              matplotlib.figure.Figure, default 'current figure'
    *args -- positional arguments to be passed to the function
    **kwargs -- keyword arguments to be passed to the function

    Returns:
    The returned object of the function.
    """
    if function not in dir(pyplot):
        print (
            "Unknown function '%s' of pyplot figure." % function,
            file = sys.stderr
        )
        return

    if figure is None:
        figure = pyplot.gcf()

    return figure.axes[0].__getattribute__(function)(*args, **kwargs)


######################
# Dynamic plotting
######################

def plotDyn(
        args: Dict[str, Dict[str, any]],
        figure: matplotlib.figure.Figure = None,
        **kwargs) -> None:
    """Dynamically create figure according to the configuration.

    Arguments:
    args -- dynamic plot arguments
    figure -- figure to plot to,
              matplotlib.figure.Figure, default 'current figure'
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
                            print (
                                "Key '%s' is not available." % a,
                                file=sys.stderr
                            )
                    else:
                        pargs.append(kwargs.get(a))
                else:
                    pargs.append(a)

            if isinstance(fargs, dict):
                globals()[function](
                    *pargs,
                    **{
                        **dict([
                            (f, i) for f, i in fargs.items() if f[0] != "_"
                        ]),
                        **{"figure": figure}
                    }
                )
            else:
                globals()[function](*pargs, **{"figure": figure})
