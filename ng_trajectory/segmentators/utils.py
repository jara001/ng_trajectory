#!/usr/bin/env python3.6
# utils.py3
"""Various utilities for segmentators.
"""
######################
# Imports & Globals
######################

import numpy

# Global variables
MAP = None
MAP_ORIGIN = None
MAP_GRID = None
MAP_LAST = None
HOOD8 = numpy.asarray([[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]])


######################
# Utilities (Map)
######################

def mapCreate(points: numpy.ndarray, origin: numpy.ndarray = None, size: numpy.ndarray = None, grid: float = None) -> None:
    """Create a cell map representation from given points.

    Arguments:
    points -- given set of points to add to the map, nx(>=2) numpy.ndarray
    origin -- origin of the map, 1x2 numpy.ndarray
    size -- size of the map, 1x2 numpy.ndarray
    grid -- when set, use this value as a grid size, otherwise it is computed, float
    """
    global MAP, MAP_ORIGIN, MAP_GRID

    print ("Creating map...")

    # Obtain grid size if not set
    _grid = grid if grid else gridCompute(points)

    print ("\tGrid:", _grid)

    # Obtain origin if not set
    _origin = origin if origin else numpy.min(points, axis = 0)

    print ("\tOrigin:", _origin)

    # Obtain size if not set
    _size = size if size else numpy.abs(
            numpy.subtract(
                numpy.max(points, axis = 0),
                numpy.min(points, axis = 0)
            )
        )

    print ("\tMin:", numpy.min(points, axis = 0))
    print ("\tMax:", numpy.max(points, axis = 0))
    print ("\tDist:", numpy.subtract(
        numpy.max(points, axis = 0),
        _origin
    ))
    print ("\tSize:", _size)

    print ("\tCell size:", (_size / _grid) + 1, ( (_size / _grid) + 1 ).astype(numpy.uint64))

    _m = numpy.zeros(( (_size / _grid) + 1).astype(numpy.uint64), dtype=numpy.uint8)

    for _p in points:
        _m[tuple(numpy.round( numpy.subtract(_p[:2], _origin) / _grid).astype(numpy.uint64))] = 100

    MAP = _m
    MAP_ORIGIN = _origin
    MAP_GRID = _grid

    print ("Map created.")

    return MAP, MAP_ORIGIN, MAP_GRID


def pointToMap(point: list) -> numpy.ndarray:
    """Converts real coordinates of a point to cell coordinates.

    Arguments:
    points -- point to convert, >=2-list

    Returns:
    cpoints -- cell coordinates of the points, 1x2 numpy.ndarray
    """
    global MAP_ORIGIN, MAP_GRID

    return numpy.round( numpy.subtract(numpy.asarray(point)[:2], MAP_ORIGIN) / MAP_GRID).astype(numpy.uint64)


def pointsToMap(points: numpy.ndarray) -> numpy.ndarray:
    """Converts real coordinates of the points to cell coordinates.

    Arguments:
    points -- points to convert, nx(>=2) numpy.ndarray

    Returns:
    cpoints -- cell coordinates of the points, nx2 numpy.ndarray
    """
    global MAP_ORIGIN, MAP_GRID

    return numpy.round( numpy.subtract(points[:, :2], MAP_ORIGIN) / MAP_GRID).astype(numpy.uint64)


def gridCompute(points: numpy.ndarray) -> float:
    """Computes square grid size from given points.

    Arguments:
    points -- points from a grid of unknown size, nx(>=2) numpy.ndarray

    Returns:
    grid_size -- size of the square grid in same units as source, float
    """
    return numpy.min(
            [
                numpy.min( numpy.subtract(u[1:], u[:-1]) ) for u in
                    [
                        numpy.unique( points[:, d] ) for d in range(2)
                    ]
            ]
        )


######################
# Utilities (Grid)
######################

def hoodObtain(cpoint: numpy.ndarray) -> numpy.ndarray:
    """Obtain the 8-neighbourhood of a cell.

    Arguments:
    cpoint -- cell coordinates of the point, 1x2 numpy.ndarray

    Returns:
    hood -- neighbour cells, (3-8)x2 numpy.ndarray

    Note: Instead of creating each point and finding whether it is
    inside the boundaries, we do this.
    """
    _hood = cpoint + HOOD8

    return _hood[
        ( ~ numpy.any( _hood < 0, axis = 1 ) )
        &
        ( _hood[:, 0] < MAP.shape[0] )
        &
        ( _hood[:, 1] < MAP.shape[1] )
    ]
