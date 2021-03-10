#!/usr/bin/env python3.6
# transform.py
"""Matryoshka transform and transform utilities.
"""
######################
# Imports & Globals
######################

import numpy

# Interpolation for Matryoshka
from scipy.interpolate import bisplrep, bisplev

# Typing support for other types
from typing import List, Tuple

# Utils functions
from ng_trajectory.utils import trajectoryReduce


# Typing
Interpolator = List[numpy.ndarray]


######################
# Utils
######################

def groupLayersCompute(layer0: numpy.ndarray, layer0_center: numpy.ndarray, layer_count: int) -> List[numpy.ndarray]:
    """Compute layers of a group in real coordinates.

    Arguments:
    layer0 -- points on first layer, mx(>=2) numpy.ndarray
    layer0_center -- center point of the first layer, 1x2 numpy.ndarray
    layer_count -- number of layers in the transformation, int

    Returns:
    layers -- points in all layers, (layer_count)-list of x2 numpy.ndarray
    """
    return groupsLayersCompute([layer0], [layer0_center], [layer_count])[0]


def groupsLayersCompute(layers0: List[numpy.ndarray], layers0_center: List[numpy.ndarray], layers_count: List[int]) \
    -> List[List[numpy.ndarray]]:
    """Compute layers of groups in real coordinates.

    Arguments:
    layers0 -- points on first layer of groups, n-list of mx(>=2) numpy.ndarray
    layers0_center -- center point of the first layer of groups, n-list of 1x2 numpy.ndarray
    layers_count -- number of layers in the transformation for each group, n-list of ints

    Returns:
    grouplayers -- points in all layers, n-list of (layer_count)-lists of x2 numpy.ndarray

    Note: This computes layers of a segment (based on the layer 0 / border). Points are
    moved towards the centroid of the segment.

    Note: Similarly to 'layerIndexToParameters', number of points in each layer
    is lowered.
    """

    layers0_size = [ len(layer) for layer in layers0 ]

    return [
        [
            trajectoryReduce(
                numpy.add(
                    numpy.multiply(
                        numpy.subtract(
                            layer0[:, :2], layers0_center[i][:, numpy.newaxis].T
                        ), 1 - (1 / layers_count[i]) * layer_index
                    ), layers0_center[i][:, numpy.newaxis].T
                ), int(layers0_size[i] - (layers0_size[i] / layers_count[i]) * layer_index)
            )
            for layer_index in range(layers_count[i])
        ] for i, layer0 in enumerate(layers0)
    ]


def layerIndexToParameters(layer_index: int, layer0_size: int, layer_count: int) -> Tuple[int, float]:
    """Compute size of a layer and its scale from its index.

    Arguments:
    layer_index -- index of a layer, int
    layer0_size -- number of points in layer0, int
    layer_count -- number of layers, int

    Returns:
    layer_params -- size of a layer and its scale, 2-tuple (int, float)

    Note: Layer size is scaled down accordingly to its index.
    Examples:
        1 layer : 100% points, scale 1

        2 layers:
            layer 0 : 100% points, scale 1
            layer 1 : 50% points, scale 0.5

        3 layers:
            layer 0 : 100% points, scale 1
            layer 1 : 66% points, scale 0.66
            layer 2 : 33% points, scale 0.33
    """
    return (int(layer0_size - ( (layer0_size / layer_count) * layer_index )), 1 - ( 1.0 / layer_count ) * layer_index)


def indicesToTransformedCoordinates(indices: List[float], layer_size: int, scale: float) -> numpy.ndarray:
    """Convert indices of a layer to transformed coordinates.

    Arguments:
    indices -- indices of points in selected layer, n-list of floats
    layer_size -- number of points in selected layer, int
    scale -- scaling factor of selected layer, float

    Returns:
    coords -- coordinates of points, nx2 numpy.ndarray

    Note: Scale is provided as a size of a square side of this coordinate frame.
    """

    _coords = []
    l = float(layer_size)

    for _i in indices:
        _point = []

        # 1 for x, -1 for y
        for _d in [1, -1]:
            _point.append(
                max(
                    min(
                        ( abs(-( ( ( (l / 2.0) + _i + _d * (l / 8.0) ) % l ) - ( l / 2.0 )) ) / ( l / 4.0 ) ) - 0.5,
                        1
                    ),
                    0
                ) * scale + ( ( 1 - scale ) / 2.0 )
            )

        _coords.append(_point)

    return numpy.asarray(_coords)


def indicesToRealCoordinates(indices: List[float], points: numpy.ndarray) -> numpy.ndarray:
    """Convert indices of a layer to real coordinates.

    Arguments:
    indices -- indices of points in selected layer, n-list of floats
    points -- points with real coordinates, nx2 numpy.ndarray

    Returns:
    rcoords -- real coordinates of points, nx2 numpy.ndarray

    Note: When index is float, result is an interpolation between integer neighbours.
    """

    _rcoords = []

    for _i in indices:
        if int(_i) == _i:
            _rcoords.append( points[ int(_i) , :] )
        else:
            _rcoords.append(
                points[ int(_i), : ] + (_i - int(_i)) * ( points[ ( int(_i) + 1 ) % points.shape[0], : ] - points [ int(_i), : ] )
            )

    return numpy.asarray(_rcoords)


######################
# Matryoshka
######################

def matryoshkaCreate(layer0: numpy.ndarray, layer0_center: numpy.ndarray, layer_count: int) -> List[Interpolator]:
    """Set up an interpolator for Matryoshka transformation.

    Arguments:
    layer0 -- points on first layer, mx(>=2) numpy.ndarray
    layer0_center -- center point of the first layer, 1x2 numpy.ndarray
    layer_count -- number of layers in the transformation, int

    Returns:
    ip2d -- Matryoshka mapping, 2-list of bisplrep
    """

    layers = groupLayersCompute(layer0, layer0_center, layer_count)
    layer0_size = len(layer0)

    # Method, where we learn the interpolator all layers at once and use that information
    # for computing of the transformation
    # Note: This is much more precise.
    _tc = numpy.empty( (0, 2) )
    _rc = numpy.empty( (0, 2) )

    for _layer_index in range(layer_count):
        layer_params = layerIndexToParameters(_layer_index, layer0_size, layer_count)
        indices = range(layer_params[0])
        tc = indicesToTransformedCoordinates(indices, *layer_params)
        rc = indicesToRealCoordinates(indices, layers[int(_layer_index)])

        _tc = numpy.vstack( (_tc, tc) )
        _rc = numpy.vstack( (_rc, rc) )

    # transformedCoordinatesToRealCoordinates
    _ip2d = []

    for _d in range(_rc.shape[1]):
        _ip2d.append(
            bisplrep(_tc[:, 0], _tc[:, 1], _rc[:, _d])
        )

    return _ip2d


def matryoshkaMap(matryoshka: List[Interpolator], coords: numpy.ndarray) -> numpy.ndarray:
    """Transform a point through Matryoshka mapping.

    Arguments:
    matryoshka -- mapping, 2-list of bisplrep
    coords -- points in transformed coordinates to convert, nx2 numpy.ndarray

    Returns:
    rcoords -- points in real coordinates, nxp numpy.ndarray
    """

    _rcoords = []

    for _c in coords:
        _dims = []

        for _interpolator in matryoshka:
            _dims.append(
                #_interpolator(_c[0], _c[1])[0]
                bisplev(_c[0], _c[1], _interpolator)
            )

        _rcoords.append(_dims)

    return numpy.asarray(_rcoords)
