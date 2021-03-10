#!/usr/bin/env python3.6
# transform.py
"""Entrypoint for ng_trsajectory."""
######################
# Imports & Globals
######################

from ng_trajectory.utils import *

# Typing
Interpolator = List[numpy.ndarray]


######################
# Configuration import
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
