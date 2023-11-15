#!/usr/bin/env python3.6
# transform.py
"""Matryoshka transform and transform utilities."""
######################
# Imports & Globals
######################

import numpy

# Interpolation for Matryoshka
from scipy.interpolate import bisplrep, bisplev

# Typing support for other types
from typing import List, Tuple

# Utils functions
from ng_trajectory.interpolators.utils import trajectoryReduce, trajectorySort
from .interpolate import trajectoryInterpolate


# Typing
Interpolator = List[numpy.ndarray]


######################
# Utils
######################

def pointsFilter(points: numpy.ndarray, grid: float = None) -> numpy.ndarray:
    """Filter out points that are not necessary in trajectory planning.

    Arguments:
    points -- points to be filtered, nx2 numpy.ndarray

    Returns:
    fpoints -- filtered points, mx2 numpy.ndarray

    Note: It is expected that points are aligned to a square grid.
    """
    _points = []

    # Obtain grid size if not set
    _grid = grid if grid else numpy.min(
        [
            numpy.min(numpy.subtract(u[1:], u[:-1])) for u in
            [
                numpy.unique(points[:, d]) for d in range(points.shape[1])
            ]
        ]
    )

    # Convert points to cell-oriented list
    _cells = [
        [
            int(numpy.round(_p[_d] / _grid)) for _d in range(points.shape[1])
        ] for _p in points
    ]
    _cells_copy = [
        [
            int(numpy.round(_p[_d] / _grid)) for _d in range(points.shape[1])
        ] for _p in points
    ]

    # Treat points as list
    pointsL = points.tolist()

    for _p in pointsL:
        # Remove unnecessary points.
        # _X_  ___  ___  _X_  |  _XX  __X  ___  ___  ___  ___  X__  XX_
        # _XX  _XX  XX_  XX_  |  _X_  _XX  _XX  _X_  _X_  XX_  XX_  _X_
        # ___  _X_  _X_  ___  |  ___  ___  __X  _XX  XX_  X__  ___  ___
        # Convert points to a cell
        _cell = [
            int(numpy.round(_p[_d] / _grid)) for _d in range(points.shape[1])
        ]

        # Ranges
        _xr = range(-1, 2)
        _yr = range(-1, 2)

        # Count number of points in the area (x-wise, y-wise)
        x = sum([
            any([
                [_cell[0] + _x, _cell[1] + _y] in _cells for _x in _xr
            ]) for _y in _yr
        ])
        y = sum([
            any([
                [_cell[0] + _x, _cell[1] + _y] in _cells for _y in _yr
            ]) for _x in _xr
        ])

        if x < 3 and y < 3:
            # Return nearby points back to the loop
            for _xr in range(-1, 2):
                for _yr in range(-1, 2):
                    if _xr == _yr == 0:
                        continue

                    # Get index of the cell
                    _nearbyc = [_cell[0] + _xr, _cell[1] + _yr]

                    # Find whether it is a valid border cell and try to find it
                    if (
                        _nearbyc in _cells_copy
                        and pointsL[_cells_copy.index(_nearbyc)] in _points
                    ):
                        _nearbyp = pointsL[_cells_copy.index(_nearbyc)]
                        _points.remove(_nearbyp)
                        pointsL.append(_nearbyp)

            _cells.remove(_cell)

        else:
            _points.append(_p)

    return numpy.asarray(_points)


######################
# Utilities (Groups)
######################

def groupsCenterCompute(groups: List[numpy.ndarray]) -> numpy.ndarray:
    """Compute centers of the groups.

    Arguments:
    groups -- points split into groups, n-list of mx2 numpy.ndarray

    Returns:
    centers -- center points of the groups, nx2 numpy.ndarray
    """
    return numpy.asarray([numpy.mean(_g, axis = 0) for _g in groups])


def groupsBorderObtain(
        groups: List[numpy.ndarray],
        grid: float = None) -> List[numpy.ndarray]:
    """Obtain border points of the groups.

    Arguments:
    groups -- points split into groups, n-list of mx2 numpy.ndarray

    Returns:
    borders -- border points of the groups, n-list of px2 numpy.ndarray

    Note: Border points are not ordered.
    Note: It is expected that points are aligned to a square grid.
    Note: It is also expected that the group shape is rather convex.
    TODO: Consider using set instead of numpy.unique / list.
    TODOd: This version is expecting mostly convex areas. Highly unconvex
    areas are missing some of the border points.
    """
    _borders = []

    for _i, _g in enumerate(groups):
        _border = []

        # Obtain grid size if not set
        _grid = grid if grid else numpy.min(
            [
                numpy.min(numpy.subtract(u[1:], u[:-1])) for u in
                [
                    numpy.unique(_g[:, d]) for d in range(_g.shape[1])
                ]
            ]
        )

        # Go through all dimensions
        for _d in range(_g.shape[1]):

            # Find unique values in that dimension
            for _u in numpy.unique(_g[:, _d]):

                # Append points with max / min values in another dimension
                _border.append(
                    numpy.min(
                        _g[numpy.where(_g[:, _d] == _u), :],
                        axis = 1
                    ).tolist()[0]
                )

                _border.append(
                    numpy.max(
                        _g[numpy.where(_g[:, _d] == _u), :],
                        axis = 1
                    ).tolist()[0]
                )

                # Append inner borders
                # Obtain values in the dimension
                _v = _g[numpy.where(_g[:, _d] == _u), :][0]
                _v = numpy.delete(_v, _d, axis = 1)  # Select other dimensions

                # Sort them
                # Enforce the axis as otherwise it is not sorted
                # in ascending order everytime.
                _v = numpy.sort(_v, axis = 0)

                # Find distances between concurrent points
                _dists = numpy.subtract(
                    numpy.roll(_v, 1, axis=1)[1:],
                    _v[:-1]
                )

                # Find points in the distance larger than 1.5x _grid
                _bords = numpy.where(
                    _dists > (_grid * 1.5)
                )[0]
                # DO NOT FORGET TO ADD 1 TO INDICES!

                # Add these points to the border
                # for _b in _bords:
                #     _border.append(
                #         _g[
                #             numpy.intersect1d(
                #                 numpy.where(_g[:, _d] == _u)[0],
                #                 numpy.where(_g[:, ^_d] == _b)[0]
                #             )
                #         ].tolist()
                #     )
                # ^^ Intersect would restrict dimensions.

                # Reuse '_v' array as it is sorted. At this point,
                # using 'where(_g...)' is NOT USABLE AT ALL because
                # the indices are not the same!
                for _b in _bords:
                    _border.append(
                        ([_u] + _v[_b].tolist()) if _d == 0
                        else (_v[_b].tolist() + [_u])
                    )

                    _border.append(
                        ([_u] + _v[_b + 1].tolist()) if _d == 0
                        else (_v[_b + 1].tolist() + [_u])
                    )


        _borders.append(_border)

    return [
        numpy.unique(numpy.asarray(b), axis = 0) for b in _borders
    ]


def groupsBorderBeautify(
        borders: List[numpy.ndarray],
        border_length: int) -> List[numpy.ndarray]:
    """Filter, sort and interpolate the borders to get smoother points.

    Arguments:
    borders -- border points of the groups, n-list of x2 numpy.ndarray

    Returns:
    bborders -- beautified border points of the groups,
                n-list of (border_length)x2 numpy.ndarray

    Note: This function is in beta, especially the point filtering.
    """
    global GROUP_ID

    bborders = []

    for group_i, border in enumerate(borders):
        # FIXME: Temporarily hidden as we are working with 0.02 map in Stage.
        border_filtered = pointsFilter(border)  # 0.05

        border_sorted = trajectorySort(border_filtered, verify_sort = True)

        border_interpolated = trajectoryInterpolate(
            border_sorted,
            border_length
        )

        bborders.append(border_interpolated)

    return bborders


def groupLayersCompute(
        layer0: numpy.ndarray,
        layer0_center: numpy.ndarray,
        layer_count: int) -> List[numpy.ndarray]:
    """Compute layers of a group in real coordinates.

    Arguments:
    layer0 -- points on first layer, mx(>=2) numpy.ndarray
    layer0_center -- center point of the first layer, 1x2 numpy.ndarray
    layer_count -- number of layers in the transformation, int

    Returns:
    layers -- points in all layers, (layer_count)-list of x2 numpy.ndarray
    """
    return groupsLayersCompute([layer0], [layer0_center], [layer_count])[0]


def groupsLayersCompute(
        layers0: List[numpy.ndarray],
        layers0_center: List[numpy.ndarray],
        layers_count: List[int]) -> List[List[numpy.ndarray]]:
    """Compute layers of groups in real coordinates.

    Arguments:
    layers0 -- points on first layer of groups, n-list of mx(>=2) numpy.ndarray
    layers0_center -- center point of the first layer of groups,
                      n-list of 1x2 numpy.ndarray
    layers_count -- number of layers in the transformation for each group,
                    n-list of ints

    Returns:
    grouplayers -- points in all layers,
                   n-list of (layer_count)-lists of x2 numpy.ndarray

    Note: This computes layers of a segment (based on the layer 0 / border).
          Points are moved towards the centroid of the segment.

    Note: Similarly to 'layerIndexToParameters', number of points in each layer
          is lowered.
    """
    layers0_size = [len(layer) for layer in layers0]

    return [
        [
            trajectoryReduce(
                numpy.add(
                    numpy.multiply(
                        numpy.subtract(
                            layer0[:, :2],
                            layers0_center[i][:, numpy.newaxis].T
                        ), 1 - (1 / layers_count[i]) * layer_index
                    ), layers0_center[i][:, numpy.newaxis].T
                ),
                int(
                    layers0_size[i]
                    - (layers0_size[i] / layers_count[i])
                    * layer_index
                )
            )
            for layer_index in range(layers_count[i])
        ] for i, layer0 in enumerate(layers0)
    ]


def layerIndexToParameters(
        layer_index: int,
        layer0_size: int,
        layer_count: int) -> Tuple[int, float]:
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
    return (
        int(layer0_size - ((layer0_size / layer_count) * layer_index)),
        1 - (1.0 / layer_count) * layer_index
    )


def indicesToTransformedCoordinates(
        indices: List[float],
        layer_size: int,
        scale: float) -> numpy.ndarray:
    """Convert indices of a layer to transformed coordinates.

    Arguments:
    indices -- indices of points in selected layer, n-list of floats
    layer_size -- number of points in selected layer, int
    scale -- scaling factor of selected layer, float

    Returns:
    coords -- coordinates of points, nx2 numpy.ndarray

    Note: Scale is provided as a size of a square side of
          this coordinate frame.
    """
    _coords = []
    l = float(layer_size)  # noqa: E741

    for _i in indices:
        _point = []

        # 1 for x, -1 for y
        for _d in [1, -1]:
            _point.append(
                max(
                    min(
                        ( abs(-( ( ( (l / 2.0) + _i + _d * (l / 8.0) ) % l ) - ( l / 2.0 )) ) / ( l / 4.0 ) ) - 0.5,  # noqa: E201,E202,E501
                        1
                    ),
                    0
                ) * scale + ((1 - scale) / 2.0)
            )

        _coords.append(_point)

    return numpy.asarray(_coords)


def indicesToRealCoordinatesInt(
        indices: List[int],
        points: numpy.ndarray) -> numpy.ndarray:
    """Convert indices (int only) of a layer to real coordinates.

    Arguments:
    indices -- indices of points in selected layer, n-list of ints
    points -- points with real coordinates, nx2 numpy.ndarray

    Returns:
    rcoords -- real coordinates of points, nx2 numpy.ndarray

    Note: When approximating the layer with float indices, use
    `indicesToRealCoordinates` instead that can interpolate points.
    """
    return points[indices, :]


def indicesToRealCoordinates(
        indices: List[float],
        points: numpy.ndarray) -> numpy.ndarray:
    """Convert indices of a layer to real coordinates.

    Arguments:
    indices -- indices of points in selected layer, n-list of floats
    points -- points with real coordinates, nx2 numpy.ndarray

    Returns:
    rcoords -- real coordinates of points, nx2 numpy.ndarray

    Note: When index is float, result is an interpolation
          between the integer neighbours.
    """
    _rcoords = []

    for _i in indices:
        if int(_i) == _i:
            _rcoords.append(points[int(_i), :])
        else:
            _rcoords.append(
                points[int(_i), :]
                + (_i - int(_i)) * (
                    points[(int(_i) + 1) % points.shape[0], :]
                    - points[int(_i), :]
                )
            )

    return numpy.asarray(_rcoords)


######################
# Matryoshka
######################

def matryoshkaCreate(
        layer0: numpy.ndarray,
        layer0_center: numpy.ndarray,
        layer_count: int) -> List[Interpolator]:
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

    # Method, where we learn the interpolator all layers at once
    # and use that information for computing of the transformation
    # Note: This is much more precise.
    _tc = numpy.empty((0, 2))
    _rc = numpy.empty((0, 2))

    for _layer_index in range(layer_count):
        layer_params = layerIndexToParameters(
            _layer_index,
            layer0_size,
            layer_count
        )
        indices = range(layer_params[0])
        tc = indicesToTransformedCoordinates(indices, *layer_params)
        rc = indicesToRealCoordinatesInt(indices, layers[int(_layer_index)])

        _tc = numpy.vstack((_tc, tc))
        _rc = numpy.vstack((_rc, rc))

    # transformedCoordinatesToRealCoordinates
    _ip2d = []

    for _d in range(_rc.shape[1]):
        _ip2d.append(
            bisplrep(_tc[:, 0], _tc[:, 1], _rc[:, _d])
        )

    return _ip2d


def matryoshkaMap(
        matryoshka: List[Interpolator],
        coords: numpy.ndarray) -> numpy.ndarray:
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
                # _interpolator(_c[0], _c[1])[0]
                bisplev(_c[0], _c[1], _interpolator)
            )

        _rcoords.append(_dims)

    return numpy.asarray(_rcoords)
