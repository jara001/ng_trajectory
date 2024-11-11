#!/usr/bin/env python3.6
# main.py
"""Compute criterion using speed profile and lap time.

[1] N. R. Kapania, J. Subosits, and J. Christian Gerdes, ‘A Sequential
Two-Step Algorithm for Fast Generation of Vehicle Racing Trajectories’,
Journal of Dynamic Systems, Measurement, and Control, vol. 138, no. 9,
p. 091005, Sep. 2016, doi: 10.1115/1.4033311.
"""
######################
# Imports & Globals
######################

import sys
import os
import numpy

from . import profiler

from ng_trajectory.interpolators.utils import (
    pointDistance,
    trajectoryClosestIndex
)

from ng_trajectory.segmentators.utils import (
    filterPoints,
    pointInBounds,
    pointToMap,
    pointsToMap,
    pointsToWorld,
    getMap, getMapOrigin, getMapGrid,
    validCheck
)

from ng_trajectory.log import (
    logfileName,
    log, logvv,
    print0
)

import ng_trajectory.plot as ngplot

from ng_trajectory.parameter import ParameterList

from multiprocessing import Queue

from itertools import chain  # Join generators

from scipy import spatial
import math

# Global variables
CENTERLINE = None
REFERENCE_PROGRESS = None
OVERTAKING_POINTS = Queue()

MAP_INSIDE = None
MAP_OUTSIDE = None

# Parameters
P = ParameterList()
P.createAdd("overlap", 0, int, "Size of the trajectory overlap. 0 disables this.", "")
P.createAdd("_mu", 0.2, float, "Friction coeficient", "init")
P.createAdd("_g", 9.81, float, "Gravity acceleration coeficient", "init")
P.createAdd("_m", 3.68, float, "Vehicle mass", "init")
P.createAdd("_ro", 1.2, float, "Air density", "init")
P.createAdd("_A", 0.3, float, "Frontal reference aerodynamic area", "init")
P.createAdd("_cl", 1, float, "Drag coeficient", "init")
P.createAdd("v_0", 0, float, "Initial speed [m.s^-1]", "init")
P.createAdd("v_lim", 4.5, float, "Maximum forward speed [m.s^-1]", "init")
P.createAdd("a_acc_max", 0.8, float, "Maximum longitudal acceleration [m.s^-2]", "init")
P.createAdd("a_break_max", 4.5, float, "Maximum longitudal decceleration [m.s^-2]", "init")
P.createAdd("_lf", 0.191, float, "Distance from center of mass to the front axle [m]", "init")
P.createAdd("_lr", 0.139, float, "Distance from center of mass to the rear axle [m]", "init")
P.createAdd("reference", None, str, "Name of the file to load (x, y, t, v) reference path that cannot be close. Currently, the only supported type is .npy", "init")
P.createAdd("reference_dist", 1.0, float, "Minimum allowed distance from the reference at given time [m].", "init")
P.createAdd("reference_rotate", 0, int, "Number of points to rotate the reference trajectory.", "init")
P.createAdd("reference_laptime", 0, float, "Lap time of the given reference. 0 = estimated from data", "init")
P.createAdd("save_solution_csv", "$", str, "When non-empty, save final trajectory to this file as CSV. Use '$' to use log name instead.", "init")
P.createAdd("plot", False, bool, "Whether a graphical representation should be created.", "init (viz.)")
P.createAdd("plot_reference", False, bool, "Whether the reference trajectory should be plotted.", "init (viz.)")
P.createAdd("plot_reference_width", 0.4, float, "Linewidth of the reference trajectory. 0 = disabled", "init (viz.)")
P.createAdd("plot_solution", False, bool, "Whether the optimized solution should be plotted. (Using 'plot_reference_width'.)", "init (viz.)")
P.createAdd("plot_timelines", False, bool, "Whether the lines between points in the same time should be plotted.", "init (viz.)")
P.createAdd("plot_timelines_size", 1, float, "Size of the points of the timelines endpoints. 0 = disabled", "init (viz.)")
P.createAdd("plot_timelines_width", 0.6, float, "Linewidth of the timelines. 0 = disabled", "init (viz.)")
P.createAdd("plot_overtaking", True, bool, "Whether to plot places where an overtaking occurs. (Has to be supported by optimizer.)", "init (viz.)")
P.createAdd("favor_overtaking", 0, float, "Penalty value to add to the lap time when overtaking does not occur.", "init")
P.createAdd("friction_map", None, str, "Name of the file to load (x, y, mu*100) with friction map.", "init")
P.createAdd("friction_map_inverse", False, bool, "When True, invert the values in the friction map.", "init")
P.createAdd("friction_map_expand", False, bool, "When True, values from the friction map are expanded over the whole map using flood fill.", "init")
P.createAdd("friction_map_plot", False, bool, "When True, friction map is plotted.", "init")
P.createAdd("friction_map_save", False, bool, "When True, friction map is saved alongside the log files.", "init")
P.createAdd("friction_map_yaml", None, str, "(Requires pyyaml) Name of the yaml configuration of the original map that was used to create '.npy' files. Map file specified in the configuration has to exist.")
P.createAdd("car_width", 0.3, float, "Width of the car when using rectangle vehicle shape.", "")
P.createAdd("car_length", 0.55, float, "Length of the car when using rectangle vehicle shape.", "")
P.createAdd("car_shape", "rectangle", str, "Vehicle shape to use when calculating collisions during overtaking. ['circle', 'rectangle']", "")
P.createAdd("ego_dist_overtake", 0.1, float, "How much in front of opponent ego car needs to be to have a right of way.", "")
P.createAdd("use_safe_zone_seconds", 10.0, float, "How many seconds after crash EGO cannot enter safety zone.", "")


######################
# Utilities
######################

def fmap_expand(friction_map, known_values):
    """Expand the known values over the whole friction map.

    Arguments:
    friction_map -- map of the environment with friction, numpy.ndarray
    known_values -- known values of the friction, used for expansion,
                    nx3 numpy.ndarray

    Returns:
    updated friction_map
    """
    cxy = pointsToMap(known_values[:, :2])
    help_map = numpy.ones_like(friction_map, dtype = bool)

    help_map[cxy[:, 0], cxy[:, 1]] = False


    queue = cxy.tolist()

    while len(queue) > 0:
        cell_x, cell_y = queue.pop(0)

        for _a, _b in [(-1, -1), (-1, 0), (-1, 1),
                       (+0, -1),          (+0, 1),   # noqa: E241
                       (+1, -1), (+1, 0), (+1, 1)]:

            # Try does catch larger values but not negative
            if cell_x + _a < 0 or cell_y + _b < 0:
                continue

            try:
                _cell = help_map[cell_x + _a, cell_y + _b]
                _cx = cell_x + _a
                _cy = cell_y + _b
            except IndexError:
                continue

            # Expand the value if not yet done
            if _cell:
                friction_map[_cx, _cy] = friction_map[cell_x, cell_y]
                help_map[_cx, _cy] = False

                queue.append((_cx, _cy))

    return friction_map


def saveMap(filename: str, map_data: numpy.ndarray):
    """Save the map into a file as npy.

    Arguments:
    filename -- name of the file to save map into (no extension), str
    map_data -- map to save, nx(>=2) numpy.ndarray

    Note: We expect that the map is a derivation of segmentator internal map,
          and their parameters are the same.

    Note: Requires pyyaml.

    Raises:
    ImportError -- when pyyaml is not available

    Source:
    https://stackoverflow.com/questions/49720605/pixel-coordinates-vs-drawing-coordinates
    """
    param_name = "friction_map_yaml"
    yaml_filename = P.getValue(param_name)

    def perr(*args, **kwargs):
        """Print an error message."""
        print0 (
            "profile: Unable to save friction_map:",
            *args, file = sys.stderr, **kwargs
        )

    # Try to import pyyaml
    try:
        from yaml import safe_load, dump
    except ImportError as e:
        perr (str(e))
        return

    # Check that we have the configuration file
    if yaml_filename is None:
        perr ("parameter '%s' is empty" % param_name)
        return

    if not os.access(yaml_filename, os.R_OK):
        perr (
            "%s '%s'" % (
                "permission denied"
                if os.access(yaml_filename, os.F_OK)
                else "no such file",
                filename
            )
        )
        return

    # Open the configuration
    try:
        with open(yaml_filename, "r") as f:
            conf = safe_load(f)
            logvv (
                "Loaded map configuration '%s': %s"
                % (yaml_filename, conf)
            )
    except Exception as e:
        perr ("configuration loading: %s" % str(e))
        return


    # Check fields in the configuration
    REQUIRED = {"image": str, "origin": list}

    for field, field_type in REQUIRED.items():
        if field not in conf:
            perr ("field '%s' is not in the configuration" % field)
            return

        if not isinstance(conf[field], field_type):
            perr (
                "field '%s' is type %s but should be %s"
                % (field, type(conf[field]), field_type)
            )
            return


    # Check that image exists
    if not os.access(conf["image"], os.R_OK):
        perr (
            "%s '%s'" % (
                "permission denied"
                if os.access(conf["image"], os.F_OK)
                else "no such file",
                conf["image"]
            )
        )
        return


    # # Process the map # #
    from PIL import Image

    # Load the original image in 'xy' format
    """
               X
        +------------>
        |
        |  0,0   1,0
      Y |
        |  0,1   1,1
        |
        v
    """
    im = Image.open(conf["image"])

    # Convert it into 'yx' numpy ndarray
    """
               Y
        +------------>
        |
        |  0,0   0,1
      X |
        |  1,0   1,1
        |
        v
    """
    nd = numpy.array(im)
    # Clear the map -- we just need the size.
    # Which is transposed now; im.size = nd.shape.T
    nd[:] = 0
    height = nd.shape[0]


    # Compute origin difference
    """
    The real map has origin in the left bottom corner.
    In addition, as we shrink the map (data) during creation,
    we have no idea, where is the real origin.

    That information is extracted from the yaml file, so we
    just have to compute the translation.

        ^           ^
     Y  |           |
     (  |  0,4      |  0,1   1,1
     o  |         Y |
     r  |  0,3      |  0,0   1,0
     i  |           |
     g  |  0,2      +----------->
     i  |                 X
     n  |  0,1   1,1
     a  |
     l  |  0,0   1,0   2,0   3,0
     )  |
        +----------------------->
               X(original)
    """
    trans = ((getMapOrigin() - conf["origin"][:2]) / getMapGrid()).astype(int)


    # Color the map; use inverted colors to represent the friction
    for (cx, cy), value in numpy.ndenumerate(map_data):
        # Shift the coordinates
        cxo, cyo = [cx, cy] + trans

        # Now we perform multiple steps at once to properly create the map;
        #   - to workaround 'xy' -> 'yx' we reverse the coordinates
        #     (x, y) = (y, x)
        #   - to get around the origin placement, we say that
        #     (y, x) = (0, 0) = (height, 0)
        nd[height - cyo, cxo] = 255 - value


    # Save the image
    try:
        Image.fromarray(nd).save(filename + ".pgm")
    except Exception as e:
        perr (str(e))
        return

    # Save the YAML
    conf["image"] = filename + ".pgm"
    conf["negate"] = 1
    conf["mode"] = "raw"

    try:
        with open(filename + ".yaml", "w") as f:
            dump(conf, f)
    except Exception as e:
        perr (str(e))
        return


######################
# Functions
######################

def determine_opponent_side(traj_pos_now, traj_pos_prev, opponent_pos):
    """
    * Helper function to determine a side of opponent relative to ego trajectory
    *
    * @param traj_pos_now a 1D ndarray of current trajectory point
    * @param traj_pos_prev a 1D ndarray of previous trajectory point
    * @param opponent_pos a 1D ndarray of opponent position 
    * @return 1.0 === opponent left, 0.0 === opponent mid, -1.0 === opponent right
    """
    # Compute required vectors
    ego_vector = traj_pos_now - traj_pos_prev
    opponent_vector = opponent_pos - traj_pos_prev
    # 1.0 - left, 0.0 - mid, -1.0 right
    # Compute vector multiplication to determine side of the opponent_pos
    return numpy.sign(ego_vector[0] * opponent_vector[1] - ego_vector[1] * opponent_vector[0])

def do_polygons_intersect(a: numpy.array, b: numpy.array) -> bool:
    """
    * Helper function to determine whether there is an intersection between the two polygons described
    * by the lists of vertices. Uses the Separating Axis Theorem
    *
    * @param a an ndarray of connected points [[x_1, y_1], [x_2, y_2],...] that form a closed polygon
    * @param b an ndarray of connected points [[x_1, y_1], [x_2, y_2],...] that form a closed polygon
    * @return true if there is any intersection between the 2 polygons, false otherwise

    Source: https://stackoverflow.com/questions/10962379/how-to-check-intersection-between-2-rotated-rectangles
    https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/previousinformation/physics4collisiondetection/2017%20Tutorial%204%20-%20Collision%20Detection.pdf
    """

    polygons = [a, b]

    for i in range(len(polygons)):

        # for each polygon, look at each edge of the polygon, and determine if it separates
        # the two shapes
        polygon = polygons[i]
        for i1 in range(len(polygon)):

            # grab 2 vertices to create an edge
            i2 = (i1 + 1) % len(polygon)
            p1 = polygon[i1]
            p2 = polygon[i2]

            # find the line perpendicular to this edge
            normal = p2 - p1

            # for each vertex in the first shape, project it onto the line perpendicular to the edge
            # and keep track of the min and max of these values
            projected = normal @ a.T
            minA = numpy.min(projected)
            maxA = numpy.max(projected)

            # for each vertex in the second shape, project it onto the line perpendicular to the edge
            # and keep track of the min and max of these values
            projected = normal @ b.T
            minB = numpy.min(projected)
            maxB = numpy.max(projected)

            # if there is no overlap between the projects, the edge we are looking at separates the two
            # polygons, and we know there is no overlap
            if (maxA < minB) or (maxB < minA):
                return False
    return True

def get_rect_points(center: numpy.ndarray, dims: numpy.ndarray, angle: float) -> numpy.array:
        """
        returns four corners of the rectangle.
        bottom left is the first conrner, from there it goes
        counter clockwise.
        """
        center = numpy.asarray(center)
        length, breadth = dims
        length_2 = length/2
        breadth_2 = breadth/2
        corners = numpy.array([[-length_2, -breadth_2],
                            [length_2, -breadth_2],
                            [length_2, breadth_2],
                            [-length_2, breadth_2]])
        sin_m_angle = numpy.sin(-angle)
        cos_m_angle = numpy.cos(-angle)

        rot = numpy.array([[cos_m_angle, sin_m_angle],
                           [-sin_m_angle, cos_m_angle]])
        corners = rot.dot(corners.T) + center[:, None]
        return corners.T  # n x 2

def init(**kwargs) -> None:
    """Initialize criterion."""
    global REFERENCE, CENTERLINE, OVERTAKING_POINTS, MAP_OUTSIDE, MAP_INSIDE

    profiler.parametersSet(**kwargs)

    P.updateAll(kwargs)

    # Recreating the Queue here makes the parent process use
    # different Queue than the ProcessPool.
    # OVERTAKING_POINTS = Queue()

    if P.getValue("save_solution_csv") == "":
        P.update("save_solution_csv", None)
    elif P.getValue("save_solution_csv") == "$":
        P.update(
            "save_solution_csv",
            logfileName() + ".csv"
        )

    if P.getValue("reference") is not None:
        REFERENCE = numpy.load(P.getValue("reference"))[:, :4]
        # REFERENCE = numpy.hstack((numpy.roll(REFERENCE[:, :2], -P.getValue("reference_rotate"), axis=0), REFERENCE[:, 2:]))
        # TODO: Lap time should be given, not estimated like this.
        lap_time = P.getValue("reference_laptime")

        if lap_time == 0.0:
            # Lap time estimate
            lap_time = REFERENCE[-1, 2] + numpy.mean([
                REFERENCE[-1, 2] - REFERENCE[-2, 2],
                REFERENCE[1, 2] - REFERENCE[0, 2]
            ])

        # roll trajectory and correct time stamps
        REFERENCE = numpy.roll(
            REFERENCE,
            -P.getValue("reference_rotate"),
            axis = 0
        )
        REFERENCE[:, 2] = REFERENCE[:, 2] - REFERENCE[0, 2]
        REFERENCE[REFERENCE[:, 2] < 0, 2] += lap_time

        # TODO use REFERENCE (x, y, t, v) and VALID_POINTS (n x 2) - valid points to create other track representations
        # compte trajectory normals
        vec_ = REFERENCE[1:, :2] - REFERENCE[:-1, :2]
        trajectory_norms = numpy.array([-vec_[:, 1], vec_[:, 0]]).T / numpy.linalg.norm(vec_, axis=1)[:, numpy.newaxis]

        print("INFO")

        from ng_trajectory.segmentators.utils import MAP_GRID, MAP
        from scipy.ndimage import morphology
        from skimage.segmentation import flood_fill
        import copy

        # create a new map
        map_ = copy.copy(MAP)

        # create proper borders on the map
        map_[0:, 0] = 0
        map_[0, 0:] = 0
        map_[0:, -1] = 0
        map_[-1, 0:] = 0
        # default map is created 

        # calc robot mask
        car_width = 0.4  # with safety region
        robot_radius_grid = int(numpy.round(car_width / MAP_GRID))
        y, x = numpy.ogrid[-robot_radius_grid:robot_radius_grid + 1, -robot_radius_grid:robot_radius_grid + 1]
        mask = x ** 2 + y ** 2 <= robot_radius_grid ** 2

        # ----- MAP INNER -----
        # use floodfill on outer wall to delete it
        MAP_INSIDE = copy.copy(map_)
        MAP_INSIDE = flood_fill(MAP_INSIDE, (0, 0), 100)

        # find inner wall and save some pixel position
        inner_wall_pos = numpy.where(MAP_INSIDE == 0)
        inner_wall_pos = (inner_wall_pos[0][0], inner_wall_pos[1][0])

        # inflate inner wall
        MAP_INSIDE = 100 - morphology.grey_dilation(100 - MAP_INSIDE, footprint=mask)

        # ----- MAP OUTER -----
        # use floodfill on inner wall to delete it
        MAP_OUTSIDE = copy.copy(map_)
        MAP_OUTSIDE = flood_fill(MAP_OUTSIDE, inner_wall_pos, 100)

        # inflate outer wall
        MAP_OUTSIDE = 100 - morphology.grey_dilation(100 - MAP_OUTSIDE, footprint=mask)

        log (
            "Loaded reference with '%d' points, lap time %fs."
            % (len(REFERENCE), lap_time)
        )
    else:
        REFERENCE = None

    if P.getValue("friction_map") is not None:
        fmap = numpy.load(P.getValue("friction_map"))

        # Filter out points outside of the map
        """
        This can happen when the friction map is slightly different, e.g.,
        it is built for non-inflated map of the track.
        """
        fmap = filterPoints(fmap)

        if P.getValue("friction_map_inverse"):
            fmap[:, 2] = 255 - fmap[:, 2]

        FRICTION_MAP = getMap().copy()

        # Set default value to the _mu parameter.
        FRICTION_MAP[:] = profiler._mu * 100

        # Set all obtained values.
        cxy = pointsToMap(fmap[:, :2])
        FRICTION_MAP[cxy[:, 0], cxy[:, 1]] = fmap[:, 2]

        # Expand the fmap is required
        if P.getValue("friction_map_expand"):
            FRICTION_MAP = fmap_expand(FRICTION_MAP, fmap)

        profiler.FRICTION_MAP = FRICTION_MAP

        # Save and plot the map
        if P.getValue("friction_map_save"):

            saveMap(logfileName() + ".fmap", FRICTION_MAP)

        if P.getValue("friction_map_save") and P.getValue("friction_map_plot"):
            fig = ngplot.figureCreate()
            ngplot.axisEqual(figure = fig)

            if False:
                # Plot everything
                _sc = ngplot.pointsScatter(
                    pointsToWorld(
                        numpy.asarray(list(numpy.ndindex(FRICTION_MAP.shape)))
                    ),
                    s = 0.5,
                    c = FRICTION_MAP.flatten() / 100.0,
                    cmap = "gray_r",
                    vmin = 0.0,
                    figure = fig
                )

            # Points to plot
            _ptp = numpy.asarray(numpy.where(getMap() == 100)).T

            _sc = ngplot.pointsScatter(
                pointsToWorld(_ptp),
                s = 0.5,
                c = FRICTION_MAP[_ptp[:, 0], _ptp[:, 1]] / 100.0,
                cmap = "gray_r",
                vmin = 0.0,
                vmax = 1.0,
                figure = fig
            )

            ngplot._pyplot(
                _sc,
                function = "colorbar",
                figure = fig
            )

            ngplot.figureSave(
                filename = logfileName() + ".fmap.png",
                figure = fig
            )

            ngplot.figureClose(figure = fig)

        log (
            "Loaded friction map from '%s'."
            % P.getValue("friction_map")
        )

        uqs, cnt = numpy.unique(fmap[:, 2], return_counts = True)

        log (
            "\n".join([
                "\t%.2f: %3.2f%%" % (_u, _r)
                for _u, _r in zip(
                    uqs / 100.0,
                    (cnt / (float(sum(cnt)))) * 100.0
                )
            ])
        )


def compute(
        points: numpy.ndarray,
        overlap: int = None,
        penalty: float = 100.0,
        **overflown) -> float:
    """Compute the speed profile using overlap.

    Arguments:
    points -- points of a trajectory with curvature, nx3 numpy.ndarray
    overlap -- size of trajectory overlap, int, default None/0 (disabled)
    penalty -- penalty value applied to the incorrect solutions,
               float, default 100.0
    **overflown -- arguments not caught by previous parts

    Returns:
    t -- time of reaching the last point of the trajectory, [s], float
         minimization criterion
    """
    global REFERENCE, CENTERLINE, REFERENCE_PROGRESS, OVERTAKING_POINTS, MAP_OUTSIDE, MAP_INSIDE

    # Get overlap parameter
    if overlap is None:
        overlap = P.getValue("overlap")

    profiler.CENTERLINE = CENTERLINE
  
    # ---------------[Create trajectory from path]---------------
    # points (x, y) --> trajectory (_v, _a, _t), _v -> speed, _a -> acceleration, _t -> time
    _v, _a, _t = profiler.profileCompute(
        points,
        overlap,
        lap_time = True,
        save = (
            P.getValue("save_solution_csv")
            if (
                not overflown.get("optimization", True)
                and P.getValue("save_solution_csv") is not None
            ) else None
        )
    )
    criterion = _t[-1]  # default criterion of the optimization is a lap time

    # test if acceleration is within limits
    for i in range(len(_a)):
        if _a[i] > P.getValue("a_acc_max") or _a[i] < -P.getValue("a_break_max"):
            return float(penalty * abs(_a[i]))

    invalid_points = []
    if REFERENCE is not None:
        is_collision = numpy.zeros((len(REFERENCE), ), dtype=bool)
        closest_indices = (-1) * numpy.ones((len(REFERENCE), ), dtype=int)
        ego_headings = numpy.zeros((len(points), ), dtype=numpy.double)
        opponent_headings = numpy.zeros((len(REFERENCE), ), dtype=numpy.double)

    collision_model = 1  # 0 - round cars, 1 - square cars

    if REFERENCE is not None:  # Do this only if we want to compute overtaking
        # ---------------[ Get indexes closest in time ]---------------
        _d = P.getValue("reference_dist")
        for _i, (rx, ry, rt, v) in enumerate(REFERENCE):

            # Use this commented part only if optimizing closed path
            _ci = 0
            __t = 0
            # Selected last
            selected_last = False

            while True:
               # Find closest point in time domain
               _ci = (abs(_t[:-1] + __t - rt)).argmin()
               # In case that we select the last point
               # Do it again for next repetition of the trajectory
               if _ci == len(_t) - 2:
                   if selected_last:
                       # Extra condition for non-closed paths.
                       break
                   __t += _t[-1]
                   selected_last = True
               else:
                   break

            # This should end this loop when we run out of points on the trajectory (for open trajectories only)
            if selected_last:
                break

            closest_indices[_i] = _ci

            # Calculate heading of the ego vehicle
            ego_pos = points[_ci, :]
            if len(points) <= _ci + 2:  # n -> n x 3
                ego_vector = [ego_pos[0] - points[_ci - 2 , 0], ego_pos[1] - points[_ci - 2 , 1]]
            else:
                ego_vector = [points[_ci + 2, 0] - points[_ci, 0], points[_ci + 2, 1] - points[_ci, 1]]
            ego_headings[_ci] = numpy.arctan2(ego_vector[1], ego_vector[0])

            # Calculate heading of the opponent vehicle
            # TODO calculate what you can in INIT
            opponent_pos = [rx, ry]
            if len(REFERENCE) <= _i + 2:
                opponent_vector = [opponent_pos[0] - REFERENCE[_i - 2, 0], opponent_pos[1] - REFERENCE[_i - 2, 1]]
            else:
                opponent_vector = [REFERENCE[_i + 2, 0] - rx, REFERENCE[_i + 2, 1] - ry]
            opponent_headings[_i] = numpy.arctan2(opponent_vector[1], opponent_vector[0])

            # Collision detection
            if P.getValue("car_shape") == "circle": # round cars
                if pointDistance([rx, ry], points[_ci, :]) < _d:  # TODO CHECK
                    is_collision[_i] = True
                    if not overflown.get("optimization", True):  # just for the plot
                        invalid_points.append([rx, ry])

            elif P.getValue("car_shape") == "rectangle":  # square cars

                dist_check_poly = numpy.sqrt(numpy.power(P.getValue('car_length'), 2.0) + numpy.power(P.getValue('car_width'), 2.0))  # Constant
                car_distance = pointDistance([rx, ry], points[_ci, :])
                if car_distance < P.getValue('car_width'):
                    # Cars are always colliding
                    if not overflown.get("optimization", True):
                        invalid_points.append([rx, ry])  # only to print invalid points. Equivalent to this should be: invalid_points ===  REFERENCE[is_collision, :2]. So we probably do not even need to create this
                    is_collision[_i] = True
                elif car_distance < dist_check_poly:
                    # Check polygon intersection only if cars are too close
                    # Calculate all vertices of the ego vehicle (rectangle representation)
                    corners_ego = get_rect_points(points[_ci, :2],
                                              (
                                                    P.getValue('car_length'),
                                                    P.getValue('car_width')
                                               ),
                                              ego_headings[_ci])

                    # Calculate all vertices of the opponent vehicle (rectangle representation)
                    corners_opponent = get_rect_points(REFERENCE[_i , :2],
                                                (
                                                        P.getValue('car_length'),
                                                        P.getValue('car_width')
                                                ),
                                                opponent_headings[_i])
                    if do_polygons_intersect(corners_ego, corners_opponent):
                        if not overflown.get("optimization", True):
                            invalid_points.append([rx, ry])  # only to print invalid points. Equivalent to this should be: invalid_points ===  REFERENCE[is_collision, :2]. So we probably do not even need to create this
                        is_collision[_i] = True

        # ---------------[ Visualization ]---------------
        if not overflown.get("optimization", True) and P.getValue("plot"):

            # Last time sample
            ts = int(_t[-1]) - 1

            if P.getValue("plot_timelines"):
                for ts in (
                    range(int(_t[-1])) if overlap > 0
                    else chain(range(int(_t[-1]) + 1), _t[-1])
                ):
                    _closest = numpy.abs(
                        numpy.subtract(REFERENCE[:, 2], ts)
                    ).argmin()

                    if _closest >= len(REFERENCE) - 1:
                        ts = ts - 1
                        break

                    ngplot.pointsScatter(
                        # Trick to force 2D array.
                        REFERENCE[_closest, None, :2],
                        s = P.getValue("plot_timelines_size"),
                        color = "black"
                    )

                    if ts % 4 == 0:
                        ngplot.labelText(
                            REFERENCE[_closest, :2],
                            ts,
                            verticalalignment = "top",
                            horizontalalignment = "left",
                            fontsize = 6
                        )

                    # PR @jara001: This used to be closest to ego, but is changed to reference.
                    # _closest_p = numpy.abs(
                    #     numpy.subtract(_t[:-1], ts)
                    # ).argmin()
                    _closest_p = closest_indices[_closest]

                    ngplot.pointsScatter(
                        points[_closest_p, None, :2],
                        s = P.getValue("plot_timelines_size"),
                        color = "red"
                    )

                    if ts % 4 == 0:
                        ngplot.labelText(
                            points[_closest_p, :2],
                            ts,
                            verticalalignment = "bottom",
                            horizontalalignment = "right",
                            fontsize = 6,
                            color = "red",
                        )

                    if P.getValue("car_shape") == "circle":  # round cars
                        # plot EGO car as a round obstacle
                        ngplot.circlePlot(points[_closest_p , :2], P.getValue("reference_dist") / 2.0, color="blue")
                        # plot opponent car as a round obstacle
                        ngplot.circlePlot(REFERENCE[_closest , :2], P.getValue("reference_dist") / 2.0, color="red")
                    elif P.getValue("car_shape") == "rectangle":  # rectangle cars
                        # plot EGO car as a round obstacle
                        ngplot.rectanglePlot(points[_closest_p , :2], 
                                         P.getValue('car_length'), 
                                         P.getValue('car_width'),
                                         angle=ego_headings[_closest_p],
                                         color="blue")
                        # plot opponent car as a round obstacle
                        ngplot.rectanglePlot(REFERENCE[_closest , :2],  
                                            P.getValue('car_length'), 
                                            P.getValue('car_width'),
                                            angle=opponent_headings[_closest],
                                            color="red")

                    ngplot.pointsPlot(
                        numpy.vstack(
                            (points[_closest_p, :2], REFERENCE[_closest, :2])
                        ),
                        #color = "red",
                        color = "green",
                        linewidth = P.getValue("plot_timelines_width"),
                        linestyle = (
                            "--" if pointDistance(
                                points[_closest_p, :2], REFERENCE[_closest, :2]
                            ) < 5.0 else " "
                        )
                    )

            if P.getValue("plot_reference"):
                # Plot only to the last time point
                # That is specified by ts, which can be altered by previous if.
                _closest = numpy.abs(
                    numpy.subtract(REFERENCE[:, 2], ts)
                ).argmin()
                _closest_p = numpy.abs(numpy.subtract(_t[:-1], ts)).argmin()

                ngplot.pointsPlot(
                    # PR @jara001: This used to plot only REFERENCE[:_closest, :2].
                    # TODO: Add parameter to control this.
                    REFERENCE[:, :2],
                    color = "black",
                    linewidth = P.getValue("plot_reference_width")
                )

                if P.getValue("plot_solution"):
                    ngplot.pointsPlot(
                        # PR @jara001: This used to plot only points[:_closest_p, :2].
                        # TODO: Add parameter to control this.
                        points[:, :2],
                        color="orange",
                        linewidth = P.getValue("plot_reference_width")
                    )

            # print invalid points (colision points)
            if len(invalid_points) > 0:
                ngplot.pointsScatter(
                    numpy.asarray(invalid_points),
                    color = "blue",
                    marker = "x",
                    s = 1
                )



    # Locate points where overtaking occurs
    # Centerline is used to obtain track progression.
    # Do not do this when not optimizing; just to avoid
    # having duplicate marker(s).
    if (P.getValue("plot_overtaking")
            and REFERENCE is not None
            and CENTERLINE is not None):
        # PR @jara001: This is currently run even when not optimizing.
        #    and overflown.get("optimization", True)):
        # It does not actually plot, just sends the data via Queue
        # to the parent process.
        # That said, plotting has to be handled by the optimizer.
        if REFERENCE_PROGRESS is None:
            REFERENCE_PROGRESS = [
                trajectoryClosestIndex(CENTERLINE, REFERENCE[_i, :2])
                for _i in range(len(REFERENCE))
            ]

        crashed = False
        crash_time = None
        crash_side = 0  # 0 -> none, 1 -> left, 2 -> right 
        data_to_save = []
        overtaken = False

        # PR @jara001: There are two important changes here.
        #  1) It does not work with indices but meters now.
        #  2) REFERENCE has 4 columns instead of 3.
        average_opponent_dist = 0.0
        num_runs = 0

        # CEMTRELINE_PROGRESS_METERS = numpy.sqrt(numpy.sum(numpy.square(CENTERLINE[:-1, :] - CENTERLINE[1:, :]), axis=1))
        REFERENCE_PROGRESS_METERS = numpy.sqrt(numpy.sum(numpy.square(REFERENCE[:-1, :] - REFERENCE[1:, :]), axis=1))
        REFERENCE_LENGTH_METERS = numpy.sum(REFERENCE_PROGRESS_METERS)
        OP_PROGRESS_CUMSUM = numpy.cumsum(REFERENCE_PROGRESS_METERS)
        OP_PROGRESS_CUMSUM = numpy.hstack((0.0, OP_PROGRESS_CUMSUM))

        EGO_PROGRESS_METERS = numpy.sqrt(numpy.sum(numpy.square(points[:-1, :2] - points[1:, :2]), axis=1))
        EGO_LENGTH_METERS = numpy.sum(EGO_PROGRESS_METERS[:])

        num_conlisions = numpy.sum(is_collision)
        kd = spatial.KDTree(REFERENCE[:, :2])
        _, min_dist_idx = kd.query(points[closest_indices, :2])

        pd_meters_prev = OP_PROGRESS_CUMSUM[min_dist_idx[0]]
        if (pd_meters_prev > 0.0):
            pd_meters_prev -= REFERENCE_LENGTH_METERS

        overtaking_point = None
        for _i, (rx, ry, time_progress, rv) in enumerate(REFERENCE):
            if closest_indices[_i] == -1:
                break

            # Calculate trajectory progress of the opponent in meters
            rd_meters = OP_PROGRESS_CUMSUM[_i]
            # Calculate trajectory progress of the ego in meters (progress on opponent's trajectory)
            pd_meters = OP_PROGRESS_CUMSUM[min_dist_idx[_i]]
            if abs(pd_meters_prev - pd_meters) > 1.0:
                pd_meters -= REFERENCE_LENGTH_METERS

            # check for overtake without collision
            if (pd_meters >= rd_meters) and (not overtaken):
                if EGO_LENGTH_METERS - numpy.sum(EGO_PROGRESS_METERS[:closest_indices[_i]]) > 10.0:  # TODO change this to PROGRESS ON OPPONENT ?? maybe...
                    overtaken = True
                    overtaking_point = points[closest_indices[_i], :2]
            elif pd_meters < rd_meters and overtaken: 
                if not crashed:
                    overtaken = False

            # If not crashed before and crashed right now
            if (not crashed) and is_collision[_i]:
                # Find out who caused the collision
                if (pd_meters - rd_meters < P.getValue("ego_dist_overtake")) or not overtaken:
                    # EGO crashed from behind --> EGO's fault
                    return float(penalty * num_conlisions + 2045)
                elif pd_meters - rd_meters >= P.getValue("ego_dist_overtake"):
                    # Opponent crashed from behind --> Opponent's fault
                    crashed = True

                    # 1.0 - left, 0.0 - mid, -1.0 right
                    crash_side = determine_opponent_side(traj_pos_now=points[closest_indices[_i], :2],
                                                         traj_pos_prev=points[closest_indices[_i] - 1, :2],
                                                         opponent_pos=REFERENCE[_i, :2])

                    # TODO start using inflated part of the map. remember the start time and which side to use
                    crash_time = time_progress

                    if not overflown.get("optimization", True) and P.getValue("plot"):
                        ngplot.pointsPlot(numpy.vstack((REFERENCE[min_dist_idx[_i], :2], points[closest_indices[_i], :2])), color = "blue", linewidth = P.getValue("plot_timelines_width"))
                        ngplot.pointsPlot(numpy.vstack((REFERENCE[_i, :2], points[closest_indices[_i], :2])), color = "green", linewidth = P.getValue("plot_timelines_width"))

                        # plot inflated map
                        if crash_side == 1 or crash_side == 0:  # left MAP_OUTSIDE
                            ngplot.imgPlotMetric(MAP_OUTSIDE, color="red", s=1, marker='.', alpha=0.1)
                        elif crash_side == -1:  # right MAP_INSIDE
                            ngplot.imgPlotMetric(MAP_INSIDE, color="red", s=1, marker='.', alpha=0.1)
                        print(f"Crashed {crash_side}")

                        ngplot.pointsPlot(numpy.vstack((points[closest_indices[_i-1], :2], points[closest_indices[_i], :2])), color = "green", linewidth = P.getValue("plot_timelines_width"))
                        # opponent position during overtake
                        ngplot.pointsPlot(REFERENCE[_i, :2].reshape((1, 2)), marker='.', color="green", linewidth=P.getValue("plot_timelines_width"))  

            # Check current trajectory point is inside the protected area
            if crashed and (crash_time + P.getValue("use_safe_zone_seconds") > time_progress):
                if not pointInBounds(points[closest_indices[_i]]):  # TODO check this
                    return float(penalty)
                point_map = pointToMap(points[closest_indices[_i]])
                if crash_side == -1 and not MAP_INSIDE[point_map[0], point_map[1]] != 0:
                    return float(penalty)
                if crash_side == 1 and not MAP_OUTSIDE[point_map[0], point_map[1]] != 0:
                    return float(penalty)

            pd_meters_prev = pd_meters

            # Additional criterion to push ego car in front of the opponent
            average_opponent_dist += (rd_meters - pd_meters)
            num_runs += 1.0

            #      time      ; time ego  ; RX ; RY ; RV ;           EGO X                ;               EGO Y            ;            EGO V        ; kappa ego ; crashed ; overtaken
            # time_progress  ; time ego  ; rx ; ry ; rv ; points[closest_indices[_i], 0] ; points[closest_indices[_i], 1] ; _v[closest_indices[_i]] ; kappa ego ; crashed ; overtaken
            if not overflown.get("optimization", True):
                # yaw_rate, steer_front
                ego_x = points[closest_indices[_i], 0]
                ego_y = points[closest_indices[_i], 1]
                kappa_ego = points[closest_indices[_i], 2]
                _lr = 0.139
                _lf = 0.191
                _beta = math.atan(_lr * kappa_ego)
                vx_ego = _v[closest_indices[_i]] * math.cos(_beta)
                vy_ego = _v[closest_indices[_i]] * math.sin(_beta)
                ego_yaw = ego_headings[closest_indices[_i]]
                ego_omega_radps = _v[closest_indices[_i]] * math.cos(_beta) * kappa_ego
                ego_delta_rad = (_lf + _lr) * kappa_ego
                data_to_save.append([time_progress, _t[closest_indices[_i]], rx, ry, rv,
                                     ego_x, ego_y, vx_ego, kappa_ego, ego_yaw, vy_ego, _a[closest_indices[_i]], ego_omega_radps, ego_delta_rad,
                                     crashed, overtaken])

        average_opponent_dist = average_opponent_dist / num_runs

        # Ego's last position   points[-1]
        # Ego's last orientation   ego_headings[-1]

        # Opponent    opponent_headings       REFERENCE

        success = False
        criterion = _t[-1]

        use_end_cost = True
        if use_end_cost:
            idx = numpy.argmin(numpy.sum(numpy.square(REFERENCE[:, :2] - points[-2, :2].reshape((1, 2))), axis=1))
            dist_error = numpy.sum(numpy.square(REFERENCE[idx, :2].reshape((1, 2)) - points[-1, :2].reshape((1, 2))), axis=1)
            heading_error = numpy.abs(ego_headings[-2] - opponent_headings[idx])
            speed_error = numpy.abs(REFERENCE[idx, 3] - _v[-2])
            criterion += 0.4 * dist_error + 5.0 * heading_error # + 5.0 * speed_error
            if not overflown.get("optimization", True):
                print(f"average_opponent_dist {average_opponent_dist}")
                print(f"dist_error: {dist_error}    heading_error: {heading_error}    speed_error: {speed_error}")

        if not crashed:
            # No crash
            if overtaken:
                # Overtake
                success = True
            else:
                # No overtake
                criterion += average_opponent_dist + P.getValue("favor_overtaking") * 2.0
        else:
            # Crash (not my mistake -> he should change his trajectory) -> successful overtake
            criterion += average_opponent_dist + P.getValue("favor_overtaking") # + 0.05 * num_conlisions  # -> bad only if I am on the outside
            success = True

        if success:
            if not overflown.get("optimization", True):
                with open("success.res", "w") as f:
                        f.write("1")

            if overflown.get("optimization", True):
                if overtaking_point is not None:
                    OVERTAKING_POINTS.put(overtaking_point)
                else:
                    print("-----------------------------------------ERROR---------------------------------------")

        # Time instead of distance at the start position

        # data collection
        if not overflown.get("optimization", True):
            # FIXME: This probably fails at some occasions.
            file_name = logfileName().replace("matryoshka.log", "save.npy")
            numpy.save(file_name, data_to_save)

    return float(criterion)
