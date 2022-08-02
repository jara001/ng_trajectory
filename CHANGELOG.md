# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
## 1.7.0 - 2022-08-02
### Added
- Interpolators
    - Arguments `from_left` for `trajectoryClosest*()` to get closest point from the path beginning.
- Penalizers
    - Variable `INVALID_POINTS` to store invalid points so that they are not computed multiple times.
    - [**NEW**] _Curvature_
        - Penalizer 'Curvature' operates on the curvature of feasible paths.
- Selectors
    - _Uniform_distance_
        - Parameter `rotate` for rotating the selection.
        - Parameter `fixed_points` to fix some points during selection, and to split the selection into multiple parts.
- Script 'ng_plot' for generating maps and GIFs from log files.
- Parameter '-p' for 'ng_graph' to show log statistics in plot-friendly csv format.

### Changed
- Selectors
    - _Curvature2_
        - Parameters of the selector are not reset to default state every `select()`.

### Fixed
- Optimizers
    - _Braghin_
        - Invalid points plotting is hidden behind `plot` parameter.
- Properly compute quantiles in 'ng_graph'.

## 1.6.0 - 2022-05-24
### Added
- Interpolators
    - Function `trajectoryRotate()` for rotating the trajectory points along the line. Used by selectors.
    - Function `trajectoryClosestIndex()` to receive index of the closest point.
    - Functions `trajectoryFarthest()` and `trajectoryFarthestIndex()` to receive the most distant points.
- Penalizers
    - Parameter `optimization` (default to True) that when True indicates that the Penalizer is run during optimization sequence.
    - Penalizer `utils` with optimization methods.
    - _Centerline_
        - Methods `sum` and `avg` for computing the resulting penalty.
        - Parameters `huber_loss` and `huber_delta` for using a Huber loss function when computing the penalty.
    - [**NEW**] _Segment_
        - Penalizer 'Segment' operates on the distance of the invalid points from the valid area.
- Segmentators
    - Functions `hood4Obtain()` and `hood8Obtain()` for obtaining the neighbour cells in the grid map.
    - Function `borderCheck()` to check whether cell point is on valid area border.
    - Function `pointToWorld()` as a approximate inverse function of `pointToMap()`.
    - Function `validCheck()` to find whether cell point is valid.
- Selectors
    - [**NEW**] _Uniform_time_
        - Selector 'Uniform_time' equidistantly samples the path with respect to the time estimated by 'profile' criterion.
- New dependency `tqdm`.
- Script 'ng_graph' for generating graphs from configuration files and log files.

### Changed
- Selectors
    - _Uniform_
        - Use `trajectoryRotate()` instead of the current implementation.
- Penalizer `init()` now receives the full combined keywords dictionary. Therefore, e.g., `method` for `centerline` penalizer can be varied.
- Target `build-wagon` is not run when using `build` target.

### Fixed
- Script 'ng_generate_data' now properly handles images with alpha channel and images without unknown area.
- Generated documentation is automatically commited on new release.

## 1.5.3 - 2022-04-05
### Added
- Target `build-wagon` for creating wagons for 'x86_64' and 'aarch64'.

### Changed
- Target `uninstall` is generic.
- Renamed original `build` to `build-wheel`.
- Target `build` now runs `build-wheel` and `build-wagon`.

### Fixed
- Sorting bug while obtaining borders of the segments.

## 1.5.2 - 2022-03-14
### Fixed
- Citation file should now contain correct fields and values to be properly parsed by GitHub.

## 1.5.1 - 2022-03-04
### Added
- Argument `-y` for 'ng_generate_data' to obtain map information from a YAML file (used in ROS maps).
- Citation (cff) file.
- Reference to the original paper.
- License file.

## 1.5.0 - 2022-01-03
### Added
- Interpolators
    - Function `pointsDistance()` for computing distances between points in a set.
    - Parameter `verify_sort` for `trajectorySort()` that removes outliers from the trajectory.
- Optimizers
    - _Matryoshka_
        - Use `verify_sort` when obtaining borders.
        - ~~Support for penalizing the optimization results using borderlines. The algorithm returns `PENALTY * d`, where `d` is the distance to the nearest borderline point that the candidate point belongs to.~~
        - ~~Parameter `use_borderlines` that activates borderline-based penalization.~~
        - Parameter `plot_mapping` that shows the transformation by mapping a grid onto the track. (Use only for demonstration.)
- [**NEW**] Penalizers
    - New group of algorithms which are used for evaluating whether the candidate is incorrect. In that case penalty is computed.
    - In constrast to other algorithms, `init()` of penalizers should be executed during `init()` of optimizers.
    - _Count_
        - Penalizer 'Count' is the basic penalizer used in ng_trajectory. It counts number of points outside the valid area.
    - _Borderlines_
        - Penalizer 'Borderlines' calculates minimum of maximum of the distances between misplaced points and their associated borderline (sets of points on the borders between two adjacent segments).
        - ~~**Warning** Currently, this works only for Matryoshka.~~
        - **Warning** Currently, this works only for Flood Fill segmentator.
    - _Centerline_
        - Penalizer 'Centerline' finds the minimum of maximum of the distances between misplaced points and their associated part of the centerline.
        - Parameter `method` that allows selecting whether the penalty is minimum or maximum of all found distances.
- Segmentators
    - _Flood Fill_
        - Function `segmentDistance()` that computes distance from a point to a segment.
        - Parameter `reserve_width` that creates a reserved region for each segment in order to touch both (outer and inner) walls.
        - Parameter `reserve_selected` with list of segments that should use the reservation method. When unset, all are used.
        - Parameter `reserve_distance` that sets the distance from the wall-segment connector that is reserved for the segment.
        - ~~Parameter `create_borderlines` that creates a nested dict of borderlines between segments.~~
        - Parameter `plot_flood` to plot the flooded areas into the figure. (Only for demonstration.)
- Selectors
    - _Curvature_
        - Parameter `downsample_factor` that controls downsampling prior to the interpolation.
        - Option `show_plot` that hides the plot even though it is saved to file.
        - Parameter `split_peaks` that was used for some time that adds +-1 points around the peaks.
    - [**NEW**] _Fixed_
        - Selector 'Fixed' returns preset list of points.
    - [**NEW**] _Curvature2_
        - Selector 'Curvature2' uses metric units for peaks identification and operates more automatically than 'Curvature'.
    - [**NEW**] _Uniform_distance_
        - Selector 'Uniform_distance' equidistantly samples the path.
- Intermediate results are stored inside logs, allowing to pause the experiments. (Only at the loop end.)
- Checking for already present logs and resuming the experiment at the loop it has ended.
- Variable `__version__` to be used for scripts.
- Current version is reported on every 'ng_run' start and at the beginning of every log (when logging enabled).
- Script 'ng_curvature_gui' for testing the Curvature selector parameters.
- Function `figureClose()` for closing the figures. It should be called after saving the figure to free used memory.
- Parameter `--gendoc` for 'ng_help' to generate the README in the repository.
- Script 'release.sh' as a guideline for successful creating of a new release.

### Changed
- Segmentators
    - _Flood Fill_
        - Remove empty segments caused mostly by a selected group center outside of the valid area. This will probably change in the future.
- Track is plotted using lighter gray that is not in the standard palette. Therefore the colors should not overlap anymore.
- Algorithms are passed to the optimizer as modules not callables. Therefore it is possible to call other functions of the module as well.
- Figures created during the optimization are closed after saving.

### Fixed
- Selectors
    - _Curvature_
        - Track with identified turns is plotted with equal axes.

## 1.4.3 - 2021-11-12
### Changed
- Un-nested plot functions return created objects.

### Fixed
- Selectors: Resolve missing peaks for Curvature selector.
- Optimizers: Plot in Matryoshka only when it is allowed.
- Add PHONY target to Makefile to get around 'target is up-to-date'.
- Figure/pyplot wrappers are protected by `plot_only` decorator.

## 1.4.2 - 2021-11-09
### Added
- Makefile for building/installing/removing the package.
- Functions `reset()` and `resetAll()` to restore default states of parameters.
- Parameter 'reset' for `updateAll()` to automatically reset all parameters.
- Documentation exported from 'ng_help' to README.

### Fixed
- Segmentators: Flood_fill is no longer using old parameters when restarting the cascade.

## 1.4.1 - 2021-11-08
### Changed
- Selectors: Curvature selector no longer imports `pyplot` directly.
- 'ng_generate_data': Numpy is using Python builtin data types.

### Fixed
- Running the scripts without `matplotlib` no longer raises an exception.

## 1.4.0 - 2021-09-14
### Changed
- Optimizers: Matryoshka does not sort centerline before the selection (for segmentation).

## 1.3.0 - 2021-06-08
### Added
- Script 'ng_help' for showing help for the algorithms and their parameters.
- Utilizy 'parameter' for documented usage of parameters within the algorithms.

### Changed
- Use newly introduced ParameterList inside the algorithms, at least for the documentation.

## 1.2.2 - 2021-06-07
### Added
- Selectors: Curvature_sample selector (based on the Botta, 2012 article) by Ondra Benedikt.

### Changed
- Optimizers: Default number of workers is set to the number of logical processors.

## 1.2.1 - 2021-05-27
### Fixed
- 'ng_generate_data': Resampling method used when resizing the image is now explicitly set to 'PIL.Image.BILINEAR' as it was changed in an older version of Pillow package.
- 'ng_generate_data': Add `Pillow>=4.2.0` dependency to the setup file.

## 1.2.0 - 2021-05-27
### Added
- Optimizers: Final graph now contains invalid points as well (if present).
- Selectors: When using negative selection, leave the number of points on the selector.
- Selectors: Curvature selector by Ondra Benedikt.
- Script 'ng_generate_data' for generating numpy files for the algorithm from an image of the map.

### Changed
- Optimizers: Individual plotting is now forced on the main figure, instead of the global one.

## 1.1.1 - 2021-05-20
### Added
- Segmentators/Utils: `gridCompute()` to obtain square grid size of the points.
- Substitute not selected algorithms with a stub.
- Parameter `silent_stub` that suppresses errors when calling stub.

### Changed
- Selectors: Uniform selector now treats rotate as factor between two consecutive points, instead of rotating the whole path.

### Fixed
- Optimizers: Invalidity of points is compared to the specified or computed square grid size instead of a fixed value.
- Optimizers: Constant for penalty is no longer fixed and can be set via parameter `penalty`.

## 1.1.0 - 2021-05-18
### Added
- Automatic versioning of the package.
- Optimizers: Braghin supports parameter `path_reduction` for reducing the path prior to the interpolation.
- *Selectors* as a class of methods used for retrieving group centers from the centerline.
- Selectors: Uniform selector that supports selection rotation.
- Script 'ng_run' for invoking the algorithm directly from Terminal.

### Changed
- Optimizers: Braghin's parameters `endpoint_distance` and `endpoint_accuracy` are modifiable.
- Optimizers: Braghin during its transformation works with the full length of the centerline instead of a subset.

### Fixed
- Exception message is shown when parsing JSON fails.
- Variate run is using string formatting to properly display non-int variables.

## 1.0.0 - 2021-03-16
### Added
- Reimplementation of `ng_trajectory` as a standalone Python package.
