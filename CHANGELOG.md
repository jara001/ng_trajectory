# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
## 1.11.0 - 2023-11-21
### Added
- Criterions
    - _Profile_
        - Parameter 'friction_map' to load a friction map from '.npy' file.
        - Use friction for computing maximum velocity from the friction map if available.
        - Parameter 'friction_map_inverse' to invert the values in the friction map.
        - Support for zero friction (impassable area).
        - CSV file contains progress along the centerline [m].
- Segmentators
    - Function `getMap` to obtain current internal map used by the segmentators.
- 'ng_generate_data'
    - Parameter `--friction-map` to create a friction map from an image.
    - Parameter `--fonly/--friction-map-only` to skip generating map data and generate only friction map data.
    - Parameter `--friction-map-inverse` to inverse the colors in the friction map image.

### Changed
- Criterions
    - _Profile_
        - Time difference between points is constrained to 10s.
        - Generated CSV file holds lap time and path length with the first trajectory point.

### Fixed
- Criterions
    - _Profile_
        - Avoid crashing when logfile is not given.
- Segmentators
    - Catch and handle OverflowError when identifying invalid points.

## 1.10.0 - 2023-11-15
### Changed
- Update all files to mitigate majority of the flake8 errors.

### Fixed
- Plot documentation is now properly formatted.

## 1.9.5 - 2023-11-14
### Added
- Criterions
    - _Profile_
        - Parameter `favor_overtaking` that is added as a penalty to the lap time when overtaking does not occur.
        - Parameter `reference_laptime` to set the lap time of the reference instead of estimating it.
- Optimizers
    - _Matryoshka_
        - Experimental parameter '_experimental_mm_max' that limits mapping to only first 'n' segments.
- Segmentators
    - Function `pointsToWorld` to convert coordinates of multiple cells at once.
- Expose all parameters in plotter for `figureSave`, `trackPlot`, `bordersPlot` and `indicesPlot`.

### Changed
- Criterions
    - _Profile_
        - Non-closed paths display more points when `plot_timelines` is set to True.

### Fixed
- Criterions
    - _Profile_
        - Properly detect overtaking points when crossing "0" progress.
        - Return correct lap time on non-closed path.
        - Reference is properly rotated, so the virtual car does not slow down at wrong places.
        - Properly compute delta in the saved trajectory.
        - Properly compute heading in the saved trajectory.
- Bound version of `bayesian-optimization<=1.4.0` to resolve import errors.

## 1.9.4 - 2023-06-29
### Added
- Criterions
    - _Profile_
        - Parameter `plot_overtaking` to plot the places where an overtaking occurred during the optimization.
- Optimizers
    - _Matryoshka_
        - Skip segments fixed by points that are outside of the valid area.
        - Save all overtaking points into a file at the end of the cascade step.
- GitHub URL to the package manifest.

### Changed
- Criterions
    - _Profile_
        - Solutions are saved as CSV by default. Pass empty string / `None` to `save_solution_csv` to disable it.

### Fixed
- Criterions
    - _Profile_
        - Properly use Queue to display the overtaking points.

## 1.9.3 - 2023-06-06
### Added
- Criterions
    - _Profile_
        - Parameter `save_solution_csv` treates `$` as name of the current log.
        - Check for overlap consistency, as short overlaps might not smoothen the velocity enough.
        - Display points that invalid because of the distance to the reference.
        - Show points where overtaking occurs (across whole budget).

### Changed
- 'ng_plot'
    - Parameter `-O` is now optional. When not given, plots are displayed directly in a sequence.

## 1.9.2 - 2023-05-25
### Added
- 'ng_generate_data'
    - Parameter `--inflate` that inflates the walls before processing the map.

### Changed
- 'ng_generate_data'
    - Function for filtering points is no longer imported from the package.
    - Generated image is not in grayscale; centerline points are shown green, start points are red.

### Fixed
- Criterions
    - _Profile_
        - Parameter `overlap` is properly handled via ParameterList.
- 'ng_generate_data'
    - Centerline is properly filtered when more then one is present.
    - Duplicit points are removed before filtering to avoid unfilterable path sections.

## 1.9.1 - 2023-05-24
### Fixed
- Criterions
    - _Profile_
        - Parameter `reference_rotate` no longer flips axes.
- 'ng_generate_data'
    - Points of the centerline are not duplicated anymore.
- Require numpy versions `<1.24` as newer ones are not compatible with `nevergrad==0.3.0`.

## 1.9.0 - 2023-05-18
### Added
- Criterions
    - _Profile_
        - Argument `lap_time` for `profileCompute()` to obtain lap time of the trajectory.
        - Parameter `reference` to set file for loading reference trajectory that needs to be avoided in time.
        - Parameter `reference_dist` to set the minimum allowed distance from the reference trajectory.
        - Parameter `reference_rotate` to set the rotation of reference trajectory.
        - Parameters `plot_reference`, `plot_reference_width` to control plotting of reference trajectory.
        - Parameter `plot_solution` to control plotting of optimized solution.
        - Parameters `plot_timelines`, `plot_timelines_size`, `plot_timelines_width` to control plotting of lines between points of the same time.
        - Parameter `save_solution_csv` to save final solution to a CSV file.
        - Parameters `_lf` and `_lr` to set the car wheelbase for the saved trajectory.
    - Parameter `optimization` (default to True) that when True indicates that the criterion is run during optimization sequence (similarly to penalizer).
    - Parameter `penalty` is passed to criterion during optimization.
    - When enabled, final trajectory is saved to a csv file.
- Interpolators
    - _Cubic_spline_
        - Parameter `closed_loop` to set interpolator to create closed/unclosed lines.
- Optimizers
    - _Matryoshka_
        - Function `indicesToRealCoordinatesInt()` to convert indices using slice.
        - Parameter `load_matryoshka` to set file for loading Matryoshka instead of creating it.
        - Parameter `save_matryoshka` to set file for saving Matryoshka after it is created.
        - Parameter `plot_group_indices` (default to True) to turn off group indices in the plot.
        - Parameter `plot_group_borders` (default to True) to turn off group borders in the plot.
        - Parameter `fixed_segments` to determine points (and their segments) that are not optimized.
- Penalizers
    - Function `eInvalidPoints()` to iterate over invalid points obtained by segmentated map.
- Segmentators
    - _Flood_fill_
        - Parameter `parallel_flood` to run the segmentation with a ProcessPool.
    - Function `pointInBounds()` along with global `MAP_BOUNDS` to check whether point lies inside the map bounds.
- 'ng_generate_data'
    - Repeatable parameter `-v` to increase verbosity.
    - Centerline is generated between all walls, but only the section without dead-ends is returned. If this fails, centerline for the largest wall is returned instead.
    - Inflated map is saved when verbosity is set to at least 2.
- 'ng_run'
    - Parameter `--help` to show help for the command.
    - Support for `+[PARAMETER]` parameters that modifies the loaded configuration.
    - Support for `+[PARAMETER]/[PARAMETER]` to modify subdictionaries.
- Documentation for package plotter.
- Function `configurationMerge()` to merge another configuration to self.
- Variate run can properly handle floats when generating a name of the log file.

### Changed
- Criterions
    - _Profile_
        - Criterion is using the true `lap_time`.
        - Overlap is upper bound by the number of points.
- Optimizers
    - _Matryoshka_
        - Use list comprehension in `groupsCenterCompute()`.
        - Force new transformation when number of segments does not correspond to the current mapping.
        - Matryoshka is plotted even with held transformation.
- Penalizers
    - _Count_
        - Use segmentated map for getting the invalid points instead of numpy vectors.
    - Use `eInvalidPoints()` where applicable.
- 'ng_generate_data'
    - Arguments are parsed by argparse instead of getopt.
    - Map inflation is much faster because of helper array.
    - Centerline points are filtered by Matryoshka's `pointsFilter()`.
    - Quantization `PIL.quantize()` uses median cut method instead of maximum coverage.

### Fixed
- Criterions
    - _Profile_
        - Profiling uses curvature that is not modified for straight sections.
- Optimizers
    - _Matryoshka_
        - ParameterList is updated with keyword arguments.
    - Set `optimization` parameter of criterions/penalizers to True during optimizer initialization.
- 'ng_help' no longer requires an argument for `-h`.
- Matplotlib internal variables should not be longer garbage collected from different thread.

## 1.8.0 - 2023-03-15
### Added
- Criterions
    - [**NEW**] _Jazar_model_
        - Criterion 'Jazar_model' computes speed profile using model from Jazar's book.
- 'ng_graph'
    - Parameter `--show-legend` to display figure legend.
    - Parameter `--loc-legend` to set position of shown legend.
    - Parameter `--label` to set legend labels for each dataset.
    - Parameter `--title` to set the title of generated figure.
    - Parameter `-j %d` to enable parallel processing of log files.
    - Parameters `--width` and `--height` to set the size of the generated figure.
    - Parameter `--tight` to adjust figure padding.
    - Parameter `--variates` to select which 'x' values should be displayed in the figure.
    - Parameter `--datasets` to determine dataset indices for configuration files.
    - Parameter `--no-header` to skip header when using `-p`.
    - When using `--merged-min` with `-l` logs are merged together (and output new "merged-min" statistics).
    - Show segment value when using `--segments` together with `-lp`.
    - Parameter `--marker-avg` to set the marker style of dataset averages.
- Field `_label` in a configuration file is used when `--label` not given.

### Changed
- 'ng_graph'
    - Logs described by JSON files are looked for inside the folder of configuration file instead of the current folder.
    - Data loading with '--merged-min' skips some steps to make the process faster.
    - Input files are not opened right away, but only when actually used.
    - Progress is shown when processing logs with `-g`.
    - When using `--merged-min`, statistics of a merged log show:
        - Rate: Success rate of individual runs (% of finding a solution).
        - Length: Number of successful runs.

### Fixed
- 'ng_graph'
    - Parameter `-r` is now shown in the correct group.
    - Parameters are now set properly and should not "eat" more values than expected.
    - Parameter `--segments` is now properly handled.
    - Standard deviation is not computed when only one log is obtained.
    - Success rate of a log is not computed when empty.

## 1.7.2 - 2023-01-05
### Fixed
- Interpolators
    - Argument `from_left` is properly handled inside `trajectoryClosest()`.
- Selectors
    - _Uniform_time_
        - A proper exception is raised when passing negative `remain` value.

## 1.7.1 - 2022-08-09
### Added
- Parameter `-r` for 'ng_graph' to recursively dig through the subfolders when looking for the log files.
- Parameter `-g` for 'ng_graph' to show a graph for log files.
- Parameter `--segments` for 'ng_graph' to set number of segments for used log files.
- Multiple log files can be processed at once in 'ng_graph'.

### Changed
- Parameters for 'ng_graph' are now grouped.

### Fixed
- Log's filename is properly shown in 'ng_graph' with '-l'.

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
