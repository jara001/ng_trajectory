# Optimization toolbox ng_trajectory

**ng_trajectory** is a toolbox for solving the optimal racing line problem using various methods and approaches. The main idea stands on using a genetic algorithm, although other approaches may be used as well.

Currently we distinguish 6 groups of algorithms:
1. **Selectors**
	Selectors take the input path and select a subset of these points.
2. **Segmentators**
	Segmentators split the track into segments using the selected point subset.
3. **Interpolators**
	Interpolators are used for interpolating the selected point subset in order to get, e.g., curvature.
4. **Optimizers**
	Optimizers take the data from the previous three parts and use them to find the optimal racing line.
5. **Criterions**
	Criterions are used for obtaining the fitness value given the waypoints.
6. **Penalizers**
	Penalizers are used to decide whether the candidate is invalid, and in that case compute its penalty.

The whole application does run multiple times:
- variating "variate" parameter,
- repeating "loop" times,
- optimization "cascade".

The configuration file is using "parent-nesting" parameter handling. This means that the parameter defined on the top level is visible in lower levels (i.e., instead of specifying segmentator for each part of the cascade, it can be set on the top level).

## Minimal version of the configuration:
```json
{
	"_version": 2,
	"loops": 1,
	"groups": 20,
	"interpolator": "cubic_spline",
	"segmentator": "flood_fill",
	"selector": "uniform",
	"cascade": [
		{
			"algorithm": "matryoshka",
			"budget": 10,
			"layers": 5,
			"criterion": "profile",
			"criterion_args": {
				"overlap": 100
			}
		}
	],
	"start_points": "start_points.npy",
	"valid_points": "valid_points.npy",
	"logging_verbosity": 2
}
```

## Available algorithms:

### Optimizers

Optimizers are main parts of the whole process. They take waypoints and run optimization in order to find the best path possible.


_ng_trajectory.optimizers.*_

- _matryoshka_  Matryoshka transformation for track optimization (2D).
- _braghin_     Braghin's transformation method for track optimization (1D).


#### Matryoshka
_optimizers.matryoshka_

Matryoshka transformation for track optimization (2D).

This optimizers segmentates the track into 2D sequence of segments. Each waypoint of the racing line is situated into one of these segments. Interpolating them in order yields a path.

In order to efficiently move inside the segments, a homeomorphism transformation is created (Matryoshka).

For the optimization itself, Nevergrad is used.


```html
Init (matryoshka) parameters:
hold_matryoshka (bool) = False [Whether the transformation should be created only once.]
grid (list) = computed by default [X-size and y-size of the grid used for points discretization.]

Init (general) parameters:
budget (int) = 100 [Budget parameter for the genetic algorithm.]
groups (int) = 8 [Number of groups to segmentate the track into.]
workers (int) = os.cpu_count() [Number threads for the genetic algorithm.]
penalty (float) = 100 [Constant used for increasing the penalty criterion.]
criterion (module) = None [Module to evaluate current criterion.]
criterion_args (dict) = {} [Arguments for the criterion function.]
interpolator (module) = None [Module to interpolate points.]
interpolator_args (dict) = {} [Arguments for the interpolator function.]
segmentator (module) = None [Module to segmentate track.]
segmentator_args (dict) = {} [Arguments for the segmentator function.]
selector (module) = None [Module to select path points as segment centers.]
selector_args (dict) = {} [Arguments for the selector function.]
penalizer (module) = None [Module to evaluate penalty criterion.]
penalizer_init (dict) = {} [Arguments for the init part of the penalizer function.]
penalizer_args (dict) = {} [Arguments for the penalizer function.]
logging_verbosity (int) = 2 [Index for verbosity of the logger.]

Init (viz.) parameters:
plot (bool) = False [Whether a graphical representation should be created.]
plot_mapping (bool) = False [Whether a grid should be mapped onto the track (to show the mapping).]
```


#### Braghin
_optimizers.braghin_

Braghin's transformation method for track optimization (1D).

This optimizer is an implementation of an approach described in [1]. The track is characterized by "cuts", a 1D lines placed on the track, where on each cut there is a single path waypoint. Since we are aware of their order, we can just simply interpolate these points to receive a path.

The optimization itself is done using Nevergrad.

[1]: F. Braghin, F. Cheli, S. Melzi, and E. Sabbioni. 2008. Race driver model. Computers & Structures 86, 13 (July 2008), 1503–1516.


```html
Init (braghin) parameters:
hold_transform (bool) = False [Whether the transformation should be created only once.]
endpoint_distance (float) = 0.2 [Starting distance from the center for creating transformation.]
endpoint_accuracy (float) = 0.02 [Accuracy of the center-endpoint distance for transformation.]
line_reduction (float) = 3 [Factor by which the number of line points is lowered before internal interpolation.]
grid (list) = computed by default [X-size and y-size of the grid used for points discretization.]

Init (general) parameters:
budget (int) = 100 [Budget parameter for the genetic algorithm.]
groups (int) = 8 [Number of groups to segmentate the track into.]
workers (int) = os.cpu_count() [Number threads for the genetic algorithm.]
penalty (float) = 100 [Constant used for increasing the penalty criterion.]
criterion (module) = None [Module to evaluate current criterion.]
criterion_args (dict) = {} [Arguments for the criterion function.]
interpolator (module) = None [Module to interpolate points.]
interpolator_args (dict) = {} [Arguments for the interpolator function.]
segmentator (module) = None [Module to segmentate track.]
segmentator_args (dict) = {} [Arguments for the segmentator function.]
selector (module) = None [Module to select path points as segment centers.]
selector_args (dict) = {} [Arguments for the selector function.]
penalizer (module) = None [Module to evaluate penalty criterion.]
penalizer_init (dict) = {} [Arguments for the init part of the penalizer function.]
penalizer_args (dict) = {} [Arguments for the penalizer function.]
logging_verbosity (int) = 2 [Index for verbosity of the logger.]

Init (viz.) parameters:
plot (bool) = False [Whether a graphical representation should be created.]
plot_cuts (bool) = True [Whether cuts should be plotted if plot is enabled.]
plot_reduced_line (bool) = False [Whether reduced line should be plotted if plot is enabled.]
```


### Criterions

Criterions are used for calculating a fitness value during the optimization.


_ng_trajectory.criterions.*_

- _curvature_  Curvature criterion for fitness evaluation.
- _length_     Length criterion for fitness evaluation.
- _profile_    Profile criterion for fitness evaluation.


#### Curvature
_criterions.curvature_

Curvature criterion for fitness evaluation.

This criterion computes fitness value from curvature of the path. Since we expect that the input data already contain curvature, the fitness itself is computed as:

	sum( (k_i)^2 )


#### Length
_criterions.length_

Length criterion for fitness evaluation.

This criterion computes fitness value from length of the path. We calculate real segment-based length, i.e., sum of all sub- segment parts of the path.


#### Profile
_criterions.profile_

Profile criterion for fitness evaluation.

This criterion computes fitness value by simulating the vehicle over the input path. There are various parameters to be set. But mostly, we focus on a simple vehicle model and simple environment interaction by friction coefficient, air density, etc.

Note: The parameters shown below are not synced with the algorithm itself. Therefore, pay attention to any updates.


```html
Parameters:
overlap (int) = 0 [Size of the trajectory overlap. 0 disables this.]

Init parameters:
_mu (float) = 0.2 [Friction coeficient]
_g (float) = 9.81 [Gravity acceleration coeficient]
_m (float) = 3.68 [Vehicle mass]
_ro (float) = 1.2 [Air density]
_A (float) = 0.3 [Frontal reference aerodynamic area]
_cl (float) = 1 [Drag coeficient]
v_0 (float) = 0 [Initial speed [m.s^-1]]
v_lim (float) = 4.5 [Maximum forward speed [m.s^-1]]
a_acc_max (float) = 0.8 [Maximum longitudal acceleration [m.s^-2]]
a_break_max (float) = 4.5 [Maximum longitudal decceleration [m.s^-2]]
```


### Interpolators

Interpolators are used for interpolating the waypoints / path subsets in order to get full (continuous) path.


_ng_trajectory.interpolators.*_

- _cubic_spline_  Cubic spline interpolator.


#### Cubic Spline
_interpolators.cubic_spline_

Cubic spline interpolator.

This interpolator connects the racing line waypoints using cubic spline. Therefore, the resulting path is differentiable two times, and its curvature is continuous and "smooth". The curvature is computed as follows:

	K = (x' * y'' - y' * x'') / ( x'**2 + y'**2 )**(3/2)

Source: https://www.math24.net/curvature-radius/

Interpolation is done by CubicSpline from scipy.interpolate.

Note: It is expected that the input points describe a continuous path (end-start).


```html
Parameters:
int_size (int) = 400 [Number of points in the interpolation.]
```


### Segmentators

Segmentators are used for splitting the track into segments, based on the selection of their centers.


_ng_trajectory.segmentators.*_

- _euclidean_   Track segmentator based on euclidean distance from the center.
- _flood_fill_  Track segmentator based on the flood fill.


#### Euclidean
_segmentators.euclidean_

Track segmentator based on euclidean distance from the center.

This segmentator splits the track into segments based on the distance of the individual track parts from the group centers.

Note: Even though this is fast, it can missalign points (e.g., when they are behind a close wall).


```html
Parameters:
range_limit (float) = 0 [Maximum distance from the center of the segment. 0 disables this.]
```


#### Flood Fill
_segmentators.flood_fill_

Track segmentator based on the flood fill.

This segmentator splits the track into segments by flood fill algorithm from the centers.


```html
Parameters:
range_limit (float) = 0 [Maximum distance from the center of the segment. 0 disables this.]
reserve_width (bool) = False [When true, the segments are reserved a path towards both walls.]
reserve_selected (list) = [] [IDs of segments that should use the reservation method, when empty, use all.]
reserve_distance (float) = 2.0 [Distance from the line segment that is reserved to the segment.]
plot_flood (bool) = False [Whether the flooded areas should be plotted.]

Init parameters:
hold_map (bool) = False [When true, the map is created only once.]
```


### Selectors

Selectors are used for obtaining a subset of path's points, which are later used for track segmentation.


_ng_trajectory.selectors.*_

- _curvature_         Points selector based on the path's shape.
- _uniform_           Uniform selector.
- _curvature_sample_  Sampling selector based on the curvature.
- _uniform_distance_  Uniform distance selector.
- _fixed_             Fixed selector.
- _uniform_time_      Uniform time selector.
- _curvature2_        Selector that utilizes the curvature of the path.


#### Curvature
_selectors.curvature_

Points selector based on the path's shape.

This selector obtains a subset of path points in order to segment the track more intelligently. Points are situated in turns, with some filling on the straight sections.

The algorithm works as follows:
(i) positive and negative peaks on the curvature are found and populated by cuts,
(ii) close cuts are merged to avoid redundancy,
(iii) long cut-less sections of the track are artificially filled with equidistant cuts,
(iv) sections of the track between two consecutive cuts where the sign of the curvature changes are filled with additional cuts, and
(v) close cuts are filtered once again.

The current version segments the track automatically, given several parameters.

Note: The number of segments is determined differently: - -1 is selection based on dx - -2 is selection based on dy - -3 is selection based on curvature


```html
Parameters:
track_name (str) = unknown [Name of the track.]
plot (bool) = False [Whether the images are generated.]
show_plot (bool) = True [Whether the generated images are shown.]
interpolation_factor (float) = 24.0 [Factor to reduce number of points prior to the interpolation.]
peaks_height (float) = 0.0 [Minimum absolute height of peaks.]
peaks_merge (int) = 0 [Width of the area used for peaks merging.]
peaks_filling (int) = 1000000 [Width of the area for filling the points.]
downsample_factor (int) = 4 [Downsample factor used prior to the interpolation.]
split_peaks (bool) = False [Whether we want to split the height peaks.]
```


#### Uniform
_selectors.uniform_

Uniform selector.

This selector uniformly samples the input path. It is not equidistant, but rather index-equidistant.


```html
Init parameters:
rotate (float) = 0 [Parameter for rotating the subset selection. 0 is not rotated. <0, 1)]
```


#### Curvature Sample
_selectors.curvature_sample_

Sampling selector based on the curvature.

This selector samples points of the path according to their curvature, based on [1]. Sampling is non-repetitive.

Note: This means, that the result is different everytime.

[1]: Matteo Botta, Vincenzo Gautieri, Daniele Loiacono, and Pier Luca Lanzi. 2012. Evolving the optimal racing line in a high-end racing game. In 2012 IEEE Conferenceon Computational Intelligence and Games (CIG). 108–115. ISSN: 2325-4289


```html
Parameters:
interpolation_size (int) = 100 [Number of points used for interpolation.]
```


#### Uniform Distance
_selectors.uniform_distance_

Uniform distance selector.

This selector uniformly samples the input path so that the selected points are equidistantly spaced.


```html
Init parameters:
sampling_distance (float) = 1.0 [[m] Distance of super-sampling before the interpolation, skipped when 0.]
distance (float) = 0 [[m] Distance between the individual points, ignored when 0, used when requesting negative number of points.]
rotate (float) = 0 [Parameter for rotating the input path. 0 is not rotated. <0, 1)]
fixed_points (list) = [] [Points to be used in the selection upon calling 'select'.]
```


#### Fixed
_selectors.fixed_

Fixed selector.

This is not a selector as other, but it just imitates one. Fixed selector takes points from the arguments and returns them upon calling.


```html
Init parameters:
points (list) =  [Points to be returned upon calling 'select', list of points]
```


#### Uniform Time
_selectors.uniform_time_

Uniform time selector.

This selector uniformly samples the input path, so that the points are equidistantly spaced in time.

Following algorithms are used:
- 'profile' criterion for computing the time,
- 'cubic_spline' interpolator for smoothing the input,
- 'uniform_distance' selector for resampling the input.


```html
Parameters:
overlap (int) = 0 [Size of the trajectory overlap. 0 disables this.]

Init parameters:
rotate (float) = 0 [Parameter for rotating the input path. 0 is not rotated. <0, 1)]
_mu (float) = 0.2 [Friction coeficient]
_g (float) = 9.81 [Gravity acceleration coeficient]
_m (float) = 3.68 [Vehicle mass]
_ro (float) = 1.2 [Air density]
_A (float) = 0.3 [Frontal reference aerodynamic area]
_cl (float) = 1 [Drag coeficient]
v_0 (float) = 0 [Initial speed [m.s^-1]]
v_lim (float) = 4.5 [Maximum forward speed [m.s^-1]]
a_acc_max (float) = 0.8 [Maximum longitudal acceleration [m.s^-2]]
a_break_max (float) = 4.5 [Maximum longitudal decceleration [m.s^-2]]
sampling_distance (float) = 1.0 [[m] Distance of super-sampling before the interpolation, skipped when 0.]
distance (float) = 0 [[m] Distance between the individual points, ignored when 0, used when requesting negative number of points.]
fixed_points (list) = [] [Points to be used in the selection upon calling 'select'.]
```


#### Curvature2
_selectors.curvature2_

Selector that utilizes the curvature of the path.

In contrast to 'Curvature' selector, this one is using metric distances not indices.

Prior to the algorithm, the path can be resampled twice.
- First resampling (using 'sampling_distance') gets rid of the wavy curvature induced by too many points.
- Second resampling (using 'point_distance') sets the distance between subsequent path points.

The algorithm works as follows:
(i) Path is resampled if required / requested.
(ii) Peaks are detected on the original and inverted absolute path curvature. - Peaks are curvatures above 'peaks_height'. - Only one peak can be detected within 'peaks_distance'.
(iii) Both arrays of peaks are merged.
(iv) Turn boundaries are selected by moving the peaks in the positive and negative 'peaks_bounds'. These are used instead of peaks now on.
(v) Detected peaks are accompanied by filled "dummy" peaks that are added to empty spaces to ensure maximum distance between two consecutive peaks 'peaks_filling'.
(vi) Too close peaks 'peaks_merge' are merged together (averaged to the closest path point).

Note: Parameter 'remain' of the select function is completely ignored.


```html
Parameters:
track_name (str) = unknown [Name of the track.]
plot (bool) = False [Whether the images are generated.]
show_plot (bool) = True [Whether the generated images are shown.]
point_distance (float) = 0.1 [[m] Distance between consecutive points of the path, skipped when 0.]
sampling_distance (float) = 1.0 [[m] Distance of super-sampling before the interpolation, skipped when 0.]
peaks_height (float) = 1.0 [[m^-1] Minimum absolute height of peaks.]
peaks_distance (int) = 16 [Minimum distance between two identified peaks.]
peaks_bounds (int) = 8 [Distance to the turn boundaries (created pseudo-peaks), skipped when 0.]
peaks_filling (float) = 10.0 [[m] Maximum distance between two consecutive peaks in the final array.]
peaks_merge (int) = 0 [Maximum distance between two subsequent peaks to be merged.]
```


### Penalizers

Penalizers are used for checking whether the candidate solution is correct. When it is incorrect, they calculate a penalty.


_ng_trajectory.penalizers.*_

- _segment_      Segment penalizer.
- _curvature_    Curvature penalizer.
- _centerline_   Centerline penalizer.
- _count_        Count penalizer.
- _borderlines_  Borderlines penalizer.


#### Segment
_penalizers.segment_

Segment penalizer.

This penalizer detects all misplaced points.

The penalty is calculated as a distance between invalid point and valid points.


```html
Init. parameters:
debug (bool) = False [Whether debug plot is ought to be shown.]
method (str) = max [Optimization method for final penalty -- min / max / sum / avg.]
huber_loss (bool) = False [Whether to use Huber loss for computing the fitness.]
huber_delta (float) = 1.0 [(Requires 'huber_loss'). Delta used for computing the fitness.]
```


#### Curvature
_penalizers.curvature_

Curvature penalizer.

This penalizer finds all points that exceed the maximum admitted curvature.


```html
Parameters:
k_max (float) = 1.5 [Maximum allowed curvature in abs [m^-1]]
```


#### Centerline
_penalizers.centerline_

Centerline penalizer.

This penalizer detects all misplaced points. Each point is associated with a section of the track centerline based upon its location.

The penalty is calculated as maximum distance between invalid point and points of the borderline.

Final penalty is the minimum of all of these distances.

Note: Initialization of this is done only once; we expect that the centerline is present there (as it usually is).

Note: Huber loss used for computing the fitness (when 'huber_loss' is True) is defined [1]:

	L(a) = 0.5 * a^2 if abs(a) <= delta else delta * ( abs(a) - 0.5 * delta )

Important: Change the method to 'max', as default 'min' is not performing very well. Experimental evaluation showed that 'min' is 10times less successful than 'max'.
```json
	"penalizer_init": {
		"method": "max"
	}
```

[1]: https://en.wikipedia.org/wiki/Huber_loss


```html
Init. parameters:
method (str) = min [Optimization method for final penalty -- min / max / sum / avg.]
huber_loss (bool) = False [Whether to use Huber loss for computing the fitness.]
huber_delta (float) = 1.0 [(Requires 'huber_loss'). Delta used for computing the fitness.]
```


#### Count
_penalizers.count_

Count penalizer.

This penalizer simply counts all invalid points (outside of the valid area) and returns their count.


#### Borderlines
_penalizers.borderlines_

Borderlines penalizer.

Borderlines are sets of points on the borders between to adjacent segments. We have borderlines for each segment and for each neighbour
(usually resulting into n * 2 arrays).

This penalizer detects all misplaced points. Each point is associated with a borderline based upon its location -- e.g., points in between selected points of segments #5 and #6 belong to borderline 5-6.

The penalty is calculated as maximum distance between invalid point and points of the borderline.

Final penalty is the minimum of all of these distances.


## General parameters:

```html
_version (int) = None [Version of the configuration.]
_comment (str) = None [Commentary of the configuration file.]
cascade (list) = None [List of dicts, that is performed loops-times. Req. 'algorithm': OPTIMIZER]
start_points (str) = None [Name of the file with initial solution (i.e., centerline).]
valid_points (str) = None [Name of the file with valid positions of the track.]
```

## Optimization parameters:

```html
loops (int) = None [Number of repetitions.]
groups (int) = None [Number of segments on the track.]
variate (str) = None [Name of the field that contains multiple values. Its values are varied, run loop-cascade times.]
criterion (str) = None [Name of the function to evaluate current criterion.]
criterion_init (dict) = {} [Arguments for the init part of the criterion function.]
criterion_args (dict) = {} [Arguments for the criterion function.]
interpolator (str) = None [Name of the function to interpolate points.]
interpolator_init (dict) = {} [Arguments for the init part of the interpolator function.]
interpolator_args (dict) = {} [Arguments for the interpolator function.]
segmentator (str) = None [Name of the function to segmentate track.]
segmentator_init (dict) = {} [Arguments for the init part of the segmentator function.]
segmentator_args (dict) = {} [Arguments for the segmentator function.]
selector (str) = None [Name of the function to select path points as segment centers.]
selector_init (dict) = {} [Arguments for the init part of the selector function.]
selector_args (dict) = {} [Arguments for the selector function.]
penalizer (str) = None [Name of the function to evaluate penalty criterion.]
penalizer_init (dict) = {} [Arguments for the init part of the penalizer function.]
penalizer_args (dict) = {} [Arguments for the penalizer function.]
```

## Utility parameters:

```html
logging_verbosity (int) = 1 [Index to the verbosity of used logger.]
prefix (str) = None [Prefix of the output log file. When unset, use terminal.]
plot (bool) = None [When true, images are plotted.]
plot_args (list) = None [List of dicts with information for plotter. 1 el. is used prior to the optimization, 2nd after.]
silent_stub (bool) = False [When set, the application does not report that an algorithm for some part is missing.]
```

