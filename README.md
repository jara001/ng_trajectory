# Optimization toolbox ng_trajectory

**ng_trajectory** is a toolbox for solving the optimal racing line problem using various methods and approaches. The main idea stands on using a genetic algorithm, although other approaches may be used as well.

The main purpose of this package is to demonstrate our proposed Matryoshka mapping [1].

To view the package documentation either run `ng_help`, or see [doc/README.md](./doc/README.md).


## Requirements

- `Python>=3.6`
- `nevergrad==0.3.0`
- `scipy>=0.18.0`
- `numpy>=1.12.0`
- `Pillow>=4.2.0`
- `tqdm`


### Optional requirements

- `matplotlib` for plotting the results
- `rospy` for publishing the results to various ROS topics
- `shapely` (EXPERIMENTAL) for computing the Matryoshka transformation with a different method


## Citing

[1]: J. Klapálek, A. Novák, M. Sojka and Z. Hanzálek, "Car Racing Line Optimization with Genetic Algorithm using Approximate Homeomorphism," 2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2021, pp. 601-607, doi: 10.1109/IROS51168.2021.9636503.
