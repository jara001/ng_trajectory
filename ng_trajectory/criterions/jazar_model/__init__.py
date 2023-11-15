#!/usr/bin/env python3.6
"""Model simulation according to the simplified model from Jazar [1].

This model is built on the 'Two-Wheel Planar Vehicle Dynamics'
equations of motion, page 143, with some addition assumptions.

Path profiling is done similarly to [2].

Note: The parameters shown below are not synced with the algorithm
itself. Therefore, pay attention to any updates.

[1] R. N. Jazar, Advanced Vehicle Dynamics. Cham: Springer International
Publishing, 2019. doi: 10.1007/978-3-030-13062-6.
[2] N. R. Kapania, J. Subosits, and J. Christian Gerdes, ‘A Sequential
Two-Step Algorithm for Fast Generation of Vehicle Racing Trajectories’,
Journal of Dynamic Systems, Measurement, and Control, vol. 138, no. 9,
p. 091005, Sep. 2016, doi: 10.1115/1.4033311.
"""
from .main import init, compute  # noqa: F401
