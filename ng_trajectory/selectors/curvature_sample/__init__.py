#!/usr/bin/env python3.6
"""Sampling selector based on the curvature.

This selector samples points of the path according
to their curvature, based on [1].
Sampling is non-repetitive.

Note: This means, that the result is different
everytime.

[1]: Matteo Botta, Vincenzo Gautieri, Daniele Loiacono,
     and Pier Luca Lanzi. 2012. Evolving the optimal
     racing line in a high-end racing game. In 2012
     IEEE Conferenceon Computational Intelligence and
     Games (CIG). 108–115. ISSN: 2325-4289
"""
from .main import init, select
