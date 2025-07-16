#!/usr/bin/env python3.6
# interpolators.py
"""ABC for interpolators."""

from abc import (
    ABC,
    abstractmethod,
)
from typing import (
    Any,
    Dict,
    Optional,
)

import numpy


class InterpolatorABC(ABC, object):
    """Abstract class for Interpolator algorithms."""

    def __init__(self):
        """Initialize the interpolator."""
        super(InterpolatorABC, self).__init__()


    @abstractmethod
    def init(self) -> Optional[Dict[str, Any]]:
        """Initialize interpolator."""
        raise NotImplementedError


    @abstractmethod
    def interpolate(self) -> numpy.ndarray:
        """Interpolate points."""
        raise NotImplementedError
