#!/usr/bin/env python3.6
# penalizers.py
"""ABC for penalizers."""

from abc import (
    ABC,
    abstractmethod,
)
from typing import (
    Any,
    Dict,
    Optional,
)


class PenalizerABC(ABC, object):
    """Abstract class for Penalizer algorithms."""

    def __init__(self):
        """Initialize the penalizer."""
        super(PenalizerABC, self).__init__()

        self.INVALID_POINTS = []


    @abstractmethod
    def init(self) -> Optional[Dict[str, Any]]:
        """Initialize criterion."""
        raise NotImplementedError


    @abstractmethod
    def penalize(self) -> float:
        """Compute penalty for a candidate solution."""
        raise NotImplementedError
