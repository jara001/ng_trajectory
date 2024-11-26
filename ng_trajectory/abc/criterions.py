#!/usr/bin/env python3.6
# criterions.py
"""ABC for criterions."""

from abc import (
    ABC,
    abstractmethod,
)
from typing import (
    Any,
    Dict,
    Optional,
)


class CriterionABC(ABC, object):
    """Abstract class for Criterion algorithms."""

    def __init__(self):
        """Initialize the criterion."""
        super(CriterionABC, self).__init__()


    @abstractmethod
    def init(self) -> Optional[Dict[str, Any]]:
        """Initialize criterion."""
        raise NotImplementedError


    @abstractmethod
    def compute(self) -> float:
        """Compute criterion value."""
        raise NotImplementedError
