#!/usr/bin/env python3.6
"""Dummy penalizer.

This effectively disables the penalizer, as it passes any candidate.
"""
from .main import NonePenalizer as Penalizer  # noqa: F401
