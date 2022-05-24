#!/usr/bin/env python3.6
# utils.py3
"""Various utilities for penalizers.
"""
######################
# Imports & Globals
######################

# Parameters
from ng_trajectory.parameter import *
P = ParameterList()
P.createAdd("method", "min", str, "Optimization method for final penalty -- min / max / sum / avg.", "Init.")
P.createAdd("huber_loss", False, bool, "Whether to use Huber loss for computing the fitness.", "Init.")
P.createAdd("huber_delta", 1.0, float, "(Requires 'huber_loss'). Delta used for computing the fitness.", "Init.")


######################
# Optimization methods
######################

METHODS = {
    "min": {
        "function": lambda old, new: min(old, new),
        "initial": 1000,
        "after": lambda result, invalid_count: result,
    },
    "max": {
        "function": lambda old, new: max(old, new),
        "initial": 0,
        "after": lambda result, invalid_count: result,
    },
    "sum": {
        "function": lambda old, new: old + new,
        "initial": 0,
        "after": lambda result, invalid_count: result,
    },
    "avg": {
        "function": lambda old, new: old + new,
        "initial": 0,
        "after": lambda result, invalid_count: result / invalid_count if invalid_count > 0 else result,
    },
}

METHOD = METHODS["min"]["function"]
INITIAL = METHODS["min"]["initial"]
AFTER = METHODS["min"]["after"]
HUBER_LOSS = False
HUBER_DELTA = 0.0
