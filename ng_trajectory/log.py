#!/usr/bin/env python3.6
# log.py
"""Logging utility for the ng_trajectory package.

This should handle all printing from the experiments.
"""
######################
# Imports & Globals
######################

import sys

from typing import TextIO


# Global variables
LOGGING_VERBOSITY = 0
LOGFILE = sys.stdout


######################
# Verbose decorator
######################

def verbose(level):
    """Decorate to conditionally suppress printing."""
    def wrapper(f, *args, **kwargs):
        def cond_print(*args, **kwargs):
            global LOGGING_VERBOSITY

            _level = LOGGING_VERBOSITY

            # Check for level in the kwargs
            if "level" in kwargs:
                _level = kwargs.get("level")
                del kwargs["level"]

            if _level >= level:
                f(*args, **kwargs)
        return cond_print
    return wrapper


######################
# Functions
######################

def verbositySet(level: int) -> None:
    """Set the global verbosity level.

    Arguments:
    level -- verbosity amount, int
    """
    global LOGGING_VERBOSITY

    LOGGING_VERBOSITY = level


def logfileFlush() -> None:
    """Flush the logfile buffer."""
    LOGFILE.flush()


def logfileName() -> str:
    """Get the name of the current logfile."""
    return LOGFILE.name


def logfileSet(logfile: TextIO) -> None:
    """Set the global logfile.

    Arguments:
    logfile -- opened file ready for writing, TextIO
    """
    global LOGFILE

    LOGFILE = logfile


def logfileReset() -> None:
    """Reset the global logfile back to stdout."""
    global LOGFILE

    LOGFILE = sys.stdout


######################
# Print functions
######################

def print0(*args, logfile = None, **kwargs) -> None:
    """Print 'args' in a custom way.

    When 'logfile' is passed (and not stdout), the print
    is also done into the logfile.

    This function ensures, that the line is not repeated
    in the terminal.
    """
    global LOGFILE

    if logfile is None:
        logfile = LOGFILE

    print (*args, **kwargs)

    if logfile != sys.stdout:
        log (*args, **{**kwargs, **{"logfile": logfile}})


@verbose(1)
def printv(*args, **kwargs) -> None:
    """Print only when verbosity is set to >= 1.

    Checks global verbosity setting, or passed 'level' value.
    """
    print0(*args, **kwargs)


@verbose(2)
def printvv(*args, **kwargs) -> None:
    """Print only when verbosity is set to >= 2.

    Checks global verbosity setting, or passed 'level' value.
    """
    print0(*args, **kwargs)


@verbose(3)
def printvvv(*args, **kwargs) -> None:
    """Print only when verbosity is set to >= 3.

    Checks global verbosity setting, or passed 'level' value.
    """
    print0(*args, **kwargs)


######################
# Log functions
######################

def log(*args, logfile = None, **kwargs) -> None:
    """Log 'args' into a log file, or show it in the terminal."""
    global LOGFILE

    if logfile is None:
        logfile = LOGFILE

    print (*args, **{**kwargs, **{"file": logfile}})


@verbose(1)
def logv(*args, **kwargs) -> None:
    """Log only when verbosity is set to >= 1.

    Checks global verbosity setting, or passed 'level' value.
    """
    log(*args, **kwargs)


@verbose(2)
def logvv(*args, **kwargs) -> None:
    """Log only when verbosity is set to >= 2.

    Checks global verbosity setting, or passed 'level' value.
    """
    log(*args, **kwargs)


@verbose(3)
def logvvv(*args, **kwargs) -> None:
    """Log only when verbosity is set to >= 3.

    Checks global verbosity setting, or passed 'level' value.
    """
    log(*args, **kwargs)
