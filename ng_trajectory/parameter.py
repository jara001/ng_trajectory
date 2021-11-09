#!/usr/bin/env python3.6
# main.py
"""Parameter utils for ng_trajectory.

Inspired by autopsy reconfigure.
"""
######################
# Imports & Globals
######################

from typing import Dict


######################
# Parameter class
######################

class Parameter(object):
    """Object that represents a parameter."""

    def __init__(self, name: str, default: any, type: any = None, description: str = "", group: str = ""):
        """Initializes a parameter object.

        name -- name of the parameter, str
        default -- default value of the parameter, any
        type -- type of the parameter, any, default type of 'default'
        description -- description of the parameter, str, default ""
        group -- parameter group this belongs into, str, default ""

        Note: Groups are used to distinguish init/run parameters.
        """

        self.name = name
        self.default = default
        self.value = default
        self.type = type if type is not None else default.__class__
        self.description = description
        self.group = group


    def get(self):
        """Obtains value of the parameter."""
        return self.value


    def set(self, value: any):
        """Sets a value to the parameter."""
        self.value = value


    def reset(self):
        """Resets the value of the parameter to the default value."""
        self.value = self.default


    def __str__(self):
        """Formats the parameter as a string."""
        return "%s (%s) = %s [%s]" % (self.name, str(self.type.__name__), str(self.value), str(self.description))




######################
# ParameterList class
######################

class ParameterList(object):
    """Object that represents a list of parameters."""

    def __init__(self):
        self.parameters = {}


    def add(self, parameter: Parameter):
        """Adds a parameter to the list."""
        self.parameters[parameter.name] = parameter


    def createAdd(self, *args, **kwargs):
        """Creates and add parameter to the list."""
        _parameter = Parameter(*args, **kwargs)
        self.parameters[_parameter.name] = _parameter


    def dict(self):
        """Return internal dict."""
        return self.parameters


    def get(self, name: str):
        """Obtains a parameter."""
        return self.parameters.get(name)


    def getValue(self, name: str):
        """Obtains value of a parameter."""
        return self.parameters.get(name).get()


    def iterate(self):
        """Iterate over items in the list."""
        for item in self.parameters.items():
            yield item


    def reset(self, name: str):
        """Resets a value of a parameter to its default state."""
        self.parameters.get(name).reset()


    def resetAll(self):
        """Resets values of all parameters to their default states."""
        for _p in self.parameters:
            self.reset(_p)


    def update(self, name: str, value: any):
        """Updates a value of a parameter."""
        self.parameters.get(name).set(value)


    def updateAll(self, kwargs: Dict[str, any], reset: bool = True):
        """Updates values of all parameters.

        reset -- When True all parameters are reset first.
        """
        if reset:
            self.resetAll()

        for _p, _v in kwargs.items():
            if _p in self.parameters:
                self.update(_p, _v)


    def __str__(self):
        """Formats the list as a string."""
        return "\n".join([ str(parameter) for parameter in self.parameters.values() ])
