"""
Interpolators are used for interpolating the waypoints / path subsets
in order to get full (continuous) path.
"""
#https://stackoverflow.com/questions/1057431/how-to-load-all-modules-in-a-folder
from os.path import dirname, basename, isfile, join, isdir
import glob

modules = glob.glob(join(dirname(__file__), "*"))

__all__ = [ basename(f)[:-3] for f in modules if isfile(f) and not f.endswith('__init__.py')]
__all__ += [ basename(f) for f in modules if isdir(f) and not f.endswith('__')]

from . import *
