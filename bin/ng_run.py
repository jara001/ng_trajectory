#!/usr/bin/env python3.6
# ng_run.py
"""Script for running the package.
"""
######################
# Imports & Globals
######################

import sys, os


######################
# Main
######################

if __name__ == "__main__":
    # Expect one argument
    if len(sys.argv) > 1:
        if os.path.exists(sys.argv[1]):

            from ng_trajectory import configurationLoad, execute

            if configurationLoad(sys.argv[1]):
                print (execute())
            else:
                print ("Unable to load configuration from given file.", file=sys.stderr)
        else:
            print ("File %s does not exist." % sys.argv[1], file=sys.stderr)
    else:
        print ("Expected one argument with path to the configuration file.", file=sys.stderr)

