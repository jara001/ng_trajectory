#!/usr/bin/env python
# _version.py
"""Script for managing versions."""

class Version():
    """Simple class for parsing versions and creating a Py-compatible format.

    We take into account following types:
    1.0.0
    1.0.0-1-1578sdfe
    1.0.0-1-1484fdgd-dirty
    """

    def __init__(self, git_version):
        version = (git_version if "-" not in git_version else git_version[:git_version.index("-")]).split(".")

        self.MAJOR = version[0]
        self.MINOR = version[1]
        self.PATCH = version[2] if len(version) > 1 else None

        # Dirty
        if "-" in git_version:
            git_version = git_version[git_version.index("-")+1:]

            self.DEV = git_version[-5:] == "dirty"

            if self.DEV:
                git_version = git_version[:-6]
        else:
            self.DEV = False

        # Commit
        self.POSTPATCH = None
        if len(git_version) > 0 and "-" in git_version:
            self.POSTPATCH = git_version[:git_version.index("-")]
            git_version = git_version[git_version.index("-")+1:]

        self.OTHER = git_version


    def __str__(self):
        return "%s.%s" % (self.MAJOR, self.MINOR) + (".%s" % self.PATCH if self.PATCH else "") + (".post%s" % self.POSTPATCH if self.POSTPATCH else "") + (".dev" if self.DEV else "")


    def __repr__(self):
        return self.__str__() + " (" + self.OTHER + ")"
