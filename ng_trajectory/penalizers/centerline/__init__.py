#!/usr/bin/env python3.6
"""Centerline penalizer.

This penalizer detects all misplaced points. Each point is associated
with a section of the track centerline based upon its location.

The penalty is calculated as maximum distance between invalid point
and points of the borderline.

Final penalty is the minimum of all of these distances.

Note: Initialization of this is done only once; we expect that the
centerline is present there (as it usually is).

Important: Change the method to 'max', as default 'min' is not
performing very well. Experimental evaluation showed that 'min'
is 10times less successful than 'max'.
```json
	"penalizer_init": {
		"method": "max"
	}
```
"""
from .main import init, penalize
