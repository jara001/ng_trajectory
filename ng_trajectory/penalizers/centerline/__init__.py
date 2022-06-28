#!/usr/bin/env python3.6
"""Centerline penalizer.

This penalizer detects all misplaced points. Each point is associated
with a section of the track centerline based upon its location.

The penalty is calculated as maximum distance between invalid point
and points of the borderline.

Final penalty is the minimum of all of these distances.

Note: Initialization of this is done only once; we expect that the
centerline is present there (as it usually is).

Note: Huber loss used for computing the fitness (when 'huber_loss'
is True) is defined [1]:

	L(a) = 0.5 * a^2 if abs(a) <= delta else delta * ( abs(a) - 0.5 * delta )

Important: Change the method to 'max', as default 'min' is not
performing very well. Experimental evaluation showed that 'min'
is 10times less successful than 'max'.
```json
	"penalizer_init": {
		"method": "max"
	}
```

[1]: https://en.wikipedia.org/wiki/Huber_loss
"""
from .main import init, penalize, INVALID_POINTS
