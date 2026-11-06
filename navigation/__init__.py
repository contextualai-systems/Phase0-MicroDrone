"""Navigation: decides *where* the drone should go; Motion decides nothing.

Jeff's boundary for Phase 0: Motion executes commands, higher layers decide
what those commands are. This package is the higher layer for movement
targets:

* `tracking.standoff_target` -- tracking geometry (bearing/range) to the pose
  Motion flies with a TRACK_TARGET command (moved here out of MotionStub).
* `routes.build_route` -- a list of points to the waypoint poses Motion's
  `WaypointExecutor` flies.

Nothing here calls Motion or changes mission state; it only returns poses.
"""

from navigation.routes import build_route
from navigation.tracking import StandoffTarget, standoff_target

__all__ = ["StandoffTarget", "standoff_target", "build_route"]
