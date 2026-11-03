"""Tracking geometry -> a pose for Motion to fly.

Moved out of `MotionStub.track_target()` so Motion never sees a bird. The
coordinator gives Navigation the declared synthetic tracking geometry
(INTEGRATION_README §4 `tracking_geometry`: bearing and range in the world
frame) and the current vehicle pose; Navigation returns the standoff pose;
the coordinator sends it to Motion as a TRACK_TARGET command after Safety.

Frames follow INTEGRATION_README §4: local ENU metres, yaw/bearing in
degrees, 0 = east, counter-clockwise positive.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from motion_engine.contracts import Pose
from safety_layer import config

#: Default standoff distance (INTEGRATION_README §5 "Tracking standoff").
DEFAULT_STANDOFF_M: float = 15.0

CLOSE_TO_STANDOFF = "close_to_standoff"
BACK_OFF_TO_STANDOFF = "back_off_to_standoff"
HOLD_STANDOFF = "hold_standoff"


@dataclass(frozen=True)
class StandoffTarget:
    """Navigation's proposed tracking pose and why."""

    pose: Pose
    #: One of CLOSE_TO_STANDOFF / BACK_OFF_TO_STANDOFF / HOLD_STANDOFF.
    decision: str
    #: Signed distance along the bearing (m); negative means backing away.
    travel_m: float


def _wrap_360(deg: float) -> float:
    return deg % 360.0


def standoff_target(
    vehicle: Pose,
    bearing_deg: float,
    distance_m: float,
    standoff_m: float = DEFAULT_STANDOFF_M,
    altitude_m: Optional[float] = None,
    tolerance_m: Optional[float] = None,
) -> StandoffTarget:
    """The pose `standoff_m` short of a target at `bearing_deg`/`distance_m`.

    `bearing_deg` is world-frame (0 = east, CCW +), measured from `vehicle`.
    The pose lies `distance_m - standoff_m` along that bearing and faces the
    target. If the target is already inside the standoff, that distance is
    negative and the pose backs away to restore it. `altitude_m=None` keeps
    the vehicle's current altitude. `tolerance_m` (default: the shared
    arrival tolerance) decides when the drone is already at the standoff.

    Raises ValueError on non-finite input or a negative distance/standoff:
    unavailable geometry is the caller's validity check (hold, don't guess).
    """
    values = (vehicle.x, vehicle.y, vehicle.z, bearing_deg, distance_m, standoff_m)
    if altitude_m is not None:
        values += (altitude_m,)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError("tracking geometry must be finite numbers")
    if distance_m < 0.0 or standoff_m < 0.0:
        raise ValueError("distance_m and standoff_m must be non-negative")
    tol = config.POSITION_ARRIVAL_TOLERANCE_M if tolerance_m is None else tolerance_m

    travel = distance_m - standoff_m
    b = math.radians(bearing_deg)
    pose = Pose(
        vehicle.x + travel * math.cos(b),
        vehicle.y + travel * math.sin(b),
        vehicle.z if altitude_m is None else altitude_m,
        _wrap_360(bearing_deg),
    )
    if travel > tol:
        decision = CLOSE_TO_STANDOFF
    elif travel < -tol:
        decision = BACK_OFF_TO_STANDOFF
    else:
        decision = HOLD_STANDOFF
    return StandoffTarget(pose=pose, decision=decision, travel_m=travel)
