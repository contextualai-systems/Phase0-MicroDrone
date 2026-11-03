"""Turn a list of points into waypoint poses for Motion's WaypointExecutor.

Navigation (or the mission layer) decides *which* points to visit; this
helper only fills in altitude and heading so every waypoint is a complete
`Pose`. Motion's `WaypointExecutor` then flies them in order and reports
progress. Validation against the geofence/ceiling happens in the executor,
against the same limits Safety uses.
"""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple, Union

from motion_engine.contracts import Pose

Point = Union[Tuple[float, float], Tuple[float, float, float], Pose]


def _heading_deg(a: Pose, b: Pose, fallback: float) -> float:
    dx, dy = b.x - a.x, b.y - a.y
    if math.hypot(dx, dy) < 1e-9:
        return fallback
    return math.degrees(math.atan2(dy, dx)) % 360.0


def build_route(
    points: Iterable[Point],
    altitude_m: Optional[float] = None,
    start: Optional[Pose] = None,
    face_direction_of_travel: bool = True,
) -> List[Pose]:
    """Waypoint poses (local ENU) from `(x, y)`, `(x, y, z)` or `Pose` points.

    * `(x, y)` points need `altitude_m`; `(x, y, z)` and `Pose` keep their z
      unless `altitude_m` is given, which overrides every waypoint's z.
    * With `face_direction_of_travel`, each waypoint's yaw points along the
      leg that reaches it (from `start`, or the previous waypoint), so the
      camera faces where the drone is going. A zero-length leg keeps the
      previous heading. Otherwise `Pose` yaw is kept and tuples get yaw 0.

    Raises ValueError for an empty route or a 2-D point without `altitude_m`.
    """
    raw: Sequence[Point] = list(points)
    if not raw:
        raise ValueError("a route needs at least one point")
    poses: List[Pose] = []
    for i, p in enumerate(raw):
        if isinstance(p, Pose):
            x, y, z, yaw = p.x, p.y, p.z, p.yaw_deg
        elif len(p) == 3:
            (x, y, z), yaw = p, 0.0
        elif len(p) == 2:
            if altitude_m is None:
                raise ValueError(f"point {i} has no altitude; pass altitude_m")
            (x, y), z, yaw = p, altitude_m, 0.0
        else:
            raise ValueError(f"point {i} must be (x, y), (x, y, z) or a Pose")
        if altitude_m is not None:
            z = altitude_m
        poses.append(Pose(float(x), float(y), float(z), float(yaw)))

    if face_direction_of_travel:
        prev = start if start is not None else poses[0]
        heading = prev.yaw_deg
        faced = []
        for p in poses:
            heading = _heading_deg(prev, p, heading)
            faced.append(Pose(p.x, p.y, p.z, heading))
            prev = p
        poses = faced
    return poses
