"""WaypointExecutor -- Motion flies a list of poses and reports progress.

Jeff's split for Phase 0: Motion owns *waypoint navigation as a mechanism*,
higher layers decide *which* waypoints to fly. So this class never chooses a
route. It is handed a list of `Pose`s (e.g. from `navigation.build_route`)
and turns it, one waypoint at a time, into ordinary MOVE_TO contract
`Command`s, then watches `VehicleFeedback` to report progress and arrival.

It does **not** bypass the shared tick (INTEGRATION_README §3). The
coordinator still runs every command through Safety and `MotionStub.submit()`:

    executor = WaypointExecutor()                      # same limits as Safety
    executor.load("patrol-1", route, now_ns)
    each tick:
        cmd = executor.command("TRACKING", now_ns)     # None when done/failed
        if cmd is not None:
            cmd, _ = check_for_motion(cmd, assessment, now_ns)   # Safety
            executor.on_ack(motion.submit(cmd, now_ns))
        motion.step(0.1, now_ns=now_ns)
        progress = executor.update(motion.feedback(now_ns))

Rules:

* **Validated up front.** An empty route, a non-finite pose, or a waypoint
  outside the geofence / ceiling / floor fails the whole route at `load()`
  with a stable reason, before anything flies. Limits default to
  `MotionLimits.from_config()`, i.e. Safety's numbers.
* **One command per waypoint.** The same `command_id` is renewed every tick
  (Motion treats it as a renewal), and the next waypoint gets the next ID.
* **Arrival comes from Motion.** A waypoint counts as reached only when the
  feedback says `arrived` *for this executor's command ID*. A Safety hold or
  an expiry that replaced the command does not count; the executor re-issues
  the waypoint under a fresh ID on the next tick.
* **Rejection fails the route.** If Motion rejects the executor's command,
  the route is FAILED with Motion's reason and no more commands are issued.
  Whether to retry, hold or abort is the mission layer's decision.
* **Completion sends nothing more.** After the last arrival `command()`
  returns None; the coordinator decides what comes next (normally HOVER).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence

try:
    from motion_engine.contracts import (
        Command, CommandAck, LogSink, MotionCommandType, Pose, VehicleFeedback,
    )
    from motion_engine.motion_stubs import GROUND_Z_M, MotionLimits
except ModuleNotFoundError:  # pragma: no cover -- only motion_engine/ on sys.path
    from contracts import (  # type: ignore[no-redef]
        Command, CommandAck, LogSink, MotionCommandType, Pose, VehicleFeedback,
    )
    from motion_stubs import GROUND_Z_M, MotionLimits  # type: ignore[no-redef]

__all__ = ["WaypointExecutor", "WaypointProgress", "WaypointStatus", "WaypointReason"]

_LOG_MODULE = "MOTION"
_NS_PER_S = 1_000_000_000


class WaypointStatus(str, Enum):
    IDLE = "idle"                # nothing loaded (or cancelled)
    IN_PROGRESS = "in_progress"  # flying toward `index`
    COMPLETE = "complete"        # every waypoint reached
    FAILED = "failed"            # invalid route or Motion rejected a command


class WaypointReason:
    """Stable reason strings (compare against these, never free text)."""

    LOADED = "route_loaded"
    WAYPOINT_REACHED = "waypoint_reached"
    ROUTE_COMPLETE = "route_complete"
    CANCELLED = "cancelled"
    EMPTY_ROUTE = "empty_route"
    INVALID_ROUTE_ID = "invalid_route_id"
    INVALID_WAYPOINT = "invalid_waypoint"
    WAYPOINT_OUTSIDE_LIMITS = "waypoint_outside_limits"
    COMMAND_REJECTED = "command_rejected"


@dataclass(frozen=True)
class WaypointProgress:
    """Progress report for the coordinator / logs. Read-only snapshot."""

    route_id: Optional[str]
    status: WaypointStatus
    #: 0-based index of the waypoint being flown; None unless IN_PROGRESS.
    index: Optional[int]
    total: int
    #: How many waypoints have been reached so far.
    reached: int
    target: Optional[Pose]
    distance_to_target_m: Optional[float]
    command_id: Optional[str]
    reason: Optional[str]
    #: Extra detail for a failure (e.g. which waypoint, Motion's reason).
    detail: Optional[str] = None

    @property
    def complete(self) -> bool:
        return self.status == WaypointStatus.COMPLETE

    @property
    def failed(self) -> bool:
        return self.status == WaypointStatus.FAILED

    def to_dict(self) -> Dict[str, Any]:
        return {
            "route_id": self.route_id,
            "status": self.status.value,
            "index": self.index,
            "total": self.total,
            "reached": self.reached,
            "target": self.target.to_dict() if self.target is not None else None,
            "distance_to_target_m": self.distance_to_target_m,
            "command_id": self.command_id,
            "reason": self.reason,
            "detail": self.detail,
        }


class WaypointExecutor:
    """Flies a validated list of poses through ordinary MOVE_TO commands."""

    def __init__(
        self,
        limits: Optional[MotionLimits] = None,
        home: Pose = Pose(0.0, 0.0, 0.0, 0.0),
        validity_ns: Optional[int] = None,
        logger: Optional[LogSink] = None,
        source: str = "waypoint_executor",
    ) -> None:
        self.limits = limits if limits is not None else MotionLimits.from_config()
        self.home = home
        if validity_ns is None:
            from safety_layer import config  # lazy, as in MotionLimits.from_config
            validity_ns = int(round(config.COMMAND_VALIDITY_S * _NS_PER_S))
        if not isinstance(validity_ns, int) or validity_ns <= 0:
            raise ValueError("validity_ns must be a positive int")
        self.validity_ns = validity_ns
        self.source = source
        self._logger = logger
        self._reset(None, [])

    # -- state ---------------------------------------------------------

    def _reset(self, route_id: Optional[str], waypoints: List[Pose]) -> None:
        self._route_id = route_id
        self._waypoints = waypoints
        self._index = 0
        self._status = WaypointStatus.IDLE
        self._reason: Optional[str] = None
        self._detail: Optional[str] = None
        self._attempt = 0
        self._command_id: Optional[str] = None
        self._accepted = False   # Motion accepted the current command_id
        self._distance: Optional[float] = None

    @property
    def waypoints(self) -> List[Pose]:
        return list(self._waypoints)

    @property
    def progress(self) -> WaypointProgress:
        active = self._status == WaypointStatus.IN_PROGRESS
        return WaypointProgress(
            route_id=self._route_id,
            status=self._status,
            index=self._index if active else None,
            total=len(self._waypoints),
            reached=self._index,
            target=self._waypoints[self._index] if active else None,
            distance_to_target_m=self._distance if active else None,
            command_id=self._command_id if active else None,
            reason=self._reason,
            detail=self._detail,
        )

    # -- lifecycle -----------------------------------------------------

    def load(self, route_id: str, waypoints: Sequence[Pose], now_ns: int = 0) -> WaypointProgress:
        """Validate and start a route. Replaces any route already loaded."""
        wps = list(waypoints) if waypoints is not None else []
        self._reset(route_id if isinstance(route_id, str) else None, wps)
        if not isinstance(route_id, str) or not route_id:
            return self._fail(WaypointReason.INVALID_ROUTE_ID, None, now_ns)
        if not wps:
            return self._fail(WaypointReason.EMPTY_ROUTE, None, now_ns)
        for i, wp in enumerate(wps):
            if not isinstance(wp, Pose) or not wp.is_finite():
                return self._fail(WaypointReason.INVALID_WAYPOINT, f"waypoint {i}", now_ns)
            problem = self._outside_limits(wp)
            if problem is not None:
                return self._fail(
                    WaypointReason.WAYPOINT_OUTSIDE_LIMITS, f"waypoint {i}: {problem}", now_ns
                )
        self._status = WaypointStatus.IN_PROGRESS
        self._reason = WaypointReason.LOADED
        self._new_command_id()
        self._log("waypoint_route_loaded", {"route_id": route_id, "total": len(wps)}, now_ns,
                  decision=WaypointReason.LOADED)
        return self.progress

    def cancel(self, now_ns: int = 0) -> WaypointProgress:
        """Stop issuing commands. The coordinator sends the replacement hold."""
        if self._status == WaypointStatus.IN_PROGRESS:
            self._log("waypoint_route_cancelled", {"route_id": self._route_id,
                      "index": self._index}, now_ns, decision=WaypointReason.CANCELLED)
        self._status = WaypointStatus.IDLE
        self._reason = WaypointReason.CANCELLED
        self._distance = None
        return self.progress

    # -- per tick --------------------------------------------------------

    def command(self, mission_state: str, now_ns: int) -> Optional[Command]:
        """The MOVE_TO for the current waypoint, or None when not flying.

        Call once per tick. Returns the same `command_id` (a renewal) until
        the waypoint is reached or the command stopped being Motion's active
        one, in which case a fresh ID is minted.
        """
        if self._status != WaypointStatus.IN_PROGRESS:
            return None
        return Command(
            command_id=self._command_id,
            kind=MotionCommandType.MOVE_TO,
            mission_state=mission_state,
            issued_ns=now_ns,
            expires_ns=now_ns + self.validity_ns,
            source=self.source,
            reason=f"waypoint {self._index + 1}/{len(self._waypoints)}",
            target=self._waypoints[self._index],
        )

    def on_ack(self, ack: CommandAck) -> WaypointProgress:
        """Record Motion's answer. Acks for other IDs (e.g. a Safety
        replacement) are ignored: they did not fly this waypoint."""
        if self._status != WaypointStatus.IN_PROGRESS or ack.command_id != self._command_id:
            return self.progress
        if ack.accepted:
            self._accepted = True
            return self.progress
        return self._fail(WaypointReason.COMMAND_REJECTED,
                          f"waypoint {self._index}: {ack.reason}", ack.timestamp_ns)

    def update(self, feedback: VehicleFeedback) -> WaypointProgress:
        """Advance on arrival; re-issue if the command was displaced."""
        if self._status != WaypointStatus.IN_PROGRESS:
            return self.progress
        target = self._waypoints[self._index]
        self._distance = feedback.pose.distance_to(target)
        ours = feedback.active_command_id == self._command_id
        if self._accepted and ours and feedback.arrived:
            self._log("waypoint_reached", {"route_id": self._route_id, "index": self._index,
                      "x": target.x, "y": target.y, "z": target.z},
                      feedback.timestamp_ns, decision=WaypointReason.WAYPOINT_REACHED)
            self._index += 1
            self._reason = WaypointReason.WAYPOINT_REACHED
            if self._index >= len(self._waypoints):
                self._status = WaypointStatus.COMPLETE
                self._reason = WaypointReason.ROUTE_COMPLETE
                self._distance = None
                self._log("waypoint_route_complete", {"route_id": self._route_id,
                          "total": len(self._waypoints)}, feedback.timestamp_ns,
                          decision=WaypointReason.ROUTE_COMPLETE)
            else:
                self._new_command_id()
                self._distance = feedback.pose.distance_to(self._waypoints[self._index])
        elif self._accepted and not ours:
            # Safety's hold, an expiry, or another command replaced ours.
            # Motion will not accept a reused ID, so fly it again under a new one.
            self._new_command_id()
        return self.progress

    # -- internals -------------------------------------------------------

    def _new_command_id(self) -> None:
        self._attempt += 1
        self._command_id = f"{self._route_id}-wp{self._index}-{self._attempt}"
        self._accepted = False

    def _outside_limits(self, wp: Pose) -> Optional[str]:
        lim = self.limits
        if wp.z > GROUND_Z_M + lim.max_altitude_m:
            return "above_ceiling"
        if wp.z < GROUND_Z_M + lim.min_altitude_m:
            return "below_floor"
        if wp.horizontal_distance_to(self.home) > lim.geofence_radius_m:
            return "outside_geofence"
        return None

    def _fail(self, reason: str, detail: Optional[str], now_ns: int) -> WaypointProgress:
        self._status = WaypointStatus.FAILED
        self._reason = reason
        self._detail = detail
        self._distance = None
        self._log("waypoint_route_failed", {"route_id": self._route_id, "detail": detail},
                  now_ns, decision=reason)
        return self.progress

    def _log(self, event: str, fields: Dict[str, Any], now_ns: Any,
             decision: Optional[str] = None) -> None:
        if self._logger is None:
            return
        t = now_ns / _NS_PER_S if isinstance(now_ns, (int, float)) and math.isfinite(now_ns) else None
        self._logger.log(_LOG_MODULE, event, fields=fields, decision=decision, t=t)
