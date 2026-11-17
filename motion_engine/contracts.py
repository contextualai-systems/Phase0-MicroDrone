"""Motion Engine side of the shared Phase-0 contract (INTEGRATION_README.md §4).

The integration README asks every module to talk to the central coordinator
through one versioned contract (`schema_version = 1`): commands go *in*,
an acknowledgement comes straight back, and feedback reports progress on
later ticks. This file holds the Motion Engine's half of that contract:

* `Command`         -- what the coordinator dispatches after Safety approves it
* `CommandAck`      -- Motion's immediate answer: accepted or rejected, and why
* `DroneState`      -- the `vehicle` section of the snapshot, read each tick
                       (`VehicleFeedback` is kept as an alias of the same class)

plus the value types they share (`Pose`, `Velocity`, `Orientation`,
`MotionCommandType`, `FlightMode`), a stable set
of reason strings, a simulation clock, and a tiny in-memory log sink.

PROVISIONAL LOCATION: the README suggests the shared types eventually live in
`state_machine/contracts.py` and the logger in `state_machine/logging_format.py`,
owned by the state-machine team. Neither exists yet. Until they do, the
Motion types live here with the README's field names, so moving them is a
cut-and-paste plus a re-export, not a rename.

Conventions (README §4):

* Time is integer simulation nanoseconds (`timestamp_ns`), starting at 0.
  Wall time is never used for a decision.
* Position is local ENU metres (x east, y north, z up), pad and ground at
  z = 0. Yaw is degrees, 0 = east, counter-clockwise positive.
* A missing required field is a *validation result* (a rejection with a
  reason), never a silently-filled "safe" default.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Protocol, Tuple

SCHEMA_VERSION: int = 1

#: The only frame accepted in this milestone.
FRAME_LOCAL_ENU: str = "local_enu"

#: Source id stamped on Motion feedback.
MOTION_SOURCE_ID: str = "motion_stub"


# --------------------------------------------------------------------------
# Value types
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Pose:
    """Position (m, local ENU, z up) plus heading (deg, 0 = east, CCW +).

    Frozen on purpose: `MotionStub.pose` hands this out to the coordinator
    and the synthetic world every tick. If it were mutable, one consumer
    doing `pose.z = 0` would silently teleport the drone for everyone else.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_deg: float = 0.0

    def distance_to(self, other: "Pose") -> float:
        """3-D Euclidean distance in metres (yaw ignored)."""
        return math.sqrt(
            (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2
        )

    def horizontal_distance_to(self, other: "Pose") -> float:
        """Distance in the x/y plane only, metres."""
        return math.hypot(self.x - other.x, self.y - other.y)

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.yaw_deg)

    def is_finite(self) -> bool:
        return all(_is_number(v) and math.isfinite(v) for v in self.as_tuple())

    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z, "yaw_deg": self.yaw_deg}


@dataclass(frozen=True)
class Velocity:
    """Linear velocity (m/s, local ENU: vx east, vy north, vz up) plus an
    optional yaw rate (deg/s, counter-clockwise positive).

    Used twice, never mixed up:

    * as a *setpoint* on a `SET_VELOCITY` `Command` (`yaw_rate_deg_s=None`
      means "keep the current heading"), and
    * as the *estimated* velocity in `DroneState.velocity`.
    """

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate_deg_s: Optional[float] = None

    def is_finite(self) -> bool:
        values = [self.vx, self.vy, self.vz]
        if self.yaw_rate_deg_s is not None:
            values.append(self.yaw_rate_deg_s)
        return all(_is_number(v) and math.isfinite(v) for v in values)

    @property
    def horizontal_speed_mps(self) -> float:
        return math.hypot(self.vx, self.vy)

    def to_dict(self) -> Dict[str, Optional[float]]:
        return {
            "vx": self.vx,
            "vy": self.vy,
            "vz": self.vz,
            "yaw_rate_deg_s": self.yaw_rate_deg_s,
        }


@dataclass(frozen=True)
class Orientation:
    """Attitude in degrees: roll, pitch, yaw (yaw 0 = east, CCW +).

    The kinematic stub has no attitude dynamics, so it always reports
    roll = pitch = 0 explicitly; yaw equals `Pose.yaw_deg`.
    """

    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

    def is_finite(self) -> bool:
        return all(
            _is_number(v) and math.isfinite(v)
            for v in (self.roll_deg, self.pitch_deg, self.yaw_deg)
        )

    def to_dict(self) -> Dict[str, float]:
        return {"roll_deg": self.roll_deg, "pitch_deg": self.pitch_deg, "yaw_deg": self.yaw_deg}


class FlightMode(str, Enum):
    """What the vehicle is doing, coarse-grained like PX4's nav_state.

    Derived by Motion from the active command; on PX4 it is read from
    `/fmu/out/vehicle_status.nav_state`.
    """

    DISARMED = "disarmed"
    ON_GROUND = "on_ground"          # armed, grounded, nothing flying it
    TAKEOFF = "takeoff"
    POSITION = "position"            # flying to a position target
    VELOCITY = "velocity"            # flying a velocity setpoint
    HOLD = "hold"                    # hover / stop / expired or boundary hold
    LAND = "land"                    # land or descend to dock
    EMERGENCY_LAND = "emergency_land"


class MotionCommandType(str, Enum):
    """Every command kind Motion understands. One-to-one with MotionStub's
    public command methods; the string values are what goes in the logs."""

    ARM = "ARM"
    DISARM = "DISARM"
    TAKEOFF = "TAKEOFF"
    MOVE_TO = "MOVE_TO"
    HOVER = "HOVER"
    SET_YAW = "SET_YAW"
    SET_ORIENTATION = "SET_ORIENTATION"
    TRACK_TARGET = "TRACK_TARGET"
    SET_VELOCITY = "SET_VELOCITY"
    GOTO_DOCK_APPROACH = "GOTO_DOCK_APPROACH"
    DESCEND_TO_DOCK = "DESCEND_TO_DOCK"
    LAND = "LAND"
    EMERGENCY_LAND = "EMERGENCY_LAND"
    STOP = "STOP"


#: Kinds the coordinator may send through `MotionStub.submit()`.
#: SET_ORIENTATION is not one: roll/pitch are not part of this milestone.
CONTRACT_KINDS = frozenset(MotionCommandType) - {MotionCommandType.SET_ORIENTATION}

#: Kinds that need a target pose in the command.
TARGET_KINDS = frozenset({
    MotionCommandType.TAKEOFF,           # only target.z is used (climb in place)
    MotionCommandType.MOVE_TO,
    MotionCommandType.TRACK_TARGET,      # Navigation's proposed tracking pose
    MotionCommandType.SET_YAW,           # only target.yaw_deg is used
    MotionCommandType.GOTO_DOCK_APPROACH,  # the approach point itself
    MotionCommandType.DESCEND_TO_DOCK,   # the pad pose
})

#: Kinds that need a `Velocity` setpoint (and no target pose) in the command.
#: Not a lifecycle kind, so `expires_ns` is mandatory: the expiry is the
#: dead-man timer that turns a stale velocity into a hold.
VELOCITY_KINDS = frozenset({MotionCommandType.SET_VELOCITY})

#: Kinds that move the vehicle horizontally, so Motion only accepts them in
#: the air (`not_airborne` otherwise). Only TAKEOFF leaves the ground: a
#: grounded drone must never lift off on a MOVE_TO and skip the launch checks.
AIRBORNE_ONLY_KINDS = frozenset({
    MotionCommandType.MOVE_TO,
    MotionCommandType.TRACK_TARGET,
    MotionCommandType.GOTO_DOCK_APPROACH,
    MotionCommandType.SET_VELOCITY,
})

#: Entry actions / lifecycle steps (README §3). These may omit `expires_ns`;
#: every other kind must carry one and be renewed by the coordinator.
LIFECYCLE_KINDS = frozenset({
    MotionCommandType.ARM,
    MotionCommandType.DISARM,
    MotionCommandType.TAKEOFF,
    MotionCommandType.LAND,
    MotionCommandType.EMERGENCY_LAND,
})

#: Kinds whose job is to put the vehicle on the ground.
LANDING_KINDS = frozenset({
    MotionCommandType.LAND,
    MotionCommandType.DESCEND_TO_DOCK,
    MotionCommandType.EMERGENCY_LAND,
})


class Reason:
    """Stable reason strings for acks, feedback and log decisions.

    Compare against these constants, never against free text. Optional
    human-readable detail goes in a separate `detail` field.
    """

    # accepted
    ACCEPTED = "accepted"
    RENEWED = "renewed"
    ALREADY_EMERGENCY_LANDING = "already_emergency_landing"
    # rejected -- malformed input
    INVALID_COMMAND = "invalid_command"
    UNSUPPORTED_SCHEMA = "unsupported_schema"
    UNSUPPORTED_FRAME = "unsupported_frame"
    UNSUPPORTED_KIND = "unsupported_kind"
    MISSING_TARGET = "missing_target"
    MISSING_EXPIRY = "missing_expiry"
    FUTURE_TIMESTAMP = "future_timestamp"
    COMMAND_ID_REUSED = "command_id_reused"
    # rejected -- not executable right now
    COMMAND_EXPIRED = "command_expired"
    NOT_ARMED = "not_armed"
    AIRBORNE = "airborne"
    EMERGENCY_DESCENT_ACTIVE = "emergency_descent_active"
    TARGET_OUTSIDE_LIMITS = "target_outside_limits"
    MISSING_VELOCITY = "missing_velocity"
    VELOCITY_OUTSIDE_LIMITS = "velocity_outside_limits"
    NOT_AIRBORNE = "not_airborne"
    # velocity boundary holds (feedback `status_reason`, log decision)
    GEOFENCE_BOUNDARY = "geofence_boundary"
    ALTITUDE_CEILING_BOUNDARY = "altitude_ceiling_boundary"
    ALTITUDE_FLOOR_BOUNDARY = "altitude_floor_boundary"
    # DroneState.validate()
    INVALID_STATE = "invalid_state"
    INVALID_TIMESTAMP = "invalid_timestamp"
    MISSING_FIELD = "missing_field"
    INCONSISTENT_STATE = "inconsistent_state"
    # feedback / log -- Motion acted on its own
    #: Held in expired_hold past `MotionLimits.comm_loss_land_timeout_ns`
    #: with no new accepted command, so Motion started a landing in place.
    COMM_LOSS_LANDING = "comm_loss_landing"


class CommandStatus(str, Enum):
    """Execution status of the active command, reported in feedback.

    Deliberately separate states, because the README says acceptance,
    arrival, settled landing and disarming are different facts:
    """

    NONE = "none"                    # nothing active (e.g. on the pad, disarmed)
    IN_PROGRESS = "in_progress"      # accepted, still moving toward the target
    ARRIVED = "arrived"              # within arrival tolerance -- NOT "landed"
    LANDED = "landed"                # landing command settled exactly at z = 0
    EXPIRED_HOLD = "expired_hold"    # command expired; Motion replaced it with a hold
    BOUNDARY_HOLD = "boundary_hold"  # velocity reached a limit; Motion stopped there and holds
    COMM_LOSS_LANDING = "comm_loss_landing"  # held too long with no coordinator; landing in place
    EMERGENCY_DESCENT = "emergency_descent"


# --------------------------------------------------------------------------
# Command / acknowledgement / feedback
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Command:
    """One command from the coordinator (after Safety), README §4.

    `expires_ns` is required for every kind except `LIFECYCLE_KINDS`. The
    coordinator renews a tracking/hold/docking target by re-sending the
    *same* `command_id` with a later `expires_ns`; Motion treats that as a
    renewal (no restart, no new log line) as long as kind and target match.

    `max_horizontal_speed_mps` / `max_vertical_speed_mps` can only lower the
    configured limits, never raise them.

    `velocity` is used only by `SET_VELOCITY` (which must not carry a
    `target`); every other kind must leave it None. It is the last field so
    existing positional/keyword construction is unaffected.
    """

    command_id: str
    kind: MotionCommandType
    mission_state: str
    issued_ns: int
    expires_ns: Optional[int]
    source: str
    reason: str
    target: Optional[Pose] = None
    frame: str = FRAME_LOCAL_ENU
    max_horizontal_speed_mps: Optional[float] = None
    max_vertical_speed_mps: Optional[float] = None
    schema_version: int = SCHEMA_VERSION
    velocity: Optional[Velocity] = None

    def validate(self) -> Optional[str]:
        """Return a `Reason` if the command is malformed, else None.

        Only checks the command on its own; whether it can run *now*
        (armed, expired, inside limits) is MotionStub's call.
        """
        if self.schema_version != SCHEMA_VERSION:
            return Reason.UNSUPPORTED_SCHEMA
        if not isinstance(self.command_id, str) or not self.command_id:
            return Reason.INVALID_COMMAND
        for text in (self.mission_state, self.source, self.reason):
            if not isinstance(text, str):
                return Reason.INVALID_COMMAND
        try:
            kind = MotionCommandType(self.kind)
        except ValueError:
            return Reason.UNSUPPORTED_KIND
        if kind not in CONTRACT_KINDS:
            return Reason.UNSUPPORTED_KIND
        if self.frame != FRAME_LOCAL_ENU:
            return Reason.UNSUPPORTED_FRAME
        if not _is_int(self.issued_ns) or self.issued_ns < 0:
            return Reason.INVALID_COMMAND
        if self.expires_ns is None:
            if kind not in LIFECYCLE_KINDS:
                return Reason.MISSING_EXPIRY
        elif not _is_int(self.expires_ns) or self.expires_ns <= self.issued_ns:
            return Reason.INVALID_COMMAND
        if kind in TARGET_KINDS:
            if self.target is None:
                return Reason.MISSING_TARGET
            if not isinstance(self.target, Pose) or not self.target.is_finite():
                return Reason.INVALID_COMMAND
        if kind in VELOCITY_KINDS:
            if self.velocity is None:
                return Reason.MISSING_VELOCITY
            if not isinstance(self.velocity, Velocity) or not self.velocity.is_finite():
                return Reason.INVALID_COMMAND
            if self.target is not None:
                return Reason.INVALID_COMMAND  # position + velocity is ambiguous
        elif self.velocity is not None:
            return Reason.INVALID_COMMAND      # a velocity on a non-velocity kind
        for cap in (self.max_horizontal_speed_mps, self.max_vertical_speed_mps):
            if cap is not None and not (_is_number(cap) and math.isfinite(cap) and cap > 0.0):
                return Reason.INVALID_COMMAND
        return None

    def same_request_as(self, other: "Command") -> bool:
        """True if `other` asks for exactly the same thing (renewal check)."""
        return (
            MotionCommandType(self.kind) == MotionCommandType(other.kind)
            and self.target == other.target
            and self.velocity == other.velocity
            and self.max_horizontal_speed_mps == other.max_horizontal_speed_mps
            and self.max_vertical_speed_mps == other.max_vertical_speed_mps
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "command_id": self.command_id,
            "kind": _enum_value(self.kind),
            "mission_state": self.mission_state,
            "target": self.target.to_dict() if isinstance(self.target, Pose) else None,
            "velocity": self.velocity.to_dict() if isinstance(self.velocity, Velocity) else None,
            "frame": self.frame,
            "max_horizontal_speed_mps": self.max_horizontal_speed_mps,
            "max_vertical_speed_mps": self.max_vertical_speed_mps,
            "issued_ns": self.issued_ns,
            "expires_ns": self.expires_ns,
            "source": self.source,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class CommandAck:
    """Motion's immediate answer to one `Command`.

    `accepted=True` means "I am now executing this", not "done". Progress
    toward the target is reported later through `DroneState` under the
    same `command_id`. `active_command_id` is whatever Motion is executing
    after handling this command -- on a rejection it is the *previous*
    command, so the coordinator can see what is still flying.
    """

    command_id: str
    accepted: bool
    reason: str
    timestamp_ns: int
    kind: Optional[str] = None
    active_command_id: Optional[str] = None
    detail: Optional[str] = None
    schema_version: int = SCHEMA_VERSION

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "command_id": self.command_id,
            "kind": self.kind,
            "accepted": self.accepted,
            "reason": self.reason,
            "detail": self.detail,
            "active_command_id": self.active_command_id,
            "timestamp_ns": self.timestamp_ns,
        }


@dataclass(frozen=True)
class DroneState:
    """The DroneState contract: the `vehicle` section of the snapshot (README §4).

    Purpose: Motion's single report of the vehicle's *estimated* state plus
    its progress on the active command, published once per coordinator tick
    (0.1 s) after `MotionStub.step()`. Documented in motion_engine/README.md
    ("DroneState contract"). `VehicleFeedback` is an alias of this class, so
    existing imports, `isinstance` checks and keyword construction keep
    working.

    Estimated state (what the vehicle is doing): `pose` (position + yaw),
    `velocity`, `orientation`, `frame`, `flight_mode`, `armed`, `airborne`,
    `grounded`, `timestamp_ns`, `valid`. Commanded state (what it was asked
    to do) is kept separate: `active_command_id`, `active_kind`, `target`,
    `target_velocity`, `status`, `status_reason` and the error fields.

    Arrival and landing are separate on purpose (README §8): `arrived` can
    be true while still airborne (e.g. 0.2 m above the pad), so docking
    must use `landing_settled` + `grounded`, never `arrived`, to confirm
    touchdown.

    The fields added for the DroneState contract (`velocity`, `orientation`,
    `flight_mode`, `target_velocity`, `status_reason`) default so that older
    keyword construction still works; `validate()` then reports the missing
    `velocity` / `flight_mode` instead of a silently "safe" value.
    `orientation=None` is derived from `pose.yaw_deg` with roll = pitch = 0.
    """

    timestamp_ns: int
    pose: Pose
    armed: bool
    airborne: bool
    grounded: bool
    emergency_descent: bool
    landing_settled: bool
    active_command_id: Optional[str]
    active_kind: Optional[str]
    target: Optional[Pose]
    status: CommandStatus
    arrived: bool
    distance_to_target_m: Optional[float]
    horizontal_error_m: Optional[float]
    vertical_error_m: Optional[float]
    yaw_error_deg: Optional[float]
    valid: bool = True
    reason: Optional[str] = None
    source_id: str = MOTION_SOURCE_ID
    synthetic: bool = True
    schema_version: int = SCHEMA_VERSION
    # -- DroneState additions (all optional for backward compatibility) --
    velocity: Optional[Velocity] = None
    orientation: Optional[Orientation] = None
    frame: str = FRAME_LOCAL_ENU
    flight_mode: Optional[FlightMode] = None
    target_velocity: Optional[Velocity] = None
    status_reason: Optional[str] = None

    def __post_init__(self) -> None:
        if self.orientation is None and isinstance(self.pose, Pose):
            object.__setattr__(
                self, "orientation", Orientation(0.0, 0.0, self.pose.yaw_deg)
            )

    @property
    def position(self) -> Pose:
        """Same as `pose`; the name START_HERE's vehicle-state table uses."""
        return self.pose

    def validate(self) -> Optional[str]:
        """Return a `Reason` if this state is malformed or self-contradictory.

        Mirrors `Command.validate()`: never raises, returns the first problem
        found, None when the state is usable.
        """
        if self.schema_version != SCHEMA_VERSION:
            return Reason.UNSUPPORTED_SCHEMA
        if not _is_int(self.timestamp_ns) or self.timestamp_ns < 0:
            return Reason.INVALID_TIMESTAMP
        if self.frame != FRAME_LOCAL_ENU:
            return Reason.UNSUPPORTED_FRAME
        if self.velocity is None or self.flight_mode is None or self.status is None:
            return Reason.MISSING_FIELD
        if not isinstance(self.pose, Pose) or not self.pose.is_finite():
            return Reason.INVALID_STATE
        if not isinstance(self.velocity, Velocity) or not self.velocity.is_finite():
            return Reason.INVALID_STATE
        if not isinstance(self.orientation, Orientation) or not self.orientation.is_finite():
            return Reason.INVALID_STATE
        try:
            mode = FlightMode(self.flight_mode)
            CommandStatus(self.status)
        except ValueError:
            return Reason.INVALID_STATE
        bools = (self.armed, self.airborne, self.grounded, self.emergency_descent,
                 self.landing_settled, self.arrived, self.valid, self.synthetic)
        if not all(isinstance(b, bool) for b in bools):
            return Reason.INVALID_STATE  # strings used as booleans are invalid
        for opt in (self.target, ):
            if opt is not None and (not isinstance(opt, Pose) or not opt.is_finite()):
                return Reason.INVALID_STATE
        if self.target_velocity is not None and (
            not isinstance(self.target_velocity, Velocity) or not self.target_velocity.is_finite()
        ):
            return Reason.INVALID_STATE
        for num in (self.distance_to_target_m, self.horizontal_error_m,
                    self.vertical_error_m, self.yaw_error_deg):
            if num is not None and not (_is_number(num) and math.isfinite(num)):
                return Reason.INVALID_STATE
        # Consistency between flags.
        if self.airborne == self.grounded:
            return Reason.INCONSISTENT_STATE
        if self.landing_settled and not self.grounded:
            return Reason.INCONSISTENT_STATE
        if self.airborne and not self.armed:
            return Reason.INCONSISTENT_STATE
        if not self.armed and mode != FlightMode.DISARMED:
            return Reason.INCONSISTENT_STATE
        if self.armed and mode == FlightMode.DISARMED:
            return Reason.INCONSISTENT_STATE
        if self.emergency_descent != (mode == FlightMode.EMERGENCY_LAND):
            return Reason.INCONSISTENT_STATE
        if abs(_wrap_180(self.orientation.yaw_deg - self.pose.yaw_deg)) > 1e-6:
            return Reason.INCONSISTENT_STATE
        if not self.valid and not self.reason:
            return Reason.MISSING_FIELD  # an invalid state must say why
        return None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "timestamp_ns": self.timestamp_ns,
            "valid": self.valid,
            "reason": self.reason,
            "source_id": self.source_id,
            "synthetic": self.synthetic,
            "frame": self.frame,
            "pose": self.pose.to_dict(),
            "velocity": self.velocity.to_dict() if isinstance(self.velocity, Velocity) else None,
            "orientation": (
                self.orientation.to_dict() if isinstance(self.orientation, Orientation) else None
            ),
            "flight_mode": _enum_value(self.flight_mode),
            "armed": self.armed,
            "airborne": self.airborne,
            "grounded": self.grounded,
            "emergency_descent": self.emergency_descent,
            "landing_settled": self.landing_settled,
            "active_command_id": self.active_command_id,
            "active_kind": self.active_kind,
            "target": self.target.to_dict() if self.target is not None else None,
            "target_velocity": (
                self.target_velocity.to_dict() if self.target_velocity is not None else None
            ),
            "status": _enum_value(self.status),
            "status_reason": self.status_reason,
            "arrived": self.arrived,
            "distance_to_target_m": self.distance_to_target_m,
            "horizontal_error_m": self.horizontal_error_m,
            "vertical_error_m": self.vertical_error_m,
            "yaw_error_deg": self.yaw_error_deg,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "DroneState":
        """Inverse of `to_dict()` (e.g. for the README example payload).

        Does not validate; call `validate()` on the result. Raises KeyError /
        TypeError / ValueError on a structurally broken payload.
        """
        def pose(v: Optional[Dict[str, Any]]) -> Optional[Pose]:
            return None if v is None else Pose(v["x"], v["y"], v["z"], v["yaw_deg"])

        def vel(v: Optional[Dict[str, Any]]) -> Optional[Velocity]:
            return None if v is None else Velocity(v["vx"], v["vy"], v["vz"], v.get("yaw_rate_deg_s"))

        o = d.get("orientation")
        mode = d.get("flight_mode")
        return cls(
            timestamp_ns=d["timestamp_ns"],
            pose=pose(d["pose"]),
            armed=d["armed"],
            airborne=d["airborne"],
            grounded=d["grounded"],
            emergency_descent=d["emergency_descent"],
            landing_settled=d["landing_settled"],
            active_command_id=d["active_command_id"],
            active_kind=d["active_kind"],
            target=pose(d["target"]),
            status=CommandStatus(d["status"]),
            arrived=d["arrived"],
            distance_to_target_m=d["distance_to_target_m"],
            horizontal_error_m=d["horizontal_error_m"],
            vertical_error_m=d["vertical_error_m"],
            yaw_error_deg=d["yaw_error_deg"],
            valid=d["valid"],
            reason=d["reason"],
            source_id=d["source_id"],
            synthetic=d["synthetic"],
            schema_version=d["schema_version"],
            velocity=vel(d.get("velocity")),
            orientation=None if o is None else Orientation(o["roll_deg"], o["pitch_deg"], o["yaw_deg"]),
            frame=d.get("frame", FRAME_LOCAL_ENU),
            flight_mode=None if mode is None else FlightMode(mode),
            target_velocity=vel(d.get("target_velocity")),
            status_reason=d.get("status_reason"),
        )


#: Backward-compatible name. `VehicleFeedback is DroneState`.
VehicleFeedback = DroneState


# --------------------------------------------------------------------------
# Clock and logging stand-ins
# --------------------------------------------------------------------------


class SimulationClock:
    """Integer-nanosecond simulation clock (README §3/§4), starting at 0.

    Owned by the coordinator; Motion only reads it. Calling the instance
    returns seconds as a float, so it can be passed straight in as
    `MotionStub(clock=...)` and log timestamps become simulation time --
    which is what makes runs reproducible at any wall-clock speed.
    """

    def __init__(self, start_ns: int = 0) -> None:
        self.timestamp_ns = int(start_ns)

    def advance(self, dt_s: float) -> int:
        self.timestamp_ns += int(round(dt_s * 1e9))
        return self.timestamp_ns

    def __call__(self) -> float:
        return self.timestamp_ns / 1e9


class LogSink(Protocol):
    """What MotionStub needs from a logger: one `log()` call per record.

    Matches the agreed six-column format (timestamp, module, event, state,
    synthetic_truth, decision). The shared implementation is planned for
    `state_machine/logging_format.py`; anything with this method works.
    """

    def log(
        self,
        module: Any,
        event: str,
        *,
        fields: Optional[Dict[str, Any]] = None,
        state: Optional[str] = None,
        truth: Optional[Dict[str, Any]] = None,
        decision: Optional[str] = None,
        t: Optional[float] = None,
    ) -> Any: ...


@dataclass
class LogEntry:
    t: Optional[float]
    module: str
    event: str
    fields: Dict[str, Any] = field(default_factory=dict)
    state: Optional[str] = None
    truth: Optional[Dict[str, Any]] = None
    decision: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "t": self.t,
            "module": self.module,
            "event": self.event,
            "fields": dict(self.fields),
            "state": self.state,
            "synthetic_truth": self.truth,
            "decision": self.decision,
        }


class MemoryLogSink:
    """Minimal in-memory `LogSink`: keeps every record in `.records`.

    Used by the motion tests and usable by anyone wiring Motion up before
    the shared logger lands. Not a replacement for it.
    """

    def __init__(self) -> None:
        self.records: List[LogEntry] = []

    def log(self, module, event, *, fields=None, state=None, truth=None, decision=None, t=None):
        entry = LogEntry(
            t=t,
            module=_enum_value(module),
            event=event,
            fields=dict(fields or {}),
            state=state,
            truth=truth,
            decision=decision,
        )
        self.records.append(entry)
        return entry


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def _is_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _is_int(v: Any) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _wrap_180(deg: float) -> float:
    out = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if out == -180.0 else out


def _enum_value(v: Any) -> Any:
    return v.value if isinstance(v, Enum) else v
