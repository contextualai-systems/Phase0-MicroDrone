"""MotionStub -- the kinematic motion stand-in for the Phase-0 mission layer.

Phase-0 has no flight controller, no motors and no simulator in the mission
loop. What the central coordinator (`state_machine/`, see
INTEGRATION_README.md) needs from "motion" is much smaller than a flight stack: it needs to *issue* a command
("go to X, Y, Z", "hover", "land on the dock") and then *observe progress over
time* -- the drone is 12 m away from the approach point, then 7 m, then it has
arrived -- so that transitions such as DOCKING_APPROACH -> DOCKED happen for a
reason the log can show, not instantly.

So this module is a pure kinematic stub: it holds a synthetic `Pose` and each
`step(dt)` moves it straight toward the active target at a capped speed. No
mass, no drag, no attitude dynamics, no PID. That is deliberate -- the point is
to exercise the *decision* layer, and a kinematic model is predictable enough
that tests can say "at 5 m/s, 20 m takes 4 s" and mean it.

Two ways in:

* `submit(command, now_ns)` / `feedback(now_ns)` / `step(dt, now_ns)` -- the
  shared contract from INTEGRATION_README.md §3-§4. The coordinator sends a
  `contracts.Command`, gets a `CommandAck` back immediately, advances Motion
  exactly once per tick, and reads `DroneState` (alias `VehicleFeedback`).
  Commands carry an ID
  and an expiry; an expired target is replaced by a hold *before* the next
  movement step, so a stale target can never keep flying.
* The direct methods (`arm()`, `move_to()`, `hover()` ...) -- still here for
  scripts and older callers. `submit()` is a thin validating layer on top of
  them, so both paths produce the same motion and the same log lines.

Why every command method carries a `# PHASE-1 HOOK:` comment: this class is the
seam where the real vehicle gets plugged in. Each public command is shaped like
the real PX4 command it will become, and the comment right where the synthetic
behaviour happens names that command, which link carries it, and how the
fields map. Phase-1 replaces the body of each method, not the call sites in the
state machine. `grep -n "PHASE-1 HOOK" motion_engine/motion_stubs.py` is the
to-do list.

Link convention (motion_engine/README.md, "Our split"):

* Operator / golf-cart style commands -- arm, disarm, takeoff, land, goto,
  mode changes -- go over **MAVLink** (`COMMAND_LONG` / `SET_POSITION_TARGET_*`).
* High-rate streamed setpoints (a real offboard loop at 20-50 Hz, which is what
  `track_position` becomes when it is renewed every tick) go over **uXRCE-DDS**:
  `/fmu/in/trajectory_setpoint` paired with `/fmu/in/offboard_control_mode`.
* Pose/state comes *back* over uXRCE-DDS (`/fmu/out/vehicle_local_position`).

Coordinate frame (INTEGRATION_README.md §4): local metres, origin =
dock / cart pad at launch, `x` = east, `y` = north, `z` = altitude **up**
(positive). Yaw in degrees, 0 = +x (east), counter-clockwise positive -- i.e.
ENU. PX4's local frame is **NED** `(north, east, down)`, so on the way out
Phase-1 converts:

    position:  (x, y, z)_enu  ->  (y, x, -z)_ned
    yaw:       yaw_ned_rad = radians(wrap_180(90 - yaw_enu_deg))
               (NED yaw is measured from north, clockwise positive)

and on the way in (vehicle_local_position) the inverse:
`x = ned.y, y = ned.x, z = -ned.z, yaw_enu = 90 - degrees(ned.heading)`.
"""

from __future__ import annotations

import math
import numbers
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

try:
    from motion_engine.contracts import (
        CONTRACT_KINDS,
        LANDING_KINDS,
        VELOCITY_KINDS,
        Command,
        CommandAck,
        CommandStatus,
        DroneState,
        FlightMode,
        LogSink,
        MotionCommandType,
        Orientation,
        Pose,
        Reason,
        Velocity,
        VehicleFeedback,
    )
except ModuleNotFoundError:  # pragma: no cover -- only motion_engine/ on sys.path
    from contracts import (  # type: ignore[no-redef]
        CONTRACT_KINDS,
        LANDING_KINDS,
        VELOCITY_KINDS,
        Command,
        CommandAck,
        CommandStatus,
        DroneState,
        FlightMode,
        LogSink,
        MotionCommandType,
        Orientation,
        Pose,
        Reason,
        Velocity,
        VehicleFeedback,
    )

# `Pose` and `MotionCommandType` moved to contracts.py (they are part of the
# shared contract); they are re-exported here so existing
# `from motion_engine.motion_stubs import Pose` imports keep working.
__all__ = [
    "GROUND_Z_M", "AIRBORNE_THRESHOLD_M", "Pose", "MotionCommandType",
    "MotionCommand", "MotionLimits", "MotionStub", "Velocity", "wrap_180", "wrap_360",
]


#: Local-frame altitude of the ground / dock pad. The origin *is* the dock pad
#: at launch, so ground is z = 0 by construction.
GROUND_Z_M: float = 0.0

#: Above this altitude (relative to ground) the vehicle counts as airborne.
#: Matches the spec's `z > 0.05`; small enough that a vehicle sitting on the
#: pad with a sliver of numeric noise does not read as flying.
AIRBORNE_THRESHOLD_M: float = 0.05

#: Log module name. A plain string so this file has no runtime dependency on
#: the shared logger (planned for `state_machine/logging_format.py`).
_LOG_MODULE: str = "MOTION"

#: Slack on the "remaining error fits in one step -> snap" test. Without it,
#: float accumulation (ten steps of 0.1 s are not exactly 1 s) leaves a
#: 1e-15 m residual that needs one extra tick and never reads as "on target".
_SNAP_EPS_M: float = 1e-9


# --------------------------------------------------------------------------
# Small value types
# --------------------------------------------------------------------------


def _is_real(v: Any) -> bool:
    """Any real number (int, float, numpy scalar, Fraction ...), but not a
    bool -- True is an int in Python, and a bool dt is a caller bug."""
    return isinstance(v, numbers.Real) and not isinstance(v, bool)


def wrap_360(deg: float) -> float:
    """Normalise an angle into [0, 360)."""
    out = deg % 360.0
    # `-1e-18 % 360` is 360.0 in floating point; fold that back to 0.
    return 0.0 if out >= 360.0 else out


def wrap_180(deg: float) -> float:
    """Normalise an angle into (-180, 180]. Used for shortest-direction yaw."""
    out = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if out == -180.0 else out


@dataclass
class MotionCommand:
    """An accepted command, as recorded in `MotionStub.history`.

    `target` is the pose actually flown to -- i.e. *after* geofence clamping --
    so replaying history reproduces the flight. `speed_mps` is the horizontal
    speed cap in effect for this command (0 for commands that do not move).
    """

    type: MotionCommandType
    target: Optional[Pose]
    speed_mps: float
    issued_at: float
    note: str = ""
    #: Contract command ID when issued through `submit()`, else None.
    command_id: Optional[str] = None
    #: True if the geofence changed the requested target (direct API only;
    #: `submit()` rejects instead of clamping).
    clamped: bool = False
    requested: Optional[Pose] = None
    #: The velocity setpoint for SET_VELOCITY (target is None then).
    velocity: Optional[Velocity] = None


@dataclass
class MotionLimits:
    """Kinematic caps and the geofence.

    Horizontal and vertical speed are capped *independently* (like PX4's
    MPC_XY_VEL_MAX vs MPC_Z_VEL_MAX_UP/DN): a diagonal climb does not borrow
    vertical speed from the horizontal budget. Vertical is kept slower because
    an altitude mistake near the cart is the one that hurts.
    """

    max_horizontal_speed_mps: float = 5.0
    max_vertical_speed_mps: float = 2.0
    max_yaw_rate_dps: float = 90.0
    #: Geofence altitude floor for *flight* commands. Landing commands
    #: (descend_to_dock, land, emergency_land) are exempt -- a floor that
    #: forbade reaching the ground would forbid landing.
    min_altitude_m: float = 0.0
    #: Geofence ceiling (m above the pad).
    max_altitude_m: float = 30.0
    #: Horizontal geofence radius around `home` (m).
    geofence_radius_m: float = 150.0
    arrival_tolerance_m: float = 0.25
    yaw_tolerance_deg: float = 3.0
    #: Synthetic emergency descent rate (m/s). Deliberately gentle: in Phase-0
    #: "emergency" means "the decision layer gave up", not "motors failed".
    emergency_descent_mps: float = 1.0
    #: Link-loss failsafe (simulation ns): after this long in `expired_hold`
    #: with no new accepted command, an airborne, armed vehicle lands in
    #: place. Shared with Safety as `COMM_LOSS_LAND_TIMEOUT_NS`.
    comm_loss_land_timeout_ns: int = 10_000_000_000

    @classmethod
    def from_config(cls, source: Any = None) -> "MotionLimits":
        """Build the limits from Safety's config, the single source of truth.

        INTEGRATION_README §4 says Safety and Motion must receive the same
        configured limits. `safety_layer/config.py` owns those numbers, so the
        coordinator builds Motion's limits from it instead of relying on the
        defaults above. `test_limits_config.py` asserts the defaults still
        equal this, so a change to one side without the other fails CI.

        `source` is any object with the config's attribute names (a module or
        a namespace); `None` means `safety_layer.config`. Fields the config
        does not define (`min_altitude_m`, and `emergency_descent_mps` /
        `comm_loss_land_timeout_ns` unless the config adds
        `EMERGENCY_DESCENT_MPS` / `COMM_LOSS_LAND_TIMEOUT_NS`) keep the
        defaults above.
        `yaw_tolerance_deg` comes from `DOCK_YAW_TOLERANCE_DEG`, the one yaw
        tolerance in the README §5 table.
        """
        if source is None:
            # Imported lazily so motion_stubs has no import-time dependency on
            # safety_layer (safety_layer already imports motion contracts).
            from safety_layer import config as source  # type: ignore[no-redef]
        return cls(
            max_horizontal_speed_mps=float(source.MAX_HORIZONTAL_SPEED_MPS),
            max_vertical_speed_mps=float(source.MAX_VERTICAL_SPEED_MPS),
            max_yaw_rate_dps=float(source.MAX_YAW_RATE_DPS),
            max_altitude_m=float(source.ALTITUDE_CEILING_M),
            geofence_radius_m=float(source.GEOFENCE_RADIUS_M),
            arrival_tolerance_m=float(source.POSITION_ARRIVAL_TOLERANCE_M),
            yaw_tolerance_deg=float(source.DOCK_YAW_TOLERANCE_DEG),
            emergency_descent_mps=float(
                getattr(source, "EMERGENCY_DESCENT_MPS", cls.emergency_descent_mps)
            ),
            # Defined in Safety's config; the getattr fallback keeps any other
            # config source working, as for EMERGENCY_DESCENT_MPS above.
            comm_loss_land_timeout_ns=int(
                getattr(source, "COMM_LOSS_LAND_TIMEOUT_NS", cls.comm_loss_land_timeout_ns)
            ),
        )


# --------------------------------------------------------------------------
# The stub
# --------------------------------------------------------------------------


class MotionStub:
    """Kinematic stand-in for the flight controller.

    Usage:
        motion = MotionStub(logger)
        motion.arm(); motion.takeoff(12.0)
        while not motion.at_target():
            motion.step(0.1)
        motion.move_to(20.0, 5.0, 12.0, yaw_deg=90.0)

    Every command returns True when accepted and False when refused (not
    armed, airborne disarm, emergency landing in progress). Refusals are
    logged with a `decision=` explaining why, so a mission log reads as a
    complete account of what was asked and what actually happened.

    Contract usage (one coordinator tick, INTEGRATION_README.md §3):
        ack = motion.submit(command, now_ns)     # accepted / rejected + reason
        motion.step(0.1, now_ns=now_ns)          # exactly once per tick
        fb = motion.feedback(now_ns)             # pose, armed/airborne, progress

    Pass the coordinator's simulation clock as `clock` (e.g.
    `contracts.SimulationClock`) so log timestamps are simulation time too.
    """

    def __init__(
        self,
        logger: Optional[LogSink] = None,
        limits: Optional[MotionLimits] = None,
        home: Pose = Pose(0.0, 0.0, 0.0, 0.0),
        clock: Callable[[], float] = time.time,
    ) -> None:
        self._logger = logger
        self.limits = limits if limits is not None else MotionLimits()
        self.home = home
        self._clock = clock

        self._pose: Pose = home
        self._target: Optional[Pose] = None
        self._armed: bool = False
        self._active: Optional[MotionCommand] = None
        self._history: List[MotionCommand] = []

        # Speed caps for the active command (set per command).
        self._h_speed: float = self.limits.max_horizontal_speed_mps
        self._v_speed: float = self.limits.max_vertical_speed_mps

        # "arrived" is logged once per target, not once per tick.
        self._arrival_logged: bool = True
        self._emergency: bool = False

        # Optional context stamped onto every MOTION record (state/truth
        # columns of the agreed log format). The mission loop may set this
        # each tick; MotionStub never interprets it.
        self._log_state: Optional[str] = None
        self._log_truth: Optional[Dict[str, Any]] = None

        # Contract bookkeeping (submit / feedback / step(now_ns)).
        self._now_ns: int = 0
        self._active_contract: Optional[Command] = None
        self._dispatching: Optional[Command] = None
        self._seen_ids: Set[str] = set()
        self._expired_hold: bool = False
        # Simulation time the current expired hold began (None = not holding
        # after an expiry), and whether Motion is landing on its own because
        # the hold outlasted `limits.comm_loss_land_timeout_ns`.
        self._expired_since_ns: Optional[int] = None
        self._comm_loss_landing: bool = False

        # Velocity mode: a SET_VELOCITY setpoint is flown instead of a target
        # pose (`_target` is None while this is set). `_boundary_reason` is the
        # stable Reason of the limit a velocity run stopped at, if any.
        self._velocity_sp: Optional[Velocity] = None
        self._boundary_reason: Optional[str] = None
        # Estimated velocity: displacement over the last good step.
        self._est_velocity: Velocity = Velocity(0.0, 0.0, 0.0, 0.0)

    # -- read-only state ----------------------------------------------

    @property
    def pose(self) -> Pose:
        return self._pose

    @property
    def target(self) -> Optional[Pose]:
        return self._target

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def airborne(self) -> bool:
        return self._pose.z > GROUND_Z_M + AIRBORNE_THRESHOLD_M

    @property
    def active_command(self) -> Optional[MotionCommand]:
        return self._active

    @property
    def history(self) -> List[MotionCommand]:
        return list(self._history)

    @property
    def velocity_setpoint(self) -> Optional[Velocity]:
        """The SET_VELOCITY setpoint being flown, or None in position mode."""
        return self._velocity_sp

    @property
    def velocity(self) -> Velocity:
        """Estimated velocity: displacement over the last step (m/s, deg/s)."""
        return self._est_velocity

    @property
    def comm_loss_landing(self) -> bool:
        """True from the comm-loss escalation until the next accepted command."""
        return self._comm_loss_landing

    @property
    def emergency_landing(self) -> bool:
        """True from `emergency_land()` until touchdown + auto-disarm."""
        return self._emergency

    def set_log_context(
        self, *, state: Optional[str] = None, truth: Optional[Dict[str, Any]] = None
    ) -> None:
        """Attach mission state / synthetic truth to subsequent MOTION records."""
        self._log_state = state
        self._log_truth = truth

    def at_target(self, tol: Optional[float] = None) -> bool:
        """True when position is within `tol` (default arrival tolerance) and
        yaw within `limits.yaw_tolerance_deg` of the active target."""
        if self._target is None:
            return False
        tol = self.limits.arrival_tolerance_m if tol is None else tol
        yaw_err = abs(wrap_180(self._target.yaw_deg - self._pose.yaw_deg))
        return (
            self._pose.distance_to(self._target) <= tol
            and yaw_err <= self.limits.yaw_tolerance_deg
        )

    @property
    def grounded(self) -> bool:
        return not self.airborne

    @property
    def landing_settled(self) -> bool:
        """True only when a landing has actually finished on the ground.

        Requires z exactly at ground (the stub snaps onto targets), the
        landing target's x/y reached too, *and* that the last thing Motion did
        was a landing (or the disarm after one). `at_target()` is not enough:
        its arrival tolerance is true 0.25 m above the pad
        (INTEGRATION_README.md §8). x/y matters because horizontal and
        vertical speed are capped separately: a descent that starts off the
        pad can reach z = 0 while still sliding toward it, and that is not a
        finished landing.
        """
        if abs(self._pose.z - GROUND_Z_M) > _SNAP_EPS_M or self._active is None:
            return False
        if self._active.type not in LANDING_KINDS | {MotionCommandType.DISARM}:
            return False
        t = self._target
        if t is None:
            return True
        return (
            abs(t.z - GROUND_Z_M) <= _SNAP_EPS_M
            and self._pose.horizontal_distance_to(t) <= _SNAP_EPS_M
        )

    # -- shared contract (INTEGRATION_README.md §3-§4) ---------------------

    def submit(self, command: Command, now_ns: int) -> CommandAck:
        """Validate and execute one coordinator command; return the ack.

        Accepting replaces the active target immediately, i.e. before the
        next `step()`. Rejecting leaves the previous command active -- the
        coordinator must send a replacement (normally a hold), and an
        un-renewed previous command still expires on its own.

        Re-sending the active `command_id` with the same kind/target and a
        later expiry is a renewal: acknowledged with reason `renewed`, no
        restart and no new log line. Reusing an old ID for anything else is
        rejected.

        Unlike the direct methods, a target outside the limits is rejected
        (`target_outside_limits`) instead of clamped: an unexpected clamp is
        a command failure the coordinator must replan around (README §4).
        """
        # PHASE-1 HOOK: this is where a validated contract Command turns into
        #   the PX4 traffic listed on each direct method below (MAVLink
        #   COMMAND_LONG for arm/takeoff/land, XRCE /fmu/in/trajectory_setpoint
        #   + /fmu/in/offboard_control_mode for streamed targets). The ack must
        #   then wait for COMMAND_ACK (MAVLink) or the first matching
        #   /fmu/out/vehicle_status / vehicle_control_mode update instead of
        #   being immediate, and the expiry becomes the offboard stream
        #   timeout (COM_OF_LOSS_T) on the vehicle side.
        self._now_ns = max(self._now_ns, now_ns) if isinstance(now_ns, int) else self._now_ns
        kind_value = getattr(command, "kind", None)
        name = kind_value.value.lower() if isinstance(kind_value, MotionCommandType) else str(kind_value).lower()
        cmd_id = getattr(command, "command_id", None)

        def reject(reason: str, detail: Optional[str] = None) -> CommandAck:
            self._log(name, {"command_id": cmd_id}, decision=f"rejected_{reason}")
            return self._ack(command, False, reason, now_ns, detail)

        if not isinstance(command, Command):
            return reject(Reason.INVALID_COMMAND, "not a contracts.Command")
        if not isinstance(now_ns, int) or isinstance(now_ns, bool):
            return reject(Reason.INVALID_COMMAND, "now_ns must be an int")
        problem = command.validate()
        if problem is not None:
            return reject(problem)
        kind = MotionCommandType(command.kind)
        if command.issued_ns > now_ns:
            return reject(Reason.FUTURE_TIMESTAMP)
        if (
            kind != MotionCommandType.EMERGENCY_LAND
            and command.expires_ns is not None
            and now_ns >= command.expires_ns
        ):
            return reject(Reason.COMMAND_EXPIRED)

        active = self._active_contract
        if active is not None and command.command_id == active.command_id:
            if not command.same_request_as(active):
                return reject(Reason.COMMAND_ID_REUSED, "same id, different request")
            self._active_contract = command  # new expiry; no restart, no log
            return self._ack(command, True, Reason.RENEWED, now_ns)
        if command.command_id in self._seen_ids:
            return reject(Reason.COMMAND_ID_REUSED)

        if self._emergency:
            if kind == MotionCommandType.EMERGENCY_LAND:
                self._seen_ids.add(command.command_id)
                return self._ack(command, True, Reason.ALREADY_EMERGENCY_LANDING, now_ns)
            return reject(Reason.EMERGENCY_DESCENT_ACTIVE)
        if not self._armed and kind not in (MotionCommandType.ARM, MotionCommandType.DISARM):
            return reject(Reason.NOT_ARMED)
        if kind == MotionCommandType.DISARM and self.airborne:
            return reject(Reason.AIRBORNE)

        if kind in VELOCITY_KINDS:
            if not self.airborne:
                return reject(Reason.NOT_AIRBORNE, "take off before flying a velocity")
            problem = self._velocity_limit_problem(
                command.velocity, command.max_horizontal_speed_mps, command.max_vertical_speed_mps
            )
            if problem is not None:
                return reject(Reason.VELOCITY_OUTSIDE_LIMITS, problem)

        requested = self._requested_pose(kind, command.target)
        if requested is not None:
            landing = kind == MotionCommandType.DESCEND_TO_DOCK
            _, clamped = self._clamp(requested, landing=landing)
            if clamped:
                return reject(Reason.TARGET_OUTSIDE_LIMITS)

        self._seen_ids.add(command.command_id)
        self._dispatching = command
        try:
            ok = self._dispatch(kind, command.target, command.velocity)
        finally:
            self._dispatching = None
        if not ok:  # pragma: no cover -- every refusal is pre-checked above
            return self._ack(command, False, Reason.INVALID_COMMAND, now_ns, "refused by stub")
        if command.max_horizontal_speed_mps is not None:
            self._h_speed = min(self._h_speed, command.max_horizontal_speed_mps)
        if command.max_vertical_speed_mps is not None:
            self._v_speed = min(self._v_speed, command.max_vertical_speed_mps)
        self._active_contract = command
        return self._ack(command, True, Reason.ACCEPTED, now_ns)

    def feedback(self, now_ns: Optional[int] = None) -> DroneState:
        """The `vehicle` snapshot section (the DroneState contract).

        Read-only: never changes state.
        """
        # PHASE-1 HOOK: on hardware every field here is read, not computed:
        #   pose from XRCE /fmu/out/vehicle_local_position (NED -> ENU as in
        #   step()), armed from /fmu/out/vehicle_status.arming_state, grounded
        #   and landing_settled from /fmu/out/vehicle_land_detected (landed,
        #   maybe_landed), and `valid` from xy_valid / z_valid plus message age.
        #   velocity from vehicle_local_position vx/vy/vz (NED -> ENU:
        #   vx_enu = vy_ned, vy_enu = vx_ned, vz_enu = -vz_ned; check v_xy_valid /
        #   v_z_valid), orientation from /fmu/out/vehicle_attitude q (FRD/NED ->
        #   roll, -pitch, 90 - yaw for FLU/ENU), and flight_mode from
        #   /fmu/out/vehicle_status.nav_state (OFFBOARD, AUTO_LOITER, AUTO_LAND,
        #   AUTO_TAKEOFF ...) or MAVLink HEARTBEAT custom_mode.
        t = self._target
        p = self._pose
        if self._emergency:
            status = CommandStatus.EMERGENCY_DESCENT
        elif self._comm_loss_landing and not self.landing_settled:
            status = CommandStatus.COMM_LOSS_LANDING
        elif self._expired_hold:
            status = CommandStatus.EXPIRED_HOLD
        elif self._boundary_reason is not None:
            status = CommandStatus.BOUNDARY_HOLD
        elif self.landing_settled:
            status = CommandStatus.LANDED
        elif self._velocity_sp is not None:
            status = CommandStatus.IN_PROGRESS
        elif t is None:
            status = CommandStatus.NONE
        elif self.at_target():
            status = CommandStatus.ARRIVED
        else:
            status = CommandStatus.IN_PROGRESS
        if status == CommandStatus.EXPIRED_HOLD:
            status_reason: Optional[str] = Reason.COMMAND_EXPIRED
        elif status == CommandStatus.BOUNDARY_HOLD:
            status_reason = self._boundary_reason
        elif self._comm_loss_landing:
            # During the descent (`comm_loss_landing`) and after touchdown
            # (`landed`) until the next accepted command, like `reason` below.
            status_reason = Reason.COMM_LOSS_LANDING
        else:
            status_reason = None
        active = self._active
        return DroneState(
            timestamp_ns=self._now_ns if now_ns is None else now_ns,
            pose=p,
            armed=self._armed,
            airborne=self.airborne,
            grounded=self.grounded,
            emergency_descent=self._emergency,
            landing_settled=self.landing_settled,
            active_command_id=active.command_id if active is not None else None,
            active_kind=active.type.value if active is not None else None,
            target=t,
            status=status,
            arrived=self.at_target(),
            distance_to_target_m=p.distance_to(t) if t is not None else None,
            horizontal_error_m=p.horizontal_distance_to(t) if t is not None else None,
            vertical_error_m=(t.z - p.z) if t is not None else None,
            yaw_error_deg=abs(wrap_180(t.yaw_deg - p.yaw_deg)) if t is not None else None,
            velocity=self._est_velocity,
            orientation=Orientation(0.0, 0.0, p.yaw_deg),  # kinematic stub: no roll/pitch
            flight_mode=self._flight_mode(),
            target_velocity=self._velocity_sp,
            status_reason=status_reason,
            # Stays set after touchdown (status `landed`) so a coordinator that
            # comes back can tell this landing was Motion's, not its own.
            reason=Reason.COMM_LOSS_LANDING if self._comm_loss_landing else None,
        )

    # -- arming ---------------------------------------------------------

    def arm(self) -> bool:
        """Arm the motors. Idempotent."""
        if self._refuse_during_emergency("arm"):
            return False
        # PHASE-1 HOOK: MAVLink COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM (400),
        #   param1 = 1 (arm), param2 = 0 (never 21196/force outside the bench).
        #   Arming is operator/golf-cart territory -> MAVLink per README. Wait for
        #   COMMAND_ACK == MAV_RESULT_ACCEPTED and confirm arming_state ==
        #   ARMED on /fmu/out/vehicle_status before returning True; PX4 can
        #   refuse (preflight checks, no position lock) and this stub cannot.
        self._armed = True
        self._accept(MotionCommandType.ARM, None, 0.0, "arm", {})
        return True

    def disarm(self) -> bool:
        """Disarm. Refused while airborne -- disarming in the air is a crash."""
        if self._refuse_during_emergency("disarm"):
            return False
        if self.airborne:
            self._log("disarm", {"z": self._pose.z}, decision="rejected_airborne")
            return False
        # PHASE-1 HOOK: MAVLink COMMAND_LONG MAV_CMD_COMPONENT_ARM_DISARM (400),
        #   param1 = 0 (disarm), param2 = 0. PX4 also auto-disarms after landing
        #   (COM_DISARM_LAND), so on hardware this may already be done; treat
        #   "already disarmed" in /fmu/out/vehicle_status as success.
        self._armed = False
        self._target = None
        self._velocity_sp = None
        self._arrival_logged = True
        self._accept(MotionCommandType.DISARM, None, 0.0, "disarm", {})
        return True

    # -- flight commands ----------------------------------------------

    def takeoff(self, altitude_m: float) -> bool:
        """Climb vertically in place to `altitude_m` (m above the pad)."""
        if not self._precheck("takeoff", {"z": altitude_m}):
            return False
        requested = Pose(self._pose.x, self._pose.y, altitude_m, self._pose.yaw_deg)
        target, clamped = self._clamp(requested)
        # PHASE-1 HOOK: MAVLink COMMAND_LONG MAV_CMD_NAV_TAKEOFF (22).
        #   param4 = yaw (NaN = keep current), param5/6 = lat/lon (NaN = here),
        #   param7 = altitude -- PX4 reads param7 as AMSL, so send
        #   home_amsl + target.z (z is already "up", no NED flip needed for
        #   this field). Alternatively set MIS_TAKEOFF_ALT = target.z and send
        #   with param7 = NaN. Operator command -> MAVLink per README.
        self._set_target(target, self.limits.max_horizontal_speed_mps, self.limits.max_vertical_speed_mps)
        self._accept(
            MotionCommandType.TAKEOFF, target, 0.0, "takeoff",
            {"z": target.z}, clamped=clamped, requested=requested,
        )
        return True

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        yaw_deg: Optional[float] = None,
        speed_mps: Optional[float] = None,
    ) -> bool:
        """Fly straight to (x, y, z). `yaw_deg=None` keeps the commanded heading.

        `speed_mps` lowers (never raises) both speed caps for this command:
        the horizontal cap becomes min(speed, max_h), vertical min(speed, max_v).
        """
        if yaw_deg is None:
            # Keep the heading already commanded (so an in-progress set_yaw
            # is not cancelled), falling back to the current heading.
            yaw = self._target.yaw_deg if self._target is not None else self._pose.yaw_deg
        else:
            yaw = wrap_360(yaw_deg)
        if not self._precheck("move_to", {"x": x, "y": y, "z": z, "yaw": yaw}):
            return False
        requested = Pose(x, y, z, yaw)
        target, clamped = self._clamp(requested)
        h_speed, v_speed = self._speed_caps(speed_mps)
        # PHASE-1 HOOK: MAVLink SET_POSITION_TARGET_LOCAL_NED (#84) for a
        #   one-shot goto from the operator/cart (README: goto -> MAVLink), or,
        #   if this becomes a streamed offboard loop, XRCE
        #   /fmu/in/trajectory_setpoint + /fmu/in/offboard_control_mode
        #   (position=True) published together at >= 2 Hz (PX4 drops out of
        #   offboard if the stream stops).
        #   NED conversion: position = [target.y, target.x, -target.z]
        #   (north, east, down); yaw = radians(wrap_180(90 - target.yaw_deg)).
        #   MAVLink fields: coordinate_frame = MAV_FRAME_LOCAL_NED (1),
        #   type_mask = ignore vx/vy/vz, afx/afy/afz, yaw_rate (0x0DF8);
        #   x = north, y = east, z = down, yaw = NED yaw (rad).
        #   Speed cap -> MAV_CMD_DO_CHANGE_SPEED (178) or MPC_XY_VEL_MAX/MPC_Z_VEL_MAX_*.
        self._set_target(target, h_speed, v_speed)
        self._accept(
            MotionCommandType.MOVE_TO, target, h_speed, "move_to",
            {"x": target.x, "y": target.y, "z": target.z, "yaw": target.yaw_deg},
            clamped=clamped, requested=requested,
        )
        return True

    def hover(self) -> bool:
        """Hold the current pose (target = where the drone is right now)."""
        # PHASE-1 HOOK: PX4 Hold -- MAVLink MAV_CMD_DO_SET_MODE (AUTO.LOITER) or
        #   MAV_CMD_NAV_LOITER_UNLIM (17) at the current position; in offboard,
        #   keep streaming the current pose as a hold setpoint on XRCE
        #   /fmu/in/trajectory_setpoint + /fmu/in/offboard_control_mode.
        #   Details (and why "stop publishing" is not "hold") in _hold().
        return self._hold(MotionCommandType.HOVER, "hover")

    def stop(self) -> bool:
        """Same as hover(), recorded as STOP -- an interrupt, not a plan."""
        # PHASE-1 HOOK: identical to hover() -- MAV_CMD_DO_SET_MODE to PX4 Hold
        #   (AUTO.LOITER) or MAV_CMD_NAV_LOITER_UNLIM over MAVLink; in offboard,
        #   stream the current /fmu/out/vehicle_local_position as the
        #   /fmu/in/trajectory_setpoint. "Stop" must win over any queued goto,
        #   so on hardware also clear any pending mission item.
        return self._hold(MotionCommandType.STOP, "stop")

    def set_yaw(self, yaw_deg: float) -> bool:
        """Rotate in place to `yaw_deg` (ENU, 0 = east, CCW +)."""
        yaw = wrap_360(yaw_deg)
        if not self._precheck("set_yaw", {"yaw": yaw}):
            return False
        base = self._target if self._target is not None else self._pose
        target = Pose(base.x, base.y, base.z, yaw)
        # PHASE-1 HOOK: MAVLink COMMAND_LONG MAV_CMD_CONDITION_YAW (115):
        #   param1 = NED heading deg = wrap_360(90 - yaw_deg), param2 = yaw rate
        #   (deg/s, limits.max_yaw_rate_dps), param3 = 0 (shortest direction),
        #   param4 = 0 (absolute). Caveat: PX4 only honours CONDITION_YAW in
        #   mission/auto modes; in offboard send the same heading as the `yaw`
        #   field (radians(wrap_180(90 - yaw_deg))) of
        #   /fmu/in/trajectory_setpoint with the position held.
        self._set_target(target, self._h_speed, self._v_speed)
        self._accept(MotionCommandType.SET_YAW, target, 0.0, "set_yaw", {"yaw": yaw})
        return True

    def set_orientation(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> bool:
        """Set attitude. Only yaw is simulated; roll/pitch are logged only.

        A kinematic point model has no attitude, and a multicopter cannot hold
        a non-zero roll/pitch without accelerating anyway -- so roll/pitch are
        recorded for the log and otherwise ignored.
        """
        yaw = wrap_360(yaw_deg)
        fields = {"roll": roll_deg, "pitch": pitch_deg, "yaw": yaw}
        if not self._precheck("set_orientation", fields):
            return False
        base = self._target if self._target is not None else self._pose
        target = Pose(base.x, base.y, base.z, yaw)
        # PHASE-1 HOOK: MAVLink SET_ATTITUDE_TARGET (#82) or XRCE
        #   /fmu/in/vehicle_attitude_setpoint + /fmu/in/offboard_control_mode
        #   (attitude=True). Build q_d (w, x, y, z) for body FRD relative to NED:
        #   roll_frd = roll_deg, pitch_frd = -pitch_deg (FLU -> FRD flips pitch),
        #   yaw_ned = 90 - yaw_deg; plus thrust_body[2] = -hover_thrust.
        #   WARNING: attitude mode abandons position hold -- the vehicle drifts.
        #   For "just face this way" use set_yaw() instead; true attitude
        #   control belongs to a gimbal/camera-pointing phase, not the mission.
        decision = None if (roll_deg == 0.0 and pitch_deg == 0.0) else "roll_pitch_not_simulated"
        self._set_target(target, self._h_speed, self._v_speed)
        self._accept(
            MotionCommandType.SET_ORIENTATION, target, 0.0, "set_orientation",
            fields, decision=decision,
        )
        return True

    def track_position(
        self, x: float, y: float, z: float, yaw_deg: Optional[float] = None
    ) -> bool:
        """Follow a tracking target that Navigation already computed.

        Motion is mechanism-only: it never sees a bird. Navigation turns the
        tracking geometry into a pose (`navigation.tracking.standoff_target`)
        and Motion just flies it. Same kinematics as `move_to`, recorded as
        TRACK_TARGET (log event `track_target`) so the log shows *why* the
        drone moved. The old bearing/distance method was removed from Motion
        for exactly that reason.
        """
        yaw = (
            (self._target.yaw_deg if self._target is not None else self._pose.yaw_deg)
            if yaw_deg is None else wrap_360(yaw_deg)
        )
        fields = {"x": x, "y": y, "z": z, "yaw": yaw}
        if not self._precheck("track_target", fields):
            return False
        requested = Pose(x, y, z, yaw)
        target, clamped = self._clamp(requested)
        # PHASE-1 HOOK: streamed every tick -> XRCE /fmu/in/trajectory_setpoint
        #   + /fmu/in/offboard_control_mode (position=True), a streamed
        #   setpoint because it is renewed every mission tick (a low-rate
        #   fallback is MAVLink SET_POSITION_TARGET_LOCAL_NED, same mapping as
        #   move_to). NED conversion: position = [target.y, target.x,
        #   -target.z], yaw = radians(wrap_180(90 - target.yaw_deg)).
        self._set_target(target, self.limits.max_horizontal_speed_mps, self.limits.max_vertical_speed_mps)
        self._accept(
            MotionCommandType.TRACK_TARGET, target, self._h_speed, "track_target",
            {"x": target.x, "y": target.y, "z": target.z, "yaw": target.yaw_deg},
            clamped=clamped, requested=requested,
        )
        return True

    def set_velocity(
        self, vx: float, vy: float, vz: float, yaw_rate_deg_s: Optional[float] = None
    ) -> bool:
        """Fly a velocity setpoint (m/s, local ENU) until replaced or expired.

        `yaw_rate_deg_s=None` keeps the current heading. Motion owns velocity
        control; the setpoint has no target, so each `step()` checks the
        geofence and ceiling/floor itself: if the next step would cross one,
        the drone stops exactly on the boundary and holds there
        (`status = boundary_hold`, `status_reason` names the limit, log
        `velocity_boundary`). That is a reported stop, not a silent clamp.

        Refused (never clamped) when not airborne, non-finite, or faster than
        the limits -- the direct API has no ack, so the refusal is logged.
        """
        fields = {"vx": vx, "vy": vy, "vz": vz, "yaw_rate": yaw_rate_deg_s}
        if not self._precheck("set_velocity", fields):
            return False
        v = Velocity(vx, vy, vz, yaw_rate_deg_s)
        if not v.is_finite():
            self._log("set_velocity", fields, decision="rejected_invalid_command")
            return False
        if not self.airborne:
            self._log("set_velocity", fields, decision=f"rejected_{Reason.NOT_AIRBORNE}")
            return False
        if self._velocity_limit_problem(v, None, None) is not None:
            self._log("set_velocity", fields, decision=f"rejected_{Reason.VELOCITY_OUTSIDE_LIMITS}")
            return False
        # PHASE-1 HOOK: streamed offboard velocity -> XRCE /fmu/in/trajectory_setpoint
        #   with position = [NaN, NaN, NaN], velocity = [vy, vx, -vz] (ENU -> NED:
        #   north, east, down), yaw = NaN and yawspeed = radians(-yaw_rate_deg_s)
        #   (CCW-up ENU -> CW-down NED flips the sign; NaN if None = hold heading),
        #   paired with /fmu/in/offboard_control_mode (velocity=True,
        #   position=False) at >= 2 Hz. MAVLink fallback:
        #   SET_POSITION_TARGET_LOCAL_NED (#84), coordinate_frame =
        #   MAV_FRAME_LOCAL_NED, type_mask = ignore position/accel/yaw (0x05C7, or
        #   0x0DC7 to also ignore yaw_rate). PX4 has no geofence-stop for raw
        #   velocity setpoints in offboard (GF_ACTION is a failsafe, not a stop
        #   at the fence), so the boundary check in step() stays on the
        #   companion side, using vehicle_local_position.
        self._set_target_velocity(v)
        self._accept(
            MotionCommandType.SET_VELOCITY, None, v.horizontal_speed_mps, "set_velocity",
            fields, velocity=v,
        )
        return True

    def goto_dock_approach(self, dock: Pose, approach_altitude_m: float = 5.0) -> bool:
        """Fly to the point `approach_altitude_m` directly above the dock."""
        fields = {"x": dock.x, "y": dock.y, "z": dock.z + approach_altitude_m}
        if not self._precheck("goto_dock_approach", fields):
            return False
        requested = Pose(dock.x, dock.y, dock.z + approach_altitude_m, wrap_360(dock.yaw_deg))
        target, clamped = self._clamp(requested)
        # PHASE-1 HOOK: MAVLink SET_POSITION_TARGET_LOCAL_NED (#84) -- a one-shot
        #   goto, so MAVLink per README -- or XRCE /fmu/in/trajectory_setpoint +
        #   /fmu/in/offboard_control_mode if the docking loop is streamed.
        #   NED conversion: position = [dock.y, dock.x, -(dock.z + approach_alt)],
        #   yaw = radians(wrap_180(90 - dock.yaw_deg)) to line up with the pad.
        #   The dock pose itself must come from the cart's GPS over MAVLink
        #   (the cart moves), converted into this local frame -- not a constant.
        self._set_target(target, self.limits.max_horizontal_speed_mps, self.limits.max_vertical_speed_mps)
        self._accept(
            MotionCommandType.GOTO_DOCK_APPROACH, target, self._h_speed, "goto_dock_approach",
            {"x": target.x, "y": target.y, "z": target.z, "yaw": target.yaw_deg},
            clamped=clamped, requested=requested,
        )
        return True

    def descend_to_dock(self, dock: Pose) -> bool:
        """Descend straight down onto the dock (x, y, z)."""
        fields = {"x": dock.x, "y": dock.y, "z": dock.z}
        if not self._precheck("descend_to_dock", fields):
            return False
        requested = Pose(dock.x, dock.y, dock.z, wrap_360(dock.yaw_deg))
        target, clamped = self._clamp(requested, landing=True)
        # PHASE-1 HOOK: precision landing -- MAVLink COMMAND_LONG MAV_CMD_NAV_LAND (21)
        #   with param2 = 2 (precision land REQUIRED; 1 = opportunistic),
        #   param4 = yaw, param5/6 = dock lat/lon. PX4 then flies the PLD_*
        #   precision-land state machine (PLD_HACC_RAD, PLD_FAPPR_ALT,
        #   PLD_SRCH_TOUT ...) using the landing target from CV: XRCE
        #   /fmu/in/landing_target_pose (vision pipeline -> XRCE per README), or
        #   MAVLink LANDING_TARGET (#149) if the marker is tracked off-board.
        #   Here the pad is at dock.z -> NED down = -dock.z.
        self._set_target(target, self.limits.max_horizontal_speed_mps, self.limits.max_vertical_speed_mps)
        self._accept(
            MotionCommandType.DESCEND_TO_DOCK, target, self._h_speed, "descend_to_dock",
            {"x": target.x, "y": target.y, "z": target.z},
            clamped=clamped, requested=requested,
        )
        return True

    def land(self) -> bool:
        """Land in place (straight down to the ground)."""
        # PHASE-1 HOOK: MAVLink COMMAND_LONG MAV_CMD_NAV_LAND (21), param2 = 0
        #   (no precision), param4 = NaN (keep yaw), param5/6 = NaN (land here),
        #   or MAV_CMD_DO_SET_MODE -> AUTO.LAND. Operator command -> MAVLink.
        #   PX4 auto-disarms after touchdown (COM_DISARM_LAND); this stub does
        #   not -- the caller disarms explicitly.
        return self._land_in_place("land")

    def _land_in_place(
        self, name: str, extra: Optional[Dict[str, Any]] = None, decision: Optional[str] = None
    ) -> bool:
        """The land path shared by `land()` and the comm-loss escalation."""
        if not self._precheck(name, {"x": self._pose.x, "y": self._pose.y}):
            return False
        target = Pose(self._pose.x, self._pose.y, GROUND_Z_M, self._pose.yaw_deg)
        # (PX4 mapping: see the PHASE-1 HOOK in land().)
        self._set_target(target, self.limits.max_horizontal_speed_mps, self.limits.max_vertical_speed_mps)
        self._accept(
            MotionCommandType.LAND, target, 0.0, name,
            {"x": target.x, "y": target.y, "z": target.z, **(extra or {})},
            decision=decision,
        )
        return True

    def emergency_land(self) -> bool:
        """SYNTHETIC ONLY: descend in place at `emergency_descent_mps`.

        Latches: until touchdown every other command is refused (logged
        `decision=ignored_emergency_landing`). On touchdown the stub
        auto-disarms and the latch clears. A repeat call while already
        emergency-landing is accepted silently (idempotent, no duplicate log).
        """
        if self._emergency:
            return True
        if not self._armed:
            self._log("emergency_land", {}, decision="rejected_not_armed")
            return False
        target = Pose(self._pose.x, self._pose.y, GROUND_Z_M, self._pose.yaw_deg)
        # PHASE-1 HOOK: MAVLink COMMAND_LONG MAV_CMD_NAV_LAND (21) with
        #   param5/6 = NaN (land exactly here, no transit), or PX4 Land mode via
        #   MAV_CMD_DO_SET_MODE; MAVLink because it must also work from the cart
        #   when the companion computer is the thing that failed.
        #   Last resort only: MAV_CMD_DO_FLIGHTTERMINATION (185) param1 = 1
        #   (XRCE: /fmu/in/vehicle_command, VEHICLE_CMD_DO_FLIGHTTERMINATION).
        #   Tradeoff: LAND keeps the vehicle controlled but needs working
        #   attitude/position estimation and some battery; TERMINATION cuts the
        #   motors and drops the drone (and needs CBRK_FLIGHTTERM disabled) --
        #   only justified for a fly-away or loss of control, never for
        #   "battery low". Phase-0 only simulates the controlled LAND.
        self._emergency = True
        self._set_target(target, 0.0, self.limits.emergency_descent_mps)
        self._accept(
            MotionCommandType.EMERGENCY_LAND, target, 0.0, "emergency_land",
            {"x": target.x, "y": target.y, "descent_mps": self.limits.emergency_descent_mps},
            decision="synthetic_only",
        )
        return True

    # -- integration ----------------------------------------------------

    def step(self, dt: float, now_ns: Optional[int] = None) -> Pose:
        """Advance the synthetic pose `dt` seconds toward the target.

        Horizontal and vertical motion are capped independently; yaw turns
        the short way round at `max_yaw_rate_dps`. Each axis snaps exactly
        onto its target when the remaining error fits inside one step, so
        arrival is exact rather than asymptotic. Does not log per tick --
        only the one-off `arrived(...)` and emergency auto-disarm.

        `now_ns` (simulation time of this tick) enables command expiry: if
        the active contract command has expired, it is replaced by a hold at
        the current pose *before* moving (INTEGRATION_README.md §3). Emergency
        descent never expires.
        """
        # PHASE-1 HOOK: on hardware nothing is integrated here. Pose comes from
        #   XRCE /fmu/out/vehicle_local_position (NED): x = msg.y (east),
        #   y = msg.x (north), z = -msg.z (up), yaw_deg = wrap_360(90 -
        #   degrees(msg.heading)); check msg.xy_valid / msg.z_valid first.
        #   Touchdown comes from /fmu/out/vehicle_land_detected.landed and
        #   armed state from /fmu/out/vehicle_status.arming_state, not from z.
        #   step() becomes "read latest pose, check arrival"; the setpoint
        #   stream (trajectory_setpoint + offboard_control_mode) is re-published
        #   here at the control rate if the active command is an offboard one.
        if now_ns is not None:
            self._now_ns = max(self._now_ns, now_ns)
            self._expire_if_needed(now_ns)
            self._escalate_if_comm_lost(now_ns)
        if not _is_real(dt) or not math.isfinite(dt):
            # A NaN dt would poison the pose for the rest of the run (and
            # feedback would still say valid=True); an inf dt would jump the
            # drone straight onto its target. Treat both as "no time passed".
            # Expiry above still ran: a bad dt must not keep a stale target alive.
            self._log("bad_dt", {"dt": repr(dt)}, decision="ignored_bad_dt")
            return self._pose
        if dt <= 0.0:
            return self._pose
        if not self._armed or (self._target is None and self._velocity_sp is None):
            self._est_velocity = Velocity(0.0, 0.0, 0.0, 0.0)
            return self._pose
        if self._velocity_sp is not None:
            return self._step_velocity(dt)

        p, t = self._pose, self._target

        # Horizontal (x/y together, so a diagonal is flown straight).
        dx, dy = t.x - p.x, t.y - p.y
        d_h = math.hypot(dx, dy)
        max_h = self._h_speed * dt
        if d_h <= max_h + _SNAP_EPS_M:
            x, y = t.x, t.y
        else:
            x, y = p.x + dx / d_h * max_h, p.y + dy / d_h * max_h

        # Vertical, own cap.
        dz = t.z - p.z
        max_v = self._v_speed * dt
        z = t.z if abs(dz) <= max_v + _SNAP_EPS_M else p.z + math.copysign(max_v, dz)

        # Yaw, shortest direction.
        dyaw = wrap_180(t.yaw_deg - p.yaw_deg)
        max_yaw = self.limits.max_yaw_rate_dps * dt
        yaw = t.yaw_deg if abs(dyaw) <= max_yaw + _SNAP_EPS_M else wrap_360(p.yaw_deg + math.copysign(max_yaw, dyaw))

        self._pose = Pose(x, y, z, wrap_360(yaw))
        self._est_velocity = self._velocity_between(p, self._pose, dt)

        if not self._arrival_logged and self.at_target():
            self._arrival_logged = True
            self._log(
                "arrived",
                {"x": self._pose.x, "y": self._pose.y, "z": self._pose.z},
            )

        if self._emergency and self._pose.z == t.z:
            # Auto-disarm on touchdown. On PX4 this is COM_DISARM_LAND acting on
            # vehicle_land_detected; here it is immediate.
            self._emergency = False
            self._armed = False
            self._target = None
            self._h_speed = self.limits.max_horizontal_speed_mps
            self._v_speed = self.limits.max_vertical_speed_mps
            self._accept(
                MotionCommandType.DISARM, None, 0.0, "disarm", {},
                decision="auto_disarm_on_ground",
            )

        return self._pose

    # -- internals ------------------------------------------------------

    def _step_velocity(self, dt: float) -> Pose:
        """One step of velocity mode, stopping on the first limit crossed."""
        # PHASE-1 HOOK: on hardware PX4 integrates the velocity; this method
        #   becomes "read /fmu/out/vehicle_local_position, predict pose + v*dt,
        #   and if that crosses the fence/ceiling publish a zero-velocity (or
        #   position-hold) /fmu/in/trajectory_setpoint at the boundary".
        p, v, lim = self._pose, self._velocity_sp, self.limits
        dx, dy, dz = v.vx * dt, v.vy * dt, v.vz * dt
        frac, why = 1.0, None

        floor = max(lim.min_altitude_m, GROUND_Z_M)
        if dz > 0.0 and p.z + dz > lim.max_altitude_m:
            s = max(0.0, (lim.max_altitude_m - p.z) / dz)
            if s < frac:
                frac, why = s, Reason.ALTITUDE_CEILING_BOUNDARY
        if dz < 0.0 and p.z + dz < floor:
            s = max(0.0, (floor - p.z) / dz)
            if s < frac:
                frac, why = s, Reason.ALTITUDE_FLOOR_BOUNDARY
        rx, ry = p.x - self.home.x, p.y - self.home.y
        R = lim.geofence_radius_m
        if math.hypot(rx + dx, ry + dy) > R:
            # Solve |r + s*d| = R for the first s in [0, 1].
            a = dx * dx + dy * dy
            b = 2.0 * (rx * dx + ry * dy)
            c = rx * rx + ry * ry - R * R
            disc = max(0.0, b * b - 4.0 * a * c)
            s = 0.0 if c >= 0.0 else (-b + math.sqrt(disc)) / (2.0 * a)
            s = min(max(s, 0.0), 1.0)
            if s < frac:
                frac, why = s, Reason.GEOFENCE_BOUNDARY

        x, y, z = p.x + dx * frac, p.y + dy * frac, p.z + dz * frac
        if why == Reason.ALTITUDE_CEILING_BOUNDARY:
            z = lim.max_altitude_m
        elif why == Reason.ALTITUDE_FLOOR_BOUNDARY:
            z = floor
        elif why == Reason.GEOFENCE_BOUNDARY:
            r = math.hypot(x - self.home.x, y - self.home.y)
            if r > R:  # float residue only: land exactly on the fence
                x = self.home.x + (x - self.home.x) * R / r
                y = self.home.y + (y - self.home.y) * R / r
        yaw = p.yaw_deg
        if v.yaw_rate_deg_s is not None:
            yaw = wrap_360(p.yaw_deg + v.yaw_rate_deg_s * dt * frac)
        self._pose = Pose(x, y, z, yaw)
        self._est_velocity = self._velocity_between(p, self._pose, dt)

        if why is not None:
            cmd = self._active_contract
            here = self._pose
            self._set_target(here, self._h_speed, self._v_speed)  # clears velocity mode
            self._arrival_logged = True
            rec = MotionCommand(MotionCommandType.HOVER, here, 0.0, self._clock(), note="boundary_hold")
            self._active = rec
            self._history.append(rec)
            # The velocity command is finished: renewing its ID is rejected
            # (command_id_reused), so the coordinator has to replan.
            self._active_contract = None
            self._boundary_reason = why
            self._log(
                "velocity_boundary",
                {
                    "command_id": cmd.command_id if cmd is not None else None,
                    "limit": why, "x": here.x, "y": here.y, "z": here.z,
                },
                decision="hold_at_boundary",
            )
        return self._pose

    @staticmethod
    def _velocity_between(a: Pose, b: Pose, dt: float) -> Velocity:
        return Velocity(
            (b.x - a.x) / dt, (b.y - a.y) / dt, (b.z - a.z) / dt,
            wrap_180(b.yaw_deg - a.yaw_deg) / dt,
        )

    def _velocity_limit_problem(
        self, v: Optional[Velocity], h_cap: Optional[float], v_cap: Optional[float]
    ) -> Optional[str]:
        """Detail text if `v` is faster than the limits (or a command's lower
        caps), else None. Rejected, never scaled down."""
        if v is None or not v.is_finite():
            return "non-finite velocity"
        lim = self.limits
        max_h = lim.max_horizontal_speed_mps if h_cap is None else min(lim.max_horizontal_speed_mps, h_cap)
        max_v = lim.max_vertical_speed_mps if v_cap is None else min(lim.max_vertical_speed_mps, v_cap)
        if v.horizontal_speed_mps > max_h:
            return f"horizontal {v.horizontal_speed_mps:.3f} m/s > {max_h} m/s"
        if abs(v.vz) > max_v:
            return f"vertical {abs(v.vz):.3f} m/s > {max_v} m/s"
        if v.yaw_rate_deg_s is not None and abs(v.yaw_rate_deg_s) > lim.max_yaw_rate_dps:
            return f"yaw rate {abs(v.yaw_rate_deg_s):.3f} deg/s > {lim.max_yaw_rate_dps} deg/s"
        return None

    def _flight_mode(self) -> FlightMode:
        if not self._armed:
            return FlightMode.DISARMED
        if self._emergency:
            return FlightMode.EMERGENCY_LAND
        if self._velocity_sp is not None:
            return FlightMode.VELOCITY
        a = self._active
        K = MotionCommandType
        if a is None or a.type in (K.ARM, K.DISARM):
            return FlightMode.ON_GROUND if self.grounded else FlightMode.HOLD
        if a.type == K.TAKEOFF:
            return FlightMode.TAKEOFF
        if a.type in (K.LAND, K.DESCEND_TO_DOCK, K.EMERGENCY_LAND):
            return FlightMode.LAND
        if a.type in (K.HOVER, K.STOP):
            return FlightMode.HOLD
        return FlightMode.POSITION

    def _requested_pose(self, kind: MotionCommandType, target: Optional[Pose]) -> Optional[Pose]:
        """The pose a contract command would ask for, before clamping."""
        if target is None:
            return None
        if kind == MotionCommandType.TAKEOFF:
            return Pose(self._pose.x, self._pose.y, target.z, self._pose.yaw_deg)
        if kind in (
            MotionCommandType.MOVE_TO,
            MotionCommandType.TRACK_TARGET,
            MotionCommandType.GOTO_DOCK_APPROACH,
            MotionCommandType.DESCEND_TO_DOCK,
        ):
            return target
        return None  # SET_YAW: yaw only, nothing to clamp

    def _dispatch(
        self, kind: MotionCommandType, t: Optional[Pose], v: Optional[Velocity] = None
    ) -> bool:
        """Route a validated contract command to the matching direct method."""
        K = MotionCommandType
        if kind == K.ARM:
            return self.arm()
        if kind == K.DISARM:
            return self.disarm()
        if kind == K.TAKEOFF:
            return self.takeoff(t.z)
        if kind == K.MOVE_TO:
            return self.move_to(t.x, t.y, t.z, yaw_deg=t.yaw_deg)
        if kind == K.TRACK_TARGET:
            return self.track_position(t.x, t.y, t.z, yaw_deg=t.yaw_deg)
        if kind == K.SET_YAW:
            return self.set_yaw(t.yaw_deg)
        if kind == K.SET_VELOCITY:
            return self.set_velocity(v.vx, v.vy, v.vz, v.yaw_rate_deg_s)
        if kind == K.HOVER:
            return self.hover()
        if kind == K.STOP:
            return self.stop()
        if kind == K.GOTO_DOCK_APPROACH:
            # The contract carries the approach point itself; the direct
            # method wants pad + height. Pad and ground are z = 0 here.
            return self.goto_dock_approach(
                Pose(t.x, t.y, GROUND_Z_M, t.yaw_deg), approach_altitude_m=t.z - GROUND_Z_M
            )
        if kind == K.DESCEND_TO_DOCK:
            return self.descend_to_dock(t)
        if kind == K.LAND:
            return self.land()
        if kind == K.EMERGENCY_LAND:
            return self.emergency_land()
        return False  # pragma: no cover -- CONTRACT_KINDS is exhaustive

    def _expire_if_needed(self, now_ns: int) -> None:
        cmd = self._active_contract
        if (
            cmd is None
            or cmd.expires_ns is None
            or MotionCommandType(cmd.kind) == MotionCommandType.EMERGENCY_LAND
            or now_ns < cmd.expires_ns
        ):
            return
        self._active_contract = None
        if not self._armed or (self._target is None and self._velocity_sp is None):
            return
        if self.landing_settled:
            # The landing already finished on the ground: there is nothing left
            # to hold. Keep the landing as the active record so feedback keeps
            # saying `landed` / `landing_settled` -- the state machine disarms
            # on DOCKED from `grounded`, and Docking is meant to confirm
            # touchdown from `landing_settled`. Turning it into an expired hold
            # here would un-land the drone. (landing_settled requires x/y on
            # the target too, so a touchdown still sliding toward the pad is
            # not settled and gets the normal hold.)
            return
        # Hold exactly where we are, at the current speed caps, before this
        # tick's movement -- the expired target must not keep pulling.
        here = self._pose
        self._set_target(here, self._h_speed, self._v_speed)
        self._arrival_logged = True
        rec = MotionCommand(MotionCommandType.HOVER, here, 0.0, self._clock(), note="expired_hold")
        self._active = rec
        self._history.append(rec)
        self._expired_hold = True
        self._expired_since_ns = now_ns
        self._log(
            "command_expired",
            {"command_id": cmd.command_id, "x": here.x, "y": here.y, "z": here.z},
            decision="hold_in_place",
        )

    def _escalate_if_comm_lost(self, now_ns: int) -> None:
        """Comm-loss failsafe: an expired hold that outlasts the timeout lands.

        The 0.2 s command validity turns a lost coordinator into a hold in
        place (`_expire_if_needed`). Holding forever would hover until the
        battery dies, and Motion cannot fly home on its own (no Navigation,
        no Safety, no battery reading), so after
        `limits.comm_loss_land_timeout_ns` of continuous expired hold it
        lands straight down where it is, through the ordinary land path.

        Not a latch (unlike EMERGENCY_LAND): any command `submit()` accepts
        cancels it, because acceptance already means fresh, valid and not
        expired, i.e. the coordinator is back and Safety has re-checked what
        it sent. A replayed or late command is rejected and cannot cancel it.
        Only an airborne, armed vehicle escalates; a grounded or disarmed one
        has nothing to land, and a settled landing never enters expired_hold.
        Simulation time only (`now_ns`); never wall time.
        """
        # PHASE-1 HOOK: on hardware this is PX4's own link-loss failsafe, so it
        #   is configured, not coded here. Offboard setpoint-stream loss
        #   (COM_OF_LOSS_T) triggers COM_OBL_RC_ACT; data-link (GCS/MAVLink)
        #   loss after COM_DL_LOSS_T triggers NAV_DLL_ACT. Set both actions to
        #   Land (land in place, as here) and COM_DL_LOSS_T to
        #   comm_loss_land_timeout_ns; check the enum values against the
        #   pinned PX4 release. Verify in SITL that a resumed stream / new
        #   offboard command regains control the way an accepted submit() does.
        since = self._expired_since_ns
        if (
            not self._expired_hold
            or since is None
            or self._comm_loss_landing
            or self._emergency
            or not self._armed
            or not self.airborne
            or now_ns - since < self.limits.comm_loss_land_timeout_ns
        ):
            return
        ok = self._land_in_place(
            "comm_loss_landing",
            {
                "held_s": round((now_ns - since) / 1e9, 3),
                "timeout_s": self.limits.comm_loss_land_timeout_ns / 1e9,
                "z_start": self._pose.z,
                "since_ns": since,
            },
            decision="land_in_place",
        )
        if ok:
            self._comm_loss_landing = True

    def _ack(
        self, command: Any, accepted: bool, reason: str, now_ns: Any, detail: Optional[str] = None
    ) -> CommandAck:
        kind = getattr(command, "kind", None)
        active = self._active
        return CommandAck(
            command_id=str(getattr(command, "command_id", "")),
            accepted=accepted,
            reason=reason,
            timestamp_ns=now_ns if isinstance(now_ns, int) and not isinstance(now_ns, bool) else self._now_ns,
            kind=kind.value if isinstance(kind, MotionCommandType) else (str(kind) if kind is not None else None),
            active_command_id=active.command_id if active is not None else None,
            detail=detail,
        )

    def _hold(self, kind: MotionCommandType, name: str) -> bool:
        if not self._precheck(name, {}):
            return False
        target = self._pose
        # PHASE-1 HOOK: hover/hold -- MAVLink MAV_CMD_DO_SET_MODE to PX4 Hold
        #   (AUTO.LOITER) or COMMAND_LONG MAV_CMD_NAV_LOITER_UNLIM (17) at the
        #   current position; if already in offboard, keep streaming a hold
        #   setpoint on XRCE /fmu/in/trajectory_setpoint (+ offboard_control_mode)
        #   = current /fmu/out/vehicle_local_position (already NED, no flip).
        #   Do not simply stop publishing: PX4 treats a dead offboard stream
        #   as a failsafe (COM_OF_LOSS_T), not as "hold".
        self._set_target(target, self._h_speed, self._v_speed)
        self._accept(
            kind, target, 0.0, name,
            {"x": target.x, "y": target.y, "z": target.z},
        )
        return True

    def _precheck(self, name: str, fields: Dict[str, Any]) -> bool:
        """Common refusal logic for commands that move the vehicle."""
        if self._refuse_during_emergency(name, fields):
            return False
        if not self._armed:
            self._log(name, fields, decision="rejected_not_armed")
            return False
        return True

    def _refuse_during_emergency(self, name: str, fields: Optional[Dict[str, Any]] = None) -> bool:
        if self._emergency:
            self._log(name, fields or {}, decision="ignored_emergency_landing")
            return True
        return False

    def _speed_caps(self, speed_mps: Optional[float]) -> Tuple[float, float]:
        h, v = self.limits.max_horizontal_speed_mps, self.limits.max_vertical_speed_mps
        if speed_mps is not None and speed_mps > 0.0:
            h, v = min(h, speed_mps), min(v, speed_mps)
        return h, v

    def _clamp(self, pose: Pose, landing: bool = False) -> Tuple[Pose, bool]:
        """Clamp into the geofence. Returns (clamped pose, whether it changed).

        `landing=True` exempts the altitude floor (you have to be allowed to
        reach the ground) but never the ceiling or the horizontal radius.
        """
        lim = self.limits
        floor = GROUND_Z_M if landing else lim.min_altitude_m
        z = min(max(pose.z, floor), lim.max_altitude_m)
        x, y = pose.x, pose.y
        dx, dy = x - self.home.x, y - self.home.y
        r = math.hypot(dx, dy)
        if r > lim.geofence_radius_m:
            scale = lim.geofence_radius_m / r
            x, y = self.home.x + dx * scale, self.home.y + dy * scale
        clamped_pose = Pose(x, y, z, pose.yaw_deg)
        changed = (x, y, z) != (pose.x, pose.y, pose.z)
        return clamped_pose, changed

    def _set_target(self, target: Pose, h_speed: float, v_speed: float) -> None:
        # Only re-arm the "arrived" log if the goal actually moved; otherwise a
        # state machine calling hover()/track_position() every tick with the
        # same answer would log "arrived" every tick.
        if self._target is None or self._target != target:
            self._arrival_logged = False
        self._target = target
        self._velocity_sp = None  # any position target ends velocity mode
        self._h_speed = h_speed
        self._v_speed = v_speed

    def _set_target_velocity(self, v: Velocity) -> None:
        """Enter velocity mode: no target pose, caps reset to the limits."""
        self._target = None
        self._velocity_sp = v
        self._arrival_logged = True  # nothing to arrive at
        self._h_speed = self.limits.max_horizontal_speed_mps
        self._v_speed = self.limits.max_vertical_speed_mps

    def _accept(
        self,
        kind: MotionCommandType,
        target: Optional[Pose],
        speed: float,
        name: str,
        fields: Dict[str, Any],
        *,
        clamped: bool = False,
        requested: Optional[Pose] = None,
        decision: Optional[str] = None,
        velocity: Optional[Velocity] = None,
    ) -> None:
        contract = self._dispatching
        cmd = MotionCommand(
            kind, target, speed, self._clock(), note=name,
            command_id=contract.command_id if contract is not None else None,
            clamped=clamped, requested=requested if clamped else None,
            velocity=velocity,
        )
        self._active = cmd
        self._history.append(cmd)
        self._expired_hold = False
        self._boundary_reason = None
        self._expired_since_ns = None
        self._comm_loss_landing = False
        if contract is None:
            # A direct call supersedes whatever the coordinator last sent.
            self._active_contract = None
        else:
            fields = dict(fields)
            fields["command_id"] = contract.command_id
        if clamped and requested is not None:
            fields = dict(fields)
            fields.update({"req_x": requested.x, "req_y": requested.y, "req_z": requested.z})
            decision = "clamped_to_geofence" if decision is None else f"{decision},clamped_to_geofence"
        self._log(name, fields, decision=decision)

    def _log(self, event: str, fields: Dict[str, Any], *, decision: Optional[str] = None) -> None:
        if self._logger is None:
            return
        rounded = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in fields.items()}
        self._logger.log(
            _LOG_MODULE,
            event,
            fields=rounded,
            state=self._log_state,
            truth=self._log_truth,
            decision=decision,
            t=self._clock(),
        )
