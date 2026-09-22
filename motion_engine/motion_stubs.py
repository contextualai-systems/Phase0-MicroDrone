"""MotionStub -- the kinematic motion stand-in for the Phase-0 mission layer.

Phase-0 has no flight controller, no motors and no simulator in the mission
loop. What the mission state machine (`mission/state_machine.py`) needs from
"motion" is much smaller than a flight stack: it needs to *issue* a command
("go to X, Y, Z", "hover", "land on the dock") and then *observe progress over
time* -- the drone is 12 m away from the approach point, then 7 m, then it has
arrived -- so that transitions such as DOCKING_APPROACH -> DOCKED happen for a
reason the log can show, not instantly.

So this module is a pure kinematic stub: it holds a synthetic `Pose` and each
`step(dt)` moves it straight toward the active target at a capped speed. No
mass, no drag, no attitude dynamics, no PID. That is deliberate -- the point is
to exercise the *decision* layer, and a kinematic model is predictable enough
that tests can say "at 5 m/s, 20 m takes 4 s" and mean it.

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
  `track_target` becomes when it is called every tick) go over **uXRCE-DDS**:
  `/fmu/in/trajectory_setpoint` paired with `/fmu/in/offboard_control_mode`.
* Pose/state comes *back* over uXRCE-DDS (`/fmu/out/vehicle_local_position`).

Coordinate frame (shared with the rest of `mission/`): local metres, origin =
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
import time
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING, Any, Callable, Dict, List, Optional, Tuple

if TYPE_CHECKING:  # pragma: no cover -- type hint only, see note below.
    # Imported for typing only: MotionStub never constructs a logger, it just
    # calls `.log(...)` on whatever it is handed (or nothing, if None). Keeping
    # the import out of runtime means motion_engine/ does not hard-depend on
    # mission/ existing, and there is no import cycle when
    # mission/state_machine.py imports this module.
    from mission.logging_format import Phase0Logger


#: Local-frame altitude of the ground / dock pad. The origin *is* the dock pad
#: at launch, so ground is z = 0 by construction.
GROUND_Z_M: float = 0.0

#: Above this altitude (relative to ground) the vehicle counts as airborne.
#: Matches the spec's `z > 0.05`; small enough that a vehicle sitting on the
#: pad with a sliver of numeric noise does not read as flying.
AIRBORNE_THRESHOLD_M: float = 0.05

#: Log module name. A plain string rather than `mission.logging_format.Module`
#: so this file has no runtime dependency on mission/ (Module is a `str` Enum,
#: so "MOTION" == Module.MOTION either way).
_LOG_MODULE: str = "MOTION"

#: Slack on the "remaining error fits in one step -> snap" test. Without it,
#: float accumulation (ten steps of 0.1 s are not exactly 1 s) leaves a
#: 1e-15 m residual that needs one extra tick and never reads as "on target".
_SNAP_EPS_M: float = 1e-9


# --------------------------------------------------------------------------
# Small value types
# --------------------------------------------------------------------------


def wrap_360(deg: float) -> float:
    """Normalise an angle into [0, 360)."""
    out = deg % 360.0
    # `-1e-18 % 360` is 360.0 in floating point; fold that back to 0.
    return 0.0 if out >= 360.0 else out


def wrap_180(deg: float) -> float:
    """Normalise an angle into (-180, 180]. Used for shortest-direction yaw."""
    out = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if out == -180.0 else out


@dataclass(frozen=True)
class Pose:
    """Position (m, local ENU, z up) plus heading (deg, 0 = east, CCW +).

    Frozen on purpose: `MotionStub.pose` hands this out to the state machine
    and the synthetic world every tick. If it were mutable, one consumer doing
    `pose.z = 0` would silently teleport the drone for everyone else.
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


class MotionCommandType(str, Enum):
    """Every command the stub accepts. One-to-one with the public methods."""

    ARM = "ARM"
    DISARM = "DISARM"
    TAKEOFF = "TAKEOFF"
    MOVE_TO = "MOVE_TO"
    HOVER = "HOVER"
    SET_YAW = "SET_YAW"
    SET_ORIENTATION = "SET_ORIENTATION"
    TRACK_TARGET = "TRACK_TARGET"
    GOTO_DOCK_APPROACH = "GOTO_DOCK_APPROACH"
    DESCEND_TO_DOCK = "DESCEND_TO_DOCK"
    LAND = "LAND"
    EMERGENCY_LAND = "EMERGENCY_LAND"
    STOP = "STOP"


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
    """

    def __init__(
        self,
        logger: Optional["Phase0Logger"] = None,
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

    def track_target(
        self,
        bearing_deg: float,
        distance_m: float,
        standoff_m: float = 15.0,
        altitude_m: Optional[float] = None,
    ) -> bool:
        """Close to `standoff_m` from a bird seen at `bearing_deg`/`distance_m`.

        `bearing_deg` is in the *world* (local ENU) yaw convention -- 0 = east,
        CCW positive -- measured from the drone's current position, which is
        how `mission/synthetic_world.py` reports it. The new target is the
        point `distance_m - standoff_m` along that bearing, facing the bird.
        If the bird is already closer than the standoff, that distance is
        negative and the drone backs away to restore it. `altitude_m=None`
        keeps the current target altitude.
        """
        fields = {"bearing": bearing_deg, "distance": distance_m, "standoff": standoff_m}
        if not self._precheck("track_target", fields):
            return False
        travel = distance_m - standoff_m
        b = math.radians(bearing_deg)
        base_z = self._target.z if self._target is not None else self._pose.z
        z = base_z if altitude_m is None else altitude_m
        requested = Pose(
            self._pose.x + travel * math.cos(b),
            self._pose.y + travel * math.sin(b),
            z,
            wrap_360(bearing_deg),
        )
        target, clamped = self._clamp(requested)
        if travel > self.limits.arrival_tolerance_m:
            decision = "close_to_standoff"
        elif travel < -self.limits.arrival_tolerance_m:
            decision = "back_off_to_standoff"
        else:
            decision = "hold_standoff"
        # PHASE-1 HOOK: this is called every mission tick, i.e. a streamed
        #   setpoint -> XRCE /fmu/in/trajectory_setpoint + /fmu/in/offboard_control_mode
        #   (position=True), per the README caveat about high-rate loops. (A
        #   low-rate fallback is MAVLink SET_POSITION_TARGET_LOCAL_NED, same
        #   field mapping as move_to.) NED conversion:
        #   position = [target.y, target.x, -target.z], yaw =
        #   radians(wrap_180(90 - bearing_deg)) so the camera faces the bird.
        #   Bearing/distance will come from the CV node over XRCE and are
        #   camera-relative on hardware -- add the vehicle heading from
        #   /fmu/out/vehicle_local_position before using them here.
        fields.update({"x": target.x, "y": target.y, "z": target.z})
        self._set_target(target, self.limits.max_horizontal_speed_mps, self.limits.max_vertical_speed_mps)
        self._accept(
            MotionCommandType.TRACK_TARGET, target, self._h_speed, "track_target",
            fields, clamped=clamped, requested=requested, decision=decision,
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
        if not self._precheck("land", {"x": self._pose.x, "y": self._pose.y}):
            return False
        target = Pose(self._pose.x, self._pose.y, GROUND_Z_M, self._pose.yaw_deg)
        # PHASE-1 HOOK: MAVLink COMMAND_LONG MAV_CMD_NAV_LAND (21), param2 = 0
        #   (no precision), param4 = NaN (keep yaw), param5/6 = NaN (land here),
        #   or MAV_CMD_DO_SET_MODE -> AUTO.LAND. Operator command -> MAVLink.
        #   PX4 auto-disarms after touchdown (COM_DISARM_LAND); this stub does
        #   not -- the caller disarms explicitly.
        self._set_target(target, self.limits.max_horizontal_speed_mps, self.limits.max_vertical_speed_mps)
        self._accept(
            MotionCommandType.LAND, target, 0.0, "land",
            {"x": target.x, "y": target.y, "z": target.z},
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

    def step(self, dt: float) -> Pose:
        """Advance the synthetic pose `dt` seconds toward the target.

        Horizontal and vertical motion are capped independently; yaw turns
        the short way round at `max_yaw_rate_dps`. Each axis snaps exactly
        onto its target when the remaining error fits inside one step, so
        arrival is exact rather than asymptotic. Does not log per tick --
        only the one-off `arrived(...)` and emergency auto-disarm.
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
        if dt <= 0.0 or self._target is None or not self._armed:
            return self._pose

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
        # state machine calling hover()/track_target() every tick with the
        # same answer would log "arrived" every tick.
        if self._target is None or self._target != target:
            self._arrival_logged = False
        self._target = target
        self._h_speed = h_speed
        self._v_speed = v_speed

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
    ) -> None:
        cmd = MotionCommand(kind, target, speed, self._clock(), note=name)
        self._active = cmd
        self._history.append(cmd)
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
