"""The second safety check: looking at one actual order.

The first check (safety_policy.py) answers "is descending allowed right now?"
This one answers "is THIS specific order safe to send?" -- for example,
"fly to 200 metres east, 40 metres up, at 9 m/s".

If an order is unsafe we hand back a SAFE REPLACEMENT, never a bare "no".
The integration plan is explicit about this: simply not sending a command
would leave the drone chasing whatever target it already had.

Which replacement depends on where the drone is:
  in the air    -> HOVER (or LAND if even holding isn't allowed),
                   EMERGENCY_LAND in an emergency
  on the ground -> DISARM (motors off). Hovering or landing makes no sense
                   for a drone that is already down.

Order shape comes from Motion's shared contract, so Safety and Motion agree
on field names. That file is expected to move to state_machine/contracts.py
later; only the import below should need changing.
"""

from dataclasses import dataclass, replace
from typing import Optional, Tuple

from motion_engine.contracts import (
    LIFECYCLE_KINDS,
    Command,
    MotionCommandType,
    Pose,
    Reason,
)

from safety_layer import config
from safety_layer.safety_policy import SafetyAssessment


# Which order needs which permission from the first check.
KIND_TO_ACTION = {
    MotionCommandType.ARM: "launch",
    MotionCommandType.TAKEOFF: "launch",
    MotionCommandType.MOVE_TO: "tracking",
    MotionCommandType.TRACK_TARGET: "tracking",
    MotionCommandType.SET_YAW: "tracking",
    MotionCommandType.GOTO_DOCK_APPROACH: "approach",
    MotionCommandType.DESCEND_TO_DOCK: "descent",
    MotionCommandType.HOVER: "hold",
    MotionCommandType.STOP: "hold",
    MotionCommandType.LAND: "hold",
    MotionCommandType.DISARM: "hold",
    MotionCommandType.EMERGENCY_LAND: "hold",
}

# Safety's own reason codes, for problems Motion's list doesn't name.
GEOFENCE_BREACH = "geofence_breach"
ALTITUDE_CEILING = "altitude_ceiling"
SPEED_LIMIT = "speed_limit"
ACTION_NOT_PERMITTED = "action_not_permitted"
EMERGENCY = "emergency"
UNDERGROUND_TARGET = "underground_target"


@dataclass(frozen=True)
class CommandDecision:
    """Safety's answer about one order."""

    allowed: bool                   # True = send the order unchanged
    command: Command                # the order to send (original or replacement)
    reasons: Tuple[str, ...] = ()   # short codes explaining any problem
    timestamp_ns: int = 0

    @property
    def replaced(self) -> bool:
        """True if we swapped in a different order."""
        return not self.allowed


def _safety_command(
    original: Command, kind: MotionCommandType, now_ns: int, reason: str
) -> Command:
    """Build a replacement order from Safety.

    HOVER, DISARM and EMERGENCY_LAND all act where the drone currently is, so we
    deliberately drop the original target instead of reusing it: the old
    destination is exactly what we're trying to stop.
    """
    return replace(
        original,
        command_id=f"{original.command_id}-safety",
        kind=kind,
        target=None,
        issued_ns=now_ns,
        expires_ns=None if kind in LIFECYCLE_KINDS else now_ns + _validity_ns(),
        source="safety",
        reason=reason,
        max_horizontal_speed_mps=None,
        max_vertical_speed_mps=None,
    )


def _validity_ns() -> int:
    """How long an order stays good, in nanoseconds."""
    return int(config.COMMAND_VALIDITY_S * config.NS_PER_S)


def _expired(command: Command, now_ns: int) -> bool:
    """Has this order gone stale?

    Takeoff, land and the other lifecycle orders may carry no expiry at all,
    so for those we measure age from when they were issued instead.
    """
    if command.expires_ns is not None:
        return now_ns >= command.expires_ns
    return now_ns - command.issued_ns > _validity_ns()


def _outside_limits(target: Optional[Pose]) -> Tuple[str, ...]:
    """Check a destination against the fence and the height limits."""
    if target is None:
        return ()

    problems = []

    # Distance from the pad, which sits at (0, 0).
    distance = (target.x ** 2 + target.y ** 2) ** 0.5
    if distance > config.GEOFENCE_RADIUS_M:
        problems.append(GEOFENCE_BREACH)

    if target.z > config.ALTITUDE_CEILING_M:
        problems.append(ALTITUDE_CEILING)

    # Below ground is never a real destination.
    if target.z < 0.0:
        problems.append(UNDERGROUND_TARGET)

    return tuple(problems)


def _speed_caps_ok(command: Command) -> bool:
    """Speed caps may only lower our limits, never raise them.

    Motion's contract says the same, so a command asking to go faster than
    the configured limit is malformed, not just unsafe.
    """
    horizontal = command.max_horizontal_speed_mps
    vertical = command.max_vertical_speed_mps
    if horizontal is not None and horizontal > config.MAX_HORIZONTAL_SPEED_MPS:
        return False
    if vertical is not None and vertical > config.MAX_VERTICAL_SPEED_MPS:
        return False
    return True


def _fallback_kind(assessment: SafetyAssessment) -> MotionCommandType:
    """What to send instead of a rejected order."""
    if assessment.grounded:
        # Already down: just turn the motors off.
        return MotionCommandType.DISARM
    if assessment.allows("hold"):
        # Hovering is how the drone stops in the air.
        return MotionCommandType.HOVER
    # In the air and even holding isn't permitted: something is badly
    # wrong, so come down instead.
    return MotionCommandType.LAND


def _already_the_safe_response(command: Command, emergency_kind, now_ns: int) -> bool:
    """Is this order exactly what Safety would send anyway?

    If so we pass it through unchanged instead of swapping it for an
    identical copy under a new name, which only confuses the logs.
    """
    try:
        kind = MotionCommandType(command.kind)
    except ValueError:
        return False
    return (
        kind == emergency_kind
        and command.validate() is None
        and not _expired(command, now_ns)
    )


def check_command(
    command: Command, assessment: SafetyAssessment, now_ns: int
) -> CommandDecision:
    """Check one order and return it, or a safe replacement.

    command:    the order the coordinator wants to send
    assessment: what the first safety check decided this tick
    now_ns:     the current simulation time
    """
    # 1. An emergency beats everything. In the air: come down here, now.
    #    On the ground: turn the motors off.
    if assessment.emergency:
        emergency_kind = (
            MotionCommandType.DISARM
            if assessment.grounded
            else MotionCommandType.EMERGENCY_LAND
        )
        if _already_the_safe_response(command, emergency_kind, now_ns):
            return CommandDecision(allowed=True, command=command, timestamp_ns=now_ns)
        return CommandDecision(
            allowed=False,
            command=_safety_command(command, emergency_kind, now_ns, Reason.ACCEPTED),
            reasons=(EMERGENCY,) + assessment.reasons,
            timestamp_ns=now_ns,
        )

    problems = []

    # 2. Is the order itself well formed? Motion's own check covers schema,
    #    frame, missing targets and bad numbers, so we reuse it.
    malformed = command.validate()
    if malformed is not None:
        problems.append(malformed)

    # 3. Has it gone stale?
    if _expired(command, now_ns):
        problems.append(Reason.COMMAND_EXPIRED)

    # 4. Is this kind of action allowed right now?
    try:
        kind = MotionCommandType(command.kind)
    except ValueError:
        kind = None
        problems.append(Reason.UNSUPPORTED_KIND)

    if kind is not None:
        action = KIND_TO_ACTION.get(kind)
        if action is None:
            problems.append(Reason.UNSUPPORTED_KIND)
        elif not assessment.allows(action):
            problems.append(ACTION_NOT_PERMITTED)

    # 5. Is the destination inside our limits?
    problems.extend(_outside_limits(command.target))

    # 6. Are the speed caps sane?
    if not _speed_caps_ok(command):
        problems.append(SPEED_LIMIT)

    if problems:
        return CommandDecision(
            allowed=False,
            command=_safety_command(
                command, _fallback_kind(assessment), now_ns, Reason.ACCEPTED
            ),
            reasons=tuple(dict.fromkeys(problems)),
            timestamp_ns=now_ns,
        )

    # All good: send the original order unchanged.
    return CommandDecision(allowed=True, command=command, timestamp_ns=now_ns)
