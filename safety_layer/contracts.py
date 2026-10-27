"""SafetyEvent: Safety's report for one tick.

Every tick, Safety answers two questions (see README "How decisions are made"):

    1. assess()        -> what is the drone allowed to do right now?
    2. check_command() -> is this specific order safe, or what replaces it?

A SafetyEvent puts both answers in one record with a fixed format, so every
module, the shared log and the tests read Safety's decisions the same way.
It doesn't make any new decisions. It only writes down the ones Safety made.

This is one of the six shared formats listed for Phase 0, alongside
Motion's Command and VehicleFeedback, CV's DetectionResult and the state
machine's MissionStatus.
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from safety_layer.command_check import CommandDecision
from safety_layer.safety_policy import ACTIONS, SafetyAssessment

SCHEMA_VERSION = 1


def _kind_name(kind) -> Optional[str]:
    """Write an order kind as plain text, e.g. "HOVER"."""
    if kind is None:
        return None
    return getattr(kind, "value", kind)


@dataclass(frozen=True)
class CommandCheck:
    """What Safety did with one order this tick."""

    requested_id: str                  # the order someone wanted to send
    requested_kind: str                # e.g. "TRACK_TARGET"
    allowed: bool                      # True = sent unchanged
    sent_id: str                       # what actually went to Motion
    sent_kind: str                     # e.g. "HOVER" if it was replaced
    reasons: Tuple[str, ...] = ()      # why it was replaced; empty if allowed

    @classmethod
    def from_decision(cls, requested, decision: CommandDecision) -> "CommandCheck":
        """Build one from the order that was asked for and Safety's answer."""
        return cls(
            requested_id=requested.command_id,
            requested_kind=_kind_name(requested.kind),
            allowed=decision.allowed,
            sent_id=decision.command.command_id,
            sent_kind=_kind_name(decision.command.kind),
            reasons=tuple(decision.reasons),
        )

    def to_dict(self) -> dict:
        return {
            "requested_id": self.requested_id,
            "requested_kind": self.requested_kind,
            "allowed": self.allowed,
            "sent_id": self.sent_id,
            "sent_kind": self.sent_kind,
            "reasons": list(self.reasons),
        }


@dataclass(frozen=True)
class SafetyEvent:
    """Safety's report for one tick.

    "frozen" means it can't be changed after it's made, so the log always
    shows what Safety really decided.
    """

    timestamp_ns: int                       # simulation time of this tick
    permissions: Dict[str, bool]            # launch/tracking/approach/descent/hold: allowed?
    emergency: bool                         # land in place now
    abort: bool                             # cancel the current action and hold
    battery_low: bool                       # 20% or less
    battery_critical: bool                  # 10% or less
    battery_recovered: bool                 # 40% or more: OK to launch again
    reasons: Tuple[str, ...]                # every active reason, most serious first
    commands: Tuple[CommandCheck, ...] = () # orders checked this tick (none, one, or more)
    schema_version: int = SCHEMA_VERSION
    clock: str = "simulation"
    synthetic: bool = True
    source: str = "safety"

    @classmethod
    def from_checks(cls, assessment: SafetyAssessment, commands=()) -> "SafetyEvent":
        """Build the tick's event from assess() and any check_command() results.

        commands: a list of (requested_order, CommandDecision) pairs, in the
        order they were checked. Leave it empty on a tick with no orders.
        """
        return cls(
            timestamp_ns=assessment.timestamp_ns,
            permissions=dict(assessment.permissions),
            emergency=assessment.emergency,
            abort=assessment.abort,
            battery_low=assessment.battery_low,
            battery_critical=assessment.battery_critical,
            battery_recovered=assessment.battery_recovered,
            reasons=tuple(assessment.reasons),
            commands=tuple(
                CommandCheck.from_decision(requested, decision)
                for requested, decision in commands
            ),
        )

    def validate(self) -> Optional[str]:
        """Check the event follows the rules. Returns a problem, or None if it's fine.

        These rules come from how Safety makes decisions, so a broken rule
        means a bug somewhere, not just a badly written log line.
        """
        if self.schema_version != SCHEMA_VERSION:
            return "unsupported_schema"
        if not isinstance(self.timestamp_ns, int) or isinstance(self.timestamp_ns, bool) \
                or self.timestamp_ns < 0:
            return "invalid_timestamp"

        # Exactly the five actions, each a real True/False.
        if not isinstance(self.permissions, dict) or set(self.permissions) != set(ACTIONS):
            return "invalid_permissions"
        if not all(isinstance(value, bool) for value in self.permissions.values()):
            return "invalid_permissions"

        if not all(isinstance(reason, str) and reason for reason in self.reasons):
            return "invalid_reason"

        # An emergency blocks everything, even holding still.
        if self.emergency and any(self.permissions.values()):
            return "emergency_with_permission"
        # Emergency outranks abort, so both are never set together.
        if self.emergency and self.abort:
            return "emergency_and_abort"
        # Anything at 10% or less is also at 20% or less.
        if self.battery_critical and not self.battery_low:
            return "critical_without_low"
        # Something must explain an emergency or abort.
        if (self.emergency or self.abort) and not self.reasons:
            return "missing_reason"

        for check in self.commands:
            if check.allowed and (check.sent_id != check.requested_id or check.reasons):
                return "allowed_command_changed"
            if not check.allowed and (check.sent_id == check.requested_id or not check.reasons):
                return "replacement_unexplained"

        return None

    def to_dict(self) -> dict:
        """Plain values only, ready for JSON and the shared log."""
        return {
            "timestamp_ns": self.timestamp_ns,
            "permissions": {action: self.permissions[action] for action in ACTIONS
                            if action in self.permissions},
            "emergency": self.emergency,
            "abort": self.abort,
            "battery_low": self.battery_low,
            "battery_critical": self.battery_critical,
            "battery_recovered": self.battery_recovered,
            "reasons": list(self.reasons),
            "commands": [check.to_dict() for check in self.commands],
            "schema_version": self.schema_version,
            "clock": self.clock,
            "synthetic": self.synthetic,
            "source": self.source,
        }
