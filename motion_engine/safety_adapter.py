"""Coordinator glue between Safety's command check and MotionStub.

Since PR #65, Safety (`safety_layer/command_check.py`) reads Motion's own
contract `Command` and hands back a Motion `Command`: the original if it is
allowed, otherwise a replacement Motion can execute (HOVER, LAND,
EMERGENCY_LAND, or DISARM on the ground, ID `<original id>-safety`, `source = "safety"`). The kind,
target and expiry translation this module used to do is no longer needed.

What is left is one call for the coordinator's dispatch step:

    to_submit, decision = check_for_motion(cmd, assessment, now_ns)
    ack = motion.submit(to_submit, now_ns)
    # log decision.reasons next to the ack

a guard that refuses to hand Motion anything that is not a Motion
`Command` (for example, if Safety's return type changes again), and one
clean-up of Safety's replacements (below). The tests in
`tests/test_safety_adapter.py` drive the real Safety check and the real
MotionStub together, so a contract drift between the two teams shows up
there first.

Velocity setpoints on Safety's replacements
-------------------------------------------
Safety builds a replacement by copying the original command and changing its
kind and target. A `SET_VELOCITY` original therefore yields a HOVER, LAND,
EMERGENCY_LAND or DISARM that still carries the old `velocity`, and Motion
rejects any non-velocity kind with a velocity (`invalid_command`): Safety's own
stop would fail. `check_for_motion()` clears `velocity` on such a replacement
before Motion sees it. It never changes what Safety decided: the kind, the
reasons and `allowed` are untouched, and an allowed command passes through as
the same object.

Until Safety maps `SET_VELOCITY` to a permission, Safety refuses it as
`unsupported_kind` and substitutes a HOVER, so a coordinator that sends one
gets a hold, never unchecked velocity flight.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Tuple

try:
    from motion_engine.contracts import Command, MotionCommandType
except ModuleNotFoundError:  # pragma: no cover -- only motion_engine/ on sys.path
    from contracts import Command, MotionCommandType  # type: ignore[no-redef]

from safety_layer.command_check import CommandDecision, check_command
from safety_layer.safety_policy import SafetyAssessment


def check_for_motion(
    cmd: Command,
    assessment: SafetyAssessment,
    now_ns: int,
) -> Tuple[Command, CommandDecision]:
    """Run Safety's command check; return `(command_to_submit, decision)`.

    `command_to_submit` is `cmd` itself when Safety allows it, else Safety's
    replacement, with any velocity setpoint it copied from the original
    cleared (see the module docstring). Raises `TypeError` if Safety returns
    something Motion cannot accept, rather than letting Motion reject it
    silently mid-flight.
    """
    decision = check_command(cmd, assessment, now_ns)
    if not isinstance(decision.command, Command):
        raise TypeError(
            "Safety returned a non-Motion command "
            f"({type(decision.command).__name__}); update motion_engine/safety_adapter.py"
        )
    return _without_copied_velocity(decision.command, cmd), decision


def _without_copied_velocity(out: Command, original: Command) -> Command:
    """Clear a velocity Safety copied onto a non-velocity replacement."""
    if out is original or out.velocity is None:
        return out
    if out.kind == MotionCommandType.SET_VELOCITY:
        return out
    return replace(out, velocity=None)
