"""
This is a place holder in:
safety_layer/safety_decision.py

Before drone the drone moves, the safety layer checks the model's decision
and decides if the action is permissible. It checks if the action is allowed,
the reason for the action, and when it was requested.

returns: allowed/not allowed, why, and when

"""

from dataclasses import dataclass
import time


@dataclass(frozen=True)
class SafetyDecision:
    #Safety's answer to a movement request
    allowed: bool      # True = move, False = blocked
    action: str        # actions: proceed, hold, land
    reason: str        # why this decision was made
    timestamp_ns: int  # when it was decided


def check_request(request) -> SafetyDecision:
    """Placeholder: approves every request for now.
    TODO: add boundary limits, speed limits, old-data checks, and low-battery checks."""
    return SafetyDecision(True, "proceed", "no checks yet (Phase 0 stub)", time.time_ns())