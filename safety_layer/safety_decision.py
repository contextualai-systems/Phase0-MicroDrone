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

# Weather values considered safe to fly in.
# TODO: agree this list with the team.
SAFE_WEATHER = ("clear", "cloudy")


@dataclass(frozen=True)
class SafetyDecision:
    """Safety's answer for one observation.

    frozen=True means a decision can't be changed after it's made,
    so the log always shows what safety actually decided.
    """

    allowed: bool      # True = motion may move, False = blocked
    action: str        # "proceed", "hold", or "abort"
    reason: str        # why, in plain words, for the log
    timestamp_ns: int  # when the decision was made
    dock_check: bool = False  # True = ask the docking module if it should dock


def check_observation(observation) -> SafetyDecision:
    """Decide whether the drone may move, based on one observation.

    Args:
        observation: One observation dictionary in the team's JSON format.

    Returns:
        SafetyDecision: allowed/blocked, what to do, why, and when.

    Rule: if required safety information is missing, block. An unknown
    condition is treated as unsafe, never as safe.
    """
    now_ns = time.time_ns()

    # The safety section is required. No safety data means no movement.
    safety = observation.get("safety")
    if not safety:
        return SafetyDecision(
            allowed=False,
            action="hold",
            reason="missing safety data in observation",
            timestamp_ns=now_ns,
        )

    # Each check is (condition_is_bad, reason, action).
    # "hold" = stop and wait. "abort" = stop and hand over to abort handling.
    checks = (
        (safety.get("people_nearby") is not False, "people nearby", "abort"),
        (safety.get("cart_moving") is not False, "cart is moving", "hold"),
        (safety.get("weather") not in SAFE_WEATHER, "unsafe weather", "hold"),
        (safety.get("camera_ok") is not True, "camera not working", "hold"),
        (safety.get("comm_ok") is not True, "lost communication", "abort"),
    )

    # Report every problem found, not just the first one.
    problems = [reason for is_bad, reason, _ in checks if is_bad]
    if problems:
        # If anything calls for abort, abort wins over hold.
        action = "abort" if any(
            is_bad and act == "abort" for is_bad, _, act in checks
        ) else "hold"
        return SafetyDecision(
            allowed=False,
            action=action,
            reason="blocked: " + ", ".join(problems),
            timestamp_ns=now_ns,
            dock_check=True,  # blocked, so docking may be the better option
        )

    # Nothing wrong: movement is allowed.
    return SafetyDecision(
        allowed=True,
        action="proceed",
        reason="all safety checks passed",
        timestamp_ns=now_ns,
    )


# TODO: send decisions to the shared logger (Will) once its format is set.
# TODO: confirm with Mykolas how "abort" maps to the ABORT state.
# TODO: battery is handled by docking (Elias N) — confirm safety doesn't duplicate it.