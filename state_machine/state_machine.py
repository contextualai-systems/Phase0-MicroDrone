# State machine code: picks the next state based on the current state and the observation.
# It only decides the state and reason; run_loop.py acts on the chosen state.

from enum import Enum

class State(Enum):
    IDLE = "IDLE"
    TRACKING = "TRACKING"
    HOVERING = "HOVERING"
    DOCKING_INIT = "DOCKING_INIT"
    DOCKING_APPROACH = "DOCKING_APPROACH"
    DOCKED = "DOCKED"
    ABORT = "ABORT"
    EMERGENCY_LAND = "EMERGENCY_LAND"

AIRBORNE = {
    State.TRACKING,
    State.HOVERING,
    State.DOCKING_INIT,
    State.DOCKING_APPROACH,
    State.ABORT
}

def is_airborne(state: State) -> bool:
    return state in AIRBORNE

# Safety's permission for one action: "launch", "tracking", "approach", "descent" or "hold".
# A missing permission counts as not permitted.
def is_permitted(safety: dict, action: str) -> bool:
    permissions = safety.get("permissions") or {}
    return permissions.get(action) is True


# The observation is a dictionary in the shared JSON format
def step(state: State, observation: dict) -> tuple[State, str | None]:
    bird = observation.get("bird") or {}
    safety = observation.get("safety") or {}
    docking = observation.get("docking") or {}

    bird_seen = bird.get("detected") is True
    should_dock = docking.get("should_dock") is True

    # Global rules are checked before the per-state rules, and emergency wins over abort
    # An emergency on the ground only blocks launch.
    if is_airborne(state) and safety.get("emergency") is True:
        return State.EMERGENCY_LAND, "Safety emergency"
    # TODO: abort while grounded (IDLE, DOCKED) needs grounded/at-pad feedback to return to IDLE
    if is_airborne(state) and state != State.ABORT and safety.get("abort") is True:
        return State.ABORT, "Safety abort"

    match state:
        # Idle state
        # A launch goes through HOVERING before TRACKING.
        case State.IDLE:
            if bird_seen and is_permitted(safety, "launch"):
                return State.HOVERING, "Bird detected"
        # Tracking state
        case State.TRACKING:
            if should_dock:
                return State.DOCKING_INIT, "Should dock"
            if not is_permitted(safety, "tracking"):
                return State.HOVERING, "Tracking not permitted"
            if not bird_seen:
                return State.HOVERING, "Bird not detected"

        # Hovering state
        case State.HOVERING:
            if should_dock:
                return State.DOCKING_INIT, "Should dock"
            if bird_seen and is_permitted(safety, "tracking"):
                return State.TRACKING, "Bird detected"

        # Docking initialization state
        # Hold here until Safety permits the approach (it blocks it for a moving cart or people nearby).
        case State.DOCKING_INIT:
            if is_permitted(safety, "approach"):
                return State.DOCKING_APPROACH, "Approach permitted"

        # Docking approach state
        # Abort on a docking failure. Only Docking's landed_on_pad confirms landing
        case State.DOCKING_APPROACH:
            if docking.get("phase") == "abort":
                return State.ABORT, "Docking failed"
            if not is_permitted(safety, "approach"):
                return State.ABORT, "Approach not permitted"
            if docking.get("phase") == "descend" and not is_permitted(safety, "descent"):
                return State.ABORT, "Descent not permitted"
            if docking.get("landed_on_pad") is True:
                return State.DOCKED, "Landed on pad"

        # Docked state
        # Safety owns the battery thresholds
        case State.DOCKED:
            if safety.get("battery_recovered") is True:
                return State.IDLE, "Battery recovered"

        # Abort state
        # When Safety permits holding, retry docking, otherwise stay in ABORT
        # TODO: go to EMERGENCY_LAND when not recoverable (e.g. too many failed docking attempts)
        case State.ABORT:
            if is_permitted(safety, "hold"):
                return State.DOCKING_INIT, "Retrying docking"

        # Emergency land state
        case State.EMERGENCY_LAND:
            pass

    return state, None
