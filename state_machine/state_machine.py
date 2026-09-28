# State machine code: picks the next state based on the current state and the input event.
# It also handles transitions and actions associated with each state.

from dataclasses import dataclass
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


@dataclass
class Bird:
    type: str | None = None
    detected: bool = False
    distance_m: float | None = None


@dataclass
class Battery:
    percentage: float = 100.0
    low: bool = False

# The observation is a dictionary in the team's shared JSON format
def step(state: State, observation: dict) -> tuple[State, str | None]:
    # Placeholder logic for state transition
    bird = observation.get("bird") or {}
    if state == State.IDLE:
        if bird.get("detected"):
            return State.TRACKING, "Bird detected"
    elif state == State.TRACKING:
        if not bird.get("detected"):
            return State.IDLE, "Bird not detected"
    # Add more state transition logic as needed
    return state, None
