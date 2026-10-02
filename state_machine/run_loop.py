
import copy
import time

from state_machine.state_machine import State, step

TICK_HZ = 10
TICK_MS = 1000 // TICK_HZ  # 100 ms per tick

# The demo scenario is a list of tuples, where each tuple contains:
# - The time step in milliseconds.
# - The changes to apply to the world at that time (only the listed fields change).
DEMO_SCENARIO = [
    (1000, {"bird": {"detected": True, "type": "hawk", "distance_m": 30.0}}),
    (4000, {"bird": {"detected": False, "type": None, "distance_m": None}}),
    # Low battery and a moving cart: Safety blocks everything except hold
    (5000, {"battery": {"percent": 18}, "docking": {"should_dock": True, "reason": "low_battery"},
            "safety": {"permissions": {"launch": False, "tracking": False, "approach": False, "descent": False, "hold": True},
                       "battery_low": True, "battery_recovered": False, "reasons": ["low_battery", "cart_moving"]}}),
    # The cart stops, so approach and descent are permitted; low battery still blocks launch and tracking
    (6000, {"safety": {"permissions": {"launch": False, "tracking": False, "approach": True, "descent": True, "hold": True},
                       "reasons": ["low_battery"]}}),
    (6100, {"docking": {"phase": "approach"}}),
    (7000, {"docking": {"phase": "align"}}),
    (7500, {"docking": {"phase": "descend"}}),
    (8000, {"docking": {"phase": "complete", "landed_on_pad": True, "should_dock": False, "reason": None}}),
    (9000, {"battery": {"percent": 90},
            "safety": {"permissions": {"launch": True, "tracking": True, "approach": True, "descent": True, "hold": True},
                       "battery_low": False, "battery_recovered": True, "reasons": []}}),
]


# How every drone should be initialized: default values - 
def init_world() -> dict:
    return {
        "bird": {"detected": False, "type": None, "distance_m": None},
        "battery": {"percent": 100},
        "safety": {
                "permissions": {"launch": True, "tracking": True, "approach": True, "descent": True, "hold": True},
                "emergency": False,
                "abort": False,
                "battery_low": False,
                "battery_critical": False,
                "battery_recovered": True,
                "reasons": []
                },
        "docking": {"phase": "idle", "landed_on_pad": False, "should_dock": False, "reason": None},
    }


def run_world(duration_ms: int, real_time: bool = False): 
    world = init_world()
    currState = State.IDLE
    transitions = []
    for tick in range(duration_ms // TICK_MS):
        current_time_ms = tick * TICK_MS

        # 1. Update what changes in the world at the specified time
        for event_time_ms, observation in DEMO_SCENARIO:
            if current_time_ms == event_time_ms:
                for section, values in observation.items():
                    world[section].update(values)

        # 2. Run the state machine each tick using a snapshot of current situation
        snapshot = copy.deepcopy(world)
        snapshot["timestamp_ns"] = current_time_ms * 1_000_000  # Convert milliseconds to nanoseconds
        newState, reason = step(currState, snapshot)

        # 3. Only log real transitions
        if newState != currState:
            transitions.append((current_time_ms, currState, newState, reason))
            currState = newState
        if real_time:
            time.sleep(TICK_MS / 1000)  # Sleep for the duration of one tick in seconds
    return transitions
    
if __name__ == "__main__":
    transitions = run_world(10000, False)  # Run the simulation for 10 seconds (10000 ms)
    for currentTimeMs, currState, newState, reason in transitions:
        print(f"({currentTimeMs}ms) {currState.value} -> {newState.value}: Reason: {reason}")
