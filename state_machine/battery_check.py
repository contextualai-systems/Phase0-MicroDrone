# Battery check: warns when battery drops below the low-battery threshold and
# sends the drone to DOCKING_INIT.
#
# Uses the State / Observation / Battery types from state_machine.py and follows
# the same contract as step(): (state, observation) -> (next_state, reason).
# It has no side effects on the observation; the only side effect is the warning log.

import logging
from typing import Iterator

from state_machine.state_machine import Battery, Observation, State

logger = logging.getLogger(__name__)

LOW_BATTERY_PERCENT = 20.0
LOW_BATTERY_REASON = "Low battery"

# States that can be sent to DOCKING_INIT (see the transition table in README.md).
# States already docking, docked, aborting or landing are left alone.
CAN_START_DOCKING = {State.IDLE, State.TRACKING, State.HOVERING}


def is_battery_low(battery: Battery | None) -> bool:
    """True when battery percentage is below LOW_BATTERY_PERCENT. Missing battery is not low."""
    return battery is not None and battery.percentage < LOW_BATTERY_PERCENT


def check_battery(state: State, observation: Observation) -> tuple[State, str | None]:
    """Return (DOCKING_INIT, "Low battery") when battery < 20%, otherwise (state, None)."""
    if not is_battery_low(observation.battery):
        return state, None

    percent = observation.battery.percentage
    logger.warning(
        "Battery low: %.1f%% (threshold %.0f%%) in state %s",
        percent, LOW_BATTERY_PERCENT, state.value,
    )

    if state in CAN_START_DOCKING:
        return State.DOCKING_INIT, LOW_BATTERY_REASON
    return state, None


def synthetic_battery(start: float = 100.0, drain_per_tick: float = 5.0) -> Iterator[Battery]:
    """Yield synthetic Battery readings that drain by drain_per_tick each tick, stopping at 0%."""
    percent = start
    while True:
        battery = Battery(percentage=percent, low=percent < LOW_BATTERY_PERCENT)
        yield battery
        if percent <= 0:
            return
        percent = max(0.0, percent - drain_per_tick)


if __name__ == "__main__":
    # Demo: drain a synthetic battery while tracking and show when it triggers docking.
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    state = State.TRACKING
    for battery in synthetic_battery(start=40.0, drain_per_tick=5.0):
        next_state, reason = check_battery(state, Observation(battery=battery))
        if next_state != state:
            print(f"STATE: transition {state.value} -> {next_state.value} (reason={reason}) at {battery.percentage:.0f}%")
            state = next_state
            break
        print(f"STATE: {state.value} battery={battery.percentage:.0f}%")
