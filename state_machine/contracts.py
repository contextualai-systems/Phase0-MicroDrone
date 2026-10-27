import dataclasses

from state_machine.state_machine import State


@dataclasses.dataclass(frozen=True)
class MissionStatus:
    """The mission's status for one tick, published by the coordinator (run_loop.py) every tick."""

    timestamp_ns: int  # simulation time of this tick
    tick: int  # tick number, starting at 0
    previous_state: State  # mission state at the start of the tick
    state: State  # mission state after step()
    reason: str | None  # why the state changed, or None if it did not change
    state_entered_ns: int  # simulation time when the current state was entered
    # "none", "climbing" or "complete". The demos reach "complete" once step() waits for the climb
    # before TRACKING (contract section 7, not implemented yet).
    launch_phase: str
    flight_active: bool  # whether the drone is armed or airborne after this tick's move
    bird_absent_since_ns: int | None  # first tick the bird was missing, or None while it is seen
    hover_started_ns: int | None  # when ordinary hovering began (not the launch climb), or None outside it
    return_started_ns: int | None  # when the first return began, or None if no return has started
    return_reason: str | None  # why the return started, or None if no return has started
    pending_command_failure: str | None  # why Motion rejected the last command, or None if there is none
    outcome: str | None  # "completed", "interrupted" or "failed" on the last tick, otherwise None
    schema_version: int = 1
    clock: str = "simulation"
    synthetic: bool = True

    def to_dict(self) -> dict:
        result = dataclasses.asdict(self)
        result["previous_state"] = self.previous_state.value
        result["state"] = self.state.value
        return result
