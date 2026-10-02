"""Deterministic docking transitions for synthetic Phase 0 inputs."""

from dataclasses import dataclass
from math import hypot, isfinite

from .docking_state import DockingState


@dataclass(frozen=True)
class DockingConfig:
    alignment_tolerance_m: float = 0.1
    max_alignment_age_s: float = 0.5

    def __post_init__(self):
        for value in (self.alignment_tolerance_m, self.max_alignment_age_s):
            if not isfinite(value) or value <= 0:
                raise ValueError("Docking thresholds must be finite and positive")


@dataclass(frozen=True)
class DockingInput:
    """Offsets are pad minus vehicle position in a shared local NED frame.

    Timestamps use the same simulation clock as update's now_s. Navigation
    supplies near_pad; safety supplies abort_reason; landing_confirmed must
    come from fresh vehicle status plus landing evaluation, not a land command.
    """

    docking_requested: bool = False
    near_pad: bool = False
    alignment_valid: bool = False
    alignment_timestamp_s: float | None = None
    north_error_m: float | None = None
    east_error_m: float | None = None
    landing_confirmed: bool = False
    abort_reason: str | None = None


@dataclass(frozen=True)
class DockingUpdate:
    previous_state: DockingState
    state: DockingState
    reason: str
    timestamp_s: float


class DockingController:
    """One transition per update; no flight commands or hardware access."""

    def __init__(self, config: DockingConfig | None = None):
        self.config = config if config is not None else DockingConfig()
        self._state = DockingState.IDLE
        self._transitions: list[DockingUpdate] = []
        self._last_update_s: float | None = None

    @property
    def state(self) -> DockingState:
        return self._state

    @property
    def transitions(self) -> tuple[DockingUpdate, ...]:
        return tuple(self._transitions)

    def update(self, inputs: DockingInput, now_s: float) -> DockingUpdate:
        if not isfinite(now_s) or now_s < 0:
            raise ValueError("now_s must be finite and nonnegative")
        if self._last_update_s is not None and now_s < self._last_update_s:
            raise ValueError("Simulation time must not move backwards")

        state, reason = self._next_state(inputs, now_s)
        result = DockingUpdate(self.state, state, reason, now_s)
        if state != self.state:
            self._transitions.append(result)
        self._state = state
        self._last_update_s = now_s
        return result

    def _alignment_available(self, inputs: DockingInput, now_s: float) -> bool:
        values = (
            inputs.alignment_timestamp_s,
            inputs.north_error_m,
            inputs.east_error_m,
        )
        if not inputs.alignment_valid or any(
            value is None or not isfinite(value) for value in values
        ):
            return False
        timestamp = inputs.alignment_timestamp_s
        return (
            timestamp >= 0
            and 0 <= now_s - timestamp <= self.config.max_alignment_age_s
        )

    def _next_state(
        self, inputs: DockingInput, now_s: float
    ) -> tuple[DockingState, str]:
        state = self.state
        if state in (DockingState.COMPLETE, DockingState.ABORT):
            return state, "Terminal state; create a new controller for another attempt"
        if inputs.abort_reason is not None:
            return DockingState.ABORT, f"Safety abort: {inputs.abort_reason}"
        if state == DockingState.IDLE:
            if inputs.docking_requested:
                return DockingState.APPROACH, "Docking requested"
            return state, "Waiting for docking request"
        if state == DockingState.APPROACH:
            if inputs.near_pad:
                return DockingState.SEARCH, "Navigation reports vehicle near pad"
            return state, "Waiting to reach pad vicinity"

        if not self._alignment_available(inputs, now_s):
            if state == DockingState.DESCEND:
                return DockingState.ABORT, "Alignment unavailable during descent"
            return DockingState.SEARCH, "Waiting for valid, fresh synthetic alignment"
        if state == DockingState.SEARCH:
            return DockingState.ALIGN, "Valid, fresh synthetic alignment acquired"

        aligned = hypot(inputs.north_error_m, inputs.east_error_m) <= (
            self.config.alignment_tolerance_m
        )
        if state == DockingState.ALIGN:
            if aligned:
                return DockingState.DESCEND, "Horizontal error within tolerance"
            return state, "Waiting for horizontal alignment"
        if not aligned:
            return DockingState.ALIGN, "Horizontal drift; alignment required again"
        if inputs.landing_confirmed:
            return DockingState.COMPLETE, "Landing confirmed within pad tolerance"
        return state, "Waiting for confirmed landing"
