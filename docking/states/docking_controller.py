"""Subordinate Docking controller for INTEGRATION_README Sections 6 and 8."""

from dataclasses import asdict, dataclass
from math import hypot, isfinite

from motion_engine.contracts import (
    Command, CommandStatus, MotionCommandType, Pose, VehicleFeedback,
)
from safety_layer import config as shared
from safety_layer.safety_policy import SafetyAssessment

from .docking_state import DockingState


def valid_ns(value):
    return type(value) is int and value >= 0


def finite_number(value):
    return type(value) in (int, float) and isfinite(value)


def yaw_difference(target, current):
    return (target - current + 180) % 360 - 180


@dataclass(frozen=True)
class DockingConfig:
    alignment_tolerance_m: float = shared.DOCK_ALIGNMENT_TOLERANCE_M
    position_tolerance_m: float = shared.POSITION_ARRIVAL_TOLERANCE_M
    yaw_tolerance_deg: float = shared.DOCK_YAW_TOLERANCE_DEG
    approach_altitude_m: float = shared.DOCK_APPROACH_ALTITUDE_M
    max_observation_age_ns: int = int(shared.MAX_OBSERVATION_AGE_S * shared.NS_PER_S)
    attempt_timeout_ns: int = shared.DOCK_ATTEMPT_TIMEOUT_NS
    search_timeout_ns: int = shared.DOCK_SEARCH_TIMEOUT_NS
    bird_absence_return_ns: int = shared.BIRD_ABSENCE_RETURN_NS
    hover_return_timeout_ns: int = shared.HOVER_RETURN_TIMEOUT_NS
    command_validity_ns: int = int(shared.COMMAND_VALIDITY_S * shared.NS_PER_S)
    max_horizontal_speed_mps: float = shared.MAX_HORIZONTAL_SPEED_MPS
    max_vertical_speed_mps: float = shared.MAX_VERTICAL_SPEED_MPS

    def __post_init__(self):
        for name, value in vars(self).items():
            if name.endswith('_ns'):
                good = valid_ns(value) and value > 0
            else:
                good = finite_number(value) and value > 0
            if not good:
                raise ValueError(f'{name} must be finite and positive (integer for ns)')
        if self.yaw_tolerance_deg > 180:
            raise ValueError('yaw_tolerance_deg must not exceed 180')
        if self.approach_altitude_m > shared.ALTITUDE_CEILING_M:
            raise ValueError('approach altitude exceeds shared ceiling')
        if self.max_horizontal_speed_mps > shared.MAX_HORIZONTAL_SPEED_MPS:
            raise ValueError('horizontal speed exceeds shared limit')
        if self.max_vertical_speed_mps > shared.MAX_VERTICAL_SPEED_MPS:
            raise ValueError('vertical speed exceeds shared limit')


@dataclass(frozen=True)
class PadObservation:
    pose: Pose
    timestamp_ns: int
    valid: bool = True
    frame: str = 'local_enu'
    source_id: str = 'synthetic_pad'
    synthetic: bool = True
    reason: str | None = None


@dataclass(frozen=True)
class DockingInput:
    mission_state: str = 'IDLE'
    flight_active: bool = False
    vehicle: VehicleFeedback | None = None
    pad: PadObservation | None = None
    safety: SafetyAssessment | None = None
    alignment_valid: bool = False
    alignment_timestamp_ns: int | None = None
    x_error_m: float | None = None
    y_error_m: float | None = None
    yaw_error_deg: float | None = None
    alignment_source_id: str = 'synthetic_alignment'
    cart_moving: bool | None = None
    environment_valid: bool = False
    environment_timestamp_ns: int | None = None
    bird_detected: bool | None = None
    bird_valid: bool = False
    bird_timestamp_ns: int | None = None
    # Shared mission timers belong to the coordinator, never this controller.
    bird_absent_since_ns: int | None = None
    hover_started_ns: int | None = None
    retry_requested: bool = False
    reset_requested: bool = False


@dataclass(frozen=True)
class ReturnRequest:
    should_dock: bool
    reasons: tuple[str, ...] = ()


@dataclass(frozen=True)
class DockingUpdate:
    previous_state: DockingState
    state: DockingState
    reason: str
    timestamp_ns: int
    should_dock: bool = False
    reasons: tuple[str, ...] = ()
    proposed_target: Command | None = None
    landed_on_pad: bool = False
    failure: bool = False
    failure_reason: str | None = None
    progress: str = 'idle'
    attempt_started_ns: int | None = None
    search_started_ns: int | None = None
    schema_version: int = 1
    clock: str = 'simulation'
    synthetic: bool = True

    @property
    def phase(self):
        return self.state

    def to_dict(self):
        result = asdict(self)
        result['previous_state'] = self.previous_state.value
        result['state'] = self.state.value
        result['phase'] = self.phase.value
        result['proposed_target'] = (
            self.proposed_target.to_dict() if self.proposed_target else None
        )
        return result


def fresh(timestamp_ns, now_ns, age_ns):
    return valid_ns(timestamp_ns) and 0 <= now_ns - timestamp_ns <= age_ns


def compute_return_request(inputs: DockingInput, now_ns: int,
                           config: DockingConfig) -> ReturnRequest:
    """Evaluate return causes; the coordinator owns timers and the return latch."""
    if inputs.flight_active is not True or inputs.mission_state == 'IDLE':
        return ReturnRequest(False)
    reasons = []
    safety = inputs.safety
    if isinstance(safety, SafetyAssessment) and isinstance(safety.permissions, dict):
        if safety.battery_low is True:
            reasons.append('low_battery')
        # Invalid geometry alone holds first; hover timeout requests return.
        excluded = {'invalid_alignment', 'missing_pad', 'tracking_geometry_unavailable'}
        if safety.permissions.get('tracking') is not True:
            codes = safety.reasons if isinstance(safety.reasons, tuple) else ()
            reasons.extend(code for code in codes
                           if isinstance(code, str) and code not in excluded)
            if not reasons and not safety.reasons:
                reasons.append('tracking_restricted')
    bird_ok = (inputs.bird_valid is True
               and type(inputs.bird_detected) is bool
               and fresh(inputs.bird_timestamp_ns, now_ns, config.max_observation_age_ns))
    if not bird_ok:
        reasons.append('camera_failure')
    elif inputs.bird_detected is False:
        start = inputs.bird_absent_since_ns
        if valid_ns(start) and start <= now_ns - config.bird_absence_return_ns:
            reasons.append('bird_lost')
    if inputs.mission_state == 'HOVERING':
        start = inputs.hover_started_ns
        if valid_ns(start) and start <= now_ns - config.hover_return_timeout_ns:
            reasons.append('hover_timeout')
    return ReturnRequest(bool(reasons), tuple(dict.fromkeys(reasons)))


class DockingController:
    """Return values only; never dispatch commands or mutate mission context."""

    def __init__(self, config: DockingConfig | None = None):
        self.config = config or DockingConfig()
        self._state = DockingState.IDLE
        self._transitions = []
        self._last_update_ns = None
        self._attempt_started_ns = None
        self._search_started_ns = None
        self._attempt_id = 0
        self._proposal_id = 0
        self._last_request = None
        self._failure_reason = None

    @property
    def state(self):
        return self._state

    @property
    def transitions(self):
        return tuple(self._transitions)

    def _deactivate(self):
        self._state = DockingState.IDLE
        self._attempt_started_ns = None
        self._search_started_ns = None
        self._last_request = None
        self._failure_reason = None

    def _vehicle_ok(self, inputs, now_ns):
        vehicle = inputs.vehicle
        return (isinstance(vehicle, VehicleFeedback) and vehicle.valid is True
                and vehicle.synthetic is True and vehicle.schema_version == 1
                and isinstance(vehicle.pose, Pose) and vehicle.pose.is_finite()
                and all(type(getattr(vehicle, name)) is bool for name in
                        ('grounded', 'airborne', 'armed', 'landing_settled', 'emergency_descent'))
                and not (vehicle.grounded and vehicle.airborne)
                and fresh(vehicle.timestamp_ns, now_ns, self.config.max_observation_age_ns))

    def _pad_ok(self, inputs, now_ns):
        pad = inputs.pad
        return (isinstance(pad, PadObservation) and pad.valid is True
                and pad.synthetic is True and pad.frame == 'local_enu'
                and isinstance(pad.source_id, str) and bool(pad.source_id)
                and isinstance(pad.pose, Pose) and pad.pose.is_finite()
                and pad.pose.z == 0
                and fresh(pad.timestamp_ns, now_ns, self.config.max_observation_age_ns))

    def _environment_ok(self, inputs, now_ns):
        return (inputs.environment_valid is True and type(inputs.cart_moving) is bool
                and fresh(inputs.environment_timestamp_ns, now_ns,
                          self.config.max_observation_age_ns))

    def _safety_ok(self, inputs, now_ns):
        safety = inputs.safety
        return (isinstance(safety, SafetyAssessment)
                and type(safety.emergency) is bool and type(safety.abort) is bool
                and isinstance(safety.permissions, dict)
                and all(type(value) is bool for value in safety.permissions.values())
                and isinstance(safety.reasons, tuple)
                and all(isinstance(reason, str) for reason in safety.reasons)
                and fresh(safety.timestamp_ns, now_ns, self.config.max_observation_age_ns))

    def _alignment_ok(self, inputs, now_ns):
        return (inputs.alignment_valid is True
                and isinstance(inputs.alignment_source_id, str)
                and bool(inputs.alignment_source_id)
                and fresh(inputs.alignment_timestamp_ns, now_ns,
                          self.config.max_observation_age_ns)
                and all(finite_number(v) for v in
                        (inputs.x_error_m, inputs.y_error_m, inputs.yaw_error_deg)))

    def _aligned(self, inputs):
        return (hypot(inputs.x_error_m, inputs.y_error_m)
                <= self.config.alignment_tolerance_m
                and abs(yaw_difference(inputs.yaw_error_deg, 0))
                <= self.config.yaw_tolerance_deg)

    def _landed(self, inputs, now_ns):
        vehicle = inputs.vehicle
        return (self._vehicle_ok(inputs, now_ns) and self._pad_ok(inputs, now_ns)
                and self._environment_ok(inputs, now_ns) and inputs.cart_moving is False
                and vehicle.grounded is True and vehicle.airborne is False
                and vehicle.emergency_descent is False and vehicle.landing_settled is True
                and vehicle.status == CommandStatus.LANDED
                and vehicle.active_kind in (MotionCommandType.DESCEND_TO_DOCK,
                                            MotionCommandType.LAND)
                and isinstance(vehicle.active_command_id, str)
                and bool(vehicle.active_command_id)
                and isinstance(vehicle.target, Pose) and vehicle.target.is_finite()
                and vehicle.pose.z == 0 and vehicle.target.z == 0
                and vehicle.pose.horizontal_distance_to(inputs.pad.pose)
                <= self.config.alignment_tolerance_m
                and vehicle.target.horizontal_distance_to(inputs.pad.pose)
                <= self.config.alignment_tolerance_m)

    def update(self, inputs: DockingInput, now_ns: int) -> DockingUpdate:
        if not valid_ns(now_ns):
            raise ValueError('now_ns must be nonnegative integer simulation nanoseconds')
        if self._last_update_ns is not None and now_ns < self._last_update_ns:
            raise ValueError('Simulation time must not move backwards')
        previous = self.state
        request = compute_return_request(inputs, now_ns, self.config)
        phase, reason = self._next_state(inputs, now_ns)
        if phase == DockingState.SEARCH and previous != phase:
            self._search_started_ns = now_ns
        elif phase != DockingState.SEARCH:
            self._search_started_ns = None
        self._state = phase
        target = self._propose(inputs, now_ns)
        if target is not None and target.validate() is not None:
            phase, reason, target = DockingState.ABORT, 'invalid_target', None
            self._state = phase
        if phase == DockingState.ABORT:
            self._failure_reason = reason
        result = DockingUpdate(
            previous, phase, reason, now_ns, request.should_dock, request.reasons,
            target, phase == DockingState.COMPLETE, phase == DockingState.ABORT,
            reason if phase == DockingState.ABORT else None, phase.value,
            self._attempt_started_ns, self._search_started_ns,
        )
        if phase != previous:
            self._transitions.append(result)
        self._last_update_ns = now_ns
        return result

    def _next_state(self, inputs, now_ns):
        if inputs.mission_state not in (
            'IDLE', 'HOVERING', 'TRACKING', 'DOCKING_INIT', 'DOCKING_APPROACH',
            'DOCKED', 'ABORT', 'EMERGENCY_LAND',
        ):
            return DockingState.ABORT, 'invalid_mission_state'
        if inputs.mission_state != 'DOCKING_APPROACH':
            self._deactivate()
            return DockingState.IDLE, 'docking_inactive'
        if any(type(value) is not bool for value in
               (inputs.flight_active, inputs.retry_requested, inputs.reset_requested)):
            return DockingState.ABORT, 'invalid_input'
        if inputs.reset_requested is True:
            reset_ok = (self._vehicle_ok(inputs, now_ns) and self._pad_ok(inputs, now_ns)
                        and inputs.vehicle.grounded and not inputs.vehicle.armed
                        and inputs.vehicle.pose.z == 0
                        and inputs.vehicle.pose.horizontal_distance_to(inputs.pad.pose)
                        <= self.config.alignment_tolerance_m
                        and self._environment_ok(inputs, now_ns)
                        and inputs.cart_moving is False
                        and self._safety_ok(inputs, now_ns)
                        and inputs.safety.emergency is False
                        and inputs.safety.abort is False)
            if reset_ok:
                self._deactivate()
                return DockingState.IDLE, 'docking_reset'
            return DockingState.ABORT, 'reset_rejected'
        if self.state == DockingState.COMPLETE:
            return self.state, 'landed_on_pad'
        if self.state == DockingState.ABORT and inputs.retry_requested is not True:
            return self.state, self._failure_reason or 'docking_failed'
        if not self._safety_ok(inputs, now_ns):
            return DockingState.ABORT, 'invalid_safety'
        if inputs.safety.emergency is True:
            return DockingState.ABORT, 'emergency_required'
        if inputs.safety.abort is True:
            return DockingState.ABORT, 'safety_abort'
        if inputs.safety.permissions.get('approach') is not True:
            return DockingState.ABORT, 'approach_permission_lost'
        if not self._vehicle_ok(inputs, now_ns):
            return DockingState.ABORT, 'stale_vehicle'
        if not self._pad_ok(inputs, now_ns):
            return DockingState.ABORT, 'invalid_pad'
        if not self._environment_ok(inputs, now_ns) or inputs.cart_moving is not False:
            return DockingState.ABORT, 'cart_not_stationary'
        if self.state == DockingState.IDLE or (
            self.state == DockingState.ABORT and inputs.retry_requested is True
        ):
            self._attempt_id += 1
            self._attempt_started_ns = now_ns
            self._last_request = None
            self._failure_reason = None
            return DockingState.APPROACH, 'docking_entry'
        if now_ns - self._attempt_started_ns >= self.config.attempt_timeout_ns:
            return DockingState.ABORT, 'docking_attempt_timeout'
        alignment_ok = self._alignment_ok(inputs, now_ns)
        if self.state == DockingState.APPROACH:
            vehicle, pad = inputs.vehicle.pose, inputs.pad.pose
            arrived = (hypot(vehicle.x - pad.x, vehicle.y - pad.y)
                       <= self.config.position_tolerance_m
                       and abs(vehicle.z - self.config.approach_altitude_m)
                       <= self.config.position_tolerance_m
                       and abs(yaw_difference(pad.yaw_deg, vehicle.yaw_deg))
                       <= self.config.yaw_tolerance_deg)
            if arrived:
                return (DockingState.ALIGN if alignment_ok else DockingState.SEARCH,
                        'approach_reached')
            return self.state, 'approaching_pad'
        if self.state == DockingState.SEARCH:
            if now_ns - self._search_started_ns >= self.config.search_timeout_ns:
                return DockingState.ABORT, 'alignment_search_timeout'
            if alignment_ok:
                return DockingState.ALIGN, 'alignment_acquired'
            return self.state, 'alignment_unavailable'
        if self.state == DockingState.ALIGN:
            if not alignment_ok:
                return DockingState.SEARCH, 'alignment_unavailable'
            if (self._aligned(inputs)
                    and inputs.safety.permissions.get('descent') is True):
                return DockingState.DESCEND, 'descent_permitted'
            return self.state, 'aligning_pad'
        if self.state == DockingState.DESCEND:
            if (not alignment_ok or not self._aligned(inputs)
                    or inputs.safety.permissions.get('descent') is not True):
                return DockingState.ABORT, 'descent_permission_lost'
            if self._landed(inputs, now_ns):
                return DockingState.COMPLETE, 'landed_on_pad'
            return self.state, 'descending_to_pad'
        return DockingState.ABORT, 'invalid_phase'

    def _propose(self, inputs, now_ns):
        if inputs.mission_state != 'DOCKING_APPROACH' or self.state in (
            DockingState.IDLE, DockingState.COMPLETE, DockingState.ABORT
        ):
            self._last_request = None
            return None
        pad = inputs.pad.pose
        if self.state == DockingState.SEARCH:
            kind, pose = MotionCommandType.HOVER, None
        elif self.state == DockingState.DESCEND:
            kind, pose = MotionCommandType.DESCEND_TO_DOCK, pad
        elif self.state == DockingState.ALIGN:
            vehicle = inputs.vehicle.pose
            kind = MotionCommandType.MOVE_TO
            pose = Pose(vehicle.x + inputs.x_error_m, vehicle.y + inputs.y_error_m,
                        self.config.approach_altitude_m,
                        (vehicle.yaw_deg + inputs.yaw_error_deg) % 360)
        else:
            kind = MotionCommandType.GOTO_DOCK_APPROACH
            pose = Pose(pad.x, pad.y, self.config.approach_altitude_m, pad.yaw_deg)
        request = (kind, pose)
        if request != self._last_request:
            self._proposal_id += 1
            self._last_request = request
        return Command(
            command_id=f'docking-{self._attempt_id}-{self._proposal_id}',
            kind=kind, mission_state='DOCKING_APPROACH', issued_ns=now_ns,
            expires_ns=now_ns + self.config.command_validity_ns,
            source='docking', reason=self.state.value, target=pose,
            max_horizontal_speed_mps=self.config.max_horizontal_speed_mps,
            max_vertical_speed_mps=self.config.max_vertical_speed_mps,
        )
