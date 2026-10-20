"""Failure injection for the Motion Engine (START_HERE "Definition of done").

Phase 0 is done when "missing synthetic inputs, stale data, command loss,
geofence violations, and low-battery scenarios have defined, tested
outcomes" (START_HERE.md) and "rejected/expired commands cannot leave an
unsafe previous target active" (INTEGRATION_README.md §11). This file
injects those failures into a running `MotionStub`, mid-flight, and checks
the outcome Motion is responsible for.

Every scenario is recorded with `TraceRecorder` and, at the end, the whole
run goes through `trace_checker.check_trace()`. So on top of each test's
own asserts, no scenario may ever exceed a speed/yaw limit, leave the
geofence, move while a command is expired, report a landing in the air, or
let a rejected command change what Motion is flying.

Sections:
  A. Command loss         -- the coordinator stops renewing
  B. Loop stalls / clock  -- big time jumps, a clock running backwards
  C. Stale and replayed commands
  D. Geofence and ceiling under tracking
  E. Emergency injected in every flight phase
  F. Hazards through the real Safety layer (cart motion, low/critical battery)
  G. Garbage input immunity
  H. Arrival above the pad is not a landing
  I. Bad tick length (dt) -- zero, negative, NaN, inf, non-numeric
"""

import sys
from dataclasses import replace
from fractions import Fraction
from pathlib import Path
from typing import Callable, List, Optional

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from motion_engine.contracts import (
    Command,
    CommandStatus,
    MemoryLogSink,
    MotionCommandType as K,
    Pose,
    Reason,
    SimulationClock,
    VehicleFeedback,
)
from motion_engine.motion_stubs import MotionLimits, MotionStub
from motion_engine.safety_adapter import check_for_motion
from motion_engine.trace_checker import TraceRecorder, TraceReport, check_trace
from safety_layer.safety_policy import ACTIONS, SafetyAssessment

DT = 0.1
DT_NS = 100_000_000
S = 1_000_000_000
#: INTEGRATION_README §5: a command is valid for 0.2 s unless renewed.
VALIDITY_NS = 200_000_000
PAD = Pose(0.0, 0.0, 0.0, 0.0)
LIM = MotionLimits()


def assessment(blocked=(), emergency=False):
    return SafetyAssessment(
        permissions={a: a not in blocked for a in ACTIONS}, emergency=emergency
    )


ALL_OK = assessment()
#: Cart in motion: Safety allows hold only (INTEGRATION_README §6).
CART_MOVING = assessment(blocked=("launch", "tracking", "approach", "descent"))
#: Low battery: no launch or tracking, but docking is still allowed.
LOW_BATTERY = assessment(blocked=("launch", "tracking"))


# --------------------------------------------------------------------------
# A coordinator stand-in that records everything
# --------------------------------------------------------------------------


class Rig:
    """Coordinator tick loop around a recorded `MotionStub`.

    One `tick()` = submit any commands, step Motion once by 0.1 s, advance
    the simulation clock. Every tick lands in `self.rec.samples`.
    """

    def __init__(self, limits: Optional[MotionLimits] = None) -> None:
        self.clock = SimulationClock()
        self.log = MemoryLogSink()
        self.motion = MotionStub(logger=self.log, clock=self.clock, limits=limits)
        self.rec = TraceRecorder(self.motion)
        self.rec.snapshot(self.now)
        self._n = 0

    # -- building commands ------------------------------------------------

    @property
    def now(self) -> int:
        return self.clock.timestamp_ns

    def cmd(self, kind, target: Optional[Pose] = None, *, cid: Optional[str] = None,
            lifecycle: bool = False, issued: Optional[int] = None) -> Command:
        if cid is None:
            self._n += 1
            cid = f"c{self._n}"
        issued = self.now if issued is None else issued
        return Command(
            command_id=cid, kind=kind, mission_state="TEST", issued_ns=issued,
            expires_ns=None if lifecycle else issued + VALIDITY_NS,
            source="test", reason="failure_injection", target=target,
        )

    def renewed(self, c: Command) -> Command:
        """The same request, re-issued now (what the coordinator sends each tick)."""
        if c.expires_ns is None:
            return c
        return replace(c, issued_ns=self.now, expires_ns=self.now + VALIDITY_NS)

    # -- running ----------------------------------------------------------

    def submit(self, c, now_ns: Optional[int] = None):
        return self.rec.submit(c, self.now if now_ns is None else now_ns)

    def tick(self, *cmds) -> VehicleFeedback:
        for c in cmds:
            self.submit(c)
        self.rec.step(DT, self.now)
        self.clock.advance(DT)
        return self.motion.feedback(self.now)

    def fly(self, c: Command, until: Callable[[VehicleFeedback], bool],
            max_ticks: int = 3000) -> VehicleFeedback:
        """Send `c`, renewing it every tick, until `until(feedback)` holds."""
        for _ in range(max_ticks):
            ack = self.submit(self.renewed(c))
            assert ack.accepted, ack
            fb = self.tick()
            if until(fb):
                return fb
        raise AssertionError(f"never reached the condition; pose={self.motion.pose}")

    def launch(self, altitude: float = 5.0) -> None:
        assert self.submit(self.cmd(K.ARM, lifecycle=True)).accepted
        assert self.submit(self.cmd(K.TAKEOFF, Pose(0, 0, altitude, 0), lifecycle=True)).accepted
        while self.motion.pose.z != altitude:
            self.tick()

    def goto(self, target: Pose, kind=K.MOVE_TO) -> Command:
        c = self.cmd(kind, target)
        self.fly(c, lambda fb: self.motion.pose == target)
        return c

    def check(self, **kw) -> TraceReport:
        report = check_trace(self.rec.samples, limits=self.motion.limits,
                             home=self.motion.home, **kw)
        assert report.ok, report.format_text(max_violations=20)
        return report


def records(log: MemoryLogSink, event: str):
    return [r for r in log.records if r.event == event]


def poses(rig: Rig, start: int = 0) -> List[Pose]:
    return [s.pose for s in rig.rec.samples[start:]]


# ==========================================================================
# A. Command loss -- the coordinator stops renewing mid-flight
# ==========================================================================

# (name, set-up flight, command to lose, how far in before losing it)
LOSS_CASES = {
    "move_to": (5.0, K.MOVE_TO, Pose(40.0, 0.0, 5.0, 0.0)),
    "track_target": (5.0, K.TRACK_TARGET, Pose(30.0, 30.0, 8.0, 45.0)),
    "dock_approach": (5.0, K.GOTO_DOCK_APPROACH, Pose(0.0, 0.0, 5.0, 0.0)),
    "set_yaw": (5.0, K.SET_YAW, Pose(0.0, 0.0, 0.0, 170.0)),
    "descend_to_dock": (12.0, K.DESCEND_TO_DOCK, PAD),
}


@pytest.mark.parametrize("case", sorted(LOSS_CASES))
def test_command_loss_freezes_the_vehicle_within_one_validity_window(case):
    altitude, kind, target = LOSS_CASES[case]
    rig = Rig()
    rig.launch(altitude)
    if kind == K.GOTO_DOCK_APPROACH:
        rig.goto(Pose(30.0, -20.0, 5.0, 0.0))  # start somewhere away from the pad

    c = rig.cmd(kind, target)
    for _ in range(8):                         # 0.8 s of normal, renewed flight
        assert rig.submit(rig.renewed(c)).accepted
        fb = rig.tick()
    assert fb.status == CommandStatus.IN_PROGRESS
    pose_at_loss = rig.motion.pose
    last_renewal_ns = rig.now - DT_NS

    # Renewals stop. The last one is valid until last_renewal + 0.2 s, so
    # Motion may fly exactly one more tick and must hold from the next.
    fb = rig.tick()
    one_more_tick = rig.motion.pose
    assert fb.status == CommandStatus.IN_PROGRESS
    fb = rig.tick()
    assert fb.status == CommandStatus.EXPIRED_HOLD
    assert rig.now - DT_NS >= last_renewal_ns + VALIDITY_NS

    frozen = rig.motion.pose
    assert frozen == one_more_tick, "moved in the tick the command expired"
    assert pose_at_loss.horizontal_distance_to(frozen) <= LIM.max_horizontal_speed_mps * DT + 1e-9
    assert abs(pose_at_loss.z - frozen.z) <= LIM.max_vertical_speed_mps * DT + 1e-9

    for _ in range(30):                        # 3 s with no coordinator at all
        fb = rig.tick()
        assert rig.motion.pose == frozen
        assert fb.status == CommandStatus.EXPIRED_HOLD
    assert fb.armed
    assert len(records(rig.log, "command_expired")) == 1
    assert records(rig.log, "command_expired")[0].decision == "hold_in_place"
    if kind == K.DESCEND_TO_DOCK:
        assert fb.airborne and not fb.landing_settled, "an expired descent must not count as landed"
    report = rig.check()
    assert report.metrics["expired_hold_ticks"] == 31


def test_one_dropped_renewal_is_tolerated():
    rig = Rig()
    rig.launch()
    c = rig.cmd(K.MOVE_TO, Pose(30.0, 0.0, 5.0, 0.0))
    rig.tick(c)
    rig.tick()                                  # one renewal lost on the way
    ack = rig.submit(rig.renewed(c))            # the next one arrives in time
    assert ack.accepted and ack.reason == Reason.RENEWED
    fb = rig.tick()
    assert fb.status == CommandStatus.IN_PROGRESS
    assert not records(rig.log, "command_expired")
    assert rig.motion.pose.x == pytest.approx(3 * LIM.max_horizontal_speed_mps * DT)
    rig.check()


def test_after_expiry_the_stale_id_cannot_be_revived_but_a_fresh_id_resumes():
    rig = Rig()
    rig.launch()
    goal = Pose(30.0, 0.0, 5.0, 0.0)
    c = rig.cmd(K.MOVE_TO, goal)
    rig.tick(c)
    rig.tick()
    fb = rig.tick()
    assert fb.status == CommandStatus.EXPIRED_HOLD
    held = rig.motion.pose

    late = rig.submit(rig.renewed(c))           # renewal arrives after expiry
    assert not late.accepted and late.reason == Reason.COMMAND_ID_REUSED
    rig.tick()
    assert rig.motion.pose == held

    fresh = rig.cmd(K.MOVE_TO, goal)            # coordinator re-plans with a new id
    fb = rig.fly(fresh, lambda fb: fb.status == CommandStatus.ARRIVED)
    assert rig.motion.pose == goal
    rig.check()


def test_lifecycle_commands_complete_without_any_renewal():
    rig = Rig()
    assert rig.tick(rig.cmd(K.ARM, lifecycle=True)).armed
    rig.tick(rig.cmd(K.TAKEOFF, Pose(0, 0, 6.0, 0), lifecycle=True))
    for _ in range(40):
        fb = rig.tick()
    assert rig.motion.pose.z == 6.0 and fb.status == CommandStatus.ARRIVED
    rig.tick(rig.cmd(K.LAND, lifecycle=True))
    for _ in range(40):
        fb = rig.tick()
    assert fb.landing_settled and fb.status == CommandStatus.LANDED
    assert not records(rig.log, "command_expired")
    rig.check()


def test_emergency_descent_never_expires():
    rig = Rig()
    rig.launch(4.0)
    rig.tick(rig.cmd(K.EMERGENCY_LAND, lifecycle=True))
    for _ in range(int(4.0 / LIM.emergency_descent_mps / DT) + 2):
        fb = rig.tick()
    assert not fb.armed and fb.grounded and not fb.emergency_descent
    assert not records(rig.log, "command_expired")
    rig.check()


# ==========================================================================
# B. Loop stalls and clock faults
# ==========================================================================


@pytest.mark.parametrize("stall_s,dt", [(1.0, DT), (5.0, DT), (5.0, 5.0)])
def test_loop_stall_expires_the_target_before_moving(stall_s, dt):
    """The coordinator freezes for `stall_s`; the next step must not keep
    chasing the old target, even if it is handed the whole gap as `dt`."""
    rig = Rig()
    rig.launch()
    c = rig.cmd(K.MOVE_TO, Pose(100.0, 0.0, 5.0, 0.0))
    for _ in range(3):
        rig.tick(rig.renewed(c))
    before = rig.motion.pose

    rig.clock.advance(stall_s)
    rig.rec.step(dt, rig.now)
    rig.clock.advance(dt)
    fb = rig.motion.feedback(rig.now)
    assert rig.motion.pose == before
    assert fb.status == CommandStatus.EXPIRED_HOLD
    rig.check()


def test_clock_running_backwards_does_not_revive_an_expired_command():
    rig = Rig()
    rig.launch()
    c = rig.cmd(K.MOVE_TO, Pose(50.0, 0.0, 5.0, 0.0))
    rig.tick(c)
    rig.tick()
    assert rig.tick().status == CommandStatus.EXPIRED_HOLD
    held = rig.motion.pose

    # Feed Motion a timestamp from before the expiry (not recorded: the
    # trace checker would rightly reject a backwards clock).
    rig.motion.step(DT, now_ns=c.issued_ns)
    assert rig.motion.pose == held
    assert rig.motion.feedback(rig.now).status == CommandStatus.EXPIRED_HOLD
    rig.check()


# ==========================================================================
# C. Stale and replayed commands
# ==========================================================================


def test_late_expired_command_is_rejected_and_the_current_target_keeps_flying():
    rig = Rig()
    rig.launch()
    goal = Pose(20.0, 0.0, 5.0, 0.0)
    current = rig.cmd(K.MOVE_TO, goal)
    rig.tick(current)
    for _ in range(5):
        rig.tick(rig.renewed(current))

    # An older command towards somewhere else arrives 0.5 s late.
    late = rig.cmd(K.MOVE_TO, Pose(-20.0, 0.0, 5.0, 180.0), issued=rig.now - 5 * DT_NS)
    ack = rig.submit(late)
    assert not ack.accepted and ack.reason == Reason.COMMAND_EXPIRED
    assert ack.active_command_id == current.command_id

    x_before = rig.motion.pose.x
    rig.tick(rig.renewed(current))
    assert rig.motion.pose.x > x_before, "still heading for the current goal"
    rig.fly(current, lambda fb: rig.motion.pose == goal)
    rig.check()


def test_replayed_old_command_id_is_rejected_after_a_replacement():
    rig = Rig()
    rig.launch()
    east = rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 5.0, 0.0))
    rig.tick(east)
    north = rig.cmd(K.MOVE_TO, Pose(0.0, 20.0, 5.0, 90.0))
    rig.tick(north)

    # The network delivers the first command again, freshly stamped.
    ack = rig.submit(rig.renewed(east))
    assert not ack.accepted and ack.reason == Reason.COMMAND_ID_REUSED
    assert ack.active_command_id == north.command_id
    fb = rig.tick(rig.renewed(north))
    assert fb.target == Pose(0.0, 20.0, 5.0, 90.0)
    rig.check()


def test_same_id_with_a_different_target_cannot_hijack_the_active_command():
    rig = Rig()
    rig.launch()
    c = rig.cmd(K.TRACK_TARGET, Pose(10.0, 10.0, 5.0, 45.0))
    rig.tick(c)
    hijack = replace(rig.renewed(c), target=Pose(-50.0, -50.0, 5.0, 225.0))
    ack = rig.submit(hijack)
    assert not ack.accepted and ack.reason == Reason.COMMAND_ID_REUSED
    fb = rig.tick(rig.renewed(c))
    assert fb.target == Pose(10.0, 10.0, 5.0, 45.0)
    rig.check()


# ==========================================================================
# D. Geofence and ceiling while tracking
# ==========================================================================


def test_bird_walking_out_of_the_geofence_never_drags_the_drone_out():
    """A small 20 m fence keeps this quick. The bird walks east 1 m per tick;
    the coordinator sends a new tracking target every tick. Targets past the
    fence are rejected, the last good one expires, and the drone holds."""
    rig = Rig(limits=MotionLimits(geofence_radius_m=20.0))
    rig.launch()
    rejected = 0
    for x in range(5, 41):
        ack = rig.submit(rig.cmd(K.TRACK_TARGET, Pose(float(x), 0.0, 5.0, 0.0)))
        if not ack.accepted:
            rejected += 1
            assert ack.reason == Reason.TARGET_OUTSIDE_LIMITS
            assert x > 20
        rig.tick()
    for _ in range(10):
        fb = rig.tick()
    assert rejected == 20
    assert rig.motion.pose.horizontal_distance_to(PAD) <= 20.0
    assert fb.status == CommandStatus.EXPIRED_HOLD
    report = rig.check()
    assert report.metrics["max_radius_m"] <= 20.0
    assert report.metrics["rejected_by_reason"] == {Reason.TARGET_OUTSIDE_LIMITS: 20}


@pytest.mark.parametrize("target,accepted", [
    (Pose(0.0, 0.0, 30.0, 0.0), True),                # exactly at the ceiling
    (Pose(0.0, 0.0, 30.5, 0.0), False),               # above it
    (Pose(106.0, 106.0, 5.0, 0.0), True),             # 149.9 m from home
    (Pose(107.0, 107.0, 5.0, 0.0), False),            # 151.3 m from home
    (Pose(150.0, 0.0, 5.0, 0.0), True),               # exactly on the fence
    (Pose(0.0, -150.01, 5.0, 0.0), False),
])
def test_geofence_and_ceiling_edges(target, accepted):
    rig = Rig()
    rig.launch()
    ack = rig.submit(rig.cmd(K.MOVE_TO, target))
    assert ack.accepted is accepted
    if not accepted:
        assert ack.reason == Reason.TARGET_OUTSIDE_LIMITS
    rig.tick()
    rig.check()


@pytest.mark.parametrize("bad_target", [
    Pose(200.0, 0.0, 5.0, 0.0),
    Pose(0.0, 0.0, 45.0, 0.0),
])
def test_safety_replaces_an_out_of_limits_target_and_the_drone_stops_this_tick(bad_target):
    rig = Rig()
    rig.launch()
    rig.goto(Pose(10.0, 0.0, 5.0, 0.0))
    moving = rig.cmd(K.MOVE_TO, Pose(30.0, 0.0, 5.0, 0.0))
    rig.tick(moving)
    rig.tick(rig.renewed(moving))
    before = rig.motion.pose

    to_send, decision = check_for_motion(rig.cmd(K.MOVE_TO, bad_target), ALL_OK, rig.now)
    assert not decision.allowed and to_send.kind == K.HOVER
    assert rig.submit(to_send).accepted
    rig.tick()
    assert rig.motion.pose == before, "the hold must apply before this tick's movement"
    rig.check()


# ==========================================================================
# E. Emergency injected in every flight phase
# ==========================================================================


def _phase_climb(rig):
    rig.submit(rig.cmd(K.ARM, lifecycle=True))
    rig.submit(rig.cmd(K.TAKEOFF, Pose(0, 0, 10.0, 0), lifecycle=True))
    for _ in range(20):
        rig.tick()
    return None


def _phase_tracking(rig):
    rig.launch()
    c = rig.cmd(K.TRACK_TARGET, Pose(40.0, 30.0, 8.0, 37.0))
    for _ in range(15):
        rig.tick(rig.renewed(c))
    return c


def _phase_approach(rig):
    rig.launch()
    rig.goto(Pose(25.0, 25.0, 5.0, 0.0))
    c = rig.cmd(K.GOTO_DOCK_APPROACH, Pose(0.0, 0.0, 5.0, 0.0))
    for _ in range(10):
        rig.tick(rig.renewed(c))
    return c


def _phase_descent(rig):
    rig.launch(8.0)
    c = rig.cmd(K.DESCEND_TO_DOCK, PAD)
    for _ in range(15):
        rig.tick(rig.renewed(c))
    return c


def _phase_hover(rig):
    rig.launch(6.0)
    c = rig.cmd(K.HOVER)
    for _ in range(5):
        rig.tick(rig.renewed(c))
    return c


PHASES = {
    "climb": _phase_climb,
    "tracking": _phase_tracking,
    "approach": _phase_approach,
    "descent": _phase_descent,
    "hover": _phase_hover,
}


@pytest.mark.parametrize("phase", sorted(PHASES))
def test_emergency_in_any_phase_descends_in_place_and_latches(phase):
    rig = Rig()
    ongoing = PHASES[phase](rig)
    start = rig.motion.pose
    assert rig.motion.airborne

    ack = rig.submit(rig.cmd(K.EMERGENCY_LAND, lifecycle=True))
    assert ack.accepted and ack.reason == Reason.ACCEPTED
    fb = rig.tick()
    assert fb.emergency_descent and fb.status == CommandStatus.EMERGENCY_DESCENT

    # The latch: everything ordinary is refused, a repeat is idempotent.
    for kind, target in [(K.HOVER, None), (K.LAND, None), (K.DISARM, None),
                         (K.MOVE_TO, Pose(0, 0, 5, 0)), (K.DESCEND_TO_DOCK, PAD)]:
        lifecycle = kind in (K.LAND, K.DISARM)
        ack = rig.submit(rig.cmd(kind, target, lifecycle=lifecycle))
        assert not ack.accepted and ack.reason == Reason.EMERGENCY_DESCENT_ACTIVE, kind
    if ongoing is not None:
        ack = rig.submit(rig.renewed(ongoing))   # the coordinator still renewing the old job
        assert not ack.accepted
    again = rig.submit(rig.cmd(K.EMERGENCY_LAND, lifecycle=True))
    assert again.accepted and again.reason == Reason.ALREADY_EMERGENCY_LANDING

    # At 1 m/s the touchdown tick is start.z / 0.1 m, give or take one tick.
    expected_ticks = start.z / (LIM.emergency_descent_mps * DT)
    ticks = 1
    while rig.motion.armed:
        fb = rig.tick()
        ticks += 1
        assert ticks <= expected_ticks + 1, "emergency descent slower than 1 m/s"
    assert ticks >= expected_ticks - 1, "emergency descent faster than 1 m/s"
    assert fb.grounded and not fb.emergency_descent and not fb.armed
    assert rig.motion.pose.horizontal_distance_to(start) == 0.0
    assert records(rig.log, "disarm")[-1].decision == "auto_disarm_on_ground"
    rig.check()


def test_emergency_while_armed_on_the_pad_disarms_without_flying():
    rig = Rig()
    rig.tick(rig.cmd(K.ARM, lifecycle=True))
    assert rig.submit(rig.cmd(K.EMERGENCY_LAND, lifecycle=True)).accepted
    fb = rig.tick()
    assert not fb.armed and fb.grounded and not fb.emergency_descent
    report = rig.check()
    assert report.metrics["path_length_m"] == 0.0


def test_emergency_while_disarmed_is_rejected():
    rig = Rig()
    ack = rig.submit(rig.cmd(K.EMERGENCY_LAND, lifecycle=True))
    assert not ack.accepted and ack.reason == Reason.NOT_ARMED
    rig.tick()
    rig.check()


# ==========================================================================
# F. Hazards through the real Safety layer
# ==========================================================================


def _send_through_safety(rig, c, verdict):
    to_send, decision = check_for_motion(c, verdict, rig.now)
    ack = rig.submit(to_send)
    assert ack.accepted, (decision, ack)
    return to_send, decision


def test_cart_motion_during_descent_stops_descent_before_that_ticks_movement():
    rig = Rig()
    rig.launch(5.0)
    descend = rig.cmd(K.DESCEND_TO_DOCK, PAD)
    while rig.motion.pose.z > 2.6:
        _send_through_safety(rig, rig.renewed(descend), ALL_OK)
        rig.tick()
    z_at_hazard = rig.motion.pose.z

    # The cart starts moving: Safety swaps the renewed descent for a hold.
    for _ in range(30):
        sent, decision = _send_through_safety(rig, rig.renewed(descend), CART_MOVING)
        assert sent.kind == K.HOVER and sent.command_id == f"{descend.command_id}-safety"
        assert "action_not_permitted" in decision.reasons
        fb = rig.tick()
        assert rig.motion.pose.z == z_at_hazard, "descent continued while the cart moved"
        assert fb.airborne and not fb.landing_settled

    # Cart stops. The coordinator re-plans the descent under a new id.
    resume = rig.cmd(K.DESCEND_TO_DOCK, PAD)
    while not rig.motion.landing_settled:
        _send_through_safety(rig, rig.renewed(resume), ALL_OK)
        rig.tick()
    assert rig.submit(rig.cmd(K.DISARM, lifecycle=True)).accepted
    rig.tick()
    report = rig.check()
    assert report.metrics["landed"] is True
    assert report.metrics["landing_error_m"] == 0.0


def test_hazard_with_hold_forbidden_lands_in_place_away_from_the_pad():
    rig = Rig()
    rig.launch()
    track = rig.goto(Pose(12.0, 5.0, 5.0, 0.0), kind=K.TRACK_TARGET)
    nothing_allowed = assessment(blocked=ACTIONS)
    sent, _ = _send_through_safety(rig, rig.renewed(track), nothing_allowed)
    assert sent.kind == K.LAND
    for _ in range(40):
        fb = rig.tick()
    assert fb.landing_settled and fb.grounded and fb.armed
    assert rig.submit(rig.cmd(K.DISARM, lifecycle=True)).accepted
    rig.tick()
    report = rig.check()
    assert report.metrics["landing_error_m"] == pytest.approx(13.0)


def test_low_battery_stops_tracking_but_still_allows_the_return_to_the_pad():
    rig = Rig()
    rig.launch()
    track = rig.cmd(K.TRACK_TARGET, Pose(30.0, 0.0, 5.0, 0.0))
    for _ in range(10):
        _send_through_safety(rig, rig.renewed(track), ALL_OK)
        rig.tick()

    sent, _ = _send_through_safety(rig, rig.renewed(track), LOW_BATTERY)
    assert sent.kind == K.HOVER
    stopped_at = rig.motion.pose
    rig.tick()
    assert rig.motion.pose == stopped_at

    approach = rig.cmd(K.GOTO_DOCK_APPROACH, Pose(0.0, 0.0, 5.0, 0.0))
    while rig.motion.pose != Pose(0.0, 0.0, 5.0, 0.0):
        sent, decision = _send_through_safety(rig, rig.renewed(approach), LOW_BATTERY)
        assert decision.allowed
        rig.tick()
    descend = rig.cmd(K.DESCEND_TO_DOCK, PAD)
    while not rig.motion.landing_settled:
        _send_through_safety(rig, rig.renewed(descend), LOW_BATTERY)
        rig.tick()
    report = rig.check()
    assert report.metrics["landing_error_m"] == 0.0


def test_critical_battery_mid_approach_overrides_docking_with_emergency_landing():
    rig = Rig()
    rig.launch()
    rig.goto(Pose(20.0, 0.0, 5.0, 0.0))
    approach = rig.cmd(K.GOTO_DOCK_APPROACH, Pose(0.0, 0.0, 5.0, 0.0))
    for _ in range(10):
        _send_through_safety(rig, rig.renewed(approach), ALL_OK)
        rig.tick()
    where = rig.motion.pose

    critical = assessment(emergency=True)
    while rig.motion.armed:
        sent, _ = _send_through_safety(rig, rig.renewed(approach), critical)
        assert sent.kind == K.EMERGENCY_LAND
        rig.tick()
    assert rig.motion.pose.horizontal_distance_to(where) == 0.0
    report = rig.check()
    assert report.metrics["landing_error_m"] == pytest.approx(where.x)


# ==========================================================================
# G. Garbage in, nothing changes
# ==========================================================================


def _junk(rig: Rig, active: Command):
    """(label, thing to submit, now_ns to submit it with). Built fresh per tick."""
    now = rig.now
    good = rig.cmd(K.MOVE_TO, Pose(-10.0, -10.0, 5.0, 0.0))
    return [
        ("none", None, now),
        ("string", "MOVE_TO", now),
        ("dict", {"kind": "MOVE_TO"}, now),
        ("nan_target", replace(good, target=Pose(float("nan"), 0.0, 5.0, 0.0)), now),
        ("inf_target", replace(good, target=Pose(0.0, float("inf"), 5.0, 0.0)), now),
        ("wrong_frame", replace(good, frame="ned"), now),
        ("schema_99", replace(good, schema_version=99), now),
        ("missing_expiry", replace(good, expires_ns=None), now),
        ("expiry_before_issue", replace(good, expires_ns=good.issued_ns), now),
        ("empty_id", replace(good, command_id=""), now),
        ("future_issue", replace(good, issued_ns=now + S, expires_ns=now + 2 * S), now),
        ("negative_issue", replace(good, issued_ns=-1), now),
        ("zero_speed_cap", replace(good, max_horizontal_speed_mps=0.0), now),
        ("nan_speed_cap", replace(good, max_vertical_speed_mps=float("nan")), now),
        ("unknown_kind", replace(good, kind="TELEPORT"), now),
        ("set_orientation", replace(good, kind=K.SET_ORIENTATION), now),
        ("missing_target", replace(good, target=None), now),
        ("float_now", good, now + 0.5),
        ("hijack_active_id", replace(rig.renewed(active), target=Pose(99.0, 0.0, 5.0, 0.0)), now),
    ]


JUNK_LABELS = [label for label, _, _ in _junk(Rig(), Rig().cmd(K.HOVER))]


@pytest.mark.parametrize("label", JUNK_LABELS)
def test_garbage_submitted_every_tick_does_not_change_the_flight(label):
    def fly(with_junk: bool) -> Rig:
        rig = Rig()
        rig.launch()
        c = rig.cmd(K.TRACK_TARGET, Pose(12.0, 9.0, 7.0, 37.0))
        for _ in range(40):
            assert rig.submit(rig.renewed(c)).accepted
            if with_junk:
                junk = {l: (thing, now) for l, thing, now in _junk(rig, c)}
                thing, now = junk[label]
                ack = rig.submit(thing, now)
                assert not ack.accepted, (label, ack)
            rig.tick()
        return rig

    clean, noisy = fly(False), fly(True)
    assert poses(noisy) == poses(clean)
    report = noisy.check()
    assert sum(report.metrics["rejected_by_reason"].values()) == 40


# ==========================================================================
# H. Arrival above the pad is not a landing
# ==========================================================================


def test_descent_lost_inside_arrival_tolerance_never_reports_landed():
    rig = Rig()
    rig.launch(5.0)
    descend = rig.cmd(K.DESCEND_TO_DOCK, PAD)
    step_m = LIM.max_vertical_speed_mps * DT
    while rig.motion.pose.z > 2 * step_m + 1e-9:
        rig.tick(rig.renewed(descend))
    # Lose the link: one more valid tick brings it to 0.2 m, then it holds.
    rig.tick()
    fb = rig.tick()
    assert fb.status == CommandStatus.EXPIRED_HOLD
    assert rig.motion.pose.z == pytest.approx(step_m)
    assert rig.motion.pose.z <= LIM.arrival_tolerance_m          # "close enough" ...
    for _ in range(20):
        fb = rig.tick()
        assert fb.airborne and not fb.grounded                 # ... but still flying
        assert not fb.landing_settled and fb.status != CommandStatus.LANDED
    disarm = rig.submit(rig.cmd(K.DISARM, lifecycle=True))
    assert not disarm.accepted and disarm.reason == Reason.AIRBORNE
    rig.check()


# ==========================================================================
# I. Bad tick length
# ==========================================================================


@pytest.mark.parametrize("dt", [0.0, -0.1, -5.0])
def test_zero_or_negative_dt_does_not_move_the_vehicle(dt):
    rig = Rig()
    rig.launch()
    rig.tick(rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 5.0, 0.0)))
    before = rig.motion.pose
    rig.motion.step(dt, now_ns=rig.now)
    assert rig.motion.pose == before


@pytest.mark.parametrize("dt", [float("nan"), float("inf"), float("-inf"), None, "0.1", True])
def test_bad_dt_is_ignored_and_logged(dt):
    """A non-finite or non-numeric dt is treated as "no time passed": the pose
    does not move, feedback stays finite, and the tick is logged. (Used to be
    a known bug: NaN poisoned the pose for the rest of the run while feedback
    said valid=True, and inf jumped the drone straight onto its target.)"""
    rig = Rig()
    rig.launch()
    rig.tick(rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 5.0, 0.0)))
    before = rig.motion.pose
    rig.motion.step(dt, now_ns=rig.now)
    assert rig.motion.pose == before
    fb = rig.motion.feedback(rig.now)
    assert fb.valid and fb.pose.is_finite()
    bad = [r for r in rig.log.records if r.event == "bad_dt"]
    assert len(bad) == 1 and bad[0].decision == "ignored_bad_dt"


@pytest.mark.parametrize("dt", [1, Fraction(1, 10)])
def test_any_real_number_is_a_valid_dt(dt):
    """Only non-finite and non-numeric dt values are refused: an int or a
    Fraction (like a numpy scalar) still moves the drone."""
    rig = Rig()
    rig.launch()
    rig.tick(rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 5.0, 0.0)))
    before = rig.motion.pose.x
    rig.motion.step(dt, now_ns=rig.now)
    assert rig.motion.pose.x == pytest.approx(before + LIM.max_horizontal_speed_mps * float(dt))
    assert not records(rig.log, "bad_dt")


@pytest.mark.parametrize("dt", [float("nan"), float("inf")])
def test_bad_dt_tick_leaves_a_clean_trace_and_flight_resumes(dt):
    """A bad tick mid-move leaves no trace violation, and the next good tick
    carries on toward the target at the normal speed."""
    rig = Rig()
    rig.launch()
    move = rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 5.0, 0.0))
    rig.tick(move)
    rig.motion.step(dt, now_ns=rig.now)          # the bad tick (not recorded: no time passed)
    rig.clock.advance(DT)
    x_before = rig.motion.pose.x
    rig.tick(rig.renewed(move))
    assert rig.motion.pose.x == pytest.approx(x_before + LIM.max_horizontal_speed_mps * DT)
    rig.check()


def test_bad_dt_does_not_keep_an_expired_target_alive():
    """Expiry still runs on a bad tick: once the command is stale, the next
    good tick holds instead of chasing the old target."""
    rig = Rig()
    rig.launch()
    rig.tick(rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 5.0, 0.0)))
    rig.clock.advance(1.0)                       # stall well past the 0.2 s validity
    rig.motion.step(float("nan"), now_ns=rig.now)
    assert rig.motion.feedback(rig.now).status == CommandStatus.EXPIRED_HOLD
    held = rig.motion.pose
    rig.tick()
    assert rig.motion.pose == held
    rig.check()
