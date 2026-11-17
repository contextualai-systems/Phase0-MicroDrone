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
  C. Stale (late) commands -- replayed / reused IDs: test_motion_contract.py and G
  D. Geofence and ceiling under tracking
  E. Emergency injected in every flight phase
  F. Hazards through the real Safety layer (cart motion, low/critical battery)
  G. Garbage input immunity
  H. Arrival above the pad is not a landing
  I. Bad tick length (dt) -- zero, negative, NaN, inf, non-numeric
  J. Comm-loss escalation -- an expired hold that outlasts the timeout lands
"""

from dataclasses import replace
from fractions import Fraction
from typing import List

import pytest

from motion_engine.contracts import (
    Command,
    CommandStatus,
    FlightMode,
    MotionCommandType as K,
    Pose,
    Reason,
    Velocity,
)
from motion_engine.motion_stubs import MotionLimits
from motion_engine.safety_adapter import check_for_motion
from motion_engine.tests.motion_rig import (
    ALL_OK,
    DT,
    DT_NS,
    LIM,
    PAD,
    S,
    VALIDITY_NS,
    Rig,
    assessment,
    records,
)
from safety_layer.safety_policy import ACTIONS

#: Cart in motion: Safety allows hold only (INTEGRATION_README §6).
CART_MOVING = assessment(blocked=("launch", "tracking", "approach", "descent"))
#: Low battery: no launch or tracking, but docking is still allowed.
LOW_BATTERY = assessment(blocked=("launch", "tracking"))


def poses(rig: Rig, start: int = 0) -> List[Pose]:
    return [s.pose for s in rig.rec.samples[start:]]


# ==========================================================================
# A. Command loss -- the coordinator stops renewing mid-flight
# ==========================================================================

# name -> (launch altitude, kind of the command that gets lost, its target pose or velocity)
LOSS_CASES = {
    "move_to": (5.0, K.MOVE_TO, Pose(40.0, 0.0, 5.0, 0.0)),
    "track_target": (5.0, K.TRACK_TARGET, Pose(30.0, 30.0, 8.0, 45.0)),
    "dock_approach": (5.0, K.GOTO_DOCK_APPROACH, Pose(0.0, 0.0, 5.0, 0.0)),
    "set_yaw": (5.0, K.SET_YAW, Pose(0.0, 0.0, 0.0, 170.0)),
    "descend_to_dock": (12.0, K.DESCEND_TO_DOCK, PAD),
    "set_velocity": (5.0, K.SET_VELOCITY, Velocity(3.0, 0.0, 0.0)),
}


@pytest.mark.parametrize("case", sorted(LOSS_CASES))
def test_command_loss_freezes_the_vehicle_within_one_validity_window(case):
    altitude, kind, target = LOSS_CASES[case]
    rig = Rig()
    rig.launch(altitude)
    if kind == K.GOTO_DOCK_APPROACH:
        rig.goto(Pose(30.0, -20.0, 5.0, 0.0))  # start somewhere away from the pad

    if kind == K.SET_VELOCITY:
        c = rig.cmd(kind, velocity=target)
    else:
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
    # The expired command is replaced by a hold on the spot, whatever its kind.
    assert fb.target == frozen and fb.active_kind == "HOVER"
    assert fb.flight_mode == FlightMode.HOLD and fb.target_velocity is None
    assert fb.status_reason == Reason.COMMAND_EXPIRED
    assert pose_at_loss.horizontal_distance_to(frozen) <= LIM.max_horizontal_speed_mps * DT + 1e-9
    assert abs(pose_at_loss.z - frozen.z) <= LIM.max_vertical_speed_mps * DT + 1e-9

    for _ in range(30):                        # 3 s with no coordinator at all
        fb = rig.tick()
        assert rig.motion.pose == frozen
        assert fb.status == CommandStatus.EXPIRED_HOLD
    assert fb.armed
    (expired,) = records(rig.log, "command_expired")
    assert expired.decision == "hold_in_place" and expired.fields["command_id"] == c.command_id
    # A late renewal cannot revive the expired ID.
    assert rig.submit(rig.renewed(c)).reason == Reason.COMMAND_ID_REUSED
    if kind == K.DESCEND_TO_DOCK:
        assert fb.airborne and not fb.landing_settled, "an expired descent must not count as landed"
    report = rig.check()
    assert report.metrics["expired_hold_ticks"] == 31


def test_one_dropped_renewal_is_tolerated(airborne_rig):
    rig = airborne_rig
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


def test_after_expiry_the_stale_id_cannot_be_revived_but_a_fresh_id_resumes(airborne_rig):
    rig = airborne_rig
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


def test_emergency_descent_never_expires_or_escalates():
    """An expiry on an emergency command is ignored, and a descent that lasts
    longer than the comm-loss timeout (12 s) never escalates. After touchdown
    Motion is disarmed and refuses flight commands until re-armed."""
    rig = Rig()
    rig.launch(12.0)
    rig.tick(rig.cmd(K.EMERGENCY_LAND))          # carries an expiry; must not time out
    while rig.motion.armed:
        fb = rig.tick()
        assert fb.status == CommandStatus.EMERGENCY_DESCENT or not fb.armed
    assert fb.grounded and not fb.emergency_descent
    assert fb.landing_settled and fb.status == CommandStatus.LANDED
    assert not records(rig.log, "command_expired")
    assert not records(rig.log, "comm_loss_landing")
    ack = rig.submit(rig.cmd(K.TAKEOFF, Pose(0, 0, 5, 0), lifecycle=True))
    assert not ack.accepted and ack.reason == Reason.NOT_ARMED
    rig.check()


# ==========================================================================
# B. Loop stalls and clock faults
# ==========================================================================


@pytest.mark.parametrize("stall_s,dt", [(1.0, DT), (5.0, DT), (5.0, 5.0)])
def test_loop_stall_expires_the_target_before_moving(stall_s, dt, airborne_rig):
    """The coordinator freezes for `stall_s`; the next step must not keep
    chasing the old target, even if it is handed the whole gap as `dt`."""
    rig = airborne_rig
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


def test_clock_running_backwards_does_not_revive_an_expired_command(airborne_rig):
    rig = airborne_rig
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
# C. Stale (late) commands
# ==========================================================================


def test_late_expired_command_is_rejected_and_the_current_target_keeps_flying(airborne_rig):
    rig = airborne_rig
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

    emergency = rig.cmd(K.EMERGENCY_LAND, lifecycle=True)
    ack = rig.submit(emergency)
    assert ack.accepted and ack.reason == Reason.ACCEPTED
    fb = rig.tick()
    assert fb.emergency_descent and fb.status == CommandStatus.EMERGENCY_DESCENT

    # The latch: everything ordinary is refused, a repeat is idempotent.
    for kind, target in [(K.HOVER, None), (K.LAND, None), (K.DISARM, None), (K.ARM, None),
                         (K.MOVE_TO, Pose(0, 0, 5, 0)), (K.DESCEND_TO_DOCK, PAD)]:
        lifecycle = kind in (K.LAND, K.DISARM, K.ARM)
        ack = rig.submit(rig.cmd(kind, target, lifecycle=lifecycle))
        assert not ack.accepted and ack.reason == Reason.EMERGENCY_DESCENT_ACTIVE, kind
        assert ack.active_command_id == emergency.command_id
    if ongoing is not None:
        ack = rig.submit(rig.renewed(ongoing))   # the coordinator still renewing the old job
        assert not ack.accepted
    again = rig.submit(rig.cmd(K.EMERGENCY_LAND, lifecycle=True))
    assert again.accepted and again.reason == Reason.ALREADY_EMERGENCY_LANDING
    assert len(records(rig.log, "emergency_land")) == 1, "a repeat must not restart or re-log"

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


@pytest.mark.parametrize("dt, logged", [
    (0.0, False), (-0.1, False), (-5.0, False),
    (float("nan"), True), (float("inf"), True), (float("-inf"), True),
    (None, True), ("0.1", True), (True, True),
])
def test_zero_negative_or_bad_dt_does_not_move_the_vehicle(dt, logged, airborne_rig):
    """Zero or negative dt is "no time passed". A non-finite or non-numeric dt
    is treated the same way and logged: the pose does not move and feedback
    stays finite. (Used to be a known bug: NaN poisoned the pose for the rest
    of the run while feedback said valid=True, and inf jumped the drone
    straight onto its target.)"""
    rig = airborne_rig
    rig.tick(rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 5.0, 0.0)))
    before = rig.motion.pose
    rig.motion.step(dt, now_ns=rig.now)
    assert rig.motion.pose == before
    fb = rig.motion.feedback(rig.now)
    assert fb.valid and fb.pose.is_finite()
    bad = records(rig.log, "bad_dt")
    assert len(bad) == int(logged)
    assert all(r.decision == "ignored_bad_dt" for r in bad)


@pytest.mark.parametrize("dt", [1, Fraction(1, 10)])
def test_any_real_number_is_a_valid_dt(dt, airborne_rig):
    """Only non-finite and non-numeric dt values are refused: an int or a
    Fraction (like a numpy scalar) still moves the drone."""
    rig = airborne_rig
    rig.tick(rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 5.0, 0.0)))
    before = rig.motion.pose.x
    rig.motion.step(dt, now_ns=rig.now)
    assert rig.motion.pose.x == pytest.approx(before + LIM.max_horizontal_speed_mps * float(dt))
    assert not records(rig.log, "bad_dt")


@pytest.mark.parametrize("dt", [float("nan"), float("inf")])
def test_bad_dt_tick_leaves_a_clean_trace_and_flight_resumes(dt, airborne_rig):
    """A bad tick mid-move leaves no trace violation, and the next good tick
    carries on toward the target at the normal speed."""
    rig = airborne_rig
    move = rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 5.0, 0.0))
    rig.tick(move)
    rig.motion.step(dt, now_ns=rig.now)          # the bad tick (not recorded: no time passed)
    rig.clock.advance(DT)
    x_before = rig.motion.pose.x
    rig.tick(rig.renewed(move))
    assert rig.motion.pose.x == pytest.approx(x_before + LIM.max_horizontal_speed_mps * DT)
    rig.check()


def test_bad_dt_does_not_keep_an_expired_target_alive(airborne_rig):
    """Expiry still runs on a bad tick: once the command is stale, the next
    good tick holds instead of chasing the old target."""
    rig = airborne_rig
    rig.tick(rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 5.0, 0.0)))
    rig.clock.advance(1.0)                       # stall well past the 0.2 s validity
    rig.motion.step(float("nan"), now_ns=rig.now)
    assert rig.motion.feedback(rig.now).status == CommandStatus.EXPIRED_HOLD
    held = rig.motion.pose
    rig.tick()
    assert rig.motion.pose == held
    rig.check()


# ==========================================================================
# J. Comm-loss escalation -- the coordinator never comes back
# ==========================================================================

#: Ticks of continuous expired hold before Motion lands on its own (10 s).
TIMEOUT_TICKS = LIM.comm_loss_land_timeout_ns // DT_NS


def _lose_link_mid_move(rig: Rig, c: Command = None) -> int:
    """Fly `c` (default: a MOVE_TO 40 m east) for 0.5 s, stop renewing, and
    run until Motion holds.

    Returns the simulation time of the step in which the hold began.
    """
    if c is None:
        c = rig.cmd(K.MOVE_TO, Pose(40.0, 0.0, 5.0, 0.0))
    for _ in range(5):
        assert rig.submit(rig.renewed(c)).accepted
        rig.tick()
    fb = rig.tick()
    while fb.status != CommandStatus.EXPIRED_HOLD:
        fb = rig.tick()
    return rig.now - DT_NS


def _hold_until_comm_loss_landing(rig: Rig, hold_began: int):
    """Tick through the 10 s expired hold; return (held pose, feedback of the
    tick the comm-loss landing started). Checks the hold stays put and that
    the landing starts exactly at the timeout, before that tick's movement."""
    held = rig.motion.pose
    for _ in range(TIMEOUT_TICKS - 1):          # 9.9 s more: still inside the limit
        fb = rig.tick()
        assert fb.status == CommandStatus.EXPIRED_HOLD and fb.reason is None
        assert fb.status_reason == Reason.COMMAND_EXPIRED and fb.flight_mode == FlightMode.HOLD
        assert fb.target_velocity is None
        assert rig.motion.pose == held
    assert not records(rig.log, "comm_loss_landing")
    assert not rig.motion.comm_loss_landing

    # Exactly 10 s after the hold began (timers fire at the limit).
    fb = rig.tick()
    assert rig.now - DT_NS - hold_began == LIM.comm_loss_land_timeout_ns
    assert fb.status == CommandStatus.COMM_LOSS_LANDING
    assert fb.flight_mode == FlightMode.LAND and fb.target_velocity is None
    assert fb.status_reason == Reason.COMM_LOSS_LANDING == fb.reason
    assert fb.valid and fb.validate() is None
    assert fb.active_kind == "LAND" and fb.target == Pose(held.x, held.y, 0.0, held.yaw_deg)
    assert rig.motion.pose.z == pytest.approx(held.z - LIM.max_vertical_speed_mps * DT)
    return held, fb


def _land_straight_down(rig: Rig, held: Pose, fb):
    """Tick until the comm-loss landing settles; it must not drift sideways."""
    while not fb.landing_settled:
        fb = rig.tick()
        assert rig.motion.pose.horizontal_distance_to(held) == 0.0, "landing must be straight down"
        assert fb.validate() is None, "DroneState must stay coherent while landing"
    assert fb.status == CommandStatus.LANDED and fb.grounded
    return fb


def test_escalation_lands_in_place_after_the_timeout(airborne_rig):
    rig = airborne_rig
    hold_began = _lose_link_mid_move(rig)
    held, fb = _hold_until_comm_loss_landing(rig, hold_began)
    assert fb.to_dict()["status"] == "comm_loss_landing"

    fb = _land_straight_down(rig, held, fb)
    assert fb.flight_mode == FlightMode.LAND
    assert fb.reason == Reason.COMM_LOSS_LANDING, "a returning coordinator must see why it landed"
    assert fb.status_reason == Reason.COMM_LOSS_LANDING
    assert fb.armed, "same land path as LAND: the caller disarms"
    for _ in range(50):                           # stays landed, no second escalation
        fb = rig.tick()
        assert fb.status == CommandStatus.LANDED
    assert rig.submit(rig.cmd(K.DISARM, lifecycle=True)).accepted
    fb = rig.tick()
    assert not fb.armed and fb.reason is None
    assert fb.flight_mode == FlightMode.DISARMED and fb.status_reason is None

    (rec,) = records(rig.log, "comm_loss_landing")
    assert rec.decision == "land_in_place"
    assert rec.fields["held_s"] == pytest.approx(10.0)
    assert rec.fields["timeout_s"] == pytest.approx(10.0)
    assert rec.fields["since_ns"] == hold_began
    assert rec.fields["z"] == 0.0 and rec.fields["z_start"] == pytest.approx(held.z)
    assert (rec.fields["x"], rec.fields["y"]) == (pytest.approx(held.x), pytest.approx(held.y))
    assert records(rig.log, "command_expired")[0].decision == "hold_in_place"
    report = rig.check()
    assert report.metrics["landing_error_m"] == pytest.approx(held.x)


def test_a_fresh_command_resets_the_timer(airborne_rig):
    rig = airborne_rig
    _lose_link_mid_move(rig)
    for _ in range(TIMEOUT_TICKS - 5):          # 9.5 s into the hold ...
        rig.tick()
    hover = rig.cmd(K.HOVER)                      # ... the coordinator is back for one tick
    fb = rig.tick(hover)
    assert fb.status == CommandStatus.ARRIVED
    fb = rig.tick()
    fb = rig.tick()
    assert fb.status == CommandStatus.EXPIRED_HOLD   # and gone again
    second_hold = rig.now - DT_NS
    for _ in range(TIMEOUT_TICKS - 1):           # a full new 10 s, not the remaining 0.5 s
        fb = rig.tick()
        assert fb.status == CommandStatus.EXPIRED_HOLD
    fb = rig.tick()
    assert fb.status == CommandStatus.COMM_LOSS_LANDING
    assert records(rig.log, "comm_loss_landing")[0].fields["since_ns"] == second_hold
    rig.check()


def test_renewals_inside_the_validity_window_never_start_the_timer(airborne_rig):
    rig = airborne_rig
    c = rig.cmd(K.HOVER)
    for _ in range(TIMEOUT_TICKS * 2):          # 20 s of a healthy link
        fb = rig.tick(rig.renewed(c))
        assert fb.status == CommandStatus.ARRIVED
    assert not records(rig.log, "command_expired")
    assert not records(rig.log, "comm_loss_landing")


def test_fresh_valid_command_cancels_a_comm_loss_landing_but_a_stale_one_cannot():
    rig = Rig()
    rig.launch(8.0)
    move = rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 8.0, 0.0))
    rig.tick(move)
    for _ in range(TIMEOUT_TICKS + 5):
        fb = rig.tick()
    assert fb.status == CommandStatus.COMM_LOSS_LANDING
    z_mid = rig.motion.pose.z
    assert 0.0 < z_mid < 8.0

    # Late and replayed traffic is rejected and does not stop the landing.
    late = rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 8.0, 0.0), issued=rig.now - 5 * DT_NS)
    assert rig.submit(late).reason == Reason.COMMAND_EXPIRED
    assert rig.submit(rig.renewed(move)).reason == Reason.COMMAND_ID_REUSED
    fb = rig.tick()
    assert fb.status == CommandStatus.COMM_LOSS_LANDING and rig.motion.pose.z < z_mid

    # A fresh, valid command (the coordinator is back) takes over.
    climb = rig.cmd(K.MOVE_TO, Pose(rig.motion.pose.x, 0.0, 6.0, 0.0))
    ack = rig.submit(climb)
    assert ack.accepted and ack.reason == Reason.ACCEPTED
    fb = rig.tick()
    assert fb.status == CommandStatus.IN_PROGRESS and fb.reason is None
    assert not rig.motion.comm_loss_landing
    rig.fly(climb, lambda fb: rig.motion.pose == climb.target)
    assert not records(rig.log, "comm_loss_landing")[1:], "no second escalation"
    rig.check()


def test_emergency_latch_still_dominates_a_comm_loss_landing():
    rig = Rig()
    rig.launch(6.0)
    _lose_link_mid_move(rig)
    for _ in range(TIMEOUT_TICKS + 3):
        fb = rig.tick()
    assert fb.status == CommandStatus.COMM_LOSS_LANDING
    start = rig.motion.pose

    assert rig.submit(rig.cmd(K.EMERGENCY_LAND, lifecycle=True)).accepted
    fb = rig.tick()
    assert fb.status == CommandStatus.EMERGENCY_DESCENT and fb.emergency_descent
    assert fb.reason is None
    assert rig.motion.pose.z == pytest.approx(start.z - LIM.emergency_descent_mps * DT)
    ack = rig.submit(rig.cmd(K.HOVER))
    assert not ack.accepted and ack.reason == Reason.EMERGENCY_DESCENT_ACTIVE
    while rig.motion.armed:
        fb = rig.tick()
    assert fb.grounded and not fb.emergency_descent
    assert rig.motion.pose.horizontal_distance_to(start) == 0.0
    rig.check()


def test_settled_landing_on_the_pad_does_not_escalate():
    rig = Rig()
    rig.launch(5.0)
    descend = rig.cmd(K.DESCEND_TO_DOCK, PAD)
    rig.fly(descend, lambda fb: fb.landing_settled)
    for _ in range(TIMEOUT_TICKS * 2):            # link gone for 20 s, still armed
        fb = rig.tick()
        assert fb.status == CommandStatus.LANDED and fb.landing_settled and fb.reason is None
    assert fb.armed
    assert not records(rig.log, "command_expired")
    assert not records(rig.log, "comm_loss_landing")
    rig.check()


def test_armed_on_the_ground_holding_does_not_escalate():
    """Armed on the pad with an expired hover (never took off): expired_hold
    on the ground, nothing to land, so no escalation."""
    rig = Rig()
    rig.tick(rig.cmd(K.ARM, lifecycle=True))
    rig.tick(rig.cmd(K.HOVER))
    for _ in range(TIMEOUT_TICKS * 2):
        fb = rig.tick()
    assert fb.status == CommandStatus.EXPIRED_HOLD and fb.grounded and fb.armed
    assert not records(rig.log, "comm_loss_landing")
    rig.check()


def test_disarmed_vehicle_does_not_escalate():
    rig = Rig()
    for _ in range(TIMEOUT_TICKS * 2):
        fb = rig.tick()
    assert fb.status == CommandStatus.NONE and not fb.armed
    assert not records(rig.log, "comm_loss_landing")
    rig.check()


def test_comm_loss_timeout_is_configurable_and_uses_simulation_time():
    rig = Rig(limits=MotionLimits(comm_loss_land_timeout_ns=2 * S))
    rig.launch()
    hold_began = _lose_link_mid_move(rig)
    # A frozen loop: one 3 s jump in simulation time, no ticks in between.
    rig.clock.advance(3.0)
    rig.rec.step(DT, rig.now)
    rig.clock.advance(DT)
    fb = rig.motion.feedback(rig.now)
    assert fb.status == CommandStatus.COMM_LOSS_LANDING
    (rec,) = records(rig.log, "comm_loss_landing")
    assert rec.fields["held_s"] == pytest.approx(3.1)
    assert rec.fields["since_ns"] == hold_began
    rig.check()


def test_step_without_simulation_time_never_escalates():
    """Direct-script use (`step(dt)` with no `now_ns`) has no simulation
    clock, and wall time is never used for a decision."""
    rig = Rig(limits=MotionLimits(comm_loss_land_timeout_ns=DT_NS))
    rig.launch()
    _lose_link_mid_move(rig)
    for _ in range(50):
        rig.motion.step(DT)
    assert not rig.motion.comm_loss_landing
    assert rig.motion.feedback(rig.now).status == CommandStatus.EXPIRED_HOLD


def test_expired_velocity_command_escalates_to_comm_loss_landing(airborne_rig):
    """SET_VELOCITY uses the same dead-man path as a position command:
    expiry -> expired_hold in place (velocity mode ends) -> after the
    comm-loss timeout, a straight-down landing."""
    rig = airborne_rig
    vel = rig.cmd(K.SET_VELOCITY, velocity=Velocity(2.0, 1.0, 0.0, 10.0))
    assert rig.submit(vel).accepted
    assert rig.motion.feedback(rig.now).flight_mode == FlightMode.VELOCITY
    hold_began = _lose_link_mid_move(rig, vel)
    assert rig.motion.velocity_setpoint is None          # velocity mode ended at expiry

    held, fb = _hold_until_comm_loss_landing(rig, hold_began)
    fb = _land_straight_down(rig, held, fb)
    assert fb.armed
    (rec,) = records(rig.log, "comm_loss_landing")
    assert rec.fields["since_ns"] == hold_began
    # The expired velocity ID is spent: replaying it cannot restart velocity mode.
    assert rig.submit(rig.renewed(vel)).reason == Reason.COMMAND_ID_REUSED
    rig.check()
