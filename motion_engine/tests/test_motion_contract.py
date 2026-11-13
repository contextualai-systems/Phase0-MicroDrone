"""MotionStub against the shared contract (INTEGRATION_README.md §3-§5, §8).

These are the Motion Engine deliverables from the integration README:
approach/descent, cancellation, ordinary disarm, limit and malformed-input
rejection, renewal -- all on the simulation clock, no hardware. Expiry under
command loss and the emergency latch are exercised in every flight phase by
test_failure_injection.py (sections A and E), so they are not repeated here.

Each test drives Motion the way the coordinator will: one `submit()` per
command, exactly one `step(dt, now_ns)` per 0.1 s tick, then `feedback()`.
"""

import json
import math

import pytest

from motion_engine.contracts import (
    SCHEMA_VERSION,
    CommandStatus,
    MotionCommandType as K,
    Pose,
    Reason,
)
from motion_engine.tests.motion_rig import DT_NS, PAD, VALIDITY_NS, Rig, records


# -- acknowledgement vs progress ----------------------------------------------


def test_accepted_means_executing_not_done(rig):
    assert rig.submit(rig.cmd(K.ARM, lifecycle=True)).accepted
    ack = rig.submit(rig.cmd(K.TAKEOFF, Pose(0, 0, 5.0, 0), lifecycle=True))
    assert ack.accepted and ack.reason == Reason.ACCEPTED
    assert ack.active_command_id == ack.command_id
    fb = rig.motion.feedback(rig.now)
    assert fb.status == CommandStatus.IN_PROGRESS
    assert not fb.arrived and not fb.airborne and fb.armed
    assert fb.active_command_id == ack.command_id and fb.active_kind == "TAKEOFF"


def test_launch_reports_arrival_and_airborne_and_refuses_disarm(airborne_rig):
    rig = airborne_rig
    fb = rig.motion.feedback(rig.now)
    assert fb.status == CommandStatus.ARRIVED
    assert fb.airborne and not fb.grounded and fb.armed
    assert fb.pose.z == 5.0 and fb.vertical_error_m == 0.0
    assert not fb.landing_settled
    ack = rig.submit(rig.cmd(K.DISARM, lifecycle=True))
    assert not ack.accepted and ack.reason == Reason.AIRBORNE
    assert rig.motion.armed


# -- approach / descent / ordinary disarm (README §8) ------------------------------


def test_approach_and_descent_land_settled_then_disarm(airborne_rig):
    rig = airborne_rig
    rig.goto(Pose(20.0, 10.0, 5.0, 0.0))

    approach = rig.cmd(K.GOTO_DOCK_APPROACH, Pose(PAD.x, PAD.y, 5.0, 90.0))
    trace = rig.fly(approach, lambda fb: fb.pose == fb.target)
    assert trace[-1].status == CommandStatus.ARRIVED
    assert trace[-1].airborne and not trace[-1].landing_settled

    descend = rig.cmd(K.DESCEND_TO_DOCK, Pose(PAD.x, PAD.y, 0.0, 90.0), mission_state="DOCKING_APPROACH")
    trace = rig.fly(descend, lambda fb: fb.landing_settled)

    # Arrival tolerance fires while still in the air: that must NOT read as landed.
    early = [fb for fb in trace if fb.arrived and fb.airborne]
    assert early, "expected at least one tick that is 'arrived' but still airborne"
    assert all(not fb.landing_settled and fb.status == CommandStatus.ARRIVED for fb in early)

    landed = trace[-1]
    assert landed.status == CommandStatus.LANDED
    assert landed.grounded and landed.pose.z == 0.0
    assert landed.pose.horizontal_distance_to(PAD) <= 0.25
    assert landed.armed  # ordinary landing does not auto-disarm

    ack = rig.submit(rig.cmd(K.DISARM, lifecycle=True, mission_state="DOCKED"))
    assert ack.accepted
    fb = rig.motion.feedback(rig.now)
    assert not fb.armed and fb.landing_settled and fb.status == CommandStatus.LANDED
    rig.check()


def test_flight_commands_rejected_when_not_armed(rig):
    ack = rig.submit(rig.cmd(K.TAKEOFF, Pose(0, 0, 5, 0), lifecycle=True))
    assert not ack.accepted and ack.reason == Reason.NOT_ARMED
    assert records(rig.log, "takeoff")[-1].decision == "rejected_not_armed"
    rig.tick()
    assert rig.motion.pose == PAD


#: Commands that would move an armed drone off the pad without a TAKEOFF.
GROUND_MOVES = {
    K.MOVE_TO: dict(target=Pose(5.0, 0.0, 5.0, 0.0)),
    K.TRACK_TARGET: dict(target=Pose(5.0, 0.0, 5.0, 0.0)),
    K.GOTO_DOCK_APPROACH: dict(target=Pose(0.0, 0.0, 5.0, 0.0)),
}


@pytest.mark.parametrize("kind", list(GROUND_MOVES), ids=lambda k: k.value)
def test_position_moves_are_rejected_on_the_ground(rig, kind):
    """Only TAKEOFF leaves the ground. An armed, grounded drone must not lift
    off on a MOVE_TO / TRACK_TARGET / GOTO_DOCK_APPROACH and skip the launch
    checks (e.g. after a comm-loss landing in the field)."""
    assert rig.submit(rig.cmd(K.ARM, lifecycle=True)).accepted
    ack = rig.submit(rig.cmd(kind, **GROUND_MOVES[kind]))
    assert not ack.accepted and ack.reason == Reason.NOT_AIRBORNE
    assert ack.detail == "take off before flying to a target"
    for _ in range(10):
        rig.tick()
    assert rig.motion.pose == PAD and rig.motion.armed
    rig.check()


@pytest.mark.parametrize("kind,target", [
    (K.HOVER, None), (K.STOP, None), (K.SET_YAW, Pose(0.0, 0.0, 0.0, 90.0)),
])
def test_holding_and_turning_on_the_ground_are_still_allowed(rig, kind, target):
    """These cannot lift the drone off, so they stay legal on the pad."""
    assert rig.submit(rig.cmd(K.ARM, lifecycle=True)).accepted
    assert rig.submit(rig.cmd(kind, target)).accepted
    rig.tick()
    assert rig.motion.pose.z == 0.0 and rig.motion.pose.horizontal_distance_to(PAD) == 0.0
    rig.check()


def test_emergency_latch_outranks_a_reused_id(airborne_rig):
    """While the emergency latch is set, a renewal of the command it replaced
    is told `emergency_descent_active` (not `command_id_reused`), and a
    repeated EMERGENCY_LAND, even under an ID already used, is idempotent."""
    rig = airborne_rig
    move = rig.cmd(K.MOVE_TO, Pose(20.0, 0.0, 5.0, 0.0))
    assert rig.submit(move).accepted
    emergency = rig.cmd(K.EMERGENCY_LAND, lifecycle=True)
    assert rig.submit(emergency).accepted
    rig.tick()
    assert rig.submit(rig.renewed(move)).reason == Reason.EMERGENCY_DESCENT_ACTIVE
    other = rig.cmd(K.HOVER)
    ack = rig.submit(other)
    assert ack.reason == Reason.EMERGENCY_DESCENT_ACTIVE
    repeat = rig.submit(rig.cmd(K.EMERGENCY_LAND, lifecycle=True, cid=move.command_id))
    assert repeat.accepted and repeat.reason == Reason.ALREADY_EMERGENCY_LANDING
    rig.tick()
    rig.check()


# -- cancellation / replacement before movement (README §3) -------------------------


def test_new_command_replaces_target_before_next_step(airborne_rig):
    rig = airborne_rig
    far = rig.cmd(K.MOVE_TO, Pose(100.0, 0.0, 5.0, 0.0))
    for _ in range(5):
        rig.tick(rig.renewed(far))
    here = rig.motion.pose
    assert here.x > 0.0

    hold = rig.cmd(K.HOVER, mission_state="HOVERING", reason="bird_lost")
    ack = rig.submit(hold)
    assert ack.accepted and ack.active_command_id == hold.command_id
    fb = rig.tick(rig.renewed(hold))
    assert fb.pose == here  # did not take even one more step toward x=100
    assert fb.status == CommandStatus.ARRIVED and fb.active_kind == "HOVER"
    rig.check()


def test_rejected_command_leaves_previous_active_and_reports_it(airborne_rig):
    rig = airborne_rig
    move = rig.cmd(K.MOVE_TO, Pose(10.0, 0.0, 5.0, 0.0))
    assert rig.submit(move).accepted
    bad = rig.submit(rig.cmd(K.MOVE_TO, Pose(0.0, 0.0, 500.0, 0.0)))
    assert not bad.accepted and bad.reason == Reason.TARGET_OUTSIDE_LIMITS
    assert bad.active_command_id == move.command_id
    assert rig.motion.target.z == 5.0  # not clamped-and-flown


@pytest.mark.parametrize("target,accepted", [
    (Pose(0.0, 0.0, 30.0, 0.0), True),                # exactly at the ceiling
    (Pose(0.0, 0.0, 30.5, 0.0), False),               # above it
    (Pose(106.0, 106.0, 5.0, 0.0), True),             # 149.9 m from home
    (Pose(107.0, 107.0, 5.0, 0.0), False),            # 151.3 m from home
    (Pose(150.0, 0.0, 5.0, 0.0), True),               # exactly on the fence
    (Pose(0.0, -150.01, 5.0, 0.0), False),
])
def test_geofence_and_ceiling_edges_reject_not_clamp(airborne_rig, target, accepted):
    rig = airborne_rig
    ack = rig.submit(rig.cmd(K.MOVE_TO, target))
    assert ack.accepted is accepted
    if not accepted:
        assert ack.reason == Reason.TARGET_OUTSIDE_LIMITS
        assert records(rig.log, "move_to")[-1].decision == "rejected_target_outside_limits"
    rig.tick()
    rig.check()


def test_speed_limits_in_command_only_lower(airborne_rig):
    rig = airborne_rig
    slow = rig.cmd(K.MOVE_TO, Pose(10.0, 0.0, 5.0, 0.0), max_horizontal_speed_mps=1.0)
    trace = rig.fly(slow, lambda fb: fb.pose == fb.target)
    assert len(trace) == pytest.approx(100, abs=1)  # 10 m at 1 m/s

    fast = rig.cmd(K.MOVE_TO, Pose(0.0, 0.0, 5.0, 0.0), max_horizontal_speed_mps=50.0)
    trace = rig.fly(fast, lambda fb: fb.pose == fb.target)
    assert len(trace) == pytest.approx(20, abs=1)  # capped at 5 m/s
    rig.check()


# -- renewal -------------------------------------------------------------


def test_renewal_extends_without_restart_or_extra_log(airborne_rig):
    rig = airborne_rig
    track = rig.cmd(K.TRACK_TARGET, Pose(25.0, 0.0, 5.0, 0.0), mission_state="TRACKING")
    assert rig.submit(track).accepted
    n_logs = len(rig.log.records)
    ack = rig.submit(rig.renewed(track))
    assert ack.accepted and ack.reason == Reason.RENEWED
    rig.tick()
    trace = rig.fly(track, lambda fb: fb.pose == fb.target)
    assert trace[-1].pose.x == 25.0
    # Only the one-off "arrived" record was added; renewals log nothing.
    assert [r.event for r in rig.log.records[n_logs:]] == ["arrived"]


def test_reusing_an_id_for_a_different_request_is_rejected(airborne_rig):
    rig = airborne_rig
    a = rig.cmd(K.MOVE_TO, Pose(10.0, 0.0, 5.0, 0.0), cid="same")
    assert rig.submit(a).accepted
    b = rig.cmd(K.MOVE_TO, Pose(-10.0, 0.0, 5.0, 0.0), cid="same")
    ack = rig.submit(b)
    assert not ack.accepted and ack.reason == Reason.COMMAND_ID_REUSED
    assert rig.motion.target.x == 10.0
    # An old, no-longer-active id cannot be recycled either.
    assert rig.submit(rig.cmd(K.HOVER, cid="h1")).accepted
    again = rig.submit(rig.cmd(K.MOVE_TO, Pose(1.0, 0.0, 5.0, 0.0), cid="same"))
    assert not again.accepted and again.reason == Reason.COMMAND_ID_REUSED
    # Nor can the original request be replayed verbatim, freshly stamped.
    replay = rig.submit(rig.renewed(a))
    assert not replay.accepted and replay.reason == Reason.COMMAND_ID_REUSED
    assert replay.active_command_id == "h1"
    assert rig.tick().active_kind == "HOVER"
    rig.check()


# -- malformed / stale input ----------------------------------------------------------


@pytest.mark.parametrize("build,reason", [
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), lifecycle=True), Reason.MISSING_EXPIRY),
    (lambda r: r.cmd(K.MOVE_TO, None), Reason.MISSING_TARGET),
    (lambda r: r.cmd(K.MOVE_TO, Pose(float("nan"), 0, 5, 0)), Reason.INVALID_COMMAND),
    (lambda r: r.cmd(K.MOVE_TO, Pose(math.inf, 0, 5, 0)), Reason.INVALID_COMMAND),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), frame="local_ned"), Reason.UNSUPPORTED_FRAME),
    (lambda r: r.cmd(K.SET_ORIENTATION, Pose(0, 0, 5, 0)), Reason.UNSUPPORTED_KIND),
    (lambda r: r.cmd("FLY_AWAY", Pose(0, 0, 5, 0)), Reason.UNSUPPORTED_KIND),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), schema_version=2), Reason.UNSUPPORTED_SCHEMA),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), cid=""), Reason.INVALID_COMMAND),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), max_vertical_speed_mps=-1.0), Reason.INVALID_COMMAND),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), issued=True), Reason.INVALID_COMMAND),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), issued=r.now + DT_NS), Reason.FUTURE_TIMESTAMP),
    (lambda r: r.cmd(K.MOVE_TO, Pose(1, 0, 5, 0), issued=r.now - VALIDITY_NS), Reason.COMMAND_EXPIRED),
])
def test_malformed_or_stale_commands_are_rejected_with_reason(airborne_rig, build, reason):
    rig = airborne_rig
    before = rig.motion.target
    ack = rig.submit(build(rig))
    assert not ack.accepted and ack.reason == reason
    assert rig.motion.target == before


def test_non_command_input_does_not_raise(rig):
    ack = rig.submit({"kind": "ARM"})  # a dict, not a Command
    assert not ack.accepted and ack.reason == Reason.INVALID_COMMAND
    assert not rig.motion.armed


def test_kind_may_be_given_as_its_string_value(rig):
    assert rig.submit(rig.cmd("ARM", lifecycle=True)).accepted
    assert rig.motion.armed


# -- feedback shape and determinism ------------------------------------------------------


def test_feedback_is_read_only_and_json_serialisable(airborne_rig):
    rig = airborne_rig
    a = rig.motion.feedback(rig.now)
    b = rig.motion.feedback(rig.now)
    assert a == b
    d = a.to_dict()
    json.dumps(d)
    assert d["schema_version"] == SCHEMA_VERSION and d["synthetic"] is True
    assert d["timestamp_ns"] == rig.now and isinstance(d["timestamp_ns"], int)
    assert set(d) >= {
        "pose", "armed", "airborne", "grounded", "active_command_id", "target",
        "status", "arrived", "landing_settled", "valid", "timestamp_ns",
    }


def _scripted_run():
    rig = Rig()
    rig.launch(5.0)
    script = [
        (rig.cmd(K.TRACK_TARGET, Pose(30.0, 10.0, 5.0, 45.0)), 40),
        (rig.cmd(K.HOVER), 5),
        (rig.cmd(K.GOTO_DOCK_APPROACH, Pose(0.0, 0.0, 5.0, 0.0)), 80),
        (rig.cmd(K.DESCEND_TO_DOCK, Pose(0.0, 0.0, 0.0, 0.0)), 40),
    ]
    trace, acks = [], []
    for cmd, ticks in script:
        cmd = rig.renewed(cmd)
        acks.append(rig.submit(cmd).to_dict())
        for _ in range(ticks):
            trace.append(rig.tick(rig.renewed(cmd)).to_dict())
    return trace, acks, [r.to_dict() for r in rig.log.records]


def test_identical_inputs_reproduce_identical_traces_and_logs():
    first = _scripted_run()
    second = _scripted_run()
    assert first == second
    assert first[0][-1]["landing_settled"] is True
