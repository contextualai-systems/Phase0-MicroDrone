"""SET_VELOCITY through the shared contract (submit / step / feedback).

Velocity control is Motion's: a velocity setpoint has no target, so it must
always carry an expiry (dead-man), is rejected (never scaled) when non-finite
or too fast, and stops *at* the geofence / ceiling / floor with an explicit
`boundary_hold` instead of a silent clamp. Every run also goes through the
trace checker, so no velocity test can pass while breaking a rate or fence rule.
"""

import math

import pytest

from motion_engine.contracts import (
    Command,
    CommandStatus,
    FlightMode,
    MemoryLogSink,
    MotionCommandType as K,
    Pose,
    Reason,
    SimulationClock,
    Velocity,
)
from motion_engine.motion_stubs import MotionLimits, MotionStub
from motion_engine.tests.motion_rig import DT, Rig


def _rig(z: float = 5.0, xy=None, limits=None) -> Rig:
    """A recorded rig holding exactly at altitude `z` (and at `xy`, if given)."""
    r = Rig(limits=limits)
    r.launch(z)
    if xy is not None:
        r.goto(Pose(xy[0], xy[1], z, 0.0))
    assert r.motion.airborne
    return r


def _vel(r: Rig, v=None, **kw) -> Command:
    return r.cmd(K.SET_VELOCITY, velocity=v, **kw)


# --------------------------------------------------------------------------
# happy path, renewal (expiry: test_failure_injection.py §A, set_velocity case)
# --------------------------------------------------------------------------


def test_velocity_happy_path_moves_at_commanded_rate():
    r = _rig()
    c = _vel(r, Velocity(2.0, -1.0, 0.5, yaw_rate_deg_s=10.0))
    ack = r.submit(c)
    assert ack.accepted and ack.reason == Reason.ACCEPTED and ack.kind == "SET_VELOCITY"
    start = r.motion.pose
    for i in range(10):
        fb = r.tick()
        if i < 9:
            assert r.submit(r.renewed(c)).reason == Reason.RENEWED
    p = r.motion.pose
    assert p.x - start.x == pytest.approx(2.0)
    assert p.y - start.y == pytest.approx(-1.0)
    assert p.z - start.z == pytest.approx(0.5)
    assert p.yaw_deg == pytest.approx(10.0)
    assert fb.status == CommandStatus.IN_PROGRESS
    assert fb.flight_mode == FlightMode.VELOCITY
    assert fb.target is None and fb.target_velocity == c.velocity
    assert fb.velocity.vx == pytest.approx(2.0) and fb.velocity.vz == pytest.approx(0.5)
    assert fb.active_kind == "SET_VELOCITY" and fb.active_command_id == c.command_id
    assert fb.validate() is None
    r.check()


def test_velocity_without_yaw_rate_keeps_heading():
    r = _rig()
    assert r.submit(_vel(r, Velocity(1.0, 0.0, 0.0))).accepted
    yaw = r.motion.pose.yaw_deg
    r.tick()
    assert r.motion.pose.yaw_deg == yaw


def test_position_command_replaces_velocity():
    r = _rig()
    assert r.submit(_vel(r, Velocity(1.0, 0.0, 0.0))).accepted
    r.tick()
    assert r.submit(r.cmd(K.HOVER)).accepted
    fb = r.motion.feedback(r.now)
    assert fb.target_velocity is None and fb.flight_mode == FlightMode.HOLD
    here = r.motion.pose
    r.tick()
    assert r.motion.pose == here


# --------------------------------------------------------------------------
# rejections
# --------------------------------------------------------------------------


@pytest.mark.parametrize("bad", [
    Velocity(float("nan"), 0.0, 0.0),
    Velocity(0.0, float("inf"), 0.0),
    Velocity(0.0, 0.0, -float("inf")),
    Velocity(1.0, 0.0, 0.0, yaw_rate_deg_s=float("nan")),
    Velocity("1", 0.0, 0.0),
])
def test_non_finite_velocity_rejected(bad):
    r = _rig()
    before = r.motion.pose
    ack = r.submit(_vel(r, bad))
    assert not ack.accepted and ack.reason == Reason.INVALID_COMMAND
    r.tick()
    assert r.motion.pose.is_finite() and r.motion.pose == before


@pytest.mark.parametrize("v, kw", [
    (Velocity(4.0, 4.0, 0.0), {}),                       # hypot 5.66 > 5 m/s
    (Velocity(0.0, 0.0, 2.5), {}),                       # vertical > 2 m/s
    (Velocity(0.0, 0.0, -2.5), {}),
    (Velocity(1.0, 0.0, 0.0, yaw_rate_deg_s=120.0), {}),  # > 90 deg/s
    (Velocity(3.0, 0.0, 0.0), {"max_horizontal_speed_mps": 2.0}),  # over its own cap
    (Velocity(0.0, 0.0, 1.5), {"max_vertical_speed_mps": 1.0}),
])
def test_over_speed_velocity_rejected_not_scaled(v, kw):
    r = _rig()
    ack = r.submit(_vel(r, v, **kw))
    assert not ack.accepted and ack.reason == Reason.VELOCITY_OUTSIDE_LIMITS
    assert ack.detail
    assert r.motion.velocity_setpoint is None


def test_velocity_exactly_at_limits_is_accepted():
    r = _rig()
    assert r.submit(_vel(r, Velocity(3.0, 4.0, 2.0, 90.0))).accepted
    r.tick()
    r.check()


def test_velocity_requires_expiry_and_velocity_and_no_target():
    r = _rig()
    assert r.submit(_vel(r, Velocity(1, 0, 0), lifecycle=True)).reason \
        == Reason.MISSING_EXPIRY
    assert r.submit(_vel(r)).reason == Reason.MISSING_VELOCITY
    assert r.submit(_vel(r, Velocity(1, 0, 0), target=Pose(1, 1, 5, 0))).reason == Reason.INVALID_COMMAND
    # A velocity on any other kind is malformed too.
    assert r.submit(r.cmd(K.HOVER, velocity=Velocity(1, 0, 0))).reason == Reason.INVALID_COMMAND


def test_velocity_rejected_on_ground_and_when_disarmed():
    r = Rig()
    assert r.submit(_vel(r, Velocity(1, 0, 0))).reason == Reason.NOT_ARMED
    assert r.submit(r.cmd(K.ARM, lifecycle=True)).accepted
    ack = r.submit(_vel(r, Velocity(1, 0, 0)))
    assert not ack.accepted and ack.reason == Reason.NOT_AIRBORNE


# --------------------------------------------------------------------------
# boundary hold
# --------------------------------------------------------------------------


def _fly_until_hold(r: Rig, c: Command, max_ticks: int = 400):
    for _ in range(max_ticks):
        fb = r.tick()
        if fb.status == CommandStatus.BOUNDARY_HOLD:
            return fb
        assert r.submit(r.renewed(c)).accepted
    raise AssertionError("never reached the boundary")


def test_ceiling_boundary_hold():
    r = _rig(29.0)
    c = _vel(r, Velocity(1.0, 0.0, 1.5))
    assert r.submit(c).accepted
    fb = _fly_until_hold(r, c)
    lim = r.motion.limits
    assert r.motion.pose.z == lim.max_altitude_m
    assert fb.status_reason == Reason.ALTITUDE_CEILING_BOUNDARY
    assert fb.flight_mode == FlightMode.HOLD and fb.target_velocity is None
    # Stopped exactly where the ceiling was crossed: x moved in proportion.
    assert r.motion.pose.x == pytest.approx(1.0 / 1.5)
    rec = [e for e in r.log.records if e.event == "velocity_boundary"]
    assert len(rec) == 1 and rec[0].decision == "hold_at_boundary"
    assert rec[0].fields["limit"] == Reason.ALTITUDE_CEILING_BOUNDARY
    assert rec[0].fields["command_id"] == c.command_id
    held = r.motion.pose
    # Renewing the finished velocity is refused; Motion keeps holding.
    assert r.submit(r.renewed(c)).reason == Reason.COMMAND_ID_REUSED
    for _ in range(5):
        fb = r.tick()
    assert r.motion.pose == held and fb.status == CommandStatus.BOUNDARY_HOLD
    assert fb.validate() is None
    r.check()


def test_geofence_boundary_hold():
    r = _rig(xy=(18.0, 0.0), limits=MotionLimits(geofence_radius_m=20.0))
    c = _vel(r, Velocity(3.0, 3.0, 0.0))
    assert r.submit(c).accepted
    fb = _fly_until_hold(r, c)
    p = r.motion.pose
    assert math.hypot(p.x, p.y) == pytest.approx(20.0, abs=1e-9)
    assert math.hypot(p.x, p.y) <= 20.0
    assert fb.status_reason == Reason.GEOFENCE_BOUNDARY
    r.check()


def test_floor_boundary_hold_is_not_a_landing():
    r = _rig(1.0)
    c = _vel(r, Velocity(0.0, 0.0, -2.0))
    assert r.submit(c).accepted
    fb = _fly_until_hold(r, c)
    assert r.motion.pose.z == 0.0
    assert fb.status_reason == Reason.ALTITUDE_FLOOR_BOUNDARY
    assert fb.grounded and not fb.landing_settled and fb.status != CommandStatus.LANDED
    r.check()


def test_velocity_toward_boundary_from_the_boundary_holds_without_moving():
    r = _rig(29.0)
    up = _vel(r, Velocity(0.0, 0.0, 2.0))
    r.submit(up)
    _fly_until_hold(r, up)
    here = r.motion.pose
    again = _vel(r, Velocity(0.0, 0.0, 1.0))
    assert r.submit(again).accepted
    fb = r.tick()
    assert r.motion.pose == here and fb.status == CommandStatus.BOUNDARY_HOLD
    # Moving away from the boundary is fine.
    down = _vel(r, Velocity(0.0, 0.0, -1.0))
    assert r.submit(down).accepted
    fb = r.tick()
    assert r.motion.pose.z == pytest.approx(29.9) and fb.status == CommandStatus.IN_PROGRESS
    r.check()


# --------------------------------------------------------------------------
# emergency latch
# --------------------------------------------------------------------------


def test_emergency_latch_wins_over_velocity():
    r = _rig(3.0)
    c = _vel(r, Velocity(2.0, 0.0, 0.0))
    assert r.submit(c).accepted
    r.tick()
    assert r.submit(r.cmd(K.EMERGENCY_LAND, lifecycle=True)).accepted
    x = r.motion.pose.x
    fb = r.motion.feedback(r.now)
    assert fb.flight_mode == FlightMode.EMERGENCY_LAND and fb.target_velocity is None
    assert fb.validate() is None
    # Renewing the old velocity and sending a new one are both refused, and
    # both are told the real reason: the emergency latch.
    assert r.submit(r.renewed(c)).reason == Reason.EMERGENCY_DESCENT_ACTIVE
    ack = r.submit(_vel(r, Velocity(0.0, 1.0, 0.0)))
    assert not ack.accepted and ack.reason == Reason.EMERGENCY_DESCENT_ACTIVE
    for _ in range(40):
        fb = r.tick()
    assert r.motion.pose.x == x and r.motion.pose.z == 0.0
    assert not fb.armed and fb.flight_mode == FlightMode.DISARMED
    r.check()


# --------------------------------------------------------------------------
# direct API (Safety's tracking-permission mapping: test_safety_adapter.py)
# --------------------------------------------------------------------------


def test_direct_set_velocity_refuses_instead_of_clamping():
    m = MotionStub(logger=MemoryLogSink(), clock=SimulationClock())
    assert not m.set_velocity(1.0, 0.0, 0.0)              # not armed
    m.arm()
    assert not m.set_velocity(1.0, 0.0, 0.0)              # on the ground
    m.takeoff(5.0)
    for _ in range(40):
        m.step(DT)
    assert not m.set_velocity(10.0, 0.0, 0.0)
    assert not m.set_velocity(float("nan"), 0.0, 0.0)
    assert m.set_velocity(1.0, 0.0, 0.0)
    m.step(DT)
    assert m.pose.x == pytest.approx(0.1)
    assert m.history[-1].velocity == Velocity(1.0, 0.0, 0.0)
