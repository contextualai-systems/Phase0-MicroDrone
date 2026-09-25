"""Tests for the kinematic MotionStub (motion_engine/motion_stubs.py).

The stub is kinematic on purpose, so these tests can be exact about timing:
at a 5 m/s horizontal cap, 20 m takes 4 s -- not "roughly". Logging is checked
against the real `mission.logging_format.Phase0Logger` with a fake clock, since
the log lines are part of the contract the mission layer relies on.
"""

import ast
import math
import sys
from pathlib import Path

_MOTION_ENGINE_ROOT = Path(__file__).resolve().parents[1]
_REPO_ROOT = _MOTION_ENGINE_ROOT.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from mission.logging_format import Phase0Logger
from motion_engine.motion_stubs import (
    MotionCommandType,
    MotionLimits,
    MotionStub,
    Pose,
    wrap_180,
)

DT = 0.1


class FakeClock:
    def __init__(self) -> None:
        self.t = 1_000.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def logger(clock):
    return Phase0Logger(clock=clock)


@pytest.fixture
def motion(logger, clock):
    return MotionStub(logger=logger, clock=clock)


def run_until_arrived(m: MotionStub, clock: FakeClock, max_s: float = 120.0) -> float:
    """Step until at_target(); return elapsed seconds."""
    elapsed = 0.0
    while not m.at_target():
        assert elapsed < max_s, f"did not arrive within {max_s}s; pose={m.pose} target={m.target}"
        m.step(DT)
        clock.t += DT
        elapsed += DT
    return elapsed


def settle(m: MotionStub, clock: FakeClock, max_s: float = 120.0) -> float:
    """Step until the pose sits *exactly* on the target (the stub snaps).

    `at_target()` goes true inside the arrival tolerance, up to a step early;
    tests about final pose or total travel time use this instead.
    """
    elapsed = 0.0
    while m.pose != m.target:
        assert elapsed < max_s, f"did not settle within {max_s}s; pose={m.pose} target={m.target}"
        m.step(DT)
        clock.t += DT
        elapsed += DT
    return elapsed


def airborne_at(m: MotionStub, clock: FakeClock, z: float) -> None:
    assert m.arm()
    assert m.takeoff(z)
    settle(m, clock)


def motion_records(logger, event=None):
    return [r for r in logger.records if r.module == "MOTION" and (event is None or r.event == event)]


# -- basics -----------------------------------------------------------------


def test_pose_helpers():
    a, b = Pose(0, 0, 0, 0), Pose(3, 4, 12, 90)
    assert a.distance_to(b) == pytest.approx(13.0)
    assert a.horizontal_distance_to(b) == pytest.approx(5.0)
    assert b.as_tuple() == (3, 4, 12, 90)


def test_initial_state(motion):
    assert motion.pose == Pose(0, 0, 0, 0)
    assert motion.target is None
    assert not motion.armed and not motion.airborne
    assert motion.active_command is None and motion.history == []


def test_logger_none_is_fine():
    m = MotionStub(logger=None)
    assert m.arm() and m.takeoff(5.0)
    for _ in range(100):
        m.step(DT)
    assert m.pose.z == pytest.approx(5.0)


# -- kinematics ---------------------------------------------------------------


def test_takeoff_uses_vertical_cap(motion, clock):
    motion.arm()
    motion.takeoff(10.0)
    t = settle(motion, clock)
    assert t == pytest.approx(10.0 / 2.0, abs=DT + 1e-9)  # 2 m/s vertical cap
    assert motion.pose.z == 10.0 and motion.airborne


def test_move_to_converges_in_expected_time(motion, clock):
    airborne_at(motion, clock, 10.0)
    assert motion.move_to(20.0, 0.0, 10.0)
    t = settle(motion, clock)
    assert t == pytest.approx(20.0 / 5.0, abs=DT + 1e-9)
    # Snapped exactly, not asymptotically close.
    assert motion.pose.as_tuple() == (20.0, 0.0, 10.0, 0.0)


def test_diagonal_is_straight_and_capped(motion, clock):
    airborne_at(motion, clock, 10.0)
    motion.move_to(30.0, 40.0, 10.0)
    prev = motion.pose
    motion.step(DT)
    step_len = prev.horizontal_distance_to(motion.pose)
    assert step_len == pytest.approx(5.0 * DT)
    # Heading of the step matches the 3-4-5 direction.
    assert (motion.pose.y - prev.y) / (motion.pose.x - prev.x) == pytest.approx(40.0 / 30.0)


def test_vertical_cap_is_independent(motion, clock):
    airborne_at(motion, clock, 2.0)
    motion.move_to(5.0, 0.0, 22.0)  # 5 m horizontal (1 s), 20 m vertical (10 s)
    motion.step(1.0)
    assert (motion.pose.x, motion.pose.z) == (5.0, 4.0)  # horizontal done, vertical 2 m/s
    t = 1.0 + settle(motion, clock)
    assert t == pytest.approx(10.0, abs=DT + 1e-9)


def test_speed_override_only_lowers(motion, clock):
    airborne_at(motion, clock, 10.0)
    motion.move_to(10.0, 0.0, 10.0, speed_mps=2.0)
    assert settle(motion, clock) == pytest.approx(5.0, abs=DT + 1e-9)
    motion.move_to(0.0, 0.0, 10.0, speed_mps=50.0)  # capped back to 5 m/s
    assert settle(motion, clock) == pytest.approx(2.0, abs=DT + 1e-9)


@pytest.mark.parametrize(
    "start,goal,first_step",
    [(350.0, 10.0, +9.0), (10.0, 350.0, -9.0), (90.0, 270.0, +9.0), (0.0, 45.0, +9.0)],
)
def test_yaw_takes_shortest_path(motion, clock, start, goal, first_step):
    motion.arm()
    motion.set_yaw(start)
    settle(motion, clock)
    motion.set_yaw(goal)
    before = motion.pose.yaw_deg
    motion.step(DT)  # 90 deg/s * 0.1 s = 9 deg
    assert wrap_180(motion.pose.yaw_deg - before) == pytest.approx(first_step)
    t = settle(motion, clock) + DT
    expected = abs(wrap_180(goal - start)) / 90.0
    assert t == pytest.approx(expected, abs=DT + 1e-9)
    assert motion.pose.yaw_deg == pytest.approx(goal)


def test_350_to_10_turns_plus_20(motion, clock):
    motion.arm()
    motion.set_yaw(350.0)
    settle(motion, clock)
    motion.set_yaw(10.0)
    total = 0.0
    prev = motion.pose.yaw_deg
    while motion.pose != motion.target:
        motion.step(DT)
        total += wrap_180(motion.pose.yaw_deg - prev)
        prev = motion.pose.yaw_deg
    assert total == pytest.approx(20.0)


def test_at_target_uses_arrival_tolerance(motion, clock):
    airborne_at(motion, clock, 10.0)
    motion.move_to(10.0, 0.0, 10.0)
    t = run_until_arrived(motion, clock)
    assert t == pytest.approx(2.0, abs=DT + 1e-9)
    assert motion.pose.horizontal_distance_to(motion.target) <= motion.limits.arrival_tolerance_m
    assert motion.at_target(tol=0.0) == (motion.pose == motion.target)


def test_step_zero_or_no_target_is_noop(motion):
    assert motion.step(1.0) == Pose(0, 0, 0, 0)
    motion.arm()
    motion.takeoff(5.0)
    assert motion.step(0.0) == Pose(0, 0, 0, 0)


# -- geofence -----------------------------------------------------------------


def test_geofence_clamps_ceiling_floor_and_radius(motion, clock, logger):
    lim = motion.limits
    airborne_at(motion, clock, 10.0)

    assert motion.move_to(0.0, 0.0, 100.0)
    assert motion.target.z == lim.max_altitude_m

    assert motion.move_to(0.0, 0.0, -5.0)
    assert motion.target.z == lim.min_altitude_m

    assert motion.move_to(300.0, 400.0, 10.0)  # 500 m out, fence is 150 m
    assert math.hypot(motion.target.x, motion.target.y) == pytest.approx(lim.geofence_radius_m)
    assert motion.target.y / motion.target.x == pytest.approx(400.0 / 300.0)  # same direction

    clamped = [r for r in motion_records(logger, "move_to") if r.decision == "clamped_to_geofence"]
    assert len(clamped) == 3
    assert clamped[-1].fields["req_x"] == 300.0


def test_geofence_is_centered_on_home(clock):
    m = MotionStub(home=Pose(100.0, 0.0, 0.0, 0.0), clock=clock)
    m.arm()
    m.move_to(300.0, 0.0, 10.0)
    assert m.target.x == pytest.approx(250.0)


def test_altitude_floor_blocks_flight_not_landing(clock):
    m = MotionStub(limits=MotionLimits(min_altitude_m=3.0), clock=clock)
    airborne_at(m, clock, 10.0)
    m.move_to(0.0, 0.0, 1.0)
    assert m.target.z == 3.0
    m.descend_to_dock(Pose(0, 0, 0, 0))
    assert m.target.z == 0.0


def test_unclamped_command_has_no_clamp_decision(motion, logger):
    motion.arm()
    motion.move_to(10.0, 5.0, 12.0, yaw_deg=90.0)
    rec = motion_records(logger, "move_to")[-1]
    assert rec.decision is None
    assert rec.fields == {"x": 10.0, "y": 5.0, "z": 12.0, "yaw": 90.0}
    assert rec.to_text().endswith("MOTION: move_to(x=10.0, y=5.0, z=12.0, yaw=90.0)")


# -- arming rules ---------------------------------------------------------------


@pytest.mark.parametrize(
    "call",
    [
        lambda m: m.takeoff(10.0),
        lambda m: m.move_to(1.0, 2.0, 3.0),
        lambda m: m.hover(),
        lambda m: m.stop(),
        lambda m: m.set_yaw(90.0),
        lambda m: m.set_orientation(0.0, 0.0, 90.0),
        lambda m: m.track_target(0.0, 40.0),
        lambda m: m.goto_dock_approach(Pose(0, 0, 0, 0)),
        lambda m: m.descend_to_dock(Pose(0, 0, 0, 0)),
        lambda m: m.land(),
        lambda m: m.emergency_land(),
    ],
)
def test_motion_commands_rejected_when_disarmed(motion, logger, call):
    assert call(motion) is False
    assert motion.target is None and motion.history == []
    assert motion_records(logger)[-1].decision == "rejected_not_armed"
    motion.step(1.0)
    assert motion.pose == Pose(0, 0, 0, 0)


def test_disarm_refused_while_airborne(motion, clock, logger):
    airborne_at(motion, clock, 5.0)
    assert motion.disarm() is False
    assert motion.armed
    assert motion_records(logger, "disarm")[-1].decision == "rejected_airborne"
    motion.land()
    settle(motion, clock)
    assert not motion.airborne and motion.armed  # plain land() does not auto-disarm
    assert motion.disarm() is True
    assert not motion.armed and motion.target is None


def test_history_and_active_command(motion, clock):
    clock.t = 42.0
    motion.arm()
    motion.takeoff(5.0)
    assert [c.type for c in motion.history] == [MotionCommandType.ARM, MotionCommandType.TAKEOFF]
    assert motion.active_command.type == MotionCommandType.TAKEOFF
    assert motion.active_command.issued_at == 42.0
    assert motion.active_command.target == Pose(0, 0, 5.0, 0)


# -- mission-shaped commands ------------------------------------------------------


def test_track_target_standoff_geometry(motion, clock, logger):
    airborne_at(motion, clock, 12.0)
    assert motion.track_target(bearing_deg=0.0, distance_m=40.0, standoff_m=15.0)
    t = motion.target
    assert (t.x, t.y, t.z, t.yaw_deg) == pytest.approx((25.0, 0.0, 12.0, 0.0))
    rec = motion_records(logger, "track_target")[-1]
    assert rec.decision == "close_to_standoff"


def test_track_target_bearing_north_faces_bird_and_altitude(motion, clock):
    airborne_at(motion, clock, 12.0)
    motion.track_target(bearing_deg=90.0, distance_m=30.0, standoff_m=10.0, altitude_m=15.0)
    t = motion.target
    assert (t.x, t.y, t.z, t.yaw_deg) == pytest.approx((0.0, 20.0, 15.0, 90.0), abs=1e-9)
    settle(motion, clock)
    assert motion.pose.yaw_deg == pytest.approx(90.0)


def test_track_target_backs_off_when_too_close(motion, clock, logger):
    airborne_at(motion, clock, 12.0)
    motion.track_target(bearing_deg=0.0, distance_m=5.0, standoff_m=15.0)
    assert motion.target.x == pytest.approx(-10.0)
    assert motion_records(logger, "track_target")[-1].decision == "back_off_to_standoff"


def test_dock_approach_then_descend_reaches_dock(motion, clock):
    dock = Pose(10.0, -5.0, 0.0, 90.0)
    airborne_at(motion, clock, 12.0)
    motion.move_to(40.0, 30.0, 12.0)
    settle(motion, clock)

    assert motion.goto_dock_approach(dock, approach_altitude_m=5.0)
    settle(motion, clock)
    assert motion.pose.as_tuple() == pytest.approx((10.0, -5.0, 5.0, 90.0))

    assert motion.descend_to_dock(dock)
    t = settle(motion, clock)
    assert t == pytest.approx(5.0 / 2.0, abs=DT + 1e-9)
    assert motion.pose.as_tuple() == pytest.approx(dock.as_tuple())
    assert not motion.airborne
    assert motion.disarm()


def test_hover_and_stop_hold_current_pose(motion, clock, logger):
    airborne_at(motion, clock, 10.0)
    motion.move_to(50.0, 0.0, 10.0)
    for _ in range(10):
        motion.step(DT)
    here = motion.pose
    assert motion.stop()
    assert motion.active_command.type == MotionCommandType.STOP
    motion.step(1.0)
    assert motion.pose == here
    assert motion.hover()
    assert motion.active_command.type == MotionCommandType.HOVER
    assert motion_records(logger, "stop") and motion_records(logger, "hover")


def test_set_orientation_only_yaw_moves(motion, clock, logger):
    motion.arm()
    assert motion.set_orientation(10.0, -5.0, 45.0)
    settle(motion, clock)
    assert motion.pose == Pose(0, 0, 0, 45.0)
    rec = motion_records(logger, "set_orientation")[-1]
    assert rec.fields == {"roll": 10.0, "pitch": -5.0, "yaw": 45.0}
    assert rec.decision == "roll_pitch_not_simulated"


# -- emergency land ---------------------------------------------------------------


def test_emergency_land_descends_in_place_ignores_commands_and_auto_disarms(motion, clock, logger):
    airborne_at(motion, clock, 6.0)
    motion.move_to(20.0, 0.0, 6.0)
    for _ in range(10):
        motion.step(DT)
    x0 = motion.pose.x
    assert motion.emergency_land()
    assert motion.emergency_landing
    assert motion_records(logger, "emergency_land")[-1].decision == "synthetic_only"

    # Other commands are refused while the latch is set.
    assert motion.move_to(0.0, 0.0, 20.0) is False
    assert motion.hover() is False
    assert motion.disarm() is False
    assert motion_records(logger, "move_to")[-1].decision == "ignored_emergency_landing"
    assert motion.emergency_land() is True  # idempotent...
    assert len(motion_records(logger, "emergency_land")) == 1  # ...and not re-logged

    motion.step(1.0)
    assert motion.pose.x == x0  # in place
    assert motion.pose.z == pytest.approx(5.0)  # 1 m/s emergency descent

    elapsed = 1.0
    while motion.armed:
        assert elapsed < 20.0
        motion.step(DT)
        elapsed += DT
    assert elapsed == pytest.approx(6.0, abs=DT + 1e-9)
    assert motion.pose == Pose(x0, 0.0, 0.0, 0.0)
    assert not motion.airborne and not motion.emergency_landing
    auto = motion_records(logger, "disarm")[-1]
    assert auto.decision == "auto_disarm_on_ground"
    assert motion.history[-1].type == MotionCommandType.DISARM

    # Latch cleared: the vehicle can be re-armed afterwards.
    assert motion.arm()


# -- logging ----------------------------------------------------------------------


def test_arrival_logged_exactly_once(motion, clock, logger):
    airborne_at(motion, clock, 10.0)
    motion.move_to(10.0, 0.0, 10.0)
    settle(motion, clock)
    for _ in range(50):
        motion.step(DT)
    # Re-issuing the same target (as a state machine does every tick) must not re-log.
    motion.move_to(10.0, 0.0, 10.0)
    motion.hover()
    for _ in range(10):
        motion.step(DT)
    arrived = motion_records(logger, "arrived")
    assert len(arrived) == 2  # one for takeoff, one for the move_to
    assert arrived[-1].fields == {"x": 10.0, "y": 0.0, "z": 10.0}


def test_step_does_not_log_every_tick(motion, clock, logger):
    motion.arm()
    motion.takeoff(10.0)
    n = len(logger.records)
    for _ in range(20):  # 2 s, not arrived yet
        motion.step(DT)
    assert len(logger.records) == n


def test_every_accepted_command_logs_one_motion_record(motion, clock, logger):
    calls = [
        lambda: motion.arm(),
        lambda: motion.takeoff(10.0),
        lambda: motion.move_to(5.0, 5.0, 10.0),
        lambda: motion.set_yaw(90.0),
        lambda: motion.set_orientation(0.0, 0.0, 180.0),
        lambda: motion.track_target(0.0, 40.0),
        lambda: motion.hover(),
        lambda: motion.stop(),
        lambda: motion.goto_dock_approach(Pose(0, 0, 0, 0)),
        lambda: motion.descend_to_dock(Pose(0, 0, 0, 0)),
        lambda: motion.land(),
    ]
    for call in calls:
        n = len(motion_records(logger))
        assert call() is True
        assert len(motion_records(logger)) == n + 1


def test_log_context_is_attached(motion, logger):
    motion.set_log_context(state="TRACKING", truth={"bird_distance_m": 31.9})
    motion.arm()
    rec = motion_records(logger, "arm")[-1]
    assert rec.state == "TRACKING" and rec.truth == {"bird_distance_m": 31.9}


# -- the Phase-1 contract ---------------------------------------------------------

#: Public methods that are *not* vehicle commands (queries / log plumbing).
_NON_COMMANDS = {"at_target", "set_log_context"}


def test_every_public_command_has_phase1_hook():
    """Every command (and step) must say which real PX4 command it becomes.

    Enumerates public methods from the source rather than a hard-coded list,
    so a new command added without a hook fails this test.
    """
    src_path = _MOTION_ENGINE_ROOT / "motion_stubs.py"
    source = src_path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MotionStub")
    methods = [
        n for n in cls.body
        if isinstance(n, ast.FunctionDef)
        and not n.name.startswith("_")
        and not any(isinstance(d, ast.Name) and d.id == "property" for d in n.decorator_list)
        and n.name not in _NON_COMMANDS
    ]
    names = {m.name for m in methods}
    expected = {
        "arm", "disarm", "takeoff", "move_to", "hover", "stop", "set_yaw",
        "set_orientation", "track_target", "goto_dock_approach", "descend_to_dock",
        "land", "emergency_land", "step",
    }
    assert expected <= names
    lines = source.splitlines()
    missing = [
        m.name for m in methods
        if "# PHASE-1 HOOK:" not in "\n".join(lines[m.lineno - 1:m.end_lineno])
    ]
    assert missing == [], f"methods without a '# PHASE-1 HOOK:' comment: {missing}"


def test_position_hooks_mention_ned_conversion():
    source = (_MOTION_ENGINE_ROOT / "motion_stubs.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MotionStub")
    lines = source.splitlines()
    bodies = {
        n.name: "\n".join(lines[n.lineno - 1:n.end_lineno])
        for n in cls.body if isinstance(n, ast.FunctionDef)
    }
    for name in ("move_to", "track_target", "goto_dock_approach", "step"):
        assert "NED" in bodies[name], name
    for name, cmd in {
        "arm": "MAV_CMD_COMPONENT_ARM_DISARM",
        "disarm": "MAV_CMD_COMPONENT_ARM_DISARM",
        "takeoff": "MAV_CMD_NAV_TAKEOFF",
        "move_to": "SET_POSITION_TARGET_LOCAL_NED",
        "track_target": "/fmu/in/trajectory_setpoint",
        "goto_dock_approach": "SET_POSITION_TARGET_LOCAL_NED",
        "hover": "MAV_CMD_NAV_LOITER_UNLIM",
        "stop": "MAV_CMD_NAV_LOITER_UNLIM",
        "set_yaw": "MAV_CMD_CONDITION_YAW",
        "set_orientation": "SET_ATTITUDE_TARGET",
        "descend_to_dock": "MAV_CMD_NAV_LAND",
        "land": "MAV_CMD_NAV_LAND",
        "emergency_land": "FLIGHTTERMINATION",
        "step": "/fmu/out/vehicle_local_position",
    }.items():
        assert cmd in bodies[name], (name, cmd)
