"""WaypointExecutor: Motion flies a list of poses and reports progress.

Every test drives the real MotionStub through the coordinator's tick
(INTEGRATION_README §3): executor command -> Safety's command check ->
MotionStub.submit -> step -> feedback -> executor.update. Flown paths are
recorded and run through the trace checker, so no test can pass while
breaking a speed, geofence or expiry rule along the way.
"""

import math
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from motion_engine.contracts import (
    Command, MemoryLogSink, MotionCommandType as K, Pose, SimulationClock,
)
from motion_engine.motion_stubs import MotionLimits, MotionStub
from motion_engine.safety_adapter import check_for_motion
from motion_engine.trace_checker import TraceRecorder, check_trace
from motion_engine.waypoints import (
    WaypointExecutor, WaypointReason as R, WaypointStatus as St,
)
from safety_layer import config
from safety_layer.safety_policy import ACTIONS, SafetyAssessment

DT = 0.1
ALT = 5.0
ROUTE = [Pose(20.0, 0.0, ALT, 0.0), Pose(20.0, 15.0, ALT, 90.0), Pose(0.0, 15.0, 8.0, 180.0)]


def assessment(blocked=()):
    return SafetyAssessment(permissions={a: a not in blocked for a in ACTIONS})


class Rig:
    """One coordinator: clock, recorded MotionStub, Safety, executor."""

    def __init__(self, limits=None):
        self.clock = SimulationClock()
        self.log = MemoryLogSink()
        self.motion = MotionStub(clock=self.clock, limits=limits or MotionLimits.from_config())
        self.rec = TraceRecorder(self.motion)
        self.rec.snapshot(self.clock.timestamp_ns)
        self.ex = WaypointExecutor(limits=self.motion.limits, logger=self.log)
        self.safety = assessment()
        self.acks = []
        self.sent = []

    @property
    def now(self):
        return self.clock.timestamp_ns

    def lifecycle(self, cid, kind, target=None):
        c = Command(cid, kind, "HOVERING", self.now, None, "test", "test", target=target)
        assert self.rec.submit(c, self.now).accepted

    def launch(self):
        self.lifecycle("arm", K.ARM)
        self.lifecycle("takeoff", K.TAKEOFF, Pose(0.0, 0.0, ALT, 0.0))
        while self.motion.pose.z < ALT:  # the full climb, not just arrival tolerance
            self.step()

    def step(self):
        self.rec.step(DT, self.now)
        self.clock.advance(DT)
        return self.motion.feedback(self.now)

    def tick(self, send=True):
        cmd = self.ex.command("TRACKING", self.now) if send else None
        if cmd is not None:
            to_submit, _ = check_for_motion(cmd, self.safety, self.now)
            ack = self.rec.submit(to_submit, self.now)
            self.sent.append(to_submit)
            self.acks.append(ack)
            self.ex.on_ack(ack)
        return self.ex.update(self.step())

    def fly(self, max_ticks=2000):
        for _ in range(max_ticks):
            p = self.tick()
            if p.status != St.IN_PROGRESS:
                return p
        raise AssertionError("route did not finish")

    def check(self):
        report = check_trace(self.rec.samples, limits=self.motion.limits, home=self.motion.home)
        assert report.ok, report.format_text()


@pytest.fixture
def rig():
    r = Rig()
    r.launch()
    return r


# -- nominal route -------------------------------------------------------------


def test_flies_every_waypoint_in_order_and_completes(rig):
    p = rig.ex.load("patrol", ROUTE, rig.now)
    assert p.status == St.IN_PROGRESS and p.index == 0 and p.total == 3 and p.reached == 0
    reached_at = []
    for _ in range(2000):
        before = rig.ex.progress.reached
        p = rig.tick()
        if p.reached > before:
            reached_at.append(rig.motion.pose)
        if p.status != St.IN_PROGRESS:
            break
    assert p.status == St.COMPLETE and p.complete and p.reason == R.ROUTE_COMPLETE
    assert p.reached == 3 and p.index is None and p.target is None
    assert reached_at == ROUTE                      # in order, exactly on each one
    assert rig.ex.command("TRACKING", rig.now) is None  # nothing more after the last
    rig.check()


def test_every_command_is_a_move_to_and_ids_renew_per_waypoint(rig):
    rig.ex.load("patrol", ROUTE, rig.now)
    rig.fly()
    assert all(c.kind == K.MOVE_TO and c.source == "waypoint_executor" for c in rig.sent)
    assert [c.command_id for c in rig.sent if c.command_id.endswith("wp0-1")]  # renewed
    ids = list(dict.fromkeys(c.command_id for c in rig.sent))
    assert ids == ["patrol-wp0-1", "patrol-wp1-2", "patrol-wp2-3"]
    assert all(a.accepted for a in rig.acks)
    assert all(c.expires_ns - c.issued_ns == int(config.COMMAND_VALIDITY_S * 1e9) for c in rig.sent)


def test_progress_distance_shrinks_toward_each_waypoint(rig):
    rig.ex.load("one", [Pose(30.0, 0.0, ALT, 0.0)], rig.now)
    distances = []
    while True:
        p = rig.tick()
        if p.status != St.IN_PROGRESS:
            break
        distances.append(p.distance_to_target_m)
    assert distances == sorted(distances, reverse=True)
    assert distances[0] == pytest.approx(30.0 - 5.0 * DT)


def test_arrival_is_logged_once_per_waypoint(rig):
    rig.ex.load("patrol", ROUTE, rig.now)
    rig.fly()
    events = [r.event for r in rig.log.records]
    assert events.count("waypoint_reached") == 3
    assert events[0] == "waypoint_route_loaded" and events[-1] == "waypoint_route_complete"


# -- validation before anything flies --------------------------------------------


@pytest.mark.parametrize(
    "route, reason, detail",
    [
        ([], R.EMPTY_ROUTE, None),
        ([Pose(1.0, 0.0, ALT), Pose(math.nan, 0.0, ALT)], R.INVALID_WAYPOINT, "waypoint 1"),
        ([Pose(1.0, 0.0, math.inf)], R.INVALID_WAYPOINT, "waypoint 0"),
        ([(1.0, 2.0, 3.0)], R.INVALID_WAYPOINT, "waypoint 0"),
        ([Pose(1.0, 0.0, ALT), Pose(0.0, 0.0, 31.0)], R.WAYPOINT_OUTSIDE_LIMITS, "waypoint 1: above_ceiling"),
        ([Pose(151.0, 0.0, ALT)], R.WAYPOINT_OUTSIDE_LIMITS, "waypoint 0: outside_geofence"),
        ([Pose(0.0, 0.0, -1.0)], R.WAYPOINT_OUTSIDE_LIMITS, "waypoint 0: below_floor"),
    ],
)
def test_bad_routes_fail_at_load_and_never_fly(rig, route, reason, detail):
    start = rig.motion.pose
    p = rig.ex.load("bad", route, rig.now)
    assert p.status == St.FAILED and p.failed and p.reason == reason and p.detail == detail
    assert rig.ex.command("TRACKING", rig.now) is None
    for _ in range(20):
        rig.tick()
    assert rig.motion.pose == start


def test_route_limits_come_from_safety_config(monkeypatch):
    monkeypatch.setattr(config, "GEOFENCE_RADIUS_M", 10.0)
    ex = WaypointExecutor()
    assert ex.load("r", [Pose(11.0, 0.0, ALT)]).reason == R.WAYPOINT_OUTSIDE_LIMITS
    assert ex.load("r", [Pose(9.0, 0.0, ALT)]).status == St.IN_PROGRESS


def test_route_id_is_required():
    assert WaypointExecutor().load("", ROUTE).reason == R.INVALID_ROUTE_ID


# -- interruptions ---------------------------------------------------------------


def test_motion_rejection_fails_the_route_with_motions_reason():
    r = Rig()  # never armed
    r.ex.load("patrol", ROUTE, r.now)
    p = r.tick()
    assert p.status == St.FAILED and p.reason == R.COMMAND_REJECTED
    assert p.detail == "waypoint 0: not_armed"
    assert r.ex.command("TRACKING", r.now) is None


def test_safety_hold_does_not_count_as_arrival_and_route_resumes(rig):
    rig.ex.load("patrol", ROUTE, rig.now)
    for _ in range(10):
        rig.tick()
    rig.safety = assessment(blocked=("tracking",))   # Safety swaps in a HOVER
    held_at = None
    for _ in range(20):
        p = rig.tick()
        held_at = held_at or rig.motion.pose
        assert p.reached == 0 and p.status == St.IN_PROGRESS
    assert rig.motion.pose == held_at                # held still, did not "arrive"
    assert rig.sent[-1].kind == K.HOVER and rig.sent[-1].source == "safety"
    rig.safety = assessment()
    p = rig.fly()
    assert p.status == St.COMPLETE and p.reached == 3
    assert rig.motion.pose == ROUTE[-1]
    rig.check()


def test_expired_command_is_reissued_under_a_new_id(rig):
    rig.ex.load("patrol", ROUTE, rig.now)
    for _ in range(5):
        rig.tick()
    first_id = rig.ex.progress.command_id
    for _ in range(10):                              # coordinator stalls: no renewals
        rig.tick(send=False)
    stalled = rig.motion.pose
    assert rig.motion.feedback(rig.now).status.value == "expired_hold"
    assert rig.ex.progress.command_id != first_id    # will not reuse an expired ID
    p = rig.fly()
    assert p.status == St.COMPLETE
    assert all(a.accepted for a in rig.acks)         # no COMMAND_ID_REUSED rejections
    assert stalled != ROUTE[0]
    rig.check()


def test_cancel_stops_commands_and_load_replaces_route(rig):
    rig.ex.load("patrol", ROUTE, rig.now)
    rig.tick()
    p = rig.ex.cancel(rig.now)
    assert p.status == St.IDLE and p.reason == R.CANCELLED
    assert rig.ex.command("TRACKING", rig.now) is None
    rig.ex.load("home", [Pose(0.0, 0.0, ALT, 180.0)], rig.now)
    assert rig.fly().status == St.COMPLETE
    assert rig.motion.pose == Pose(0.0, 0.0, ALT, 180.0)
    rig.check()


def test_emergency_latch_rejects_waypoints(rig):
    rig.lifecycle("em", K.EMERGENCY_LAND)
    rig.ex.load("patrol", ROUTE, rig.now)
    p = rig.tick()
    assert p.status == St.FAILED and p.detail == "waypoint 0: emergency_descent_active"


def test_progress_serializes():
    ex = WaypointExecutor()
    d = ex.load("patrol", ROUTE).to_dict()
    assert d["status"] == "in_progress" and d["target"] == ROUTE[0].to_dict()
    assert d["command_id"] == "patrol-wp0-1"
