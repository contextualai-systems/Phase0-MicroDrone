"""WaypointExecutor: Motion flies a list of poses and reports progress.

Every test drives the real MotionStub through the coordinator's tick
(INTEGRATION_README §3): executor command -> Safety's command check ->
MotionStub.submit -> step -> feedback -> executor.update. Flown paths are
recorded and run through the trace checker, so no test can pass while
breaking a speed, geofence or expiry rule along the way.
"""

import math

import pytest

from motion_engine.contracts import (
    Command, CommandStatus, MemoryLogSink, MotionCommandType as K, Pose, SimulationClock,
)
from motion_engine.motion_stubs import MotionLimits, MotionStub
from motion_engine.safety_adapter import check_for_motion
from motion_engine.trace_checker import TraceRecorder, check_trace
from motion_engine.waypoints import (
    WaypointExecutor, WaypointReason as R, WaypointStatus as St,
)
from motion_engine.tests.motion_rig import DT, assessment
from safety_layer import config

ALT = 5.0
ROUTE = [Pose(20.0, 0.0, ALT, 0.0), Pose(20.0, 15.0, ALT, 90.0), Pose(0.0, 15.0, 8.0, 180.0)]


class WaypointRig:
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
    r = WaypointRig()
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
    events = [r.event for r in rig.log.records]     # arrival logged once per waypoint
    assert events.count("waypoint_reached") == 3
    assert events[0] == "waypoint_route_loaded" and events[-1] == "waypoint_route_complete"
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
    r = WaypointRig()  # never armed
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


# -- faults in the middle of a route ------------------------------------------------
#
# Each of these starts the patrol, flies 3 s toward the first waypoint, then
# injects the fault. The question every test answers: does the route stop
# counting progress while the fault lasts, and what state is it left in?


def _mid_route(rig):
    """Load ROUTE and fly 3 s of it: about 15 m toward waypoint 0, not there yet."""
    rig.ex.load("patrol", ROUTE, rig.now)
    for _ in range(30):
        p = rig.tick()
    assert p.status == St.IN_PROGRESS and p.reached == 0
    return rig.motion.pose


def _ticks_until_disarmed(rig, max_ticks=200):
    for _ in range(max_ticks):
        p = rig.tick()
        if not rig.motion.armed:
            return p
    raise AssertionError("never touched down and disarmed")


def test_emergency_from_safety_mid_route_lands_in_place_and_never_counts_a_waypoint(rig):
    """Safety's emergency verdict turns every route command into EMERGENCY_LAND.
    The drone goes straight down where it is, the route records no arrival,
    and once it is on the ground and disarmed the route fails rather than
    relaunching."""
    start = _mid_route(rig)
    rig.safety = assessment(emergency=True)
    p = _ticks_until_disarmed(rig)
    assert rig.motion.pose.horizontal_distance_to(start) == 0.0
    assert rig.motion.pose.z == 0.0
    assert p.reached == 0
    assert not [r for r in rig.log.records if r.event == "waypoint_reached"]
    assert {c.kind for c in rig.sent[30:]} == {K.EMERGENCY_LAND}

    rig.safety = assessment()                     # emergency clears on the ground
    p = rig.tick()
    assert p.status == St.FAILED and p.reason == R.COMMAND_REJECTED
    assert p.detail == "waypoint 0: not_armed"
    assert rig.ex.command("TRACKING", rig.now) is None
    rig.check()


def test_emergency_sent_mid_route_fails_the_route_and_lands_in_place(rig):
    """The coordinator sends EMERGENCY_LAND itself (not through Safety). The
    route's next command (a renewal of the ID the emergency replaced) is
    refused with the real reason, so the route fails at the waypoint it was
    flying with `emergency_descent_active`, stops commanding, and the
    descent is straight down."""
    start = _mid_route(rig)
    rig.lifecycle("em", K.EMERGENCY_LAND)
    p = rig.tick()
    assert p.status == St.FAILED and p.reason == R.COMMAND_REJECTED
    assert p.detail == "waypoint 0: emergency_descent_active"
    assert rig.ex.command("TRACKING", rig.now) is None
    n_sent = len(rig.sent)
    _ticks_until_disarmed(rig)
    assert len(rig.sent) == n_sent, "a failed route must not keep sending"
    assert rig.motion.pose.horizontal_distance_to(start) == 0.0
    assert rig.motion.pose.z == 0.0
    rig.check()


def test_low_battery_mid_route_holds_and_the_return_goes_through_docking(rig):
    """Low battery blocks launch and tracking, and waypoint MOVE_TOs use the
    tracking permission, so the route holds in place and cannot be the way
    home. The coordinator cancels it and returns with GOTO_DOCK_APPROACH /
    DESCEND_TO_DOCK, which low battery still allows."""
    _mid_route(rig)
    rig.safety = assessment(blocked=("launch", "tracking"))
    held = None
    for _ in range(30):
        p = rig.tick()
        held = held or rig.motion.pose
        assert rig.motion.pose == held and p.reached == 0 and p.status == St.IN_PROGRESS
    assert rig.sent[-1].kind == K.HOVER and rig.sent[-1].source == "safety"

    assert rig.ex.cancel(rig.now).reason == R.CANCELLED
    pad = Pose(0.0, 0.0, 0.0, 0.0)
    validity_ns = int(config.COMMAND_VALIDITY_S * 1e9)
    for cid, kind, target, done in (
        ("approach", K.GOTO_DOCK_APPROACH, Pose(0.0, 0.0, ALT, 0.0), lambda fb: fb.pose == fb.target),
        ("descend", K.DESCEND_TO_DOCK, pad, lambda fb: fb.landing_settled),
    ):
        for _ in range(1000):                     # renewed every tick, same ID
            c = Command(cid, kind, "DOCKING_APPROACH", rig.now, rig.now + validity_ns,
                        "test", "test", target=target)
            to_send, decision = check_for_motion(c, rig.safety, rig.now)
            assert decision.allowed, decision.reasons
            assert rig.rec.submit(to_send, rig.now).accepted
            if done(rig.step()):
                break
        else:
            raise AssertionError(f"{cid} never finished")
    assert rig.motion.landing_settled and rig.motion.pose.horizontal_distance_to(pad) == 0.0
    rig.check()


def test_comm_loss_landing_mid_route_is_called_off_when_the_coordinator_returns(rig):
    """The coordinator goes silent for 12 s mid-route: Motion holds, then
    after its 10 s comm-loss timeout starts landing in place. When the
    coordinator comes back before touchdown, the executor re-issues the
    waypoint under a fresh ID (an expired one would be refused), Motion
    accepts it, calls off the landing, and the route finishes in order."""
    _mid_route(rig)
    for _ in range(120):
        rig.tick(send=False)
    fb = rig.motion.feedback(rig.now)
    assert fb.status == CommandStatus.COMM_LOSS_LANDING and fb.airborne
    assert rig.ex.progress.status == St.IN_PROGRESS and rig.ex.progress.reached == 0

    first_id = rig.sent[-1].command_id
    rig.tick()
    assert rig.acks[-1].accepted and rig.acks[-1].command_id != first_id
    assert not rig.motion.comm_loss_landing
    p = rig.fly()
    assert p.status == St.COMPLETE and p.reached == 3
    assert rig.motion.pose == ROUTE[-1]
    assert all(a.accepted for a in rig.acks), [a.reason for a in rig.acks if not a.accepted]
    rig.check()


def test_route_does_not_lift_off_after_a_comm_loss_landing_in_the_field(rig):
    """The coordinator stays silent until the comm-loss landing has touched
    down away from the pad. When it comes back, the route's next MOVE_TO is
    refused (`not_airborne`): only a checked TAKEOFF may lift the drone off,
    so the route fails and the drone stays on the ground."""
    _mid_route(rig)
    for _ in range(200):
        rig.tick(send=False)
    fb = rig.motion.feedback(rig.now)
    assert fb.status == CommandStatus.LANDED and fb.grounded and fb.armed
    landed_at = rig.motion.pose

    p = rig.tick()
    assert p.status == St.FAILED and p.reason == R.COMMAND_REJECTED
    assert p.detail == "waypoint 0: not_airborne"
    for _ in range(10):
        rig.tick()
    assert rig.motion.pose == landed_at
    rig.check()


def test_progress_serializes():
    ex = WaypointExecutor()
    d = ex.load("patrol", ROUTE).to_dict()
    assert d["status"] == "in_progress" and d["target"] == ROUTE[0].to_dict()
    assert d["command_id"] == "patrol-wp0-1"
