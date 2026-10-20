"""Motion, as driven by the state machine (state_machine/run_loop.py).

`run_loop.run_world()` turns each mission state into a Motion command
(`wanted_command()`), renews it every tick, and steps `MotionStub`. The state
machine's own tests check the states and the command order. These tests check
Motion's side of that: for every demo in `state_machine/demos.py`, whatever
the state machine sends must be accepted, the whole flight must pass the
trace checker, and the vehicle must end up where the mission says it is.

If the state machine changes how it drives Motion and Motion stops agreeing,
this file is where it shows up.
"""

import sys
from dataclasses import replace
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

import state_machine.run_loop as run_loop
from state_machine.demos import DEMOS
from state_machine.state_machine import State
from motion_engine.contracts import (
    Command,
    CommandStatus,
    MemoryLogSink,
    MotionCommandType as K,
    Pose,
    SimulationClock,
)
from motion_engine.motion_stubs import MotionStub
from motion_engine.trace_checker import TraceRecorder, check_trace

PAD = Pose(0.0, 0.0, 0.0, 0.0)
TAKEOFF_Z = 5.0  # run_loop.launch_drone() climbs to 5 m
DT = run_loop.TICK_MS / 1000


class RecordedMotion:
    """Stands in for the `MotionStub` that run_world() builds, so every tick,
    every command and every log line is recorded for checking."""

    def __init__(self, clock):
        self.log = MemoryLogSink()
        self.stub = MotionStub(logger=self.log, clock=clock)
        self.rec = TraceRecorder(self.stub)
        self.rec.snapshot(clock.timestamp_ns)
        self.sent = []       # every Command submitted, renewals included
        self.per_tick = []   # feedback after each step

    def submit(self, command, now_ns):
        self.sent.append(command)
        return self.rec.submit(command, now_ns)

    def step(self, dt, now_ns=None):
        self.rec.step(dt, now_ns)
        self.per_tick.append(self.stub.feedback(now_ns))
        return self.stub.pose

    def feedback(self, now_ns=None):
        return self.stub.feedback(now_ns)


@pytest.fixture
def run_demo(monkeypatch):
    def run(name):
        made = []

        def factory(*args, clock=None, **kwargs):
            made.append(RecordedMotion(clock))
            return made[-1]

        monkeypatch.setattr(run_loop, "MotionStub", factory)
        scenario, duration_ms = DEMOS[name]
        result = run_loop.run_world(duration_ms, scenario=scenario)
        assert len(made) == 1
        return result, made[0]

    return run


def tick_of(time_ms):
    return time_ms // run_loop.TICK_MS


@pytest.mark.parametrize("name", sorted(DEMOS))
def test_motion_accepts_everything_and_the_flight_is_clean(name, run_demo):
    result, motion = run_demo(name)

    # Every command the state machine sends (renewals included) is accepted
    acks = [ack for sample in motion.rec.samples for ack in sample.acks]
    assert acks and all(ack["accepted"] for ack in acks), [a for a in acks if not a["accepted"]]

    # The whole flight obeys every Motion rule (speeds, fence, latch, expiry, ...)
    report = check_trace(motion.rec.samples, limits=motion.stub.limits, home=motion.stub.home)
    assert report.ok, report.format_text(max_violations=20)

    # Motion never had to refuse, ignore or clamp anything
    flagged = [(r.event, r.decision) for r in motion.log.records
               if r.decision and r.decision not in ("synthetic_only", "auto_disarm_on_ground")]
    assert flagged == []

    # Commands are stamped with a real mission state, matching the transition table
    states = {s.value for s in State}
    assert {c.mission_state for c in motion.sent} <= states


@pytest.mark.parametrize("name", ["full_cycle", "docking_retry"])
def test_docking_demos_land_exactly_on_the_pad_before_docked(name, run_demo):
    result, motion = run_demo(name)
    docked_ms = next(t for t, _, new, _ in result["transitions"] if new == State.DOCKED)

    # Motion confirmed the touchdown before the state machine said DOCKED
    before = motion.per_tick[tick_of(docked_ms) - 1]
    assert before.landing_settled and before.grounded
    assert before.status == CommandStatus.LANDED

    # And it is still settled, disarmed, on the pad at the end
    vehicle = result["vehicle"]
    assert vehicle.pose.as_tuple()[:3] == PAD.as_tuple()[:3]
    assert vehicle.landing_settled and not vehicle.armed
    assert vehicle.status == CommandStatus.LANDED


def test_emergency_demo_descends_in_place_and_disarms(run_demo):
    result, motion = run_demo("emergency")
    emergency_ms = next(t for t, _, new, _ in result["transitions"] if new == State.EMERGENCY_LAND)
    start = motion.per_tick[tick_of(emergency_ms) - 1].pose

    vehicle = result["vehicle"]
    assert (vehicle.pose.x, vehicle.pose.y) == (start.x, start.y)   # straight down
    assert vehicle.grounded and not vehicle.armed and not vehicle.emergency_descent
    # Sent once, never renewed
    assert sum(c.kind == K.EMERGENCY_LAND for c in motion.sent) == 1


def test_blocked_dock_holds_still_at_takeoff_altitude(run_demo):
    result, motion = run_demo("blocked_dock")
    init_ms = next(t for t, _, new, _ in result["transitions"] if new == State.DOCKING_INIT)
    held = [fb.pose for fb in motion.per_tick[tick_of(init_ms):]]
    assert held and all(p == held[0] for p in held)
    assert held[0].z == pytest.approx(TAKEOFF_Z)
    assert result["vehicle"].armed and result["vehicle"].airborne


# --------------------------------------------------------------------------
# A settled landing stays landed when the state machine stops renewing it
# --------------------------------------------------------------------------


def _land_on_pad_then_stop_renewing():
    clock = SimulationClock()
    motion = MotionStub(clock=clock)
    rec = TraceRecorder(motion)
    rec.snapshot(clock.timestamp_ns)

    def cmd(cid, kind, target=None, lifecycle=False):
        now = clock.timestamp_ns
        return Command(cid, kind, "DOCKING_APPROACH", now,
                       None if lifecycle else now + run_loop.VALIDITY_NS,
                       "test", "alignment", target=target)

    def tick(*cmds):
        for c in cmds:
            assert rec.submit(c, clock.timestamp_ns).accepted
        rec.step(DT, clock.timestamp_ns)
        clock.advance(DT)
        return motion.feedback(clock.timestamp_ns)

    tick(cmd("arm", K.ARM, lifecycle=True), cmd("takeoff", K.TAKEOFF, Pose(0, 0, 2.0, 0), lifecycle=True))
    while motion.pose.z != 2.0:
        tick()
    descend = cmd("descend", K.DESCEND_TO_DOCK, PAD)
    fb = tick(descend)
    while not fb.landing_settled:
        fb = tick(replace(descend, issued_ns=clock.timestamp_ns,
                          expires_ns=clock.timestamp_ns + run_loop.VALIDITY_NS))
    return motion, rec, clock, tick


def test_settled_landing_survives_its_command_expiring():
    """DOCKED sends nothing until it sees grounded + armed, and Docking should
    confirm touchdown from landing_settled. If the landing command expires on
    the ground, Motion must keep reporting the landing, not swap it for an
    expired hold."""
    motion, rec, clock, tick = _land_on_pad_then_stop_renewing()
    for _ in range(10):  # well past the 0.2 s validity, no renewals
        fb = tick()
        assert fb.status == CommandStatus.LANDED
        assert fb.landing_settled and fb.grounded and fb.armed
    assert motion.pose.as_tuple()[:3] == PAD.as_tuple()[:3]

    disarm = Command("disarm", K.DISARM, "DOCKED", clock.timestamp_ns, None, "test", "alignment")
    fb = tick(disarm)
    assert not fb.armed and fb.landing_settled
    report = check_trace(rec.samples, limits=motion.limits, home=motion.home)
    assert report.ok, report.format_text()


def test_off_pad_touchdown_is_not_settled_and_holds_when_it_expires():
    """Horizontal and vertical speed are capped separately, so a descent that
    starts off the pad can reach z = 0 while still sliding toward it. That is
    not a finished landing: it must not read as landed, and if the command
    stops being renewed the drone must hold where it is, not keep sliding."""
    clock = SimulationClock()
    motion = MotionStub(clock=clock)
    rec = TraceRecorder(motion)
    rec.snapshot(clock.timestamp_ns)

    def tick(*cmds):
        for c in cmds:
            assert rec.submit(c, clock.timestamp_ns).accepted
        rec.step(DT, clock.timestamp_ns)
        clock.advance(DT)
        return motion.feedback(clock.timestamp_ns)

    def cmd(cid, kind, target=None, lifecycle=False):
        now = clock.timestamp_ns
        return Command(cid, kind, "DOCKING_APPROACH", now,
                       None if lifecycle else now + run_loop.VALIDITY_NS,
                       "test", "alignment", target=target)

    tick(cmd("arm", K.ARM, lifecycle=True), cmd("takeoff", K.TAKEOFF, Pose(0, 0, 2.0, 0), lifecycle=True))
    while motion.pose.z != 2.0:
        tick()
    move = cmd("move", K.MOVE_TO, Pose(20.0, 0.0, 2.0, 0.0))
    fb = tick(move)
    while motion.pose.x != 20.0:
        fb = tick(replace(move, issued_ns=clock.timestamp_ns,
                          expires_ns=clock.timestamp_ns + run_loop.VALIDITY_NS))
    descend = cmd("descend", K.DESCEND_TO_DOCK, PAD)
    fb = tick(descend)
    while motion.pose.z != 0.0:
        fb = tick(replace(descend, issued_ns=clock.timestamp_ns,
                          expires_ns=clock.timestamp_ns + run_loop.VALIDITY_NS))
    assert motion.pose.x > 1.0                       # on the ground, still short of the pad
    assert not fb.landing_settled and fb.status != CommandStatus.LANDED

    tick(); fb = tick(); fb = tick()                 # stop renewing: it expires
    assert fb.status == CommandStatus.EXPIRED_HOLD
    stopped = motion.pose
    for _ in range(10):
        fb = tick()
        assert motion.pose == stopped                # no sliding on an expired target
    assert not fb.landing_settled
    report = check_trace(rec.samples, limits=motion.limits, home=motion.home)
    assert report.ok, report.format_text()
