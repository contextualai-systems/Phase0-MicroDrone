"""Shared test helpers for the Motion Engine suite (not a test module).

`Rig` is a stand-in for the coordinator's tick loop around a recorded
`MotionStub`: one `tick()` = submit any commands, step Motion once by 0.1 s,
advance the simulation clock, return the feedback. Every tick lands in
`rig.rec.samples`, so `rig.check()` can run the whole flight through
`trace_checker.check_trace()`.

`conftest.py` puts the repo root on `sys.path`; test modules import from here
as `motion_engine.tests.motion_rig`.
"""

from dataclasses import replace
from typing import Callable, List, Optional

from motion_engine.contracts import (
    Command,
    MemoryLogSink,
    MotionCommandType as K,
    Pose,
    SimulationClock,
    VehicleFeedback,
    Velocity,
)
from motion_engine.motion_stubs import MotionLimits, MotionStub
from motion_engine.trace_checker import TraceRecorder, TraceReport, check_trace
from safety_layer.safety_policy import ACTIONS, SafetyAssessment

DT = 0.1
DT_NS = 100_000_000
S = 1_000_000_000
#: INTEGRATION_README §5: a command is valid for 0.2 s unless renewed.
VALIDITY_NS = 200_000_000
PAD = Pose(0.0, 0.0, 0.0, 0.0)
LIM = MotionLimits()


def assessment(blocked=(), emergency=False, abort=False) -> SafetyAssessment:
    """A Safety verdict with every action permitted except `blocked`."""
    return SafetyAssessment(
        permissions={a: a not in blocked for a in ACTIONS}, emergency=emergency, abort=abort,
    )


ALL_OK = assessment()


def records(log: MemoryLogSink, event: Optional[str] = None):
    return [r for r in log.records if event is None or r.event == event]


class Rig:
    """Coordinator tick loop around a recorded `MotionStub`."""

    def __init__(self, limits: Optional[MotionLimits] = None,
                 validity_ns: int = VALIDITY_NS) -> None:
        self.clock = SimulationClock()
        self.log = MemoryLogSink()
        self.motion = MotionStub(logger=self.log, clock=self.clock, limits=limits)
        self.rec = TraceRecorder(self.motion)
        self.rec.snapshot(self.now)
        self.validity_ns = validity_ns
        self._n = 0

    # -- building commands ------------------------------------------------

    @property
    def now(self) -> int:
        return self.clock.timestamp_ns

    def cmd(self, kind, target: Optional[Pose] = None, *, velocity: Optional[Velocity] = None,
            cid: Optional[str] = None, lifecycle: bool = False, issued: Optional[int] = None,
            mission_state: str = "TEST", source: str = "test", reason: str = "test",
            **kw) -> Command:
        """A command issued now, valid for one validity window.

        `lifecycle=True` leaves out the expiry (ARM/TAKEOFF/LAND/... style).
        """
        if cid is None:
            self._n += 1
            cid = f"c{self._n}"
        issued = self.now if issued is None else issued
        return Command(
            command_id=cid, kind=kind, mission_state=mission_state, issued_ns=issued,
            expires_ns=None if lifecycle else issued + self.validity_ns,
            source=source, reason=reason, target=target, velocity=velocity, **kw,
        )

    def renewed(self, c: Command) -> Command:
        """The same request, re-issued now (what the coordinator sends each tick)."""
        if c.expires_ns is None:
            return c
        return replace(c, issued_ns=self.now, expires_ns=self.now + self.validity_ns)

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
            max_ticks: int = 3000) -> List[VehicleFeedback]:
        """Send `c`, renewing it every tick, until `until(feedback)` holds.

        Returns the feedback of every tick flown (the last one satisfies `until`).
        """
        trace = []
        for _ in range(max_ticks):
            ack = self.submit(self.renewed(c))
            assert ack.accepted, ack
            trace.append(self.tick())
            if until(trace[-1]):
                return trace
        raise AssertionError(f"never reached the condition; pose={self.motion.pose}")

    def launch(self, altitude: float = 5.0) -> None:
        """Arm and take off, settling *exactly* at `altitude` (not just in tolerance)."""
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
