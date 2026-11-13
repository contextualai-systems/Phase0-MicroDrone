"""Motion's logging: the shared UniversalLog adapter and integrated runs.

`UniversalLogSink` adapts Motion's `LogSink` calls to UniversalLog's
`{"State", "Details"}` records. These tests check that:

* every record Motion would put in `MemoryLogSink` reaches UniversalLog with
  the same content (nothing dropped or reshaped),
* the real shared class writes one valid-JSON line per record, under the
  coordinator's mission state, into the same run file CV uses,
* mission-state enums, missing state and non-JSON values are handled
  instead of raising inside the mission loop.

Passing this logger into the integrated loop (`state_machine/run_loop.py`)
is the State Machine team's change; its tests travel with that request.
"""

import json
import logging
import math
from importlib.util import module_from_spec, spec_from_file_location

import pytest

from motion_engine.contracts import (
    Command,
    MemoryLogSink,
    MotionCommandType as K,
    Pose,
    SimulationClock,
)
from motion_engine.motion_stubs import MotionStub
from motion_engine.tests.motion_rig import DT, REPO_ROOT, VALIDITY_NS
from motion_engine.universal_logging import UniversalLogSink, log_entry
from state_machine.state_machine import State


def _drive(logger, state=None):
    """Arm, take off, fly, get one rejection, and land, on the sim clock."""
    clock = SimulationClock()
    motion = MotionStub(logger=logger, clock=clock)
    if state is not None:
        motion.set_log_context(state=state, truth={"bird_x": 4.0})
    n = 0

    def submit(kind, target=None, expires=True):
        nonlocal n
        n += 1
        now = clock.timestamp_ns
        return motion.submit(Command(
            command_id=f"c{n}", kind=kind, mission_state="TRACKING", target=target,
            issued_ns=now, expires_ns=now + VALIDITY_NS if expires else None,
            source="test", reason="test",
        ), now)

    def ticks(count):
        for _ in range(count):
            motion.step(DT, now_ns=clock.timestamp_ns)
            clock.advance(DT)

    assert submit(K.ARM, expires=False).accepted
    assert submit(K.TAKEOFF, Pose(0, 0, 3.0, 0), expires=False).accepted
    ticks(30)
    assert not submit(K.MOVE_TO, Pose(0, 0, 500.0, 0)).accepted  # above ceiling
    assert submit(K.MOVE_TO, Pose(2.0, 1.0, 3.0, 0)).accepted
    ticks(1)
    assert submit(K.LAND, expires=False).accepted
    ticks(40)
    return motion


@pytest.fixture
def universal_log_file(tmp_path):
    """Point the real shared logger at a temp file (pattern from the CV tests)."""
    logger = logging.getLogger("universal_log")
    saved = (logger.handlers[:], logger.level, logger.propagate,
             getattr(logger, "_universal_console_handler", None))
    logger.handlers = []
    logger._universal_console_handler = None
    path = tmp_path / "run.log"
    handler = logging.FileHandler(path, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(levelname)s %(message)s"))
    logger.addHandler(handler)

    def read_lines():
        handler.flush()
        return path.read_text(encoding="utf-8").splitlines()

    try:
        yield read_lines
    finally:
        handler.close()
        logger.handlers, level, logger.propagate, logger._universal_console_handler = saved
        logger.setLevel(level)


def _parse(line):
    """'LEVEL [timestamp] STATE: {json}' -> (level, state, details)."""
    level, rest = line.split(" ", 1)
    head, details = rest.split(": ", 1)
    return level, head.rsplit(" ", 1)[1], json.loads(details)


def test_universal_records_match_memory_sink_records():
    memory = MemoryLogSink()
    _drive(memory, state="TRACKING")
    written = []
    _drive(UniversalLogSink(written.append), state="TRACKING")

    assert len(written) == len(memory.records) > 0
    for record, entry in zip(written, memory.records):
        expected = entry.to_dict()
        assert record["State"] == expected.pop("state")
        assert record["Details"] == expected
        json.dumps(record, allow_nan=False)  # what UniversalLog will serialize
    events = [r["Details"]["event"] for r in written]
    assert {"arm", "takeoff", "move_to", "land"} <= set(events)
    assert any(str(r["Details"]["decision"]).startswith("rejected_") for r in written)


def test_real_universal_log_writes_one_json_line_per_record(universal_log_file):
    memory = MemoryLogSink()
    _drive(memory, state="TRACKING")
    _drive(UniversalLogSink(), state="TRACKING")

    lines = universal_log_file()
    assert len(lines) == len(memory.records)
    for line, entry in zip(lines, memory.records):
        level, state, details = _parse(line)
        assert (level, state) == ("INFO", "TRACKING")
        assert details["module"] == "MOTION"
        assert details["event"] == entry.event
        assert details["decision"] == entry.decision
        assert details["synthetic_truth"] == {"bird_x": 4.0}
        assert details["t"] == pytest.approx(entry.t)


def test_state_enum_is_written_as_its_value(universal_log_file):
    _drive(UniversalLogSink(), state=State.DOCKING_APPROACH)
    states = {_parse(line)[1] for line in universal_log_file()}
    assert states == {"DOCKING_APPROACH"}


def test_missing_state_defaults_to_idle_like_cv():
    written = []
    _drive(UniversalLogSink(written.append))
    assert {r["State"] for r in written} == {"IDLE"}


def test_unknown_state_is_kept_and_flagged_by_universal_log(universal_log_file):
    UniversalLogSink().log("MOTION", "hover", state="NOT_A_STATE", t=0.0)
    level, state, _ = _parse(universal_log_file()[0])
    assert (level, state) == ("ERROR", "NOT_A_STATE")


def test_non_json_values_do_not_break_the_logger(universal_log_file):
    class Opaque:
        def __str__(self):
            return "opaque"

    UniversalLogSink().log(
        K.HOVER, "hover", state="HOVERING", t=float("nan"),
        fields={"pose": Pose(1.0, 2.0, 3.0, 0.0), "kind": K.LAND,
                "err": math.inf, "ids": ("a", "b"), "obj": Opaque()},
    )
    _, _, details = _parse(universal_log_file()[0])
    assert details["module"] == "HOVER"
    assert details["t"] == "nan"
    assert details["fields"]["kind"] == "LAND"
    assert details["fields"]["err"] == "inf"
    assert details["fields"]["ids"] == ["a", "b"]
    assert details["fields"]["obj"] == "opaque"
    assert details["fields"]["pose"]["z"] == 3.0


def test_motion_and_cv_share_one_run_file(universal_log_file):
    path = REPO_ROOT / "cv" / "universal_logging.py"
    spec = spec_from_file_location("_test_cv_universal_logging", path)
    cv_logging = module_from_spec(spec)
    spec.loader.exec_module(cv_logging)

    cv_logging.log_entry({"State": "TRACKING", "Details": {"module": "CV", "event": "observation"}})
    log_entry({"State": "TRACKING", "Details": {"module": "MOTION", "event": "move_to"}})
    modules = [_parse(line)[2]["module"] for line in universal_log_file()]
    assert modules == ["CV", "MOTION"]
