"""Keep the "MotionCommand contract" section of motion_engine/README.md honest.

The README's JSON examples are read straight from the README (each block is
tagged `<!-- motion-command-example: NAME -->`), built into real `Command` /
`CommandAck` objects, and replayed through a `MotionStub`. The reason table
must list exactly the `contracts.Reason` values. If the code changes and the
README does not, these tests fail.
"""

import json
import re
from dataclasses import fields, replace

import pytest

from motion_engine.contracts import (
    CONTRACT_KINDS,
    LIFECYCLE_KINDS,
    Command,
    CommandAck,
    MotionCommandType as K,
    Pose,
    Reason,
)
from motion_engine.motion_stubs import MotionLimits, MotionStub
from motion_engine.tests.motion_rig import MOTION_ENGINE_ROOT

README = MOTION_ENGINE_ROOT / "README.md"
TAG = re.compile(r"<!-- motion-command-example: (\w+) -->\s*```json\n(.*?)```", re.S)


def _section() -> str:
    text = README.read_text(encoding="utf-8")
    start = text.index("## MotionCommand contract")
    end = text.index("\n## ", start + 1)
    return text[start:end]


def _examples() -> dict:
    found = {name: json.loads(body) for name, body in TAG.findall(_section())}
    assert set(found) == {"accepted_command", "accepted_ack", "rejected_command", "rejected_ack"}
    return found


def _command(d: dict) -> Command:
    d = dict(d)
    d["kind"] = K(d["kind"])
    if d["target"] is not None:
        d["target"] = Pose(**d["target"])
    return Command(**d)


def _reason_values():
    return {v for k, v in vars(Reason).items() if not k.startswith("_") and isinstance(v, str)}


@pytest.mark.parametrize("name", ["accepted_command", "rejected_command"])
def test_command_examples_round_trip(name):
    d = _examples()[name]
    assert set(d) == {f.name for f in fields(Command)}
    cmd = _command(d)
    assert cmd.validate() is None  # well-formed; the rejection is a limits check
    assert cmd.to_dict() == d


@pytest.mark.parametrize("name", ["accepted_ack", "rejected_ack"])
def test_ack_examples_have_exactly_the_ack_fields(name):
    d = _examples()[name]
    assert set(d) == {f.name for f in fields(CommandAck)}
    assert CommandAck(**d).to_dict() == d
    assert d["reason"] in _reason_values()


def test_examples_replay_through_motion_stub():
    """Fly to the state the examples describe and check Motion's acks match."""
    ex = _examples()
    accepted = _command(ex["accepted_command"])
    rejected = _command(ex["rejected_command"])
    motion = MotionStub(limits=MotionLimits.from_config())
    launch_ns = 1_000_000_000
    for cid, kind, target in (("arm-1", K.ARM, None), ("takeoff-1", K.TAKEOFF, Pose(0, 0, 5.0, 0))):
        ack = motion.submit(Command(cid, kind, "HOVERING", launch_ns, None, "state_machine", "x",
                                    target=target), launch_ns)
        assert ack.accepted
    now = launch_ns
    while now < accepted.issued_ns:
        motion.step(0.1, now_ns=now)
        now += 100_000_000

    assert motion.submit(accepted, accepted.issued_ns).to_dict() == ex["accepted_ack"]

    # The README's renewal sentence: same command, one tick later.
    renewal = replace(accepted, issued_ns=rejected.issued_ns,
                      expires_ns=rejected.issued_ns + 200_000_000)
    renewed = motion.submit(renewal, rejected.issued_ns)
    assert (renewed.accepted, renewed.reason) == (True, Reason.RENEWED)

    assert motion.submit(rejected, rejected.issued_ns).to_dict() == ex["rejected_ack"]


def test_reason_strings_match_code():
    section = _section()
    codes = _reason_values()
    for code in codes:
        assert f"`{code}`" in section, f"Reason {code!r} is not documented"
    # Every code in the validation table's reason column is a real Reason.
    table = section[section.index("### Validation rules"):section.index("### Example payloads")]
    rows = [line for line in table.splitlines() if re.match(r"\| \d+ \|", line)]
    assert len(rows) >= 20
    for row in rows:
        reason_cell = row.split("|")[3]
        for code in re.findall(r"`([^`]+)`", reason_cell):
            assert code in codes, f"README lists unknown reason {code!r}"


def test_kind_lists_match_code():
    section = _section()
    for kind in CONTRACT_KINDS:
        assert f"`{kind.value}`" in section
    lifecycle = "`ARM`, `DISARM`, `TAKEOFF`, `LAND`, `EMERGENCY_LAND`"
    assert lifecycle in section
    assert {K(k) for k in re.findall(r"`(\w+)`", lifecycle)} == set(LIFECYCLE_KINDS)
