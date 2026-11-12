"""The DroneState contract (motion_engine/README.md, "DroneState contract").

Checks the fields START_HERE requires (position, velocity, orientation,
frame, timestamp, flight mode, validity), that `to_dict()` only *added*
keys to the old `VehicleFeedback` layout, `validate()`, and that the README
example payload round-trips through JSON and validates.
"""

import dataclasses
import json
import re
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]

import pytest

from motion_engine.contracts import (
    FRAME_LOCAL_ENU,
    Command,
    CommandStatus,
    DroneState,
    FlightMode,
    MotionCommandType as K,
    Orientation,
    Pose,
    Reason,
    SimulationClock,
    Velocity,
    VehicleFeedback,
)
from motion_engine.motion_stubs import MotionStub

DT_NS = 100_000_000

#: Every key `VehicleFeedback.to_dict()` had before DroneState. Consumers
#: (run_loop snapshot, trace checker, docking logs) rely on these.
LEGACY_KEYS = {
    "schema_version", "timestamp_ns", "valid", "reason", "source_id", "synthetic",
    "pose", "armed", "airborne", "grounded", "emergency_descent", "landing_settled",
    "active_command_id", "active_kind", "target", "status", "arrived",
    "distance_to_target_m", "horizontal_error_m", "vertical_error_m", "yaw_error_deg",
}
NEW_KEYS = {"frame", "velocity", "orientation", "flight_mode", "target_velocity", "status_reason"}


def _readme_example() -> dict:
    text = (_REPO_ROOT / "motion_engine" / "README.md").read_text(encoding="utf-8")
    section = text[text.index("### DroneState contract"):]
    block = re.search(r"```json\n(.*?)\n```", section, re.S)
    assert block, "README DroneState section has no JSON example"
    return json.loads(block.group(1))


def _flying_state() -> DroneState:
    m = MotionStub(clock=SimulationClock())
    def c(cid, kind, **kw):
        return Command(cid, kind, "HOVERING", 0, kw.pop("exp", None), "t", "t", **kw)
    m.submit(c("a", K.ARM), 0)
    m.submit(c("t", K.TAKEOFF, target=Pose(0, 0, 5, 0)), 0)
    now = 0
    for _ in range(30):
        m.step(0.1, now_ns=now)
        now += DT_NS
    return m.feedback(now)


def _valid_state(**overrides) -> DroneState:
    return dataclasses.replace(DroneState.from_dict(_readme_example()), **overrides)


# --------------------------------------------------------------------------
# type and fields
# --------------------------------------------------------------------------


def test_stub_state_has_start_here_fields():
    s = _flying_state()
    assert s.timestamp_ns == 3_000_000_000
    assert s.position == s.pose == Pose(0, 0, 5, 0)
    assert s.velocity == Velocity(0.0, 0.0, 0.0, 0.0)   # takeoff finished: still
    assert s.orientation == Orientation(0.0, 0.0, 0.0)
    assert s.frame == FRAME_LOCAL_ENU
    assert s.flight_mode == FlightMode.TAKEOFF
    assert s.valid is True
    assert s.validate() is None


def test_velocity_is_estimated_from_motion():
    m = MotionStub(clock=SimulationClock())
    m.arm()
    m.takeoff(5.0)
    m.step(0.1, now_ns=0)
    s = m.feedback(DT_NS)
    assert s.velocity.vz == pytest.approx(2.0)          # climbing at the 2 m/s cap
    assert s.velocity.vx == 0.0 and s.velocity.vy == 0.0


@pytest.mark.parametrize("setup, mode", [
    (lambda m: None, FlightMode.DISARMED),
    (lambda m: m.arm(), FlightMode.ON_GROUND),
    (lambda m: (m.arm(), m.takeoff(5)), FlightMode.TAKEOFF),
    (lambda m: (m.arm(), m.takeoff(5), m.move_to(1, 1, 5)), FlightMode.POSITION),
    (lambda m: (m.arm(), m.takeoff(5), m.hover()), FlightMode.HOLD),
    (lambda m: (m.arm(), m.takeoff(5), m.land()), FlightMode.LAND),
    (lambda m: (m.arm(), m.takeoff(5), m.step(0.5), m.emergency_land()), FlightMode.EMERGENCY_LAND),
])
def test_flight_mode_follows_active_command(setup, mode):
    m = MotionStub()
    setup(m)
    s = m.feedback(0)
    assert s.flight_mode == mode
    assert s.validate() is None


def test_to_dict_keeps_legacy_keys_and_only_adds():
    d = _flying_state().to_dict()
    assert set(d) == LEGACY_KEYS | NEW_KEYS
    assert d["frame"] == "local_enu"
    assert d["flight_mode"] == "takeoff"
    assert d["orientation"] == {"roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0}
    assert d["velocity"] == {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw_rate_deg_s": 0.0}
    json.dumps(d)  # plain values only


def test_legacy_alias_and_keyword_construction_still_work():
    assert VehicleFeedback is DroneState
    assert isinstance(MotionStub().feedback(0), VehicleFeedback)
    # docking/run_docking.py builds feedback without the new fields.
    s = VehicleFeedback(
        timestamp_ns=0, pose=Pose(1, 2, 0, 45), armed=True, airborne=False, grounded=True,
        emergency_descent=False, landing_settled=True, active_command_id="x",
        active_kind="DESCEND_TO_DOCK", target=Pose(1, 2, 0, 45), status=CommandStatus.LANDED,
        arrived=True, distance_to_target_m=None, horizontal_error_m=None,
        vertical_error_m=None, yaw_error_deg=None,
    )
    assert s.orientation == Orientation(0.0, 0.0, 45)
    assert s.to_dict()["velocity"] is None
    # ...but a contract consumer is told what is missing, not handed a default.
    assert s.validate() == Reason.MISSING_FIELD


# --------------------------------------------------------------------------
# validate()
# --------------------------------------------------------------------------


@pytest.mark.parametrize("overrides, reason", [
    ({"schema_version": 2}, Reason.UNSUPPORTED_SCHEMA),
    ({"timestamp_ns": -1}, Reason.INVALID_TIMESTAMP),
    ({"timestamp_ns": 1.5}, Reason.INVALID_TIMESTAMP),
    ({"timestamp_ns": True}, Reason.INVALID_TIMESTAMP),
    ({"frame": "local_ned"}, Reason.UNSUPPORTED_FRAME),
    ({"velocity": None}, Reason.MISSING_FIELD),
    ({"flight_mode": None}, Reason.MISSING_FIELD),
    ({"pose": Pose(float("nan"), 0, 5, 1.0)}, Reason.INVALID_STATE),
    ({"velocity": Velocity(0, float("inf"), 0)}, Reason.INVALID_STATE),
    ({"orientation": Orientation(float("nan"), 0, 1.0)}, Reason.INVALID_STATE),
    ({"target_velocity": Velocity(float("nan"), 0, 0)}, Reason.INVALID_STATE),
    ({"distance_to_target_m": float("inf")}, Reason.INVALID_STATE),
    ({"flight_mode": "cruise"}, Reason.INVALID_STATE),
    ({"armed": "true"}, Reason.INVALID_STATE),
    ({"grounded": True}, Reason.INCONSISTENT_STATE),                 # airborne and grounded
    ({"airborne": False}, Reason.INCONSISTENT_STATE),                # neither
    ({"landing_settled": True}, Reason.INCONSISTENT_STATE),          # settled in the air
    ({"armed": False}, Reason.INCONSISTENT_STATE),                   # airborne but disarmed
    ({"flight_mode": FlightMode.DISARMED}, Reason.INCONSISTENT_STATE),
    ({"emergency_descent": True}, Reason.INCONSISTENT_STATE),        # mode not emergency
    ({"orientation": Orientation(0, 0, 90.0)}, Reason.INCONSISTENT_STATE),
    ({"valid": False}, Reason.MISSING_FIELD),                        # invalid with no reason
])
def test_validate_rejects(overrides, reason):
    assert _valid_state().validate() is None
    assert _valid_state(**overrides).validate() == reason


def test_validate_accepts_invalid_state_with_reason_and_never_raises():
    assert _valid_state(valid=False, reason="stale_vehicle").validate() is None
    assert _valid_state(pose="not a pose").validate() == Reason.INVALID_STATE


# --------------------------------------------------------------------------
# README example payload
# --------------------------------------------------------------------------


def test_readme_example_round_trips_through_json():
    payload = _readme_example()
    state = DroneState.from_dict(payload)
    assert state.validate() is None
    assert state.to_dict() == payload
    again = DroneState.from_dict(json.loads(json.dumps(state.to_dict())))
    assert again == state
    assert set(payload) == LEGACY_KEYS | NEW_KEYS


def test_readme_example_matches_what_the_stub_publishes():
    m = MotionStub(clock=SimulationClock())
    now = 0
    def c(cid, kind, exp=None, **kw):
        return Command(cid, kind, "TRACKING", now, exp, "t", "t", **kw)
    m.submit(c("a", K.ARM), now)
    m.submit(c("t", K.TAKEOFF, target=Pose(0, 0, 5, 0)), now)
    for _ in range(30):
        m.step(0.1, now_ns=now)
        now += DT_NS
    v = Velocity(2.0, 1.0, 0.0, yaw_rate_deg_s=10.0)
    assert m.submit(c("vel-1", K.SET_VELOCITY, exp=now + 200_000_000, velocity=v), now).accepted
    m.step(0.1, now_ns=now)
    now += DT_NS
    published = m.feedback(now).to_dict()
    expected = _readme_example()
    assert set(published) == set(expected)
    for key, want in expected.items():
        got = published[key]
        if isinstance(want, dict):
            for k, w in want.items():
                assert got[k] == pytest.approx(w), (key, k)
        else:
            assert got == want, key
