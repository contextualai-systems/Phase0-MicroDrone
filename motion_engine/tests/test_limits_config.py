"""Motion's limits and Safety's config must be the same numbers.

INTEGRATION_README §4: "Safety and Motion receive the same configured limits."
`safety_layer/config.py` owns those numbers. `MotionLimits` still has
defaults (scripts and older tests build `MotionStub()` with no arguments), so
these tests make any drift between the two a CI failure instead of a silent
disagreement about where the geofence is.
"""

import sys
from dataclasses import fields
from pathlib import Path
from types import SimpleNamespace

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from motion_engine.motion_stubs import MotionLimits, MotionStub
from safety_layer import config
from state_machine import run_loop

#: MotionLimits field -> the safety_layer.config name it must equal.
SHARED = {
    "max_horizontal_speed_mps": "MAX_HORIZONTAL_SPEED_MPS",
    "max_vertical_speed_mps": "MAX_VERTICAL_SPEED_MPS",
    "max_yaw_rate_dps": "MAX_YAW_RATE_DPS",
    "max_altitude_m": "ALTITUDE_CEILING_M",
    "geofence_radius_m": "GEOFENCE_RADIUS_M",
    "arrival_tolerance_m": "POSITION_ARRIVAL_TOLERANCE_M",
    "yaw_tolerance_deg": "DOCK_YAW_TOLERANCE_DEG",
}


@pytest.mark.parametrize("field_name, config_name", sorted(SHARED.items()))
def test_default_limits_match_safety_config(field_name, config_name):
    assert getattr(MotionLimits(), field_name) == getattr(config, config_name), (
        f"MotionLimits.{field_name} default disagrees with "
        f"safety_layer.config.{config_name}; change both or neither"
    )


def test_defaults_equal_from_config():
    assert MotionLimits() == MotionLimits.from_config()


def test_every_shared_name_exists_in_config():
    missing = [name for name in SHARED.values() if not hasattr(config, name)]
    assert missing == []


def test_unshared_fields_are_only_the_documented_ones():
    """A new MotionLimits field must be mapped to config or listed here."""
    unshared = {f.name for f in fields(MotionLimits)} - set(SHARED)
    assert unshared == {"min_altitude_m", "emergency_descent_mps"}


def test_from_config_follows_a_changed_config(monkeypatch):
    monkeypatch.setattr(config, "GEOFENCE_RADIUS_M", 42.0)
    monkeypatch.setattr(config, "MAX_HORIZONTAL_SPEED_MPS", 3.0)
    lim = MotionLimits.from_config()
    assert lim.geofence_radius_m == 42.0
    assert lim.max_horizontal_speed_mps == 3.0


def test_from_config_accepts_any_source():
    src = SimpleNamespace(**{name: 1.0 for name in SHARED.values()}, EMERGENCY_DESCENT_MPS=0.5)
    lim = MotionLimits.from_config(src)
    assert all(getattr(lim, f) == 1.0 for f in SHARED)
    assert lim.emergency_descent_mps == 0.5
    assert lim.min_altitude_m == MotionLimits().min_altitude_m


def test_run_loop_builds_motion_from_safety_config(monkeypatch):
    """The coordinator must hand Motion the config limits, not the defaults."""
    monkeypatch.setattr(config, "GEOFENCE_RADIUS_M", 149.0)  # differs from the default
    built = []

    def factory(*args, **kwargs):
        built.append(MotionStub(*args, **kwargs))
        return built[-1]

    monkeypatch.setattr(run_loop, "MotionStub", factory)
    run_loop.run_world(run_loop.TICK_MS)
    assert len(built) == 1
    assert built[0].limits == MotionLimits.from_config()
    assert built[0].limits.geofence_radius_m == 149.0
