"""Shared fixtures for the Motion Engine tests.

Puts the repo root on `sys.path` once (so test modules can import
`motion_engine`, `safety_layer`, `state_machine` directly, even when CI runs
a single file by path), and provides the coordinator `Rig` from
`motion_rig.py` as fixtures.
"""

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest

from motion_engine.tests.motion_rig import Rig


@pytest.fixture
def rig() -> Rig:
    """A fresh rig: disarmed on the pad, clock at 0."""
    return Rig()


@pytest.fixture
def airborne_rig() -> Rig:
    """A rig armed and holding exactly at 5 m above the pad."""
    r = Rig()
    r.launch(5.0)
    return r
