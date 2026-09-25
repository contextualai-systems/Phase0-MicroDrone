import logging

import pytest

from state_machine.battery_check import (
    LOW_BATTERY_REASON,
    check_battery,
    is_battery_low,
    synthetic_battery,
)
from state_machine.state_machine import Battery, Bird, Observation, State


def obs(percent):
    return Observation(bird=Bird(detected=True), battery=Battery(percentage=percent))


@pytest.mark.parametrize("state", [State.IDLE, State.TRACKING, State.HOVERING])
def test_low_battery_goes_to_docking_init(state):
    assert check_battery(state, obs(15.0)) == (State.DOCKING_INIT, LOW_BATTERY_REASON)


def test_battery_at_threshold_is_not_low():
    assert check_battery(State.TRACKING, obs(20.0)) == (State.TRACKING, None)


def test_battery_just_below_threshold_is_low():
    assert check_battery(State.TRACKING, obs(19.9)) == (State.DOCKING_INIT, LOW_BATTERY_REASON)


def test_healthy_battery_keeps_state():
    assert check_battery(State.TRACKING, obs(80.0)) == (State.TRACKING, None)


def test_missing_battery_keeps_state():
    observation = Observation(bird=Bird(detected=True), battery=None)
    assert check_battery(State.TRACKING, observation) == (State.TRACKING, None)
    assert not is_battery_low(None)


@pytest.mark.parametrize(
    "state",
    [State.DOCKING_INIT, State.DOCKING_APPROACH, State.DOCKED, State.ABORT, State.EMERGENCY_LAND],
)
def test_low_battery_does_not_interrupt_docking_or_landing(state):
    assert check_battery(state, obs(10.0)) == (state, None)


def test_low_battery_logs_warning(caplog):
    with caplog.at_level(logging.WARNING, logger="state_machine.battery_check"):
        check_battery(State.TRACKING, obs(12.0))
    assert "Battery low: 12.0%" in caplog.text


def test_healthy_battery_does_not_warn(caplog):
    with caplog.at_level(logging.WARNING, logger="state_machine.battery_check"):
        check_battery(State.TRACKING, obs(50.0))
    assert caplog.text == ""


def test_synthetic_battery_drains_and_triggers_docking():
    state = State.TRACKING
    trigger_percent = None
    for battery in synthetic_battery(start=40.0, drain_per_tick=5.0):
        state, reason = check_battery(state, Observation(battery=battery))
        if state == State.DOCKING_INIT:
            trigger_percent = battery.percentage
            break
    assert trigger_percent == 15.0
    assert reason == LOW_BATTERY_REASON


def test_synthetic_battery_stops_at_zero_and_sets_low_flag():
    readings = list(synthetic_battery(start=30.0, drain_per_tick=10.0))
    assert [b.percentage for b in readings] == [30.0, 20.0, 10.0, 0.0]
    assert [b.low for b in readings] == [False, False, True, True]
