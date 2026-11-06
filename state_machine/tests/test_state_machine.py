import pytest

from safety_layer.safety_policy import assess
from state_machine.demos import DEMOS
from state_machine.run_loop import run_world
from state_machine.state_machine import step, State

AIRBORNE_STATES = [State.TRACKING, State.HOVERING, State.DOCKING_INIT, State.DOCKING_APPROACH, State.ABORT]

def safety_with(blocked=(), **changes):
    # Safety's answer with every action permitted except the blocked ones,
    # e.g. safety_with(blocked=["approach", "descent"], reasons=["cart_moving"])
    # Field names are from Safety's SafetyAssessment (PR #61, INTEGRATION_README.md section 4)
    safety = {
        "permissions": {action: action not in blocked
                        for action in ["launch", "tracking", "approach", "descent", "hold"]},
        "emergency": False,
        "abort": False,
        "battery_low": False,
        "battery_critical": False,
        "battery_recovered": True,
        "reasons": [],
    }
    safety.update(changes)
    return safety

# What Safety blocks for a moving cart or people nearby: everything except hold (INTEGRATION_README.md section 6)
HAZARD_BLOCKED = ["launch", "tracking", "approach", "descent"]

def docking_with(**changes):
    # Default docking flags with some changed, e.g. docking_with(should_dock=True, phase="descend")
    # phase is one of: idle, approach, search, align, descend, complete, abort
    docking = {
        "should_dock": False,
        "reason": None,
        "phase": "idle",
        "landed_on_pad": False,
    }
    docking.update(changes)
    return docking

NOW_NS = 10_000_000_000  # the observation's time in the unit tests (10 s)
PAD_DOCK = {"pad_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}, "alignment_valid": True, "valid": True}

def vehicle_with(**changes):
    # Motion's feedback for a drone resting disarmed on the pad, from the previous tick (0.1 s ago),
    # with some fields changed, e.g. vehicle_with(armed=True)
    vehicle = {
        "valid": True,
        "timestamp_ns": NOW_NS - 100_000_000,
        "armed": False,
        "airborne": False,
        "grounded": True,
        "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
    }
    vehicle.update(changes)
    return vehicle

def make_observation(bird = None, battery = None, safety = None, docking = None, vehicle = None):
    # Makes observations in the team's JSON format for testing purposes
    return {
        "timestamp_ns": NOW_NS,
        "bird": bird or {"detected": False},
        "battery": battery or {"percent": 100, "low": False},
        "dock": PAD_DOCK,
        "vehicle": vehicle or vehicle_with(),
        "safety": safety or safety_with(),
        "docking": docking or docking_with(),
    }

def test_idle_to_hovering():
    # A launch goes through HOVERING before TRACKING
    observation = make_observation(bird={"detected": True})
    new_state, reason = step(State.IDLE, observation)
    assert new_state == State.HOVERING
    assert reason == "Bird detected"

def test_tracking_to_hovering():
    observation = make_observation(bird={"detected": False})
    new_state, reason = step(State.TRACKING, observation)
    assert new_state == State.HOVERING
    assert reason == "Bird not detected"

def test_idle_to_detected_bird():
    observation = make_observation(bird={"type": "sparrow", "detected": True, "distance_m": 5})
    assert step(State.IDLE, observation) == (State.HOVERING, "Bird detected")

def test_lost_bird_to_hovering():
    observation = make_observation(bird={"detected": False})
    assert step(State.TRACKING, observation) == (State.HOVERING, "Bird not detected")

def test_missing_bird_data_to_hovering():
    # Only the state is checked: the reason depends on whether bird-lost or unsafe is checked first
    assert step(State.TRACKING, {})[0] == State.HOVERING

def test_idle_does_not_launch_when_unsafe():
    observation = make_observation(bird={"detected": True},
                                   safety=safety_with(blocked=HAZARD_BLOCKED, reasons=["cart_moving"]))
    assert step(State.IDLE, observation)[0] == State.IDLE

def test_idle_does_not_launch_without_safety_data():
    # A missing permission counts as not permitted
    observation = make_observation(bird={"detected": True})
    del observation["safety"]
    assert step(State.IDLE, observation)[0] == State.IDLE

def test_idle_with_should_dock_stays_idle():
    # IDLE is already on the dock, so a grounded drone ignores return requests
    observation = make_observation(docking=docking_with(should_dock=True, reason="no_bird"))
    assert step(State.IDLE, observation)[0] == State.IDLE

def test_tracking_should_dock_to_docking_init():
    observation = make_observation(bird={"detected": True}, docking=docking_with(should_dock=True, reason="low_battery"))
    assert step(State.TRACKING, observation)[0] == State.DOCKING_INIT

def test_hovering_should_dock_to_docking_init():
    observation = make_observation(docking=docking_with(should_dock=True))
    assert step(State.HOVERING, observation)[0] == State.DOCKING_INIT

def test_hovering_to_tracking():
    observation = make_observation(bird={"detected": True})
    assert step(State.HOVERING, observation) == (State.TRACKING, "Bird detected")

def test_docked_to_idle_when_battery_recovered():
    # Safety decides when the battery has recovered, not the state machine
    recovered = make_observation(battery={"percent": 100}, safety=safety_with(battery_recovered=True))
    assert step(State.DOCKED, recovered)[0] == State.IDLE

    still_low = make_observation(battery={"percent": 18},
                                 safety=safety_with(blocked=["launch", "tracking"], battery_low=True,
                                                    battery_recovered=False, reasons=["low_battery"]))
    assert step(State.DOCKED, still_low)[0] == State.DOCKED

def test_docked_waits_until_disarmed_and_grounded_at_pad():
    # A recovered battery is not enough: Motion's fresh feedback must also show the drone
    # disarmed and grounded at the pad (INTEGRATION_README.md section 7)
    recovered = safety_with(battery_recovered=True)
    not_ready = {
        "still armed": vehicle_with(armed=True),
        "not grounded": vehicle_with(grounded=False, airborne=True,
                                     pose={"x": 0.0, "y": 0.0, "z": 1.0, "yaw_deg": 0.0}),
        "off the pad": vehicle_with(pose={"x": 1.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}),
        "stale feedback": vehicle_with(timestamp_ns=NOW_NS - 1_000_000_000),
        "invalid feedback": vehicle_with(valid=False),
    }
    for name, vehicle in not_ready.items():
        observation = make_observation(safety=recovered, vehicle=vehicle)
        assert step(State.DOCKED, observation)[0] == State.DOCKED, name

    missing = make_observation(safety=recovered)
    del missing["vehicle"]
    assert step(State.DOCKED, missing)[0] == State.DOCKED

    # Within the 0.25 m pad tolerance counts as at the pad
    nearly_centered = vehicle_with(pose={"x": 0.2, "y": 0.0, "z": 0.0, "yaw_deg": 0.0})
    assert step(State.DOCKED, make_observation(safety=recovered, vehicle=nearly_centered))[0] == State.IDLE

def test_docking_init_waits_for_approach_permission():
    # Safety blocks the approach (e.g. cart moving), so the drone holds in DOCKING_INIT
    blocked = make_observation(docking=docking_with(should_dock=True),
                               safety=safety_with(blocked=HAZARD_BLOCKED, reasons=["cart_moving"]))
    assert step(State.DOCKING_INIT, blocked)[0] == State.DOCKING_INIT

    permitted = make_observation(docking=docking_with(should_dock=True))
    assert step(State.DOCKING_INIT, permitted)[0] == State.DOCKING_APPROACH

def test_approach_waits_for_landed_on_pad():
    # Docking is still working in these phases, so the state machine waits
    for phase in ["idle", "approach", "search", "align", "descend"]:
        observation = make_observation(docking=docking_with(should_dock=True, phase=phase))
        assert step(State.DOCKING_APPROACH, observation)[0] == State.DOCKING_APPROACH, phase

    landed = make_observation(docking=docking_with(phase="complete", landed_on_pad=True))
    assert step(State.DOCKING_APPROACH, landed)[0] == State.DOCKED

def test_approach_aborts_when_docking_fails():
    failed = make_observation(docking=docking_with(should_dock=True, phase="abort"))
    assert step(State.DOCKING_APPROACH, failed)[0] == State.ABORT

def test_approach_aborts_when_permission_lost():
    no_approach = make_observation(docking=docking_with(should_dock=True, phase="align"),
                                   safety=safety_with(blocked=HAZARD_BLOCKED, reasons=["people_nearby"]))
    assert step(State.DOCKING_APPROACH, no_approach)[0] == State.ABORT

    # Descent permission only matters once docking is descending
    no_descent = make_observation(docking=docking_with(should_dock=True, phase="descend"),
                                  safety=safety_with(blocked=["descent"], reasons=["invalid_alignment"]))
    assert step(State.DOCKING_APPROACH, no_descent)[0] == State.ABORT

    aligning = make_observation(docking=docking_with(should_dock=True, phase="align"),
                                safety=safety_with(blocked=["descent"], reasons=["invalid_alignment"]))
    assert step(State.DOCKING_APPROACH, aligning)[0] == State.DOCKING_APPROACH

def test_unsafe_goes_to_hovering_not_emergency_land():
    # Tracking forbidden without an emergency (e.g. camera failure) means hold, not land
    no_camera = make_observation(bird={"detected": True},
                                 safety=safety_with(blocked=["launch", "tracking"], reasons=["camera_failure"]))
    assert step(State.TRACKING, no_camera) == (State.HOVERING, "Tracking not permitted")

def test_abort_retries_docking():
    # ABORT happens because docking went wrong, so the hazard may still be there
    # recovering means trying docking again, not landing in place
    person_at_dock = make_observation(docking=docking_with(should_dock=True),
                                      safety=safety_with(blocked=HAZARD_BLOCKED, reasons=["people_nearby"]))
    assert step(State.ABORT, person_at_dock)[0] == State.DOCKING_INIT

def test_abort_stays_when_hold_not_permitted():
    no_hold = make_observation(docking=docking_with(should_dock=True), safety=safety_with(blocked=["hold"]))
    assert step(State.ABORT, no_hold)[0] == State.ABORT

def test_emergency_land_is_final():
    assert step(State.EMERGENCY_LAND, make_observation())[0] == State.EMERGENCY_LAND

# What each demo in demos.py should do from start to finish:
# - states:   every state the mission passes through after IDLE, in order
# - commands: every new command sent to Motion, in order (a renewed command counts once)
# - grounded / armed: where the drone is when the run ends
EXPECTED_DEMOS = {
    "full_cycle": {
        "states": [State.HOVERING, State.TRACKING, State.HOVERING, State.DOCKING_INIT,
                   State.DOCKING_APPROACH, State.DOCKED, State.IDLE],
        # Docking flies the approach and descent. Docking aligns with GOTO_DOCK_APPROACH too; the drone is
        # already over the pad, so the target is the same and it renews the approach command
        "commands": ["ARM", "TAKEOFF", "HOVER", "GOTO_DOCK_APPROACH", "DESCEND_TO_DOCK", "DISARM"],
        "grounded": True, "armed": False,
    },
    "docking_retry": {
        "states": [State.HOVERING, State.TRACKING, State.HOVERING, State.DOCKING_INIT, State.DOCKING_APPROACH,
                   State.ABORT, State.DOCKING_INIT, State.DOCKING_APPROACH, State.DOCKED, State.IDLE],
        # Each attempt: GOTO_DOCK_APPROACH, then HOVER while Docking searches for the pad alignment.
        # The first search times out (STOP in ABORT); the second finds it, aligns (GOTO_DOCK_APPROACH) and descends
        "commands": ["ARM", "TAKEOFF", "HOVER", "GOTO_DOCK_APPROACH", "HOVER", "STOP", "HOVER",
                     "GOTO_DOCK_APPROACH", "HOVER", "GOTO_DOCK_APPROACH", "DESCEND_TO_DOCK", "DISARM"],
        "grounded": True, "armed": False,
    },
    "emergency": {
        "states": [State.HOVERING, State.TRACKING, State.EMERGENCY_LAND],
        "commands": ["ARM", "TAKEOFF", "EMERGENCY_LAND"],
        "grounded": True, "armed": False,  # the emergency landing disarms on touchdown
    },
    "blocked_dock": {
        "states": [State.HOVERING, State.TRACKING, State.DOCKING_INIT],
        "commands": ["ARM", "TAKEOFF", "HOVER"],
        "grounded": False, "armed": True,  # still hovering, waiting for the dock to clear
    },
    "not_aligned": {
        "states": [State.HOVERING, State.TRACKING, State.HOVERING, State.DOCKING_INIT, State.DOCKING_APPROACH,
                   State.DOCKED, State.IDLE],
        # The second HOVER is Docking searching above the pad until the alignment is valid;
        # the second GOTO_DOCK_APPROACH is Docking aligning
        "commands": ["ARM", "TAKEOFF", "HOVER", "GOTO_DOCK_APPROACH", "HOVER", "GOTO_DOCK_APPROACH",
                     "DESCEND_TO_DOCK", "DISARM"],
        "grounded": True, "armed": False,
    },
    "cart_during_descent": {
        "states": [State.HOVERING, State.TRACKING, State.HOVERING, State.DOCKING_INIT, State.DOCKING_APPROACH,
                   State.ABORT, State.DOCKING_INIT, State.DOCKING_APPROACH, State.DOCKED, State.IDLE],
        # The second attempt starts with GOTO_DOCK_APPROACH again, to climb back from where the descent stopped.
        # Aligning renews that same GOTO_DOCK_APPROACH, because the drone is already over the pad
        "commands": ["ARM", "TAKEOFF", "HOVER", "GOTO_DOCK_APPROACH", "DESCEND_TO_DOCK", "STOP",
                     "HOVER", "GOTO_DOCK_APPROACH", "DESCEND_TO_DOCK", "DISARM"],
        "grounded": True, "armed": False,
    },
    "comm_loss": {
        "states": [State.HOVERING, State.TRACKING, State.EMERGENCY_LAND],
        "commands": ["ARM", "TAKEOFF", "EMERGENCY_LAND"],
        "grounded": True, "armed": False,  # the emergency landing disarms on touchdown
    },
}

@pytest.mark.parametrize("name", sorted(DEMOS))
def test_demo_end_to_end(name):
    scenario, duration_ms = DEMOS[name]
    expected = EXPECTED_DEMOS[name]
    result = run_world(duration_ms, scenario=scenario)

    states = [new for _, _, new, _ in result["transitions"]]
    assert states == expected["states"]

    # Motion accepted every command, and they were sent in this order
    assert all(accepted for _, _, accepted, _ in result["commands"]), result["commands"]
    assert [kind for _, kind, _, _ in result["commands"]] == expected["commands"]

    vehicle = result["vehicle"]
    assert vehicle.grounded == expected["grounded"]
    assert vehicle.armed == expected["armed"]

def test_descent_waits_until_aligned():
    # Docking hovers above the pad until the alignment becomes valid at 8000 ms,
    # aligns on that tick and starts the descent on the next one
    scenario, duration_ms = DEMOS["not_aligned"]
    commands = run_world(duration_ms, scenario=scenario)["commands"]
    descents = [time_ms for time_ms, kind, _, _ in commands if kind == "DESCEND_TO_DOCK"]
    assert descents == [8100]

def test_cart_moving_stops_descent_on_the_same_tick():
    # The cart starts moving at 7000 ms, during the descent; the descent is replaced by STOP in that
    # same tick, before Motion moves the drone (INTEGRATION_README.md section 11)
    scenario, duration_ms = DEMOS["cart_during_descent"]
    commands = run_world(duration_ms, scenario=scenario)["commands"]
    assert (7000, "STOP") in [(time_ms, kind) for time_ms, kind, _, _ in commands]

def test_return_start_time_survives_abort_and_retry(monkeypatch):
    # The loop gives Safety the time the return started (context return_started_ns), for the 60 s
    # return deadline. An abort and retry must not restart it, and it clears back in IDLE
    # (INTEGRATION_README.md sections 6 and 7)
    contexts = []
    def recording_assess(snapshot, now_ns):
        contexts.append(snapshot["context"]["return_started_ns"])
        return assess(snapshot, now_ns)
    monkeypatch.setattr("state_machine.run_loop.assess", recording_assess)

    scenario, duration_ms = DEMOS["docking_retry"]
    transitions = run_world(duration_ms, scenario=scenario)["transitions"]
    assert State.ABORT in [new for _, _, new, _ in transitions]  # the retry really happens

    # One start time, from the first DOCKING_INIT at 6000 ms, kept through the abort and retry
    assert {started for started in contexts if started is not None} == {6_000_000_000}
    assert contexts[-1] is None  # cleared once the drone is back in IDLE