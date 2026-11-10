import json
from pathlib import Path

import pytest

from safety_layer.safety_policy import assess
from state_machine import run_loop
from cv.synthetic_frames import validate_scenario
from state_machine.demos import BIRD_DEMO_INIT, BIRD_GONE, BIRD_SEEN, DEMOS
from state_machine.run_loop import build_temp_fixture, run_world
from state_machine.state_machine import step, State
from motion_engine.contracts import MemoryLogSink

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

def test_tracking_waits_for_the_launch_climb():
    # Tracking only starts once the climb has finished (INTEGRATION_README.md section 7)
    observation = make_observation(bird={"detected": True})
    observation["context"] = {"launch_phase": "climbing"}
    assert step(State.HOVERING, observation) == (State.HOVERING, None)

    observation["context"] = {"launch_phase": "complete"}
    assert step(State.HOVERING, observation) == (State.TRACKING, "Bird detected")

    # An ordinary hold (not a launch) can go back to tracking straight away
    observation["context"] = {"launch_phase": "none"}
    assert step(State.HOVERING, observation) == (State.TRACKING, "Bird detected")

def test_return_request_during_the_climb_still_docks():
    # A return request doesn't wait for the climb: it cancels the rest of it
    observation = make_observation(bird={"detected": True}, docking=docking_with(should_dock=True))
    observation["context"] = {"launch_phase": "climbing"}
    assert step(State.HOVERING, observation)[0] == State.DOCKING_INIT

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

@pytest.mark.parametrize("name", sorted(DEMOS))
def test_demos_track_only_after_the_climb(name):
    # In every demo, the tick before TRACKING starts shows the launch climb complete
    scenario, duration_ms = DEMOS[name]
    statuses = run_world(duration_ms, scenario=scenario)["statuses"]
    starts = [tick for tick, status in enumerate(statuses)
              if status.state == State.TRACKING and status.previous_state != State.TRACKING]
    assert starts, "the demo should track the bird"
    for tick in starts:
        assert statuses[tick - 1].launch_phase == "complete"

def test_descent_waits_until_aligned():
    # Docking hovers above the pad until the alignment becomes valid at 9000 ms,
    # aligns on that tick and starts the descent on the next one
    scenario, duration_ms = DEMOS["not_aligned"]
    commands = run_world(duration_ms, scenario=scenario)["commands"]
    descents = [time_ms for time_ms, kind, _, _ in commands if kind == "DESCEND_TO_DOCK"]
    assert descents == [9100]

def test_cart_moving_stops_descent_on_the_same_tick():
    # The cart starts moving at 8000 ms, during the descent; the descent is replaced by STOP in that
    # same tick, before Motion moves the drone (INTEGRATION_README.md section 11)
    scenario, duration_ms = DEMOS["cart_during_descent"]
    commands = run_world(duration_ms, scenario=scenario)["commands"]
    assert (8000, "STOP") in [(time_ms, kind) for time_ms, kind, _, _ in commands]

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

    # One start time, from the first DOCKING_INIT at 7000 ms, kept through the abort and retry
    assert {started for started in contexts if started is not None} == {7_000_000_000}
    assert contexts[-1] is None  # cleared once the drone is back in IDLE


# How each demo ends (README "MissionStatus"): back on the pad is "completed", ABORT or
# EMERGENCY_LAND is "failed", and still mid-mission when time runs out is "interrupted"
EXPECTED_OUTCOMES = {
    "full_cycle": "completed",
    "docking_retry": "completed",
    "emergency": "failed",
    "blocked_dock": "interrupted",
    "not_aligned": "completed",
    "cart_during_descent": "completed",
    "comm_loss": "failed",
}

TIME_FIELDS = ["state_entered_ns", "bird_absent_since_ns", "hover_started_ns", "return_started_ns"]


@pytest.mark.parametrize("name", sorted(DEMOS))
def test_each_tick_has_a_mission_status(name):
    # One MissionStatus per tick that follows the README's validation rules,
    # and the last one has the demo's outcome
    assert set(EXPECTED_OUTCOMES) == set(DEMOS)  # a new demo needs an expected outcome
    scenario, duration_ms = DEMOS[name]
    statuses = run_world(duration_ms, scenario=scenario)["statuses"]
    assert len(statuses) == duration_ms // 100  # 100 ms per tick

    previous_state = State.IDLE
    latched_return = None
    for tick, status in enumerate(statuses):
        assert status.tick == tick
        assert status.timestamp_ns == tick * 100_000_000
        assert status.previous_state == previous_state

        # A reason exactly when the state changed, and the entry time is that tick
        changed = status.state != status.previous_state
        assert (status.reason is not None) == changed
        assert status.state_entered_ns <= status.timestamp_ns
        if changed:
            assert status.state_entered_ns == status.timestamp_ns

        if status.state != State.HOVERING:
            assert status.launch_phase == "none"
            assert status.hover_started_ns is None

        for field in TIME_FIELDS:
            value = getattr(status, field)
            assert value is None or 0 <= value <= status.timestamp_ns, field

        # The return latch: set together with its reason, unchanged until IDLE, where both are cleared
        assert (status.return_started_ns is None) == (status.return_reason is None)
        if status.state == State.IDLE:
            assert status.return_started_ns is None
        elif latched_return is not None:
            assert status.return_started_ns == latched_return
        latched_return = status.return_started_ns

        json.dumps(status.to_dict())  # it can be written to a JSON log
        previous_state = status.state

    for status in statuses[:-1]:
        assert status.outcome is None
    assert statuses[-1].outcome == EXPECTED_OUTCOMES[name]


def test_launch_phase_reaches_complete():
    # The bird leaves right after the launch, so the mission stays in HOVERING for the whole climb:
    # launch_phase goes none -> climbing -> complete, then back to none once the mission leaves HOVERING.
    scenario = [(0, {"bird": BIRD_GONE}), (1000, {"bird": BIRD_SEEN}), (1100, {"bird": BIRD_GONE})]
    statuses = run_world(4500, scenario=scenario)["statuses"]

    phases = []  # (state, launch_phase) each time either one changes
    for status in statuses:
        if not phases or phases[-1] != (status.state, status.launch_phase):
            phases.append((status.state, status.launch_phase))
    assert phases == [
        (State.IDLE, "none"),
        (State.HOVERING, "climbing"),
        (State.HOVERING, "complete"),
        (State.DOCKING_INIT, "none"),
        (State.DOCKING_APPROACH, "none"),
    ]


# CV's 15 Hz fixture: no bird for 1 s, bird 1-5 s, none 5-6 s, bird 6-8 s, none 8-12 s,
# then missing frames until it ends at 13 s
INTEGRATION_CYCLE = str(Path(__file__).resolve().parents[2] / "cv" / "scenarios" / "integration_cycle.json")
FRAME_NS = 1_000_000_000 / 15  # time between two of its frames


def birds_seen_by_step(monkeypatch, duration_ms, **run_args):
    """Run the loop and record (tick time, snapshot["bird"]) on every tick, as step() receives them."""
    birds = []
    real_step = run_loop.step

    def recording_step(state, snapshot):
        birds.append((snapshot["timestamp_ns"], snapshot["bird"]))
        return real_step(state, snapshot)

    monkeypatch.setattr(run_loop, "step", recording_step)
    run_world(duration_ms, **run_args)
    return birds


# CV's 1 Hz fixture: a bird in 3 samples (0 s, 1 s, 2 s). At 10 Hz the cached sample
# goes past Safety's 0.5 s limit before the next one arrives.
STALE_SAMPLES = str(Path(__file__).resolve().parents[2] / "cv" / "scenarios" / "stale_samples.json")
MAX_AGE_NS = 500_000_000  # Safety's MAX_OBSERVATION_AGE_S


def snapshots_seen_by_step(monkeypatch, duration_ms, **run_args):
    """Run the loop and record every tick's snapshot as step() receives it, after Safety has assessed it."""
    snapshots = []
    real_step = run_loop.step

    def recording_step(state, snapshot):
        snapshots.append(snapshot)
        return real_step(state, snapshot)

    monkeypatch.setattr(run_loop, "step", recording_step)
    run_world(duration_ms, **run_args)
    return snapshots


def test_cv_bird_keeps_its_own_sample_time(monkeypatch):
    # At 15 Hz the samples fall between the 10 Hz ticks. The bird must keep its sample's time,
    # not be restamped with the tick's time.
    birds = birds_seen_by_step(monkeypatch, 15000, scenario=[], camera_fixture=INTEGRATION_CYCLE)
    bird_at = dict(birds)  # tick time -> bird

    assert bird_at[100_000_000]["timestamp_ns"] == 66_666_667  # frame 1, not the tick's 100 ms
    assert bird_at[200_000_000]["frame_index"] == 3  # frames 2 and 3 were both due; the latest one wins
    for now_ns, bird in birds:
        # always the latest sample that is due: no later than the tick, and less than one frame before it
        assert now_ns - FRAME_NS < bird["timestamp_ns"] <= now_ns

    first_seen = next(now_ns for now_ns, bird in birds if bird["detected"] is True)
    assert first_seen == 1_000_000_000  # frame 15

    # Missing frames are missing input, not "no bird"
    missing = [bird for now_ns, bird in birds if now_ns >= 12_000_000_000]
    assert len(missing) == 10
    for bird in missing:
        assert bird["detected"] is None and bird["valid"] is False and bird["reason"] == "missing_frame"


def test_run_ends_when_the_cv_fixture_runs_out():
    # The fixture ends at 13 s, so a 15 s run stops after the tick at 12.9 s.
    # That is the planned end of the scenario, so the last tick that ran still gets the outcome.
    statuses = run_world(15000, scenario=[], camera_fixture=INTEGRATION_CYCLE)["statuses"]

    assert len(statuses) == 130
    assert statuses[-1].timestamp_ns == 12_900_000_000
    assert statuses[-1].state == State.DOCKING_APPROACH
    assert statuses[-1].outcome == "interrupted"  # still airborne when the scenario ended
    for status in statuses[:-1]:
        assert status.outcome is None


def test_motion_logger_gets_motion_records():
    # run_world passes motion_logger to MotionStub, so Motion's records reach it.
    # MemoryLogSink keeps them in memory, so the test writes no log files.
    sink = MemoryLogSink()
    scenario, duration_ms = DEMOS["full_cycle"]
    run_world(duration_ms, scenario=scenario, motion_logger=sink)

    assert sink.records  # the logger reached Motion
    assert all(record.module == "MOTION" for record in sink.records)
    assert sink.records[0].event == "arm"  # the launch is the first thing Motion is asked to do


def test_safety_blocks_launch_and_tracking_on_missing_frames(monkeypatch):
    # integration_cycle.json's last second (12-13 s) is missing frames. Safety reads the camera
    # from environment.camera_ok, so the loop has to turn missing CV input into a camera failure.
    snapshots = snapshots_seen_by_step(monkeypatch, 15000, scenario=[], camera_fixture=INTEGRATION_CYCLE)
    missing = [snapshot for snapshot in snapshots if snapshot["bird"]["valid"] is False]

    assert len(missing) == 10  # the ticks at 12.0-12.9 s
    for snapshot in missing:
        safety = snapshot["safety"]
        assert safety["permissions"]["launch"] is False
        assert safety["permissions"]["tracking"] is False
        assert "camera_failure" in safety["reasons"]


def test_safety_blocks_launch_and_tracking_on_stale_samples(monkeypatch):
    # stale_samples.json is 1 Hz. Once the cached sample is older than Safety's 0.5 s limit,
    # the camera counts as failed; while it's fresh, it doesn't.
    snapshots = snapshots_seen_by_step(monkeypatch, 3000, scenario=[], camera_fixture=STALE_SAMPLES)

    stale = [s for s in snapshots if s["timestamp_ns"] - s["bird"]["timestamp_ns"] > MAX_AGE_NS]
    fresh = [s for s in snapshots if s["timestamp_ns"] - s["bird"]["timestamp_ns"] <= MAX_AGE_NS]
    assert len(stale) == 12  # 0.6-0.9 s, 1.6-1.9 s and 2.6-2.9 s
    assert len(fresh) == 18

    for snapshot in stale:
        safety = snapshot["safety"]
        assert safety["permissions"]["launch"] is False
        assert safety["permissions"]["tracking"] is False
        assert "camera_failure" in safety["reasons"]
    for snapshot in fresh:
        assert "camera_failure" not in snapshot["safety"]["reasons"]


def test_partial_bird_update_keeps_the_rest_of_the_bird():
    # A scenario event only changes the fields it lists. A distance-only update after a sighting
    # must keep detected and type, or CV rejects the fixture at startup.
    scenario = [BIRD_DEMO_INIT, (500, {"bird": BIRD_SEEN}), (2000, {"bird": {"distance_m": 20.0}})]
    gone_before = dict(BIRD_GONE)
    fixture = build_temp_fixture(scenario, 3000)

    validate_scenario(fixture)  # raises if any frame is incomplete
    frames = [segment["bird"] for segment in fixture["segments"]]  # one frame per tick
    assert frames[20] == {"detected": True, "type": "hawk", "distance_m": 20.0}  # 2.0 s: only the distance changed
    assert frames[19] == {"detected": True, "type": "hawk", "distance_m": 30.0}  # 1.9 s: earlier frames unchanged
    assert BIRD_GONE == gone_before  # the shared constant isn't changed either

    run_world(3000, scenario=scenario)  # CV accepts it


def test_partial_no_bird_update_clears_type_and_distance():
    # {"detected": False} on its own must not leave the hawk's type and distance behind,
    # because CV rejects a "no bird" frame that still has them.
    scenario = [BIRD_DEMO_INIT, (500, {"bird": BIRD_SEEN}), (1000, {"bird": {"detected": False}})]
    fixture = build_temp_fixture(scenario, 2000)

    validate_scenario(fixture)
    assert fixture["segments"][10]["bird"] == BIRD_GONE
