from state_machine.state_machine import step, State

def make_observation(bird = None, battery = None, safety = None, docking = None):
    # Makes observations in the team's JSON format for testing purposes
    return {
        "bird": bird or {"detected": False},
        "battery": battery or {"percent": 100, "low": False},
        "safety": safety or {
            "people_nearby": False,
            "cart_moving": False,
            "weather": "clear",
            "camera_ok": True,
            "comm_ok": True,
        },
        "docking": docking or {"should_dock": False, "reason": None},
    }

def test_idle_to_tracking():
    observation = make_observation(bird={"detected": True})
    new_state, reason = step(State.IDLE, observation)
    assert new_state == State.TRACKING
    assert reason == "Bird detected"

def test_tracking_to_idle():
    observation = make_observation(bird={"detected": False})
    new_state, reason = step(State.TRACKING, observation)
    assert new_state == State.IDLE
    assert reason == "Bird not detected"

def test_idle_to_detected_bird():
    observation = make_observation(bird={"type": "sparrow", "detected": True, "distance_m": 5})
    assert step(State.IDLE, observation) == (State.TRACKING, "Bird detected")

def test_lost_bird_to_idle():
    observation = make_observation(bird={"detected": False})
    assert step(State.TRACKING, observation) == (State.IDLE, "Bird not detected")

def test_missing_bird_data_to_idle():
    assert step(State.TRACKING, {}) == (State.IDLE, "Bird not detected")
