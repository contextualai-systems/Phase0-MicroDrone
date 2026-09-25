""""
Tests for safety decision stub in (safety_layer/safety_decision.py)

Check if the safety_decision.py returns a valid answer
if "allowed/not allowed, why, and when" answered logically,
then we can proceed with said action

"""

import unittest

from safety_layer.safety_decision import check_observation


def make_observation(**overrides):
    """Build a safe observation, then apply any changes for the test."""
    safety = {
        "people_nearby": False,
        "cart_moving": False,
        "weather": "clear",
        "camera_ok": True,
        "comm_ok": True,
    }
    safety.update(overrides)
    return {"state": "TRACKING", "safety": safety}
class TestSafetyDecision(unittest.TestCase):
    def test_allows_movement_when_all_clear(self):
        decision = check_observation(make_observation())
        self.assertTrue(decision.allowed)
        self.assertEqual(decision.action, "proceed")

    def test_blocks_when_people_nearby(self):
        decision = check_observation(make_observation(people_nearby=True))
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.action, "abort")
        self.assertIn("people nearby", decision.reason)

    def test_blocks_when_cart_moving(self):
        decision = check_observation(make_observation(cart_moving=True))
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.action, "hold")

    def test_blocks_on_bad_weather(self):
        decision = check_observation(make_observation(weather="storm"))
        self.assertFalse(decision.allowed)

    def test_blocks_on_camera_failure(self):
        decision = check_observation(make_observation(camera_ok=False))
        self.assertFalse(decision.allowed)

    def test_blocks_on_comm_loss(self):
        decision = check_observation(make_observation(comm_ok=False))
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.action, "abort")

    def test_blocks_when_safety_data_missing(self):
        decision = check_observation({"state": "TRACKING"})
        self.assertFalse(decision.allowed)
        self.assertIn("missing", decision.reason)

    def test_reports_every_problem(self):
        decision = check_observation(make_observation(cart_moving=True, camera_ok=False))
        self.assertIn("cart is moving", decision.reason)
        self.assertIn("camera not working", decision.reason)

    def test_blocked_decision_suggests_dock_check(self):
        decision = check_observation(make_observation(cart_moving=True))
        self.assertTrue(decision.dock_check)

    def test_every_decision_has_reason_and_time(self):
        decision = check_observation(make_observation())
        self.assertTrue(decision.reason)
        self.assertGreater(decision.timestamp_ns, 0)


if __name__ == "__main__":
    unittest.main()