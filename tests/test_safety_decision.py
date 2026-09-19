""""
Tests for safety decision stub in (safety_layer/safety_decision.py)

Check if the safety_decision.py returns a valid answer
if "allowed/not allowed, why, and when" answered logically,
then we can proceed with said action

"""

import unittest

from safety_layer.safety_decision import check_request


class TestSafetyDecision(unittest.TestCase):
    def test_stub_allows_request(self):
        decision = check_request(request=None)
        self.assertTrue(decision.allowed)
        self.assertEqual(decision.action, "proceed")

    def test_decision_has_reason_and_time(self):
        decision = check_request(request=None)
        self.assertTrue(decision.reason)
        self.assertGreater(decision.timestamp_ns, 0)


if __name__ == "__main__":
    unittest.main()