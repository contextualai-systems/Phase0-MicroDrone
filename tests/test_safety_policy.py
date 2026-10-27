"""Tests for the safety referee.

Each test sets up a pretend situation, breaks one thing, and checks that
safety reacts the right way.

Run this from the main project folder:
    python -m unittest tests.test_safety_policy -v
"""

import unittest

from safety_layer import config
from safety_layer.safety_policy import assess

# Pretend the simulation has been running for 10 seconds.
NOW = 10 * config.NS_PER_S


def snapshot(flight_active=True, now_ns=NOW, **changes):
    """Build a situation where everything is fine, then break what we want.

    Instead of writing out the whole situation in every test, we start from
    a good one and change only the piece we're testing.
    """
    base = {
        "vehicle": {"valid": True, "timestamp_ns": now_ns, "airborne": flight_active},
        "battery": {"valid": True, "timestamp_ns": now_ns, "percent": 80.0},
        "environment": {
            "valid": True,
            "timestamp_ns": now_ns,
            "people_nearby": False,
            "cart_moving": False,
            "weather": "clear",
            "camera_ok": True,
            "comm_ok": True,
        },
        "dock": {"valid": True, "timestamp_ns": now_ns, "alignment_valid": True},
        "context": {"flight_active": flight_active},
    }
    # Apply whatever the test wanted to change.
    for section, values in changes.items():
        base.setdefault(section, {}).update(values)
    return base


class TestAllClear(unittest.TestCase):
    def test_everything_permitted_when_all_clear(self):
        """Nothing wrong, so everything should be allowed."""
        result = assess(snapshot(), NOW)
        for action in ("launch", "tracking", "approach", "descent", "hold"):
            self.assertTrue(result.allows(action), action)
        self.assertFalse(result.emergency)
        self.assertFalse(result.abort)
        self.assertEqual(result.reasons, ())  # no complaints


class TestEmergency(unittest.TestCase):
    def test_critical_battery_in_flight_is_emergency(self):
        """Nearly dead battery while flying: land right now."""
        result = assess(snapshot(battery={"percent": 5.0}), NOW)
        self.assertTrue(result.emergency)
        self.assertIn("critical_battery", result.reasons)
        self.assertFalse(result.allows("tracking"))

    def test_critical_battery_on_ground_is_not_emergency(self):
        """Same dead battery on the ground: it's already safe, just don't fly."""
        result = assess(snapshot(flight_active=False, battery={"percent": 5.0}), NOW)
        self.assertFalse(result.emergency)
        self.assertFalse(result.allows("launch"))

    def test_comm_loss_in_flight_is_emergency(self):
        """Nobody can send orders anymore, so bring it down."""
        result = assess(snapshot(environment={"comm_ok": False}), NOW)
        self.assertTrue(result.emergency)
        self.assertIn("comm_loss", result.reasons)

    def test_stale_vehicle_in_flight_is_emergency(self):
        """We don't know where the drone is anymore."""
        stale = NOW - int(2 * config.NS_PER_S)  # 2 seconds old; limit is half a second
        result = assess(snapshot(vehicle={"timestamp_ns": stale}), NOW)
        self.assertTrue(result.emergency)
        self.assertIn("stale_vehicle", result.reasons)


class TestRestrictions(unittest.TestCase):
    def test_low_battery_blocks_flying_but_allows_docking(self):
        """Low battery: stop chasing birds, but still allow coming home."""
        result = assess(snapshot(battery={"percent": 15.0}), NOW)
        self.assertTrue(result.battery_low)
        self.assertFalse(result.allows("tracking"))
        self.assertTrue(result.allows("approach"))
        self.assertTrue(result.allows("descent"))

    def test_cart_moving_blocks_descent(self):
        """Can't land on a moving cart, but hovering is still fine."""
        result = assess(snapshot(environment={"cart_moving": True}), NOW)
        self.assertFalse(result.allows("descent"))
        self.assertFalse(result.allows("approach"))
        self.assertTrue(result.allows("hold"))
        self.assertIn("cart_moving", result.reasons)

    def test_camera_failure_blocks_tracking_but_allows_docking(self):
        """No camera means no bird chasing, but landing uses the pad instead."""
        result = assess(snapshot(environment={"camera_ok": False}), NOW)
        self.assertFalse(result.allows("tracking"))
        self.assertTrue(result.allows("approach"))
        self.assertIn("camera_failure", result.reasons)

    def test_invalid_alignment_blocks_only_descent(self):
        """Not lined up with the pad: fly above it, but don't come down."""
        result = assess(snapshot(dock={"alignment_valid": False}), NOW)
        self.assertFalse(result.allows("descent"))
        self.assertTrue(result.allows("approach"))

    def test_missing_environment_blocks_flight_actions(self):
        """No information about surroundings at all, so no flying."""
        snap = snapshot()
        del snap["environment"]
        result = assess(snap, NOW)
        self.assertFalse(result.allows("tracking"))
        self.assertIn("missing_environment", result.reasons)

    def test_launch_needs_recovered_battery(self):
        """30% isn't low, but it's still not enough to start a new flight."""
        result = assess(snapshot(flight_active=False, battery={"percent": 30.0}), NOW)
        self.assertFalse(result.allows("launch"))
        self.assertFalse(result.battery_low)


class TestMultipleHazards(unittest.TestCase):
    def test_all_reasons_are_reported(self):
        """Two problems at once should both show up, not just the first."""
        result = assess(
            snapshot(environment={"cart_moving": True, "people_nearby": True}), NOW
        )
        self.assertIn("cart_moving", result.reasons)
        self.assertIn("people_nearby", result.reasons)

    def test_emergency_outranks_abort(self):
        """When both apply, the more serious one wins."""
        snap = snapshot(battery={"percent": 5.0})
        snap["context"]["operator_abort"] = True
        result = assess(snap, NOW)
        self.assertTrue(result.emergency)
        self.assertFalse(result.abort)


class TestAbort(unittest.TestCase):
    def test_operator_abort_blocks_flight_but_allows_hold(self):
        """Someone pressed stop: quit flying, but hovering is how it stops."""
        snap = snapshot()
        snap["context"]["operator_abort"] = True
        result = assess(snap, NOW)
        self.assertTrue(result.abort)
        self.assertFalse(result.allows("tracking"))
        self.assertTrue(result.allows("hold"))


class TestTimeHandling(unittest.TestCase):
    def test_uses_simulation_time_not_wall_clock(self):
        """The answer uses the pretend time we gave it, not the real clock."""
        result = assess(snapshot(now_ns=5_000), 5_000)
        self.assertEqual(result.timestamp_ns, 5_000)

    def test_future_timestamp_is_invalid(self):
        """Information stamped in the future is broken, so don't trust it."""
        future = NOW + config.NS_PER_S
        result = assess(snapshot(battery={"timestamp_ns": future}), NOW)
        self.assertIn("stale_battery", result.reasons)


class TestReturnDeadline(unittest.TestCase):
    """The trip home has 60 seconds. After that, land where you are."""

    def heading_home(self, seconds_ago, flight_active=True):
        snap = snapshot(flight_active=flight_active)
        snap["context"]["return_started_ns"] = NOW - int(seconds_ago * config.NS_PER_S)
        return assess(snap, NOW)

    def test_under_60_seconds_is_fine(self):
        result = self.heading_home(59.9)
        self.assertFalse(result.emergency)
        self.assertNotIn("return_deadline", result.reasons)

    def test_exactly_60_seconds_is_an_emergency(self):
        """Timers go off at the limit, not just after it."""
        result = self.heading_home(60)
        self.assertTrue(result.emergency)
        self.assertIn("return_deadline", result.reasons)

    def test_over_60_seconds_is_an_emergency(self):
        result = self.heading_home(75)
        self.assertTrue(result.emergency)
        self.assertFalse(result.allows("approach"))   # stop trying to dock

    def test_not_heading_home_is_fine(self):
        snap = snapshot()
        snap["context"]["return_started_ns"] = None
        self.assertFalse(assess(snap, NOW).emergency)

    def test_no_timer_field_is_fine(self):
        """Older code that doesn't send the timer shouldn't cause emergencies."""
        self.assertFalse(assess(snapshot(), NOW).emergency)

    def test_on_the_ground_is_not_an_emergency(self):
        """Already landed, so there's nothing to land."""
        result = self.heading_home(75, flight_active=False)
        self.assertFalse(result.emergency)

    def test_start_time_in_the_future_is_ignored(self):
        """A broken timer must not trigger a landing."""
        snap = snapshot()
        snap["context"]["return_started_ns"] = NOW + config.NS_PER_S
        self.assertFalse(assess(snap, NOW).emergency)


if __name__ == "__main__":
    unittest.main()
