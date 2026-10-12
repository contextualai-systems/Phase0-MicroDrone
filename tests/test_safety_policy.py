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


class TestGroundedConditions(unittest.TestCase):
    """On the ground, a serious problem means shut off, not land."""

    def test_serious_problem_on_the_ground_disarms(self):
        result = assess(snapshot(flight_active=False, environment={"comm_ok": False}), NOW)
        self.assertTrue(result.disarm)
        self.assertFalse(result.emergency)      # never start a landing on the pad
        self.assertFalse(result.allows("launch"))
        self.assertIn("comm_loss", result.reasons)

    def test_bad_weather_on_the_ground_blocks_takeoff(self):
        result = assess(snapshot(flight_active=False, environment={"weather": "storm"}), NOW)
        self.assertTrue(result.disarm)
        self.assertFalse(result.allows("launch"))
        self.assertIn("weather_hazard", result.reasons)

    def test_abort_on_the_ground_disarms(self):
        snap = snapshot(flight_active=False)
        snap["context"]["operator_abort"] = True
        result = assess(snap, NOW)
        self.assertTrue(result.abort)
        self.assertTrue(result.disarm)

    def test_abort_in_the_air_does_not_disarm(self):
        snap = snapshot()
        snap["context"]["operator_abort"] = True
        result = assess(snap, NOW)
        self.assertTrue(result.abort)
        self.assertFalse(result.disarm)         # hold instead, it's flying
        self.assertTrue(result.allows("hold"))

    def test_nothing_wrong_on_the_ground_does_not_disarm(self):
        result = assess(snapshot(flight_active=False), NOW)
        self.assertFalse(result.disarm)
        self.assertTrue(result.grounded)
        self.assertTrue(result.allows("launch"))


class TestMissingFlightStatus(unittest.TestCase):
    """If we aren't told whether it's flying, assume it is."""

    def test_missing_flight_status_counts_as_flying(self):
        snap = snapshot(environment={"comm_ok": False})
        del snap["context"]["flight_active"]
        result = assess(snap, NOW)
        self.assertTrue(result.emergency)       # land, don't disarm mid-air
        self.assertFalse(result.disarm)
        self.assertFalse(result.grounded)

    def test_drone_reporting_airborne_counts_as_flying(self):
        snap = snapshot(flight_active=False, environment={"comm_ok": False})
        snap["vehicle"]["airborne"] = True
        result = assess(snap, NOW)
        self.assertTrue(result.emergency)
        self.assertFalse(result.disarm)


if __name__ == "__main__":
    unittest.main()
