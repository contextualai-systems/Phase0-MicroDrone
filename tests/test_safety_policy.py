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
        "vehicle": {
            "valid": True,
            "timestamp_ns": now_ns,
            "airborne": flight_active,
            # On the ground means resting on the pad with the motors off.
            "grounded": not flight_active,
            "armed": flight_active,
            "pose": {"x": 0.0, "y": 0.0, "z": 0.0 if not flight_active else 10.0, "yaw_deg": 0.0},
        },
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
        "dock": {
            "valid": True,
            "timestamp_ns": now_ns,
            "alignment_valid": True,
            "pad_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0},
        },
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


class TestLaunchChecks(unittest.TestCase):
    """Before takeoff: on the ground, motors off, on the pad, signal OK,
    and safe weather. These only apply on the ground."""

    def on_ground(self, **changes):
        return assess(snapshot(flight_active=False, **changes), NOW)

    def assert_launch_blocked(self, result, reason):
        self.assertFalse(result.allows("launch"))
        self.assertIn(reason, result.reasons)
        # A launch problem is not an emergency: the drone just waits.
        self.assertFalse(result.emergency)
        self.assertTrue(result.allows("hold"))

    def test_launch_allowed_when_ready_on_the_pad(self):
        result = self.on_ground()
        self.assertTrue(result.allows("launch"))
        self.assertEqual(result.reasons, ())

    def test_not_grounded_blocks_launch(self):
        self.assert_launch_blocked(self.on_ground(vehicle={"grounded": False}), "not_grounded")

    def test_motors_on_blocks_launch(self):
        self.assert_launch_blocked(self.on_ground(vehicle={"armed": True}), "motors_armed")

    def test_unknown_motor_state_blocks_launch(self):
        snap = snapshot(flight_active=False)
        del snap["vehicle"]["armed"]
        self.assert_launch_blocked(assess(snap, NOW), "motors_armed")

    def test_off_the_pad_blocks_launch(self):
        result = self.on_ground(vehicle={"pose": {"x": 3.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}})
        self.assert_launch_blocked(result, "not_on_pad")

    def test_just_inside_the_pad_tolerance_is_fine(self):
        edge = config.DOCK_ALIGNMENT_TOLERANCE_M
        result = self.on_ground(vehicle={"pose": {"x": edge, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}})
        self.assertTrue(result.allows("launch"))

    def test_unknown_pad_position_blocks_launch(self):
        snap = snapshot(flight_active=False)
        del snap["dock"]["pad_pose"]
        self.assert_launch_blocked(assess(snap, NOW), "not_on_pad")

    def test_missing_pad_data_blocks_launch(self):
        result = self.on_ground(dock={"valid": False})
        self.assert_launch_blocked(result, "missing_pad")
        self.assertNotIn("not_on_pad", result.reasons)  # no guessing from bad data

    def test_bad_weather_on_the_ground_waits_instead_of_emergency(self):
        result = self.on_ground(environment={"weather": "storm"})
        self.assert_launch_blocked(result, "weather_hazard")

    def test_lost_signal_on_the_ground_blocks_launch(self):
        self.assert_launch_blocked(self.on_ground(environment={"comm_ok": False}), "comm_loss")

    def test_stale_vehicle_does_not_add_guessed_reasons(self):
        result = self.on_ground(vehicle={"valid": False, "grounded": False})
        self.assertFalse(result.allows("launch"))
        self.assertIn("stale_vehicle", result.reasons)
        self.assertNotIn("not_grounded", result.reasons)

    def test_launch_reasons_never_appear_in_flight(self):
        """In the air, launch reasons would get mixed into Docking's reasons."""
        result = assess(snapshot(vehicle={"pose": {"x": 40.0, "y": 0.0, "z": 10.0, "yaw_deg": 0.0}}), NOW)
        for reason in ("not_grounded", "motors_armed", "not_on_pad"):
            self.assertNotIn(reason, result.reasons)

    def test_weather_still_an_emergency_in_flight(self):
        result = assess(snapshot(environment={"weather": "storm"}), NOW)
        self.assertTrue(result.emergency)


class TestLaunchInTheRunLoop(unittest.TestCase):
    """The same rule, end to end through the real coordinator loop."""

    def test_drone_waits_on_the_pad_until_the_storm_clears(self):
        from state_machine.run_loop import run_world
        from state_machine.state_machine import State

        scenario = [
            (0, {"environment": {"weather": "storm"}}),
            (500, {"bird": {"detected": True, "type": "pigeon", "distance_m": 20.0}}),
            (2000, {"environment": {"weather": "clear"}}),
        ]
        result = run_world(3000, scenario=scenario)
        launches = [t for t in result["transitions"] if t[1] == State.IDLE and t[2] == State.HOVERING]
        self.assertEqual(len(launches), 1)
        self.assertEqual(launches[0][0], 2000)  # took off the moment the storm cleared
        # It never had to emergency-land.
        self.assertNotIn(State.EMERGENCY_LAND, [t[2] for t in result["transitions"]])


class TestLaunchTimeout(unittest.TestCase):
    """The takeoff climb has 10 seconds. After that, abort and come home."""

    def launching(self, seconds_ago, phase="climbing", **context):
        snap = snapshot()  # in the air, everything else fine
        snap["context"]["launch_phase"] = phase
        snap["context"]["launch_started_ns"] = (
            None if seconds_ago is None else NOW - int(seconds_ago * config.NS_PER_S)
        )
        snap["context"].update(context)
        return assess(snap, NOW)

    def test_under_10_seconds_is_fine(self):
        result = self.launching(9.9)
        self.assertFalse(result.abort)
        self.assertNotIn("launch_timeout", result.reasons)

    def test_exactly_10_seconds_aborts(self):
        """Timers go off at the limit, not just after it."""
        result = self.launching(10)
        self.assertTrue(result.abort)
        self.assertIn("launch_timeout", result.reasons)

    def test_abort_is_not_an_emergency(self):
        """The drone can still hold and come home; it doesn't land in place."""
        result = self.launching(12)
        self.assertFalse(result.emergency)
        self.assertTrue(result.allows("hold"))
        for action in ("launch", "tracking", "approach", "descent"):
            self.assertFalse(result.allows(action), action)

    def test_arming_counts_as_launching(self):
        self.assertTrue(self.launching(10, phase="arming").abort)

    def test_finished_climb_cancels_the_timer(self):
        """An old launch timer must never abort a later trip home."""
        for phase in ("complete", "none", None):
            with self.subTest(phase=phase):
                result = self.launching(30, phase=phase)
                self.assertFalse(result.abort)
                self.assertNotIn("launch_timeout", result.reasons)

    def test_no_timer_or_broken_timer_is_ignored(self):
        self.assertFalse(self.launching(None).abort)
        self.assertFalse(self.launching(-5).abort)  # start time in the future
        snap = snapshot()
        snap["context"].update(launch_phase="climbing", launch_started_ns=True)
        self.assertFalse(assess(snap, NOW).abort)

    def test_no_timer_field_is_fine(self):
        """Older code that doesn't send the timer shouldn't cause aborts."""
        snap = snapshot()
        snap["context"]["launch_phase"] = "climbing"
        snap["context"].pop("launch_started_ns", None)
        self.assertFalse(assess(snap, NOW).abort)

    def test_emergency_wins_over_launch_timeout(self):
        snap = snapshot(battery={"percent": 5.0})
        snap["context"].update(
            launch_phase="climbing", launch_started_ns=NOW - 12 * config.NS_PER_S
        )
        result = assess(snap, NOW)
        self.assertTrue(result.emergency)
        self.assertNotIn("launch_timeout", result.reasons)


class TestLaunchTimeoutInTheRunLoop(unittest.TestCase):
    """End to end: weak motors that can't finish the climb in time."""

    BIRD_AT_START = [(0, {"bird": {"detected": True, "type": "hawk", "distance_m": 20.0}})]

    def run_with_climb_speed(self, climb_mps, duration_ms):
        import dataclasses
        from unittest import mock

        import state_machine.run_loop as run_loop

        real_limits = run_loop.MotionLimits.from_config
        real_assess = run_loop.assess
        seen = []  # (time_ms, reasons) for every tick Safety ran

        def weak_motors(*args, **kwargs):
            return dataclasses.replace(
                real_limits(*args, **kwargs), max_vertical_speed_mps=climb_mps
            )

        def listening_assess(snap, now_ns):
            result = real_assess(snap, now_ns)
            seen.append((now_ns // 1_000_000, result.reasons))
            return result

        with mock.patch.object(run_loop.MotionLimits, "from_config", side_effect=weak_motors), \
                mock.patch.object(run_loop, "assess", side_effect=listening_assess):
            result = run_loop.run_world(duration_ms, scenario=self.BIRD_AT_START)
        return result, seen

    def test_stuck_climb_aborts_at_10_seconds_and_lands_on_the_pad(self):
        from state_machine.state_machine import State

        # 0.3 m/s needs about 17 s to reach the 5 m launch altitude.
        result, seen = self.run_with_climb_speed(0.3, 34000)
        transitions = [(t[0], t[1], t[2]) for t in result["transitions"]]
        self.assertEqual(transitions[0], (0, State.IDLE, State.HOVERING))
        self.assertEqual(transitions[1], (10000, State.HOVERING, State.ABORT))
        self.assertIn((10100, State.ABORT, State.DOCKING_INIT), transitions)
        self.assertIn(State.DOCKED, [t[2] for t in transitions])
        self.assertNotIn(State.EMERGENCY_LAND, [t[2] for t in transitions])

        timeout_ticks = [t for t, reasons in seen if "launch_timeout" in reasons]
        self.assertEqual(timeout_ticks, [10000])  # once, then the timer is cancelled

    def test_normal_climb_never_times_out(self):
        result, seen = self.run_with_climb_speed(2.0, 12000)
        self.assertFalse(any("launch_timeout" in reasons for _, reasons in seen))

    def test_no_demo_ever_hits_the_launch_timeout(self):
        from unittest import mock

        import state_machine.run_loop as run_loop
        from state_machine.demos import DEMOS

        real_assess = run_loop.assess
        for name, (scenario, duration_ms) in DEMOS.items():
            with self.subTest(demo=name):
                hits = []

                def listening_assess(snap, now_ns):
                    result = real_assess(snap, now_ns)
                    if "launch_timeout" in result.reasons:
                        hits.append(now_ns)
                    return result

                with mock.patch.object(run_loop, "assess", side_effect=listening_assess):
                    run_loop.run_world(duration_ms, scenario=scenario)
                self.assertEqual(hits, [])
