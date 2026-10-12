"""Tests for the second safety check: checking one actual order.

Run this from the main project folder:
    python -m unittest tests.test_command_check -v
"""

import unittest

from motion_engine.contracts import Command, MotionCommandType, Pose, Reason

from safety_layer import config
from safety_layer.command_check import (
    ACTION_NOT_PERMITTED,
    ALTITUDE_CEILING,
    EMERGENCY,
    GEOFENCE_BREACH,
    GROUNDED_DISARM,
    SPEED_LIMIT,
    UNDERGROUND_TARGET,
    check_command,
)
from safety_layer.safety_policy import assess
from tests.test_safety_policy import NOW, snapshot


def good_order(**changes):
    """A perfectly safe tracking order, unless a test changes something."""
    base = dict(
        command_id="cmd-1",
        kind=MotionCommandType.TRACK_TARGET,
        mission_state="TRACKING",
        issued_ns=NOW,
        expires_ns=NOW + int(0.2 * config.NS_PER_S),
        source="navigation",
        reason="track_bird",
        target=Pose(x=10.0, y=10.0, z=15.0, yaw_deg=0.0),
    )
    base.update(changes)
    return Command(**base)


def all_clear():
    """The first safety check's answer when nothing is wrong, in the air."""
    return assess(snapshot(), NOW)


def on_the_ground(**changes):
    """The first check's answer with the drone sitting on the pad."""
    return assess(snapshot(flight_active=False, **changes), NOW)


class TestGoodOrder(unittest.TestCase):
    def test_safe_order_passes_through_unchanged(self):
        decision = check_command(good_order(), all_clear(), NOW)
        self.assertTrue(decision.allowed)
        self.assertEqual(decision.command.command_id, "cmd-1")
        self.assertEqual(decision.reasons, ())

    def test_takeoff_without_an_expiry_is_fine(self):
        """Lifecycle orders may carry no expiry at all."""
        takeoff = good_order(
            kind=MotionCommandType.TAKEOFF,
            mission_state="HOVERING",
            expires_ns=None,
            target=Pose(z=5.0),
        )
        decision = check_command(takeoff, on_the_ground(), NOW)
        self.assertTrue(decision.allowed)


class TestExpiredOrders(unittest.TestCase):
    def test_order_past_its_expiry_is_replaced(self):
        old = good_order(expires_ns=NOW - 1)
        decision = check_command(old, all_clear(), NOW)
        self.assertFalse(decision.allowed)
        self.assertIn(Reason.COMMAND_EXPIRED, decision.reasons)
        self.assertEqual(decision.command.kind, MotionCommandType.HOVER)

    def test_old_takeoff_without_expiry_still_goes_stale(self):
        """No expiry field, so we measure from when it was issued."""
        stale = good_order(
            kind=MotionCommandType.TAKEOFF,
            expires_ns=None,
            issued_ns=NOW - int(0.5 * config.NS_PER_S),
            target=Pose(z=5.0),
        )
        decision = check_command(stale, on_the_ground(), NOW)
        self.assertIn(Reason.COMMAND_EXPIRED, decision.reasons)


class TestLimits(unittest.TestCase):
    def test_outside_the_fence_is_replaced(self):
        far_away = good_order(target=Pose(x=200.0, y=0.0, z=10.0))
        decision = check_command(far_away, all_clear(), NOW)
        self.assertIn(GEOFENCE_BREACH, decision.reasons)
        self.assertEqual(decision.command.kind, MotionCommandType.HOVER)

    def test_too_high_is_replaced(self):
        decision = check_command(good_order(target=Pose(z=45.0)), all_clear(), NOW)
        self.assertIn(ALTITUDE_CEILING, decision.reasons)

    def test_underground_target_is_replaced(self):
        decision = check_command(good_order(target=Pose(z=-3.0)), all_clear(), NOW)
        self.assertIn(UNDERGROUND_TARGET, decision.reasons)

    def test_speed_cap_above_our_limit_is_refused(self):
        """Caps may only slow the drone down, never speed it up."""
        fast = good_order(max_horizontal_speed_mps=9.0)   # limit is 5
        decision = check_command(fast, all_clear(), NOW)
        self.assertIn(SPEED_LIMIT, decision.reasons)

    def test_lower_speed_cap_is_fine(self):
        slow = good_order(max_horizontal_speed_mps=2.0)
        decision = check_command(slow, all_clear(), NOW)
        self.assertTrue(decision.allowed)


class TestReplacementIsSafe(unittest.TestCase):
    def test_replacement_drops_the_old_destination(self):
        """The old target is exactly what we're trying to stop."""
        decision = check_command(good_order(target=Pose(z=45.0)), all_clear(), NOW)
        self.assertIsNone(decision.command.target)

    def test_replacement_comes_from_safety(self):
        decision = check_command(good_order(target=Pose(z=45.0)), all_clear(), NOW)
        self.assertEqual(decision.command.source, "safety")
        self.assertNotEqual(decision.command.command_id, "cmd-1")

    def test_replacement_is_well_formed(self):
        """Motion must accept our replacement, or the drone keeps going."""
        decision = check_command(good_order(target=Pose(z=45.0)), all_clear(), NOW)
        self.assertIsNone(decision.command.validate())


class TestPermissionsAreRespected(unittest.TestCase):
    def test_tracking_order_blocked_when_tracking_is_not_allowed(self):
        low_battery = assess(snapshot(battery={"percent": 15.0}), NOW)
        decision = check_command(good_order(), low_battery, NOW)
        self.assertFalse(decision.allowed)
        self.assertIn(ACTION_NOT_PERMITTED, decision.reasons)

    def test_unknown_order_kind_is_refused(self):
        decision = check_command(good_order(kind="do_a_backflip"), all_clear(), NOW)
        self.assertFalse(decision.allowed)

    def test_turning_on_the_spot_follows_the_hold_rules(self):
        """SET_YAW doesn't go anywhere, so low battery shouldn't block it."""
        low_battery = assess(snapshot(battery={"percent": 15.0}), NOW)
        turn = good_order(kind=MotionCommandType.SET_YAW, target=Pose(yaw_deg=90.0))
        decision = check_command(turn, low_battery, NOW)
        self.assertTrue(decision.allowed)


class TestGroundedBehaviour(unittest.TestCase):
    """On the pad the answer is disarm, never hover or land."""

    def test_rejected_order_on_the_ground_disarms(self):
        expired = good_order(expires_ns=NOW - 1)
        decision = check_command(expired, on_the_ground(), NOW)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.command.kind, MotionCommandType.DISARM)

    def test_lost_signal_on_the_ground_disarms(self):
        grounded = on_the_ground(environment={"comm_ok": False})
        self.assertTrue(grounded.disarm)
        self.assertFalse(grounded.emergency)        # no landing manoeuvre
        decision = check_command(good_order(), grounded, NOW)
        self.assertEqual(decision.command.kind, MotionCommandType.DISARM)
        self.assertIn(GROUNDED_DISARM, decision.reasons)
        self.assertIn("comm_loss", decision.reasons)

    def test_bad_weather_on_the_ground_blocks_takeoff(self):
        grounded = on_the_ground(environment={"weather": "storm"})
        self.assertFalse(grounded.allows("launch"))
        takeoff = good_order(
            kind=MotionCommandType.TAKEOFF, expires_ns=None, target=Pose(z=5.0)
        )
        decision = check_command(takeoff, grounded, NOW)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.command.kind, MotionCommandType.DISARM)

    def test_disarm_replacement_is_well_formed(self):
        grounded = on_the_ground(environment={"comm_ok": False})
        decision = check_command(good_order(), grounded, NOW)
        self.assertIsNone(decision.command.validate())


class TestCartMovingDuringDescent(unittest.TestCase):
    """The scenario the integration plan calls out by name."""

    def test_cart_starts_moving_while_coming_down(self):
        moving = assess(snapshot(environment={"cart_moving": True}), NOW)
        descending = good_order(
            kind=MotionCommandType.DESCEND_TO_DOCK,
            mission_state="DOCKING_APPROACH",
            target=Pose(x=0.0, y=0.0, z=0.0),
        )
        decision = check_command(descending, moving, NOW)

        # The descent must be swapped out in this same tick.
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.command.kind, MotionCommandType.HOVER)
        self.assertIn(ACTION_NOT_PERMITTED, decision.reasons)


class TestEmergency(unittest.TestCase):
    def test_emergency_replaces_any_order_with_landing(self):
        dying = assess(snapshot(battery={"percent": 4.0}), NOW)
        decision = check_command(good_order(), dying, NOW)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.command.kind, MotionCommandType.EMERGENCY_LAND)
        self.assertIn(EMERGENCY, decision.reasons)

    def test_emergency_keeps_the_underlying_reasons(self):
        no_signal = assess(snapshot(environment={"comm_ok": False}), NOW)
        decision = check_command(good_order(), no_signal, NOW)
        self.assertIn("comm_loss", decision.reasons)

    def test_emergency_landing_order_is_well_formed(self):
        dying = assess(snapshot(battery={"percent": 4.0}), NOW)
        decision = check_command(good_order(), dying, NOW)
        self.assertIsNone(decision.command.validate())


class TestSeveralProblemsAtOnce(unittest.TestCase):
    def test_every_problem_is_listed(self):
        bad = good_order(target=Pose(x=300.0, y=0.0, z=50.0), expires_ns=NOW - 1)
        decision = check_command(bad, all_clear(), NOW)
        self.assertIn(GEOFENCE_BREACH, decision.reasons)
        self.assertIn(ALTITUDE_CEILING, decision.reasons)
        self.assertIn(Reason.COMMAND_EXPIRED, decision.reasons)


class TestReplacementNames(unittest.TestCase):
    """Motion rejects an old name reused for a different order."""

    def test_hover_and_emergency_landing_get_different_names(self):
        low = assess(snapshot(battery={"percent": 15.0}), NOW)
        dying = assess(snapshot(battery={"percent": 4.0}), NOW)
        hover = check_command(good_order(), low, NOW).command
        landing = check_command(good_order(), dying, NOW).command
        self.assertEqual(hover.command_id, "cmd-1-safety-hover")
        self.assertEqual(landing.command_id, "cmd-1-safety-emergency_land")

    def test_replacements_say_why_in_the_log(self):
        low = assess(snapshot(battery={"percent": 15.0}), NOW)
        dying = assess(snapshot(battery={"percent": 4.0}), NOW)
        grounded = on_the_ground(environment={"comm_ok": False})
        self.assertEqual(check_command(good_order(), low, NOW).command.reason, "safety_hold")
        self.assertEqual(check_command(good_order(), dying, NOW).command.reason, "emergency_land")
        self.assertEqual(check_command(good_order(), grounded, NOW).command.reason, "safety_disarm")


if __name__ == "__main__":
    unittest.main()
