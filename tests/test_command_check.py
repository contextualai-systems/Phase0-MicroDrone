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
    """The first safety check's answer when nothing is wrong."""
    return assess(snapshot(), NOW)


class TestGoodOrder(unittest.TestCase):
    def test_safe_order_passes_through_unchanged(self):
        decision = check_command(good_order(), all_clear(), NOW)
        self.assertTrue(decision.allowed)
        self.assertEqual(decision.command.command_id, "cmd-1")
        self.assertEqual(decision.reasons, ())

    def test_takeoff_without_an_expiry_is_fine(self):
        """Lifecycle orders may carry no expiry at all."""
        grounded = assess(snapshot(flight_active=False), NOW)
        takeoff = good_order(
            kind=MotionCommandType.TAKEOFF,
            mission_state="HOVERING",
            expires_ns=None,
            target=Pose(z=5.0),
        )
        decision = check_command(takeoff, grounded, NOW)
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
        grounded = assess(snapshot(flight_active=False), NOW)
        stale = good_order(
            kind=MotionCommandType.TAKEOFF,
            expires_ns=None,
            issued_ns=NOW - int(0.5 * config.NS_PER_S),
            target=Pose(z=5.0),
        )
        decision = check_command(stale, grounded, NOW)
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


# A drone sitting on its pad. "flight_active" is still true when the motors
# are on (just landed, or about to take off), which is when DISARM matters.
ON_THE_PAD = {"grounded": True, "airborne": False}


def on_the_pad(motors_on=True, **changes):
    vehicle = dict(ON_THE_PAD)
    vehicle.update(changes.pop("vehicle", {}))
    return assess(snapshot(flight_active=motors_on, vehicle=vehicle, **changes), NOW)


def emergency_landing_order(**changes):
    return good_order(
        command_id="sm-emergency",
        kind=MotionCommandType.EMERGENCY_LAND,
        mission_state="EMERGENCY_LAND",
        expires_ns=None,
        target=None,
        **changes,
    )


def disarm_order(**changes):
    return good_order(
        command_id="sm-disarm",
        kind=MotionCommandType.DISARM,
        mission_state="DOCKED",
        expires_ns=None,
        target=None,
        **changes,
    )


class TestOnTheGround(unittest.TestCase):
    """On the ground, the safe replacement is DISARM (motors off)."""

    def test_rejected_order_on_the_ground_becomes_disarm(self):
        stopped = on_the_pad(context={"operator_abort": True})
        decision = check_command(good_order(), stopped, NOW)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.command.kind, MotionCommandType.DISARM)
        self.assertIsNone(decision.command.target)
        self.assertEqual(decision.command.source, "safety")
        self.assertIsNone(decision.command.validate())

    def test_emergency_on_the_ground_turns_motors_off_instead_of_landing(self):
        storm = on_the_pad(environment={"weather": "storm"})
        self.assertTrue(storm.emergency)
        decision = check_command(good_order(), storm, NOW)
        self.assertEqual(decision.command.kind, MotionCommandType.DISARM)
        self.assertIn(EMERGENCY, decision.reasons)
        self.assertIn("weather_hazard", decision.reasons)

    def test_disarm_order_during_ground_emergency_passes_unchanged(self):
        dying = on_the_pad(battery={"percent": 4.0})
        order = disarm_order()
        decision = check_command(order, dying, NOW)
        self.assertTrue(decision.allowed)
        self.assertIs(decision.command, order)
        self.assertEqual(decision.reasons, ())

    def test_in_the_air_still_hovers(self):
        """The old behaviour in the air is unchanged."""
        decision = check_command(good_order(expires_ns=NOW - 1), all_clear(), NOW)
        self.assertEqual(decision.command.kind, MotionCommandType.HOVER)

    def test_unknown_ground_status_counts_as_flying(self):
        """Never tell a drone that might be flying to turn its motors off."""
        snap = snapshot(context={"operator_abort": True})
        snap["vehicle"].pop("grounded", None)
        decision = check_command(good_order(), assess(snap, NOW), NOW)
        self.assertEqual(decision.command.kind, MotionCommandType.HOVER)

    def test_airborne_flag_wins_over_grounded_flag(self):
        confused = on_the_pad(vehicle={"airborne": True}, context={"operator_abort": True})
        self.assertFalse(confused.grounded)
        decision = check_command(good_order(), confused, NOW)
        self.assertNotEqual(decision.command.kind, MotionCommandType.DISARM)

    def test_stale_position_never_counts_as_grounded(self):
        stale = on_the_pad(vehicle={"timestamp_ns": NOW - 2 * config.NS_PER_S})
        self.assertFalse(stale.grounded)
        decision = check_command(good_order(), stale, NOW)
        self.assertNotEqual(decision.command.kind, MotionCommandType.DISARM)

    def test_motion_accepts_the_disarm_on_the_ground(self):
        """End to end: Motion's simulator really turns the motors off."""
        from motion_engine.motion_stubs import MotionStub

        motion = MotionStub()
        motion.arm()
        self.assertTrue(motion.armed)
        storm = on_the_pad(environment={"weather": "storm"})
        decision = check_command(good_order(), storm, NOW)
        ack = motion.submit(decision.command, NOW)
        self.assertTrue(ack.accepted, ack.reason)
        self.assertFalse(motion.armed)


class TestEmergencyPassThrough(unittest.TestCase):
    """An EMERGENCY_LAND during an emergency is already the right answer."""

    def dying(self):
        return assess(snapshot(battery={"percent": 4.0}), NOW)

    def test_emergency_landing_passes_through_unchanged(self):
        order = emergency_landing_order()
        decision = check_command(order, self.dying(), NOW)
        self.assertTrue(decision.allowed)
        self.assertIs(decision.command, order)
        self.assertEqual(decision.command.command_id, "sm-emergency")
        self.assertEqual(decision.reasons, ())

    def test_stale_emergency_landing_is_replaced_with_a_fresh_one(self):
        order = emergency_landing_order(issued_ns=NOW - config.NS_PER_S)
        decision = check_command(order, self.dying(), NOW)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.command.kind, MotionCommandType.EMERGENCY_LAND)
        self.assertEqual(decision.command.issued_ns, NOW)

    def test_other_orders_are_still_replaced(self):
        decision = check_command(good_order(), self.dying(), NOW)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.command.kind, MotionCommandType.EMERGENCY_LAND)

    def test_disarm_in_the_air_is_still_replaced(self):
        """Motors off mid-flight is a crash, so it never passes through."""
        decision = check_command(disarm_order(), self.dying(), NOW)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.command.kind, MotionCommandType.EMERGENCY_LAND)
