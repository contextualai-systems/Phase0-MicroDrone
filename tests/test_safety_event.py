"""Tests for SafetyEvent, Safety's report for one tick.

Run this from the main project folder:
    python -m unittest tests.test_safety_event -v
"""

import json
import unittest
from dataclasses import replace
from unittest import mock

from motion_engine.contracts import MotionCommandType, Pose

from safety_layer.command_check import check_command
from safety_layer.contracts import SafetyEvent
from safety_layer.safety_policy import ACTIONS, assess
from tests.test_command_check import good_order
from tests.test_safety_policy import NOW, snapshot


def event_for(snap, *orders):
    """Run both Safety checks on a situation and build the event."""
    assessment = assess(snap, NOW)
    checked = [(order, check_command(order, assessment, NOW)) for order in orders]
    return SafetyEvent.from_checks(assessment, checked)


class TestBuildingAnEvent(unittest.TestCase):
    def test_all_clear_event(self):
        event = event_for(snapshot())
        self.assertIsNone(event.validate())
        self.assertEqual(event.timestamp_ns, NOW)
        self.assertTrue(all(event.permissions.values()))
        self.assertEqual(event.reasons, ())
        self.assertEqual(event.commands, ())       # no orders this tick

    def test_copies_the_assessment(self):
        event = event_for(snapshot(battery={"percent": 15.0}))
        self.assertTrue(event.battery_low)
        self.assertFalse(event.permissions["tracking"])
        self.assertIn("low_battery", event.reasons)
        self.assertIsNone(event.validate())

    def test_allowed_order_is_recorded(self):
        event = event_for(snapshot(), good_order())
        check = event.commands[0]
        self.assertTrue(check.allowed)
        self.assertEqual(check.requested_id, check.sent_id)
        self.assertEqual(check.sent_kind, "TRACK_TARGET")
        self.assertIsNone(event.validate())

    def test_replaced_order_shows_what_was_sent_instead(self):
        too_high = good_order(target=Pose(z=45.0))
        event = event_for(snapshot(), too_high)
        check = event.commands[0]
        self.assertFalse(check.allowed)
        self.assertEqual(check.requested_kind, "TRACK_TARGET")
        self.assertEqual(check.sent_kind, "HOVER")
        self.assertIn("altitude_ceiling", check.reasons)
        self.assertIsNone(event.validate())

    def test_several_orders_in_one_tick(self):
        """A launch sends ARM and TAKEOFF on the same tick."""
        arm = good_order(command_id="arm-1", kind=MotionCommandType.ARM,
                         expires_ns=None, target=None)
        takeoff = good_order(command_id="takeoff-1", kind=MotionCommandType.TAKEOFF,
                             expires_ns=None, target=Pose(z=5.0))
        event = event_for(snapshot(flight_active=False), arm, takeoff)
        self.assertEqual([c.requested_id for c in event.commands], ["arm-1", "takeoff-1"])
        self.assertIsNone(event.validate())

    def test_emergency_event(self):
        event = event_for(snapshot(battery={"percent": 4.0}), good_order())
        self.assertTrue(event.emergency)
        self.assertEqual(event.commands[0].sent_kind, "EMERGENCY_LAND")
        self.assertIsNone(event.validate())


class TestToDict(unittest.TestCase):
    def test_is_plain_json(self):
        """The shared log needs plain values, no Python objects."""
        event = event_for(snapshot(environment={"cart_moving": True}), good_order())
        text = json.dumps(event.to_dict())          # fails if anything isn't plain
        back = json.loads(text)
        self.assertEqual(back["source"], "safety")
        self.assertEqual(back["schema_version"], 1)
        self.assertEqual(back["clock"], "simulation")
        self.assertIn("cart_moving", back["reasons"])
        self.assertEqual(list(back["permissions"]), list(ACTIONS))


class TestValidationCatchesBugs(unittest.TestCase):
    """Each rule should catch the mistake it's there for."""

    def setUp(self):
        self.good = event_for(snapshot())

    def test_missing_action(self):
        perms = dict(self.good.permissions)
        del perms["hold"]
        self.assertEqual(replace(self.good, permissions=perms).validate(), "invalid_permissions")

    def test_permission_written_as_text(self):
        perms = dict(self.good.permissions, launch="true")
        self.assertEqual(replace(self.good, permissions=perms).validate(), "invalid_permissions")

    def test_emergency_that_still_allows_something(self):
        bad = replace(self.good, emergency=True, reasons=("comm_loss",))
        self.assertEqual(bad.validate(), "emergency_with_permission")

    def test_emergency_and_abort_together(self):
        perms = {action: False for action in ACTIONS}
        bad = replace(self.good, permissions=perms, emergency=True, abort=True,
                      reasons=("comm_loss",))
        self.assertEqual(bad.validate(), "emergency_and_abort")

    def test_abort_with_no_reason(self):
        self.assertEqual(replace(self.good, abort=True).validate(), "missing_reason")

    def test_critical_but_not_low(self):
        bad = replace(self.good, battery_critical=True, battery_low=False)
        self.assertEqual(bad.validate(), "critical_without_low")

    def test_negative_time(self):
        self.assertEqual(replace(self.good, timestamp_ns=-1).validate(), "invalid_timestamp")

    def test_replaced_order_with_no_reason(self):
        replaced = event_for(snapshot(), good_order(target=Pose(z=45.0)))
        check = replace(replaced.commands[0], reasons=())
        bad = replace(replaced, commands=(check,))
        self.assertEqual(bad.validate(), "replacement_unexplained")


class TestEveryDemoTick(unittest.TestCase):
    """Run the team's demos and check the event Safety would publish each tick.

    This doesn't change the main loop. It listens in on the two Safety calls
    the loop already makes, the same way the state machine's own tests do.
    """

    def run_demo(self, name):
        from state_machine.demos import DEMOS
        from state_machine.run_loop import run_world
        import state_machine.run_loop as loop

        ticks = []   # one [assessment, [(order, decision), ...]] per tick
        real_assess, real_check = loop.assess, loop.check_for_motion

        def listen_assess(snap, now_ns):
            result = real_assess(snap, now_ns)
            ticks.append([result, []])
            return result

        def listen_check(order, assessment, now_ns):
            sent, decision = real_check(order, assessment, now_ns)
            ticks[-1][1].append((order, decision))
            return sent, decision

        scenario, duration_ms = DEMOS[name]
        with mock.patch.object(loop, "assess", listen_assess), \
                mock.patch.object(loop, "check_for_motion", listen_check):
            run_world(duration_ms, scenario=scenario)
        return [SafetyEvent.from_checks(a, checked) for a, checked in ticks]

    def test_every_tick_of_every_demo_is_valid(self):
        from state_machine.demos import DEMOS
        for name in DEMOS:
            with self.subTest(demo=name):
                events = self.run_demo(name)
                self.assertTrue(events)
                for event in events:
                    self.assertIsNone(event.validate(), (name, event.timestamp_ns))
                    json.dumps(event.to_dict())

    def test_emergency_demo_records_the_emergency(self):
        events = self.run_demo("emergency")
        self.assertTrue(any(e.emergency and "critical_battery" in e.reasons for e in events))
        self.assertTrue(any(c.sent_kind == "EMERGENCY_LAND" for e in events for c in e.commands))


if __name__ == "__main__":
    unittest.main()
