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
    """The main loop builds one SafetyEvent per tick (result["safety_events"])."""

    def run_demo(self, name, safety_logger=None):
        from state_machine.demos import DEMOS
        from state_machine.run_loop import run_world

        scenario, duration_ms = DEMOS[name]
        return run_world(duration_ms, scenario=scenario, safety_logger=safety_logger)

    def test_one_valid_event_per_tick_in_every_demo(self):
        from state_machine.demos import DEMOS
        for name in DEMOS:
            with self.subTest(demo=name):
                result = self.run_demo(name)
                events = result["safety_events"]
                self.assertEqual(len(events), len(result["statuses"]))
                for event, status in zip(events, result["statuses"]):
                    self.assertIsNone(event.validate(), (name, event.timestamp_ns))
                    self.assertEqual(event.timestamp_ns, status.timestamp_ns)
                    json.dumps(event.to_dict())

    def test_emergency_demo_records_the_emergency(self):
        events = self.run_demo("emergency")["safety_events"]
        self.assertTrue(any(e.emergency and "critical_battery" in e.reasons for e in events))
        self.assertTrue(any(c.sent_kind == "EMERGENCY_LAND" for e in events for c in e.commands))

    def test_every_order_sent_to_motion_was_checked_by_safety(self):
        """Each command in the run results shows up as a Safety check that tick."""
        from state_machine.demos import DEMOS
        for name in DEMOS:
            with self.subTest(demo=name):
                result = self.run_demo(name)
                checked = {}
                for event in result["safety_events"]:
                    for check in event.commands:
                        checked.setdefault(event.timestamp_ns // 1_000_000, set()).add(check.sent_kind)
                for time_ms, kind, _accepted, _reason in result["commands"]:
                    kind = getattr(kind, "value", kind)
                    self.assertIn(kind, checked.get(time_ms, set()), (name, time_ms, kind))


class TestSafetyLog(unittest.TestCase):
    """What reaches the shared log (INTEGRATION_README.md section 10)."""

    def run_logged(self, name):
        from motion_engine.universal_logging import UniversalLogSink
        from state_machine.demos import DEMOS
        from state_machine.run_loop import run_world

        lines = []
        sink = UniversalLogSink(write=lines.append)
        scenario, duration_ms = DEMOS[name]
        result = run_world(duration_ms, scenario=scenario, safety_logger=sink)
        return result, [line["Details"] for line in lines]

    def test_log_lines_are_safety_events(self):
        result, lines = self.run_logged("full_cycle")
        self.assertTrue(lines)
        for line in lines:
            self.assertEqual(line["module"], "safety")
            self.assertEqual(line["event"], "safety_event")
            self.assertEqual(line["fields"]["source"], "safety")
            json.dumps(line)

    def test_every_override_is_logged(self):
        """An order Safety replaced always gets its own log line."""
        from state_machine.demos import DEMOS
        for name in DEMOS:
            with self.subTest(demo=name):
                result, lines = self.run_logged(name)
                logged_times = {line["fields"]["timestamp_ns"] for line in lines}
                for event in result["safety_events"]:
                    if any(not c.allowed for c in event.commands):
                        self.assertIn(event.timestamp_ns, logged_times)

    def test_every_change_in_safetys_answer_is_logged(self):
        result, lines = self.run_logged("emergency")
        logged = {line["fields"]["timestamp_ns"]: line["fields"] for line in lines}
        previous = None
        for event in result["safety_events"]:
            if previous is not None and event.reasons != previous.reasons:
                self.assertIn(event.timestamp_ns, logged)
                self.assertEqual(logged[event.timestamp_ns]["reasons"], list(event.reasons))
            previous = event

    def test_unchanged_ticks_are_summarized(self):
        result, lines = self.run_logged("full_cycle")
        self.assertLess(len(lines), len(result["safety_events"]) // 5)

    def test_log_time_matches_the_simulation_clock(self):
        result, lines = self.run_logged("full_cycle")
        for line in lines:
            self.assertAlmostEqual(line["t"] * 1e9, line["fields"]["timestamp_ns"], delta=1)

    def test_no_logger_means_no_logging(self):
        from state_machine.run_loop import run_world
        from state_machine.demos import DEMOS
        scenario, duration_ms = DEMOS["full_cycle"]
        result = run_world(duration_ms, scenario=scenario)  # must not raise
        self.assertTrue(result["safety_events"])


class TestWorthLogging(unittest.TestCase):
    """Which ticks go into the shared log."""

    def test_first_tick_is_always_logged(self):
        self.assertTrue(event_for(snapshot()).worth_logging(None))

    def test_same_answer_later_is_skipped(self):
        first = event_for(snapshot())
        later = replace(event_for(snapshot()), timestamp_ns=NOW + 100_000_000)
        self.assertFalse(later.worth_logging(first))

    def test_allowed_order_with_same_answer_is_skipped(self):
        first = event_for(snapshot(), good_order())
        again = event_for(snapshot(), good_order())
        self.assertTrue(again.commands[0].allowed)
        self.assertFalse(again.worth_logging(first))

    def test_replaced_order_is_always_logged(self):
        first = event_for(snapshot())
        replaced = event_for(snapshot(), good_order(expires_ns=NOW - 1))
        self.assertFalse(replaced.commands[0].allowed)
        self.assertTrue(replaced.worth_logging(first))
        self.assertTrue(replaced.worth_logging(replaced))  # even if the same as last time

    def test_new_reason_is_logged(self):
        first = event_for(snapshot())
        low = event_for(snapshot(battery={"percent": 15.0}))
        self.assertTrue(low.worth_logging(first))

    def test_reason_clearing_is_logged(self):
        low = event_for(snapshot(battery={"percent": 15.0}))
        clear = event_for(snapshot())
        self.assertTrue(clear.worth_logging(low))


if __name__ == "__main__":
    unittest.main()
