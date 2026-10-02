from dataclasses import replace
import unittest

from docking.states import DockingConfig, DockingController, DockingInput, DockingState


class DockingControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = DockingController()
        self.aligned = DockingInput(
            alignment_valid=True,
            alignment_timestamp_s=1.0,
            north_error_m=0.0,
            east_error_m=0.0,
        )

    def reach_search(self):
        self.controller.update(DockingInput(docking_requested=True), now_s=0.0)
        self.controller.update(DockingInput(near_pad=True), now_s=1.0)

    def reach_descent(self):
        self.reach_search()
        self.controller.update(self.aligned, now_s=1.0)
        self.controller.update(self.aligned, now_s=1.0)
        self.assertEqual(self.controller.state, DockingState.DESCEND)

    def test_waits_for_request_and_navigation(self):
        self.assertEqual(self.controller.state, DockingState.IDLE)
        self.controller.update(self.aligned, now_s=0.0)
        self.assertEqual(self.controller.state, DockingState.IDLE)
        self.controller.update(DockingInput(docking_requested=True), now_s=0.0)
        self.controller.update(self.aligned, now_s=1.0)
        self.assertEqual(self.controller.state, DockingState.APPROACH)

    def test_normal_sequence_records_reasons_and_confirmed_landing(self):
        self.reach_descent()
        self.controller.update(self.aligned, now_s=1.0)
        self.assertEqual(self.controller.state, DockingState.DESCEND)
        self.controller.update(
            replace(self.aligned, landing_confirmed=True), now_s=1.0
        )
        self.assertEqual(self.controller.state, DockingState.COMPLETE)
        transitions = self.controller.transitions
        self.assertEqual(
            [event.state for event in transitions],
            [DockingState.APPROACH, DockingState.SEARCH, DockingState.ALIGN,
             DockingState.DESCEND, DockingState.COMPLETE],
        )
        self.assertEqual(transitions[0].previous_state, DockingState.IDLE)
        for previous, current in zip(transitions, transitions[1:]):
            self.assertEqual(current.previous_state, previous.state)
        self.assertTrue(all(event.reason for event in transitions))
        self.assertEqual([event.timestamp_s for event in transitions], [0, 1, 1, 1, 1])

    def test_invalid_alignment_blocks_descent_and_aborts_active_descent(self):
        invalid_inputs = [
            DockingInput(),
            replace(self.aligned, alignment_valid=False),
            replace(self.aligned, alignment_timestamp_s=None),
            replace(self.aligned, alignment_timestamp_s=0.49),
            replace(self.aligned, alignment_timestamp_s=1.01),
            replace(self.aligned, alignment_timestamp_s=-0.1),
            replace(self.aligned, alignment_timestamp_s=float("nan")),
            replace(self.aligned, north_error_m=None),
            replace(self.aligned, east_error_m=float("inf")),
            replace(self.aligned, north_error_m=float("nan")),
        ]
        for inputs in invalid_inputs:
            with self.subTest(inputs=inputs):
                self.controller = DockingController()
                self.reach_search()
                self.controller.update(inputs, now_s=1.0)
                self.assertEqual(self.controller.state, DockingState.SEARCH)
                self.controller.update(self.aligned, now_s=1.0)
                self.controller.update(inputs, now_s=1.0)
                self.assertEqual(self.controller.state, DockingState.SEARCH)
                self.controller.update(self.aligned, now_s=1.0)
                self.controller.update(self.aligned, now_s=1.0)
                result = self.controller.update(
                    replace(inputs, landing_confirmed=True), now_s=1.0
                )
                self.assertEqual(result.state, DockingState.ABORT)
                self.assertIn("Alignment unavailable", result.reason)

    def test_tolerance_uses_horizontal_distance_and_accepts_boundary(self):
        self.controller = DockingController(DockingConfig(alignment_tolerance_m=0.5))
        self.reach_search()
        self.controller.update(self.aligned, now_s=1.0)
        self.controller.update(
            replace(self.aligned, north_error_m=0.4, east_error_m=0.4), now_s=1.0
        )
        self.assertEqual(self.controller.state, DockingState.ALIGN)
        self.controller.update(
            replace(self.aligned, north_error_m=-0.3, east_error_m=0.4), now_s=1.0
        )
        self.assertEqual(self.controller.state, DockingState.DESCEND)

    def test_freshness_limit_is_configurable_and_inclusive(self):
        self.controller = DockingController(DockingConfig(max_alignment_age_s=0.25))
        self.reach_search()
        self.controller.update(self.aligned, now_s=1.25)
        self.assertEqual(self.controller.state, DockingState.ALIGN)
        self.controller.update(self.aligned, now_s=1.251)
        self.assertEqual(self.controller.state, DockingState.SEARCH)

    def test_drift_during_descent_requires_realignment(self):
        self.reach_descent()
        self.controller.update(
            replace(self.aligned, north_error_m=1.0, landing_confirmed=True),
            now_s=1.0,
        )
        self.assertEqual(self.controller.state, DockingState.ALIGN)

    def test_safety_abort_takes_priority_in_every_active_state(self):
        for steps in range(5):
            with self.subTest(steps=steps):
                self.controller = DockingController()
                sequence = [DockingInput(docking_requested=True),
                            DockingInput(near_pad=True), self.aligned, self.aligned]
                for inputs in sequence[:steps]:
                    self.controller.update(inputs, now_s=1.0)
                result = self.controller.update(
                    replace(self.aligned, landing_confirmed=True,
                            abort_reason="low battery"), now_s=1.0
                )
                self.assertEqual(result.state, DockingState.ABORT)
                self.assertEqual(result.reason, "Safety abort: low battery")

    def test_terminal_states_do_not_restart(self):
        for terminal in (DockingState.COMPLETE, DockingState.ABORT):
            with self.subTest(terminal=terminal):
                self.controller = DockingController()
                self.reach_descent()
                self.controller.update(
                    replace(self.aligned, landing_confirmed=True,
                            abort_reason="operator" if terminal == DockingState.ABORT
                            else None), now_s=1.0
                )
                count = len(self.controller.transitions)
                self.controller.update(DockingInput(docking_requested=True), now_s=2.0)
                self.assertEqual(self.controller.state, terminal)
                self.assertEqual(len(self.controller.transitions), count)

    def test_landing_confirmation_does_not_skip_states(self):
        inputs = replace(self.aligned, docking_requested=True, near_pad=True,
                         landing_confirmed=True)
        for expected in (DockingState.APPROACH, DockingState.SEARCH,
                         DockingState.ALIGN, DockingState.DESCEND, DockingState.COMPLETE):
            self.controller.update(inputs, now_s=1.0)
            self.assertEqual(self.controller.state, expected)

    def test_invalid_thresholds_are_rejected(self):
        for field in ("alignment_tolerance_m", "max_alignment_age_s"):
            for value in (0, -1, float("nan"), float("inf")):
                with self.subTest(field=field, value=value):
                    with self.assertRaises(ValueError):
                        DockingConfig(**{field: value})

    def test_invalid_clock_does_not_change_state(self):
        self.controller.update(DockingInput(), now_s=1.0)
        for now_s in (-1, 0.5, float("nan"), float("inf")):
            with self.subTest(now_s=now_s):
                with self.assertRaises(ValueError):
                    self.controller.update(DockingInput(docking_requested=True), now_s)
                self.assertEqual(self.controller.state, DockingState.IDLE)
                self.assertEqual(self.controller.transitions, ())


if __name__ == "__main__":
    unittest.main()
