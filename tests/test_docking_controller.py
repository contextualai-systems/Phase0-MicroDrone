from dataclasses import replace
import unittest

from docking.states import (
    DockingConfig, DockingController, DockingInput, DockingState, PadObservation,
    compute_return_request,
)
from motion_engine.contracts import CommandStatus, MotionCommandType, Pose
from motion_engine.motion_stubs import MotionStub
from safety_layer.safety_policy import SafetyAssessment

NS = 1_000_000_000


def fixture(now_ns=0, pose=Pose(0, 0, 5)):
    vehicle = replace(MotionStub().feedback(now_ns), pose=pose, armed=True,
                      grounded=pose.z <= 0.05, airborne=pose.z > 0.05)
    return DockingInput(
        mission_state='DOCKING_APPROACH', flight_active=True, vehicle=vehicle,
        pad=PadObservation(Pose(), now_ns),
        safety=SafetyAssessment(
            {'approach': True, 'descent': True, 'hold': True, 'tracking': True},
            timestamp_ns=now_ns),
        alignment_valid=True, alignment_timestamp_ns=now_ns,
        x_error_m=-pose.x, y_error_m=-pose.y, yaw_error_deg=-pose.yaw_deg,
        cart_moving=False, environment_valid=True, environment_timestamp_ns=now_ns,
        bird_valid=True, bird_detected=True, bird_timestamp_ns=now_ns,
    )


class DockingControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = DockingController()

    def reach_descent(self):
        for expected in (DockingState.APPROACH, DockingState.ALIGN, DockingState.DESCEND):
            result = self.controller.update(fixture(), 0)
            self.assertEqual(result.state, expected)
        return result

    def landed_fixture(self, now_ns=0):
        inputs = fixture(now_ns, Pose())
        return replace(inputs, vehicle=replace(
            inputs.vehicle, landing_settled=True, status=CommandStatus.LANDED,
            active_kind='DESCEND_TO_DOCK', active_command_id='landing', target=Pose()))

    def test_defaults_and_one_transition_per_tick(self):
        self.assertEqual(self.controller.config.alignment_tolerance_m, 0.25)
        self.assertEqual(self.controller.config.yaw_tolerance_deg, 3)
        self.assertEqual(self.controller.config.search_timeout_ns, 5 * NS)
        self.assertEqual(self.controller.config.attempt_timeout_ns, 30 * NS)
        self.reach_descent()
        result = self.controller.update(self.landed_fixture(), 0)
        self.assertEqual(result.state, DockingState.COMPLETE)
        self.assertTrue(result.landed_on_pad)
        self.assertIsNone(result.proposed_target)
        self.assertEqual([r.state for r in self.controller.transitions],
                         [DockingState.APPROACH, DockingState.ALIGN,
                          DockingState.DESCEND, DockingState.COMPLETE])
        self.assertTrue(all(r.reason and type(r.timestamp_ns) is int
                            for r in self.controller.transitions))

    def test_approach_requires_position_altitude_and_yaw(self):
        self.controller.update(fixture(), 0)
        for pose in (Pose(0.3, 0, 5), Pose(0, 0, 4.7), Pose(0, 0, 5, 3.1)):
            with self.subTest(pose=pose):
                result = self.controller.update(fixture(pose=pose), 0)
                self.assertEqual(result.state, DockingState.APPROACH)
        inputs = replace(fixture(pose=Pose(0.25, 0, 4.75, 357)), alignment_valid=False)
        result = self.controller.update(inputs, 0)
        self.assertEqual(result.state, DockingState.SEARCH)
        self.assertEqual(result.proposed_target.kind, MotionCommandType.HOVER)

    def test_align_horizontal_and_wrapped_yaw_tolerances(self):
        self.controller.update(fixture(), 0)
        self.controller.update(fixture(), 0)
        for changes in ({'x_error_m': 0.2, 'y_error_m': 0.2}, {'yaw_error_deg': 3.01}):
            self.controller.update(replace(fixture(), **changes), 0)
            self.assertEqual(self.controller.state, DockingState.ALIGN)
        self.controller.update(replace(fixture(), x_error_m=0.25, yaw_error_deg=357), 0)
        self.assertEqual(self.controller.state, DockingState.DESCEND)

    def test_descent_permission_blocks_align_and_revocation_aborts(self):
        inputs = fixture()
        blocked = replace(inputs, safety=replace(inputs.safety,
                          permissions={**inputs.safety.permissions, 'descent': False}))
        self.controller.update(inputs, 0)
        self.controller.update(inputs, 0)
        result = self.controller.update(blocked, 0)
        self.assertEqual(result.state, DockingState.ALIGN)
        self.assertNotEqual(result.proposed_target.kind, MotionCommandType.DESCEND_TO_DOCK)
        self.controller.update(inputs, 0)
        result = self.controller.update(blocked, 0)
        self.assertEqual(result.state, DockingState.ABORT)
        self.assertEqual(result.failure_reason, 'descent_permission_lost')
        self.assertIsNone(result.proposed_target)

    def test_horizontal_or_yaw_drift_aborts_descent(self):
        for changes in ({'x_error_m': 0.26}, {'yaw_error_deg': 3.1}):
            with self.subTest(changes=changes):
                self.controller = DockingController()
                self.reach_descent()
                result = self.controller.update(replace(fixture(), **changes), 0)
                self.assertEqual(result.state, DockingState.ABORT)
                self.assertIsNone(result.proposed_target)

    def test_stale_missing_future_and_malformed_alignment(self):
        for changes in (
            {'alignment_valid': False}, {'alignment_valid': 'true'},
            {'alignment_timestamp_ns': None}, {'alignment_timestamp_ns': True},
            {'alignment_timestamp_ns': NS + 1}, {'alignment_timestamp_ns': NS - 500_000_001},
            {'x_error_m': None}, {'x_error_m': float('nan')}, {'y_error_m': '1'},
            {'yaw_error_deg': float('inf')},
        ):
            with self.subTest(changes=changes):
                self.controller = DockingController()
                self.reach_descent()
                result = self.controller.update(replace(fixture(NS), **changes), NS)
                self.assertEqual(result.state, DockingState.ABORT)
        self.controller = DockingController()
        self.reach_descent()
        result = self.controller.update(
            replace(fixture(NS), alignment_timestamp_ns=NS - 500_000_000), NS)
        self.assertEqual(result.state, DockingState.DESCEND)

    def test_search_timeout_boundary_wins_over_new_alignment(self):
        self.controller.update(fixture(), 0)
        self.controller.update(replace(fixture(), alignment_valid=False), 0)
        self.controller.update(replace(fixture(5 * NS - 1), alignment_valid=False), 5 * NS - 1)
        self.assertEqual(self.controller.state, DockingState.SEARCH)
        result = self.controller.update(fixture(5 * NS), 5 * NS)
        self.assertEqual(result.failure_reason, 'alignment_search_timeout')

    def test_attempt_timer_survives_repeated_search_entries(self):
        self.controller.update(fixture(), 0)
        self.controller.update(replace(fixture(NS), alignment_valid=False), NS)
        self.controller.update(fixture(2 * NS), 2 * NS)
        self.controller.update(replace(fixture(3 * NS), alignment_valid=False), 3 * NS)
        self.assertEqual(self.controller._attempt_started_ns, 0)
        self.assertEqual(self.controller._search_started_ns, 3 * NS)
        self.controller.update(fixture(4 * NS), 4 * NS)
        result = self.controller.update(fixture(30 * NS), 30 * NS)
        self.assertEqual(result.failure_reason, 'docking_attempt_timeout')
        self.assertIsNone(result.proposed_target)

    def test_approach_loss_and_emergency_abort_every_active_phase(self):
        for phase_steps in range(4):
            for hazard in ('approach', 'emergency', 'abort'):
                with self.subTest(steps=phase_steps, hazard=hazard):
                    self.controller = DockingController()
                    for _ in range(phase_steps):
                        self.controller.update(fixture(), 0)
                    inputs = fixture()
                    safety = inputs.safety
                    if hazard == 'approach':
                        safety = replace(safety, permissions={**safety.permissions, 'approach': False})
                    else:
                        safety = replace(safety, **{hazard: True})
                    result = self.controller.update(replace(inputs, safety=safety), 0)
                    self.assertTrue(result.failure)
                    self.assertFalse(result.landed_on_pad)
                    self.assertIsNone(result.proposed_target)

    def test_malformed_or_stale_required_inputs_fail_closed(self):
        original = fixture(NS)
        for changes in (
            {'vehicle': replace(original.vehicle, timestamp_ns=0)}, {'vehicle': None},
            {'pad': replace(original.pad, frame='local_ned')}, {'pad': None},
            {'pad': replace(original.pad, pose=Pose(z=1))},
            {'cart_moving': True}, {'cart_moving': 'false'},
            {'environment_valid': False}, {'environment_timestamp_ns': 0},
            {'safety': None}, {'safety': replace(original.safety, timestamp_ns=0)},
            {'safety': replace(original.safety, permissions={'approach': 'true'})},
        ):
            with self.subTest(changes=changes):
                result = DockingController().update(replace(original, **changes), NS)
                self.assertTrue(result.failure)
                self.assertIsNone(result.proposed_target)

    def test_feedback_confirmation_rejects_arrival_and_invalid_touchdown(self):
        good = self.landed_fixture()
        for changes in (
            {'pose': Pose(z=0.2), 'airborne': True, 'grounded': False},
            {'pose': Pose(z=0.01)}, {'pose': Pose(x=0.251)},
            {'landing_settled': False}, {'active_kind': 'MOVE_TO'},
            {'active_kind': 'EMERGENCY_LAND'}, {'active_command_id': None},
            {'target': Pose(z=0.1)}, {'target': Pose(x=1)}, {'valid': False},
            {'status': CommandStatus.ARRIVED},
        ):
            with self.subTest(changes=changes):
                self.controller = DockingController()
                self.reach_descent()
                inputs = replace(good, vehicle=replace(good.vehicle, arrived=True, **changes))
                result = self.controller.update(inputs, 0)
                self.assertFalse(result.landed_on_pad)
                self.assertNotEqual(result.state, DockingState.COMPLETE)
        self.controller = DockingController()
        self.reach_descent()
        moving = replace(good, cart_moving=True)
        self.assertTrue(self.controller.update(moving, 0).failure)

    def test_command_proposals_validate_and_renew_without_restarting(self):
        first = self.controller.update(fixture(pose=Pose(5, 0, 5)), 0)
        command = first.proposed_target
        self.assertIsNone(command.validate())
        self.assertEqual(command.target, Pose(0, 0, 5))
        self.assertEqual(command.frame, 'local_enu')
        self.assertEqual(command.expires_ns, 200_000_000)
        second = self.controller.update(fixture(100_000_000, Pose(4, 0, 5)), 100_000_000)
        self.assertEqual(command.command_id, second.proposed_target.command_id)
        self.assertGreater(second.proposed_target.expires_ns, command.expires_ns)
        self.assertEqual(second.attempt_started_ns, 0)

    def test_alignment_loss_returns_to_search_before_descent(self):
        self.controller.update(fixture(), 0)
        self.controller.update(fixture(), 0)
        result = self.controller.update(replace(fixture(NS), alignment_valid=False), NS)
        self.assertEqual(result.state, DockingState.SEARCH)
        self.assertEqual(result.search_started_ns, NS)
        self.assertEqual(result.proposed_target.kind, MotionCommandType.HOVER)

    def test_terminal_completion_latches_until_mission_exit(self):
        self.reach_descent()
        self.controller.update(self.landed_fixture(), 0)
        result = self.controller.update(DockingInput(mission_state='DOCKING_APPROACH'), NS)
        self.assertTrue(result.landed_on_pad)
        self.assertIsNone(result.proposed_target)
        result = self.controller.update(DockingInput(mission_state='DOCKED'), NS)
        self.assertEqual(result.phase, DockingState.IDLE)
        self.assertFalse(result.landed_on_pad)

    def test_retry_does_not_mutate_shared_return_timers(self):
        self.reach_descent()
        self.controller.update(replace(fixture(), alignment_valid=False), 0)
        inputs = replace(fixture(20 * NS), retry_requested=True,
                         bird_detected=False, bird_absent_since_ns=0, hover_started_ns=0)
        result = self.controller.update(inputs, 20 * NS)
        self.assertEqual(result.attempt_started_ns, 20 * NS)
        self.assertEqual(inputs.bird_absent_since_ns, 0)
        self.assertEqual(inputs.hover_started_ns, 0)
        self.assertEqual(result.reasons, ('bird_lost',))

    def test_invalid_mission_state_reports_failure_without_target(self):
        result = self.controller.update(replace(fixture(), mission_state='LANDING'), 0)
        self.assertEqual(result.failure_reason, 'invalid_mission_state')
        self.assertIsNone(result.proposed_target)

    def test_exit_retry_and_reset_are_explicit_and_local(self):
        self.reach_descent()
        bad = replace(fixture(), alignment_valid=False)
        self.controller.update(bad, 0)
        self.assertEqual(self.controller.update(fixture(NS), NS).state, DockingState.ABORT)
        retried = self.controller.update(replace(fixture(2 * NS), retry_requested=True), 2 * NS)
        self.assertEqual(retried.state, DockingState.APPROACH)
        self.assertEqual(retried.attempt_started_ns, 2 * NS)
        stopped = self.controller.update(replace(fixture(3 * NS), mission_state='ABORT'), 3 * NS)
        self.assertEqual(stopped.state, DockingState.IDLE)
        self.assertFalse(stopped.failure)
        self.assertIsNone(stopped.proposed_target)
        self.assertIsNone(stopped.attempt_started_ns)
        self.controller.update(fixture(4 * NS), 4 * NS)
        rejected = self.controller.update(replace(fixture(4 * NS), reset_requested=True), 4 * NS)
        self.assertEqual(rejected.failure_reason, 'reset_rejected')
        inputs = self.landed_fixture(5 * NS)
        inputs = replace(inputs, vehicle=replace(inputs.vehicle, armed=False), reset_requested=True)
        reset = self.controller.update(inputs, 5 * NS)
        self.assertEqual(reset.state, DockingState.IDLE)
        self.assertIsNone(reset.proposed_target)

    def test_targets_only_in_docking_mission_and_low_battery_is_return_not_abort(self):
        inputs = fixture()
        low = replace(inputs.safety, battery_low=True, reasons=('low_battery',),
                      permissions={**inputs.safety.permissions, 'tracking': False})
        result = self.controller.update(replace(inputs, mission_state='TRACKING', safety=low), 0)
        self.assertTrue(result.should_dock)
        self.assertEqual(result.reasons, ('low_battery',))
        self.assertEqual(result.phase, DockingState.IDLE)
        self.assertIsNone(result.proposed_target)
        result = self.controller.update(replace(inputs, safety=low), 0)
        self.assertEqual(result.state, DockingState.APPROACH)
        self.assertFalse(result.failure)

    def test_return_bird_absence_hover_timeout_and_reappearance(self):
        config = DockingConfig()
        for now_ns, expected in ((3 * NS - 1, False), (3 * NS, True)):
            inputs = replace(fixture(now_ns), mission_state='TRACKING',
                             bird_detected=False, bird_absent_since_ns=0)
            self.assertEqual(compute_return_request(inputs, now_ns, config).should_dock, expected)
        inputs = replace(fixture(3 * NS), bird_absent_since_ns=0)
        self.assertFalse(compute_return_request(inputs, 3 * NS, config).should_dock)
        inputs = replace(fixture(10 * NS), mission_state='HOVERING', hover_started_ns=0)
        self.assertEqual(compute_return_request(inputs, 10 * NS, config).reasons, ('hover_timeout',))
        self.assertFalse(compute_return_request(replace(inputs, hover_started_ns=None),
                                               10 * NS, config).should_dock)
        inputs = replace(inputs, bird_valid=False)
        self.assertIn('camera_failure', compute_return_request(inputs, 10 * NS, config).reasons)
        self.assertFalse(compute_return_request(replace(inputs, flight_active=False),
                                               10 * NS, config).should_dock)

    def test_return_restrictions_preserve_order_without_battery_thresholds(self):
        inputs = fixture()
        safety = replace(inputs.safety, battery_low=True,
                         permissions={**inputs.safety.permissions, 'tracking': False},
                         reasons=('people_nearby', 'cart_moving', 'low_battery'))
        request = compute_return_request(replace(inputs, safety=safety), 0, DockingConfig())
        self.assertEqual(request.reasons, ('low_battery', 'people_nearby', 'cart_moving'))
        geometry = replace(safety, battery_low=False, reasons=('tracking_geometry_unavailable',))
        self.assertFalse(compute_return_request(replace(inputs, safety=geometry),
                                               0, DockingConfig()).should_dock)

    def test_invalid_config_and_clock_do_not_change_state(self):
        for name, value in (('attempt_timeout_ns', 1.0), ('search_timeout_ns', True),
                            ('alignment_tolerance_m', float('nan')), ('yaw_tolerance_deg', 181)):
            with self.assertRaises(ValueError):
                DockingConfig(**{name: value})
        self.controller.update(fixture(NS), NS)
        for time in (True, 1.0, -1, NS - 1):
            with self.assertRaises(ValueError):
                self.controller.update(fixture(), time)
            self.assertEqual(self.controller.state, DockingState.APPROACH)

    def test_motion_feedback_end_to_end_confirms_landing_before_disarm(self):
        motion = MotionStub()
        motion.arm()
        motion.takeoff(5)
        motion.step(3)
        motion.move_to(2, 0, 5)
        motion.step(1)
        final = None
        for tick in range(100):
            now = tick * 100_000_000
            feedback = motion.feedback(now)
            inputs = fixture(now, feedback.pose)
            inputs = replace(inputs, vehicle=feedback)
            final = self.controller.update(inputs, now)
            if final.landed_on_pad:
                break
            self.assertFalse(final.failure, final.reason)
            self.assertIsNone(final.proposed_target.validate())
            self.assertTrue(motion.submit(final.proposed_target, now).accepted)
            motion.step(0.1, now)
        self.assertTrue(final.landed_on_pad)
        self.assertTrue(motion.armed)
        self.assertTrue(motion.disarm())
        self.assertFalse(motion.armed)


if __name__ == '__main__':
    unittest.main()
