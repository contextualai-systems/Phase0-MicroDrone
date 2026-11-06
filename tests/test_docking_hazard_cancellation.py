"""Docking → Safety → Motion checks with a small coordinator test fixture.

The fixture supplies abort handling; this is not a production mission runner.
In particular, a null Docking proposal does not cancel Motion's active target.
"""

from dataclasses import replace
import unittest

from docking.alignment import LocalPosition, calculate_alignment
from docking.states import DockingController, DockingState, PadObservation
from motion_engine.contracts import Command, CommandStatus, MotionCommandType, Pose
from motion_engine.motion_stubs import MotionStub
from motion_engine.safety_adapter import check_for_motion
from safety_layer.command_check import ACTION_NOT_PERMITTED
from safety_layer.safety_policy import assess


DT_NS = 100_000_000
DT = 0.1
PAD = Pose()


class DockingHazardCancellationTests(unittest.TestCase):
    def reset_scenario(self):
        self.controller = DockingController()
        self.motion = MotionStub()
        # Setup only: begin each docking attempt above the stationary pad.
        self.assertTrue(self.motion.arm())
        self.assertTrue(self.motion.takeoff(5))
        self.motion.step(3)
        self.now_ns = 3_000_000_000
        self.active_command = None

    def inputs(self, hazard=None):
        vehicle = self.motion.feedback(self.now_ns)
        stamp = self.now_ns
        if hazard == 'stale_alignment':
            stamp -= self.controller.config.max_observation_age_ns + 1
        alignment = calculate_alignment(
            LocalPosition(0, 0, 'test_world'),
            LocalPosition(vehicle.pose.x, vehicle.pose.y, 'test_world',
                          yaw_deg=vehicle.pose.yaw_deg),
            stamp,
        )
        cart_moving = hazard == 'cart_moving'
        snapshot = {
            'vehicle': vehicle.to_dict(),
            'battery': {'valid': True, 'timestamp_ns': self.now_ns, 'percent': 80},
            'environment': {
                'valid': True, 'timestamp_ns': self.now_ns,
                'cart_moving': cart_moving, 'people_nearby': False,
                'weather': 'clear', 'comm_ok': True, 'camera_ok': True,
            },
            'dock': {'valid': True, 'timestamp_ns': self.now_ns,
                     'alignment_valid': alignment.valid},
            'context': {'flight_active': True},
        }
        safety = assess(snapshot, self.now_ns)
        if hazard == 'descent_revoked':
            # An explicit permission fixture independent of alignment loss.
            safety = replace(safety, permissions={**safety.permissions, 'descent': False})
        self.assertFalse(safety.emergency)
        self.assertTrue(safety.allows('hold'))
        return replace(
            alignment.to_docking_input(), mission_state='DOCKING_APPROACH',
            flight_active=True, vehicle=vehicle,
            pad=PadObservation(PAD, self.now_ns), safety=safety,
            cart_moving=cart_moving, environment_valid=True,
            environment_timestamp_ns=self.now_ns,
            bird_valid=True, bird_detected=True, bird_timestamp_ns=self.now_ns,
        )

    def dispatch_and_step(self, command, safety):
        permitted, decision = check_for_motion(command, safety, self.now_ns)
        ack = self.motion.submit(permitted, self.now_ns)
        self.assertTrue(ack.accepted, ack.reason)
        self.active_command = permitted
        self.motion.step(DT, self.now_ns)
        self.now_ns += DT_NS
        return decision

    def start_descent(self, near_touchdown):
        self.reset_scenario()
        for phase in (DockingState.APPROACH, DockingState.ALIGN, DockingState.DESCEND):
            inputs = self.inputs()
            update = self.controller.update(inputs, self.now_ns)
            self.assertEqual(update.phase, phase)
            decision = self.dispatch_and_step(update.proposed_target, inputs.safety)
            self.assertTrue(decision.allowed, decision.reasons)
        # Let a real descent run before injecting the hazard. Also exercise
        # the last airborne tick, when arrival tolerance alone looks complete.
        while near_touchdown and self.motion.pose.z > 0.21:
            inputs = self.inputs()
            update = self.controller.update(inputs, self.now_ns)
            self.assertEqual(update.phase, DockingState.DESCEND)
            self.dispatch_and_step(update.proposed_target, inputs.safety)
        vehicle = self.motion.feedback(self.now_ns)
        self.assertTrue(vehicle.airborne)
        self.assertFalse(vehicle.landing_settled)
        self.assertLess(vehicle.pose.z, 5)
        self.assertGreater(self.active_command.expires_ns, self.now_ns)
        if near_touchdown:
            self.assertTrue(vehicle.arrived)

    def verify_cancellation(self, hazard, near_touchdown):
        self.start_descent(near_touchdown)
        before = self.motion.feedback(self.now_ns)
        old_descent = self.active_command
        inputs = self.inputs(hazard)
        update = self.controller.update(inputs, self.now_ns)
        self.assertEqual(update.phase, DockingState.ABORT)
        expected = ('approach_permission_lost' if hazard == 'cart_moving'
                    else 'descent_permission_lost')
        self.assertEqual(update.failure_reason, expected)
        self.assertTrue(update.failure)
        self.assertFalse(update.landed_on_pad)
        self.assertIsNone(update.proposed_target)

        # Inspect the still-live descent, even though Docking proposes nothing.
        replacement, decision = check_for_motion(old_descent, inputs.safety, self.now_ns)
        if hazard == 'stale_alignment':
            # Safety currently checks the pad section's age, not the separate
            # alignment timestamp. Docking's abort must still cancel descent.
            self.assertTrue(inputs.alignment_valid)
            replacement = Command(
                'test-abort-hold', MotionCommandType.STOP, 'ABORT', self.now_ns,
                self.now_ns + 200_000_000, 'state_machine', update.failure_reason,
            )
        else:
            self.assertFalse(decision.allowed)
            self.assertIn(ACTION_NOT_PERMITTED, decision.reasons)
            self.assertEqual(replacement.kind, MotionCommandType.HOVER)

        # The coordinator fixture dispatches the approved replacement BEFORE
        # Motion steps. The old command is not expired yet, so expiry cannot
        # accidentally make this assertion pass.
        hold_decision = self.dispatch_and_step(replacement, inputs.safety)
        self.assertTrue(hold_decision.allowed, hold_decision.reasons)
        after = self.motion.feedback(self.now_ns)
        self.assertEqual(after.pose, before.pose)
        self.assertNotEqual(after.active_command_id, old_descent.command_id)
        self.assertIsNone(replacement.target)
        self.assertEqual(after.target, before.pose)
        self.assertIn(after.active_kind, (MotionCommandType.HOVER, MotionCommandType.STOP))
        self.assertTrue(after.airborne)
        self.assertFalse(after.grounded)
        self.assertFalse(after.landing_settled)
        self.assertNotEqual(after.status, CommandStatus.LANDED)

        # Keep the hold renewed beyond the old descent's expiry. Clearing the
        # hazard alone must not release Docking's abort or restart descent.
        for _ in range(5):
            clear = self.inputs()
            latched = self.controller.update(clear, self.now_ns)
            self.assertEqual(latched.phase, DockingState.ABORT)
            self.assertFalse(latched.landed_on_pad)
            self.assertIsNone(latched.proposed_target)
            hold = replace(self.active_command, issued_ns=self.now_ns,
                           expires_ns=self.now_ns + 200_000_000)
            self.dispatch_and_step(hold, clear.safety)
            self.assertEqual(self.motion.pose, before.pose)

    def test_cart_motion_cancels_descent_before_movement(self):
        for near in (False, True):
            with self.subTest(near_touchdown=near):
                self.verify_cancellation('cart_moving', near)

    def test_stale_alignment_cancels_descent_before_movement(self):
        for near in (False, True):
            with self.subTest(near_touchdown=near):
                self.verify_cancellation('stale_alignment', near)

    def test_revoked_descent_cancels_before_movement(self):
        for near in (False, True):
            with self.subTest(near_touchdown=near):
                self.verify_cancellation('descent_revoked', near)

    def test_null_docking_proposal_does_not_cancel_active_descent(self):
        self.start_descent(near_touchdown=False)
        before = self.motion.pose
        update = self.controller.update(self.inputs('stale_alignment'), self.now_ns)
        self.assertTrue(update.failure)
        self.assertIsNone(update.proposed_target)
        # Negative control: simply omitting dispatch leaves the old target live.
        self.motion.step(DT, self.now_ns)
        self.assertLess(self.motion.pose.z, before.z)
        self.assertEqual(self.motion.feedback(self.now_ns).active_command_id,
                         self.active_command.command_id)


if __name__ == '__main__':
    unittest.main()
