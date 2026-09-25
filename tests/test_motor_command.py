from dataclasses import FrozenInstanceError
import unittest

from motion_engine.commands import MotorCommand


class MotorCommandTests(unittest.TestCase):
    def test_motor_command_defaults_to_zero_output(self):
        command = MotorCommand(motor_id="front_left")

        self.assertEqual(command.motor_id, "front_left")
        self.assertEqual(command.output, 0.0)

    def test_motor_command_preserves_supplied_output(self):
        command = MotorCommand(motor_id="rear_right", output=0.5)

        self.assertEqual(command.motor_id, "rear_right")
        self.assertEqual(command.output, 0.5)

    def test_motor_command_cannot_be_changed_after_creation(self):
        command = MotorCommand(motor_id="front_left", output=0.5)

        with self.assertRaises(FrozenInstanceError):
            command.output = 1.0

        self.assertEqual(command.output, 0.5)


if __name__ == "__main__":
    unittest.main()
