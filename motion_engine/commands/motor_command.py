from dataclasses import dataclass


@dataclass(frozen=True)
class MotorCommand:
    motor_id: str
    output: float = 0.0
