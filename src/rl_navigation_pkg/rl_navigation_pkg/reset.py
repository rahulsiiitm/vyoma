"""Reset acceptance logic shared by training and integration tests."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VehicleMotion:
    sequence: int
    velocity_x: float
    velocity_y: float
    velocity_z: float

    @property
    def speed(self) -> float:
        return math.sqrt(
            self.velocity_x ** 2 + self.velocity_y ** 2 + self.velocity_z ** 2
        )


class ResetValidator:
    """Accept a reset only after fresh, consecutively settled odometry."""

    def __init__(
        self,
        initial_sequence: int,
        maximum_speed: float = 0.15,
        required_stable_samples: int = 5,
    ) -> None:
        if required_stable_samples < 1:
            raise ValueError("required_stable_samples must be positive")
        self.initial_sequence = initial_sequence
        self.maximum_speed = maximum_speed
        self.required_stable_samples = required_stable_samples
        self.stable_samples = 0

    def observe(self, motion: VehicleMotion) -> bool:
        if motion.sequence <= self.initial_sequence:
            return False
        if not math.isfinite(motion.speed) or motion.speed > self.maximum_speed:
            self.stable_samples = 0
            return False
        self.stable_samples += 1
        return self.stable_samples >= self.required_stable_samples
