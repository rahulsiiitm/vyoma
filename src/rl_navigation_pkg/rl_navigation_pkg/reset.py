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
        return math.hypot(self.velocity_x, self.velocity_y, self.velocity_z)


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
        if not math.isfinite(maximum_speed) or maximum_speed < 0.0:
            raise ValueError("maximum_speed must be finite and nonnegative")
        self.initial_sequence = initial_sequence
        self.last_sequence = initial_sequence
        self.maximum_speed = maximum_speed
        self.required_stable_samples = required_stable_samples
        self.stable_samples = 0

    def observe(self, motion: VehicleMotion) -> bool:
        if motion.sequence <= self.last_sequence:
            return False
        self.last_sequence = motion.sequence
        if not math.isfinite(motion.speed) or motion.speed > self.maximum_speed:
            self.stable_samples = 0
            return False
        self.stable_samples += 1
        return self.stable_samples >= self.required_stable_samples
