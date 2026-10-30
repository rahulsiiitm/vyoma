"""Reset acceptance logic shared by training and integration tests."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VehicleMotion:
    sequence: int
    velocity_x: float
    velocity_y: float
    velocity_z: float
    position_x: float = 0.0
    position_y: float = 0.0
    position_z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    angular_speed: float = 0.0
    timestamp_us: int = 0

    @property
    def speed(self) -> float:
        return math.hypot(self.velocity_x, self.velocity_y, self.velocity_z)


class ResetValidator:
    """Accept a reset only after fresh, level, consecutively settled odometry."""

    def __init__(
        self,
        initial_sequence: int,
        maximum_speed: float = 0.15,
        required_stable_samples: int = 30,
        minimum_stable_duration: float = 0.0,
    ) -> None:
        if required_stable_samples < 1:
            raise ValueError("required_stable_samples must be positive")
        if not math.isfinite(maximum_speed) or maximum_speed < 0.0:
            raise ValueError("maximum_speed must be finite and nonnegative")
        if not math.isfinite(minimum_stable_duration) or minimum_stable_duration < 0.0:
            raise ValueError("minimum_stable_duration must be finite and nonnegative")
        self.initial_sequence = initial_sequence
        self.last_sequence = initial_sequence
        self.maximum_speed = maximum_speed
        self.required_stable_samples = required_stable_samples
        self.stable_samples = 0
        self.minimum_stable_duration = minimum_stable_duration
        self.stable_since_us = None

    def observe(self, motion: VehicleMotion) -> bool:
        if motion.sequence <= self.last_sequence:
            return False
        self.last_sequence = motion.sequence
        position = (motion.position_x, motion.position_y, motion.position_z)
        if (not all(math.isfinite(value) for value in (*position, motion.roll,
                                                       motion.pitch, motion.angular_speed))
                or math.hypot(motion.position_x, motion.position_y) > 0.75
                or abs(motion.position_z) > 0.5
                or abs(motion.roll) > 0.25 or abs(motion.pitch) > 0.25
                or not 0.0 <= motion.angular_speed <= 0.25
                or not math.isfinite(motion.speed) or motion.speed > self.maximum_speed):
            self.stable_samples = 0
            self.stable_since_us = None
            return False
        if self.stable_since_us is None:
            self.stable_since_us = motion.timestamp_us
        if motion.timestamp_us < self.stable_since_us:
            self.stable_samples = 0
            self.stable_since_us = None
            return False
        self.stable_samples += 1
        return (self.stable_samples >= self.required_stable_samples and
                motion.timestamp_us - self.stable_since_us >= self.minimum_stable_duration * 1e6)
