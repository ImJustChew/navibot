import math
from dataclasses import dataclass

from navibot.robot.encoders import EncoderSample


@dataclass
class Pose2D:
    x_mm: float = 0.0
    y_mm: float = 0.0
    theta_rad: float = 0.0

    @property
    def theta_deg(self) -> float:
        return math.degrees(self.theta_rad)


@dataclass(frozen=True)
class DifferentialOdometryConfig:
    wheel_diameter_mm: float = 43.0
    wheel_track_mm: float = 64.0
    pulses_per_channel: int = 7
    gear_ratio: float = 132.0


class DifferentialOdometry:
    def __init__(self, config: DifferentialOdometryConfig) -> None:
        self._config = config
        self._pose = Pose2D()
        self._previous_left = 0
        self._previous_right = 0

    @property
    def pose(self) -> Pose2D:
        return Pose2D(
            x_mm=self._pose.x_mm,
            y_mm=self._pose.y_mm,
            theta_rad=self._pose.theta_rad,
        )

    def reset(self, left: EncoderSample, right: EncoderSample) -> None:
        self._pose = Pose2D()
        self._previous_left = left.counts
        self._previous_right = right.counts

    def update(self, left: EncoderSample, right: EncoderSample) -> Pose2D:
        left_delta = left.counts - self._previous_left
        right_delta = right.counts - self._previous_right
        self._previous_left = left.counts
        self._previous_right = right.counts

        dl_mm = self.counts_to_mm(left_delta)
        dr_mm = self.counts_to_mm(right_delta)
        dc_mm = (dl_mm + dr_mm) / 2
        dtheta = (dr_mm - dl_mm) / self._config.wheel_track_mm

        theta_mid = self._pose.theta_rad + dtheta / 2
        self._pose.x_mm += dc_mm * math.cos(theta_mid)
        self._pose.y_mm += dc_mm * math.sin(theta_mid)
        self._pose.theta_rad = normalize_angle(self._pose.theta_rad + dtheta)
        return self.pose

    def counts_to_mm(self, counts: int) -> float:
        counts_per_rev = self._config.pulses_per_channel * 4 * self._config.gear_ratio
        circumference_mm = math.pi * self._config.wheel_diameter_mm
        return counts * circumference_mm / counts_per_rev


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
