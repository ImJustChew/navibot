from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CalibrationResult:
    measured_mm: float
    odometry_mm: float
    correction_multiplier: float


def mm_per_count(wheel_diameter_mm: float, pulses_per_channel: int, gear_ratio: float) -> float:
    counts_per_rev = pulses_per_channel * 4 * gear_ratio
    return math.pi * wheel_diameter_mm / counts_per_rev


def counts_for_distance(distance_mm: float, millimeters_per_count: float) -> int:
    return max(1, round(abs(distance_mm) / millimeters_per_count))


def estimate_mm_per_count_multiplier(
    commanded_distance_mm: float,
    encoder_distance_mm: float,
    fiducial_distance_mm: float,
) -> CalibrationResult:
    del commanded_distance_mm
    if encoder_distance_mm <= 0:
        raise ValueError("encoder_distance_mm must be greater than zero")
    return CalibrationResult(
        measured_mm=fiducial_distance_mm,
        odometry_mm=encoder_distance_mm,
        correction_multiplier=fiducial_distance_mm / encoder_distance_mm,
    )


def estimate_track_width_mm(
    current_track_width_mm: float,
    encoder_yaw_deg: float,
    fiducial_yaw_deg: float,
) -> float:
    if abs(fiducial_yaw_deg) < 0.001:
        raise ValueError("fiducial_yaw_deg is too close to zero")
    return current_track_width_mm * (encoder_yaw_deg / fiducial_yaw_deg)

