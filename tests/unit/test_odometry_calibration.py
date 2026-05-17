import math

from navibot.calibration.odometry import (
    CalibrationResult,
    counts_for_distance,
    estimate_mm_per_count_multiplier,
    estimate_track_width_mm,
    mm_per_count,
)


def test_distance_count_conversion_uses_quadrature_counts() -> None:
    value = mm_per_count(wheel_diameter_mm=43.0, pulses_per_channel=7, gear_ratio=100.0)

    assert value == math.pi * 43.0 / 2800.0
    assert counts_for_distance(100.0, value) == round(100.0 / value)


def test_distance_multiplier_compares_fiducial_motion_to_encoder_motion() -> None:
    result = estimate_mm_per_count_multiplier(
        commanded_distance_mm=200.0,
        encoder_distance_mm=198.0,
        fiducial_distance_mm=190.0,
    )

    assert result == CalibrationResult(
        measured_mm=190.0,
        odometry_mm=198.0,
        correction_multiplier=190.0 / 198.0,
    )


def test_track_width_estimate_scales_by_encoder_yaw_over_measured_yaw() -> None:
    assert estimate_track_width_mm(
        current_track_width_mm=105.0,
        encoder_yaw_deg=20.0,
        fiducial_yaw_deg=18.0,
    ) == 105.0 * (20.0 / 18.0)

