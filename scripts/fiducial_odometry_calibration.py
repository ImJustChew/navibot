#!/usr/bin/env python3
"""Calibrate wheel odometry against a single forward-facing ArUco marker."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from time import monotonic, sleep
from typing import Any

from navibot.calibration.odometry import (
    counts_for_distance,
    estimate_mm_per_count_multiplier,
    estimate_track_width_mm,
    mm_per_count,
)
from navibot.robot.encoders import EncoderPins, QuadratureEncoder
from navibot.robot.motors import DriverMotor, MotorPins, clamp, validate_motor_voltage
from navibot.sensors.vl53l1x_array import DEFAULT_VL53L1X_SPECS, Vl53l1xArray


@dataclass(frozen=True)
class WheelPins:
    motor: MotorPins
    encoder: EncoderPins


@dataclass(frozen=True)
class Observation:
    marker_id: int
    x_mm: float
    z_mm: float
    distance_mm: float
    bearing_deg: float
    yaw_deg: float


class Wheel:
    def __init__(
        self,
        pins: WheelPins,
        *,
        pull_up: bool,
        motor_inverted: bool,
        encoder_inverted: bool,
        brake_on_stop: bool,
    ) -> None:
        self.motor = DriverMotor(
            pins.motor,
            inverted=motor_inverted,
            brake_on_stop=brake_on_stop,
        )
        self.encoder = QuadratureEncoder(pins.encoder, pull_up=pull_up, inverted=encoder_inverted)

    def close(self) -> None:
        self.motor.close()
        self.encoder.close()


class CalibrationRig:
    def __init__(self, args: argparse.Namespace) -> None:
        try:
            from gpiozero import OutputDevice
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self.left = Wheel(
            WheelPins(
                motor=MotorPins(pwm=args.left_pwm, in1=args.left_in1, in2=args.left_in2),
                encoder=EncoderPins(a=args.left_encoder_a, b=args.left_encoder_b),
            ),
            pull_up=args.pull_up,
            motor_inverted=args.left_motor_inverted,
            encoder_inverted=args.left_encoder_inverted,
            brake_on_stop=args.brake_on_stop,
        )
        self.right = Wheel(
            WheelPins(
                motor=MotorPins(pwm=args.right_pwm, in1=args.right_in1, in2=args.right_in2),
                encoder=EncoderPins(a=args.right_encoder_a, b=args.right_encoder_b),
            ),
            pull_up=args.pull_up,
            motor_inverted=args.right_motor_inverted,
            encoder_inverted=args.right_encoder_inverted,
            brake_on_stop=args.brake_on_stop,
        )
        self._standby = OutputDevice(args.standby, initial_value=False)

    def enable(self) -> None:
        self._standby.on()

    def stop(self) -> None:
        self.left.motor.stop()
        self.right.motor.stop()

    def close(self) -> None:
        self.stop()
        self._standby.off()
        self.left.close()
        self.right.close()
        self._standby.close()


class CameraSource:
    def __init__(self, args: argparse.Namespace) -> None:
        self._picam2: Any | None = None
        self._capture: Any | None = None
        try:
            from picamera2 import Picamera2

            self._picam2 = Picamera2()
            config = self._picam2.create_preview_configuration(
                main={"size": (args.camera_width, args.camera_height), "format": "RGB888"}
            )
            self._picam2.configure(config)
            self._picam2.set_controls(camera_controls(args))
            self._picam2.start()
            sleep(1.0)
        except Exception as exc:
            import cv2

            self._capture = cv2.VideoCapture(args.camera_index)
            self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.camera_width)
            self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.camera_height)
            if not self._capture.isOpened():
                raise RuntimeError(
                    "Could not open Picamera2 or OpenCV VideoCapture camera"
                ) from exc

    def read(self) -> Any:
        if self._picam2 is not None:
            return self._picam2.capture_array()
        ok, frame = self._capture.read()
        if not ok:
            raise RuntimeError("Camera frame capture failed")
        return frame

    def close(self) -> None:
        if self._picam2 is not None:
            self._picam2.close()
        if self._capture is not None:
            self._capture.release()


def camera_controls(args: argparse.Namespace) -> dict[str, float | bool | int]:
    controls: dict[str, float | bool | int] = {
        "Contrast": args.camera_contrast,
        "Sharpness": args.camera_sharpness,
    }
    if args.camera_exposure_us > 0:
        controls.update(
            {
                "AeEnable": False,
                "ExposureTime": args.camera_exposure_us,
                "AnalogueGain": args.camera_gain,
            }
        )
    else:
        controls.update(
            {
                "AeEnable": True,
                "ExposureValue": args.camera_exposure_value,
            }
        )
    return controls


class ArucoDetector:
    def __init__(self, args: argparse.Namespace) -> None:
        import cv2
        import numpy as np

        self._cv2 = cv2
        self._np = np
        self._marker_size_mm = args.marker_size_mm
        self._marker_id = args.marker_id
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, args.aruco_dictionary)
        )
        self._parameters = cv2.aruco.DetectorParameters()
        self._detector = (
            cv2.aruco.ArucoDetector(self._dictionary, self._parameters)
            if hasattr(cv2.aruco, "ArucoDetector")
            else None
        )
        self._camera_matrix, self._dist_coeffs = self._load_camera_model(args)

    def detect(self, frame: Any) -> Observation | None:
        cv2 = self._cv2
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) if frame.ndim == 3 else frame
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self._dictionary,
                parameters=self._parameters,
            )
        if ids is None:
            return None

        for index, marker_id in enumerate(ids.flatten()):
            if self._marker_id is not None and int(marker_id) != self._marker_id:
                continue
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[index]],
                self._marker_size_mm,
                self._camera_matrix,
                self._dist_coeffs,
            )
            rvec = rvecs[0][0]
            tvec = tvecs[0][0]
            distance_mm = float(self._np.linalg.norm(tvec))
            bearing_deg = math.degrees(math.atan2(float(tvec[0]), float(tvec[2])))
            rotation, _ = cv2.Rodrigues(rvec)
            yaw_deg = math.degrees(math.atan2(float(rotation[1, 0]), float(rotation[0, 0])))
            return Observation(
                marker_id=int(marker_id),
                x_mm=float(tvec[0]),
                z_mm=float(tvec[2]),
                distance_mm=distance_mm,
                bearing_deg=bearing_deg,
                yaw_deg=yaw_deg,
            )
        return None

    def _load_camera_model(self, args: argparse.Namespace) -> tuple[Any, Any]:
        np = self._np
        if args.camera_calibration_json:
            data = json.loads(Path(args.camera_calibration_json).read_text(encoding="utf-8"))
            return (
                np.array(data["camera_matrix"], dtype=np.float32),
                np.array(data.get("dist_coeffs", [0, 0, 0, 0, 0]), dtype=np.float32),
            )

        fx = (args.camera_width / 2) / math.tan(math.radians(args.camera_hfov_deg) / 2)
        fy = fx
        cx = args.camera_width / 2
        cy = args.camera_height / 2
        return (
            np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32),
            np.zeros((5, 1), dtype=np.float32),
        )


class FrontTof:
    def __init__(self, enabled: bool, settle_seconds: float) -> None:
        self._array: Vl53l1xArray | None = None
        if not enabled:
            return
        self._array = Vl53l1xArray(specs=DEFAULT_VL53L1X_SPECS)
        self._array.start_ranging()
        sleep(settle_seconds)

    def read(self, samples: int = 5, interval_seconds: float = 0.05) -> int | None:
        if self._array is None:
            return None
        values: list[int] = []
        for _ in range(samples):
            for reading in self._array.read_all():
                if reading.name == "front" and reading.ready and reading.distance_mm is not None:
                    values.append(reading.distance_mm)
            sleep(interval_seconds)
        if not values:
            return None
        values.sort()
        return values[len(values) // 2]

    def close(self) -> None:
        if self._array is not None:
            self._array.close()


def wait_for_marker(
    camera: CameraSource,
    detector: ArucoDetector,
    *,
    timeout_seconds: float,
    label: str,
) -> Observation:
    print(f"Looking for marker: {label}", flush=True)
    deadline = monotonic() + timeout_seconds
    while monotonic() < deadline:
        observation = detector.detect(camera.read())
        if observation is not None:
            print(
                f"{label}: id={observation.marker_id} "
                f"x={observation.x_mm:.1f}mm "
                f"z={observation.z_mm:.1f}mm "
                f"distance={observation.distance_mm:.1f}mm "
                f"bearing={observation.bearing_deg:.2f}deg "
                f"yaw={observation.yaw_deg:.2f}deg",
                flush=True,
            )
            return observation
        sleep(0.05)
    raise RuntimeError(f"No marker detected for {label}")


def reset_encoders(rig: CalibrationRig) -> None:
    rig.left.encoder.reset()
    rig.right.encoder.reset()


def average_abs_counts(rig: CalibrationRig) -> float:
    left = rig.left.encoder.sample().abs_counts
    right = rig.right.encoder.sample().abs_counts
    return (left + right) / 2


def drive_forward_counts(
    rig: CalibrationRig,
    *,
    target_counts: int,
    pwm: float,
    timeout_seconds: float,
) -> tuple[int, int]:
    reset_encoders(rig)
    started = monotonic()
    rig.left.motor.forward(pwm)
    rig.right.motor.forward(pwm)
    try:
        while average_abs_counts(rig) < target_counts:
            if monotonic() - started >= timeout_seconds:
                print("Forward move timed out before target counts.", flush=True)
                break
            remaining = target_counts - average_abs_counts(rig)
            if remaining < target_counts * 0.25:
                slow_pwm = clamp(pwm * 0.55, 0.16, pwm)
                rig.left.motor.forward(slow_pwm)
                rig.right.motor.forward(slow_pwm)
            sleep(0.02)
    finally:
        rig.stop()
        sleep(0.25)
    return rig.left.encoder.sample().abs_counts, rig.right.encoder.sample().abs_counts


def rotate_counts(
    rig: CalibrationRig,
    *,
    target_counts: int,
    pwm: float,
    direction: str,
    timeout_seconds: float,
) -> tuple[int, int]:
    reset_encoders(rig)
    started = monotonic()
    if direction == "left":
        rig.left.motor.reverse(pwm)
        rig.right.motor.forward(pwm)
    else:
        rig.left.motor.forward(pwm)
        rig.right.motor.reverse(pwm)
    try:
        while average_abs_counts(rig) < target_counts:
            if monotonic() - started >= timeout_seconds:
                print("Turn move timed out before target counts.", flush=True)
                break
            remaining = target_counts - average_abs_counts(rig)
            if remaining < target_counts * 0.25:
                slow_pwm = clamp(pwm * 0.55, 0.18, pwm)
                if direction == "left":
                    rig.left.motor.reverse(slow_pwm)
                    rig.right.motor.forward(slow_pwm)
                else:
                    rig.left.motor.forward(slow_pwm)
                    rig.right.motor.reverse(slow_pwm)
            sleep(0.02)
    finally:
        rig.stop()
        sleep(0.25)
    return rig.left.encoder.sample().abs_counts, rig.right.encoder.sample().abs_counts


def pulse_rotate(rig: CalibrationRig, direction: str, pwm: float, seconds: float) -> None:
    if direction == "left":
        rig.left.motor.reverse(pwm)
        rig.right.motor.forward(pwm)
    else:
        rig.left.motor.forward(pwm)
        rig.right.motor.reverse(pwm)
    try:
        sleep(seconds)
    finally:
        rig.stop()
        sleep(0.15)


def align_to_marker(
    rig: CalibrationRig,
    camera: CameraSource,
    detector: ArucoDetector,
    *,
    tolerance_deg: float,
    pwm: float,
    timeout_seconds: float,
) -> Observation:
    deadline = monotonic() + timeout_seconds
    observation = wait_for_marker(
        camera,
        detector,
        timeout_seconds=min(3.0, timeout_seconds),
        label="align_start",
    )
    while abs(observation.bearing_deg) > tolerance_deg:
        if monotonic() >= deadline:
            print("Marker alignment timed out; continuing with current bearing.", flush=True)
            return observation
        direction = "right" if observation.bearing_deg > 0 else "left"
        pulse_rotate(rig, direction, pwm, seconds=0.08)
        observation = wait_for_marker(
            camera,
            detector,
            timeout_seconds=2.0,
            label="align",
        )
    return observation


def normalize_delta_deg(value: float) -> float:
    while value <= -180:
        value += 360
    while value > 180:
        value -= 360
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--marker-size-mm", type=float, required=True)
    parser.add_argument("--marker-id", type=int)
    parser.add_argument("--aruco-dictionary", default="DICT_4X4_50")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--camera-hfov-deg", type=float, default=62.2)
    parser.add_argument("--camera-calibration-json")
    parser.add_argument("--camera-exposure-value", type=float, default=-2.0)
    parser.add_argument("--camera-exposure-us", type=int, default=0)
    parser.add_argument("--camera-gain", type=float, default=1.0)
    parser.add_argument("--camera-contrast", type=float, default=2.0)
    parser.add_argument("--camera-sharpness", type=float, default=8.0)
    parser.add_argument("--front-tof", dest="front_tof", action="store_true", default=True)
    parser.add_argument("--no-front-tof", dest="front_tof", action="store_false")
    parser.add_argument("--tof-settle-seconds", type=float, default=0.25)
    parser.add_argument("--detect-timeout-seconds", type=float, default=8.0)
    parser.add_argument("--distance-mm", type=float, default=200.0)
    parser.add_argument("--turn-deg", type=float, default=20.0)
    parser.add_argument("--turn-direction", choices=("left", "right"), default="left")
    parser.add_argument("--wheel-diameter-mm", type=float, default=43.0)
    parser.add_argument("--wheel-track-mm", type=float, default=64.0)
    parser.add_argument("--pulses-per-channel", type=int, default=7)
    parser.add_argument("--gear-ratio", type=float, default=132.0)
    parser.add_argument("--forward-pwm", type=float, default=0.28)
    parser.add_argument("--turn-pwm", type=float, default=0.30)
    parser.add_argument("--align-first", dest="align_first", action="store_true", default=True)
    parser.add_argument("--no-align-first", dest="align_first", action="store_false")
    parser.add_argument("--align-tolerance-deg", type=float, default=2.0)
    parser.add_argument("--align-timeout-seconds", type=float, default=10.0)
    parser.add_argument("--supply-voltage", type=float, default=7.4)
    parser.add_argument("--motor-voltage-limit", type=float, default=6.0)
    parser.add_argument("--move-timeout-seconds", type=float, default=8.0)
    parser.add_argument("--pull-up", dest="pull_up", action="store_true", default=True)
    parser.add_argument("--no-pull-up", dest="pull_up", action="store_false")
    parser.add_argument(
        "--left-motor-inverted",
        dest="left_motor_inverted",
        action="store_true",
        default=True,
    )
    parser.add_argument("--left-motor-normal", dest="left_motor_inverted", action="store_false")
    parser.add_argument("--right-motor-inverted", action="store_true")
    parser.add_argument("--left-encoder-inverted", action="store_true")
    parser.add_argument("--right-encoder-inverted", action="store_true", default=True)
    parser.add_argument(
        "--right-encoder-normal",
        dest="right_encoder_inverted",
        action="store_false",
    )
    parser.add_argument("--brake-on-stop", dest="brake_on_stop", action="store_true", default=True)
    parser.add_argument("--coast-on-stop", dest="brake_on_stop", action="store_false")
    parser.add_argument("--left-pwm", type=int, default=13)
    parser.add_argument("--left-in1", type=int, default=26)
    parser.add_argument("--left-in2", type=int, default=19)
    parser.add_argument("--left-encoder-a", type=int, default=23)
    parser.add_argument("--left-encoder-b", type=int, default=24)
    parser.add_argument("--right-pwm", type=int, default=12)
    parser.add_argument("--right-in1", type=int, default=20)
    parser.add_argument("--right-in2", type=int, default=21)
    parser.add_argument("--right-encoder-a", type=int, default=27)
    parser.add_argument("--right-encoder-b", type=int, default=22)
    parser.add_argument("--standby", type=int, default=16)
    parser.add_argument("--skip-motion", action="store_true")
    parser.add_argument("--yes", action="store_true")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.marker_size_mm <= 0:
        raise ValueError("--marker-size-mm must be greater than zero")
    if args.distance_mm <= 0:
        raise ValueError("--distance-mm must be greater than zero")
    if args.turn_deg <= 0:
        raise ValueError("--turn-deg must be greater than zero")
    if args.wheel_diameter_mm <= 0 or args.wheel_track_mm <= 0:
        raise ValueError("--wheel-diameter-mm and --wheel-track-mm must be greater than zero")
    if args.pulses_per_channel <= 0 or args.gear_ratio <= 0:
        raise ValueError("--pulses-per-channel and --gear-ratio must be greater than zero")
    validate_motor_voltage(args.forward_pwm, args.supply_voltage, args.motor_voltage_limit)
    validate_motor_voltage(args.turn_pwm, args.supply_voltage, args.motor_voltage_limit)


def confirm_or_exit(args: argparse.Namespace, straight_counts: int, turn_counts: int) -> None:
    if args.yes or args.skip_motion:
        return
    print("This will move the robot. Put it on the floor facing a visible ArUco marker.")
    print(f"Straight: {args.distance_mm:g} mm -> {straight_counts} counts/wheel")
    print(f"Turn: {args.turn_deg:g} deg {args.turn_direction} -> {turn_counts} counts/wheel")
    answer = input("Type RUN to start: ")
    if answer != "RUN":
        raise SystemExit("Fiducial calibration cancelled.")


def main() -> None:
    args = parse_args()
    validate_args(args)
    millimeters_per_count = mm_per_count(
        args.wheel_diameter_mm,
        args.pulses_per_channel,
        args.gear_ratio,
    )
    straight_target_counts = counts_for_distance(args.distance_mm, millimeters_per_count)
    turn_arc_mm = math.radians(args.turn_deg) * args.wheel_track_mm / 2
    turn_target_counts = counts_for_distance(turn_arc_mm, millimeters_per_count)
    confirm_or_exit(args, straight_target_counts, turn_target_counts)

    camera = CameraSource(args)
    detector = ArucoDetector(args)
    front_tof = FrontTof(args.front_tof, args.tof_settle_seconds)
    rig: CalibrationRig | None = None
    try:
        start = wait_for_marker(
            camera,
            detector,
            timeout_seconds=args.detect_timeout_seconds,
            label="start",
        )
        if args.skip_motion:
            print("Marker detection succeeded; skipping motor motion.")
            return

        rig = CalibrationRig(args)
        rig.enable()
        if args.align_first:
            start = align_to_marker(
                rig,
                camera,
                detector,
                tolerance_deg=args.align_tolerance_deg,
                pwm=args.turn_pwm,
                timeout_seconds=args.align_timeout_seconds,
            )
        start_front_tof_mm = front_tof.read()
        if start_front_tof_mm is not None:
            print(f"front_tof_start: {start_front_tof_mm}mm", flush=True)

        left_counts, right_counts = drive_forward_counts(
            rig,
            target_counts=straight_target_counts,
            pwm=args.forward_pwm,
            timeout_seconds=args.move_timeout_seconds,
        )
        after_front_tof_mm = front_tof.read()
        if after_front_tof_mm is not None:
            print(f"front_tof_after_forward: {after_front_tof_mm}mm", flush=True)
        after_forward = wait_for_marker(
            camera,
            detector,
            timeout_seconds=args.detect_timeout_seconds,
            label="after_forward",
        )
        encoder_distance_mm = ((left_counts + right_counts) / 2) * millimeters_per_count
        fiducial_motion_mm = start.z_mm - after_forward.z_mm
        tof_motion_mm = (
            None
            if start_front_tof_mm is None or after_front_tof_mm is None
            else start_front_tof_mm - after_front_tof_mm
        )
        distance_result = estimate_mm_per_count_multiplier(
            args.distance_mm,
            encoder_distance_mm,
            fiducial_motion_mm,
        )

        before_turn = after_forward
        left_turn_counts, right_turn_counts = rotate_counts(
            rig,
            target_counts=turn_target_counts,
            pwm=args.turn_pwm,
            direction=args.turn_direction,
            timeout_seconds=args.move_timeout_seconds,
        )
        after_turn = wait_for_marker(
            camera,
            detector,
            timeout_seconds=args.detect_timeout_seconds,
            label="after_turn",
        )
        avg_turn_counts = (left_turn_counts + right_turn_counts) / 2
        encoder_turn_deg = math.degrees(
            (2 * avg_turn_counts * millimeters_per_count) / args.wheel_track_mm
        )
        measured_turn_deg = abs(
            normalize_delta_deg(after_turn.bearing_deg - before_turn.bearing_deg)
        )
        suggested_track_width = estimate_track_width_mm(
            args.wheel_track_mm,
            encoder_turn_deg,
            measured_turn_deg,
        )

        corrected_mm_per_count = millimeters_per_count * distance_result.correction_multiplier
        corrected_gear_ratio = args.gear_ratio / distance_result.correction_multiplier
        print()
        print("Calibration summary")
        print(f"  Current mm/count:       {millimeters_per_count:.6f}")
        print(f"  Encoder straight:       {encoder_distance_mm:.1f} mm")
        print(f"  Fiducial straight:      {fiducial_motion_mm:.1f} mm")
        if tof_motion_mm is not None:
            print(f"  Front TOF straight ref: {tof_motion_mm:.1f} mm")
        print(f"  mm/count multiplier:    {distance_result.correction_multiplier:.5f}")
        print(f"  Suggested mm/count:     {corrected_mm_per_count:.6f}")
        print(f"  Suggested gear ratio:   {corrected_gear_ratio:.3f}")
        print(f"  Encoder turn:           {encoder_turn_deg:.2f} deg")
        print(f"  Fiducial bearing turn:  {measured_turn_deg:.2f} deg")
        print(f"  Suggested track width:  {suggested_track_width:.1f} mm")
        print(
            "  Bad transitions:        "
            f"L={rig.left.encoder.bad_transitions} R={rig.right.encoder.bad_transitions}"
        )
    finally:
        if rig is not None:
            rig.close()
        front_tof.close()
        camera.close()


if __name__ == "__main__":
    main()
