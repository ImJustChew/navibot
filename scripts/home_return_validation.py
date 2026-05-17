#!/usr/bin/env python3
"""Validate that Navibot can leave and return to a marker-defined home pose."""

from __future__ import annotations

import argparse
from time import sleep

from fiducial_odometry_calibration import (
    ArucoDetector,
    CalibrationRig,
    CameraSource,
    FrontTof,
    align_to_marker,
    counts_for_distance,
    mm_per_count,
    pulse_rotate,
    rotate_counts,
    validate_motor_voltage,
    wait_for_marker,
)


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
    parser.add_argument("--detect-timeout-seconds", type=float, default=8.0)
    parser.add_argument("--wheel-diameter-mm", type=float, default=43.0)
    parser.add_argument("--wheel-track-mm", type=float, default=64.0)
    parser.add_argument("--pulses-per-channel", type=int, default=7)
    parser.add_argument("--gear-ratio", type=float, default=132.0)
    parser.add_argument("--reverse-away-mm", type=float, default=150.0)
    parser.add_argument("--side-turn-deg", type=float, default=18.0)
    parser.add_argument("--return-tolerance-mm", type=float, default=25.0)
    parser.add_argument("--bearing-tolerance-deg", type=float, default=3.0)
    parser.add_argument("--max-return-steps", type=int, default=24)
    parser.add_argument("--forward-pwm", type=float, default=0.22)
    parser.add_argument("--reverse-pwm", type=float, default=0.22)
    parser.add_argument("--turn-pwm", type=float, default=0.24)
    parser.add_argument("--supply-voltage", type=float, default=7.4)
    parser.add_argument("--motor-voltage-limit", type=float, default=6.0)
    parser.add_argument("--move-timeout-seconds", type=float, default=6.0)
    parser.add_argument("--front-tof", dest="front_tof", action="store_true", default=True)
    parser.add_argument("--no-front-tof", dest="front_tof", action="store_false")
    parser.add_argument("--tof-settle-seconds", type=float, default=0.25)
    parser.add_argument("--min-front-tof-mm", type=int, default=70)
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
    parser.add_argument("--yes", action="store_true")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.marker_size_mm <= 0:
        raise ValueError("--marker-size-mm must be greater than zero")
    if args.reverse_away_mm <= 0 or args.side_turn_deg <= 0:
        raise ValueError("--reverse-away-mm and --side-turn-deg must be greater than zero")
    validate_motor_voltage(args.forward_pwm, args.supply_voltage, args.motor_voltage_limit)
    validate_motor_voltage(args.reverse_pwm, args.supply_voltage, args.motor_voltage_limit)
    validate_motor_voltage(args.turn_pwm, args.supply_voltage, args.motor_voltage_limit)


def confirm_or_exit(args: argparse.Namespace) -> None:
    if args.yes:
        return
    print("This will move the robot away from the floor marker and return to marker-home.")
    print(f"Reverse away: {args.reverse_away_mm:g} mm")
    print(f"Side turn: {args.side_turn_deg:g} deg each way")
    answer = input("Type RUN to start: ")
    if answer != "RUN":
        raise SystemExit("Home return validation cancelled.")


def drive_counts(
    rig: CalibrationRig,
    *,
    target_counts: int,
    pwm: float,
    direction: str,
    timeout_seconds: float,
) -> tuple[int, int]:
    rig.left.encoder.reset()
    rig.right.encoder.reset()
    started = __import__("time").monotonic()
    if direction == "forward":
        rig.left.motor.forward(pwm)
        rig.right.motor.forward(pwm)
    else:
        rig.left.motor.reverse(pwm)
        rig.right.motor.reverse(pwm)
    try:
        while average_counts(rig) < target_counts:
            if __import__("time").monotonic() - started >= timeout_seconds:
                print(f"{direction} move timed out before target counts.", flush=True)
                break
            sleep(0.02)
    finally:
        rig.stop()
        sleep(0.2)
    return rig.left.encoder.sample().abs_counts, rig.right.encoder.sample().abs_counts


def average_counts(rig: CalibrationRig) -> float:
    return (rig.left.encoder.sample().abs_counts + rig.right.encoder.sample().abs_counts) / 2


def move_away_pattern(
    rig: CalibrationRig,
    args: argparse.Namespace,
    millimeters_per_count: float,
) -> None:
    reverse_counts = counts_for_distance(args.reverse_away_mm, millimeters_per_count)
    turn_arc_mm = __import__("math").radians(args.side_turn_deg) * args.wheel_track_mm / 2
    turn_counts = counts_for_distance(turn_arc_mm, millimeters_per_count)

    print("Leaving home...", flush=True)
    drive_counts(
        rig,
        target_counts=reverse_counts,
        pwm=args.reverse_pwm,
        direction="reverse",
        timeout_seconds=args.move_timeout_seconds,
    )
    rotate_counts(
        rig,
        target_counts=turn_counts,
        pwm=args.turn_pwm,
        direction="left",
        timeout_seconds=args.move_timeout_seconds,
    )
    rotate_counts(
        rig,
        target_counts=turn_counts * 2,
        pwm=args.turn_pwm,
        direction="right",
        timeout_seconds=args.move_timeout_seconds,
    )
    rotate_counts(
        rig,
        target_counts=turn_counts,
        pwm=args.turn_pwm,
        direction="left",
        timeout_seconds=args.move_timeout_seconds,
    )


def return_to_home(
    rig: CalibrationRig,
    camera: CameraSource,
    detector: ArucoDetector,
    front_tof: FrontTof,
    args: argparse.Namespace,
    home_z_mm: float,
    millimeters_per_count: float,
) -> tuple[float, float, int | None]:
    print("Returning to marker-home...", flush=True)
    final_tof = None
    for step in range(args.max_return_steps):
        obs = reacquire_marker(
            rig,
            camera,
            detector,
            args,
            millimeters_per_count,
            label=f"return_{step:02d}",
        )
        depth_error = obs.z_mm - home_z_mm
        final_tof = front_tof.read(samples=3)
        if final_tof is not None:
            print(f"front_tof_ref: {final_tof}mm", flush=True)

        if abs(obs.bearing_deg) > args.bearing_tolerance_deg:
            direction = "right" if obs.bearing_deg > 0 else "left"
            pulse_rotate(rig, direction, args.turn_pwm, seconds=0.06)
            continue

        if abs(depth_error) <= args.return_tolerance_mm:
            return depth_error, obs.bearing_deg, final_tof

        if depth_error > 0:
            move_mm = min(45.0, max(8.0, abs(depth_error) * 0.55))
            if final_tof is not None and final_tof <= args.min_front_tof_mm:
                print("Stopping return: front TOF is below safety threshold.", flush=True)
                return depth_error, obs.bearing_deg, final_tof
            direction = "forward"
            pwm = args.forward_pwm
        else:
            move_mm = min(45.0, max(8.0, abs(depth_error) * 0.55))
            direction = "reverse"
            pwm = args.reverse_pwm

        drive_counts(
            rig,
            target_counts=counts_for_distance(move_mm, millimeters_per_count),
            pwm=pwm,
            direction=direction,
            timeout_seconds=args.move_timeout_seconds,
        )

    obs = reacquire_marker(
        rig,
        camera,
        detector,
        args,
        millimeters_per_count,
        label="return_final",
    )
    return obs.z_mm - home_z_mm, obs.bearing_deg, final_tof


def reacquire_marker(
    rig: CalibrationRig,
    camera: CameraSource,
    detector: ArucoDetector,
    args: argparse.Namespace,
    millimeters_per_count: float,
    *,
    label: str,
):
    try:
        return wait_for_marker(
            camera,
            detector,
            timeout_seconds=args.detect_timeout_seconds,
            label=label,
        )
    except RuntimeError:
        print(f"{label}: marker lost; backing up and scanning", flush=True)

    drive_counts(
        rig,
        target_counts=counts_for_distance(35.0, millimeters_per_count),
        pwm=args.reverse_pwm,
        direction="reverse",
        timeout_seconds=args.move_timeout_seconds,
    )
    for direction in ("left", "right", "right", "left", "left", "right"):
        for _ in range(3):
            pulse_rotate(rig, direction, args.turn_pwm, seconds=0.06)
            obs = detector.detect(camera.read())
            if obs is not None:
                print(
                    f"{label}_reacquired: z={obs.z_mm:.1f}mm "
                    f"bearing={obs.bearing_deg:.2f}deg yaw={obs.yaw_deg:.2f}deg",
                    flush=True,
                )
                return obs
    return wait_for_marker(
        camera,
        detector,
        timeout_seconds=args.detect_timeout_seconds,
        label=f"{label}_retry",
    )


def main() -> None:
    args = parse_args()
    validate_args(args)
    confirm_or_exit(args)
    millimeters_per_count = mm_per_count(
        args.wheel_diameter_mm,
        args.pulses_per_channel,
        args.gear_ratio,
    )

    camera = CameraSource(args)
    detector = ArucoDetector(args)
    front_tof = FrontTof(args.front_tof, args.tof_settle_seconds)
    rig: CalibrationRig | None = None
    try:
        start = wait_for_marker(
            camera,
            detector,
            timeout_seconds=args.detect_timeout_seconds,
            label="home_initial",
        )
        rig = CalibrationRig(args)
        rig.enable()
        home = align_to_marker(
            rig,
            camera,
            detector,
            tolerance_deg=args.bearing_tolerance_deg,
            pwm=args.turn_pwm,
            timeout_seconds=10.0,
        )
        home_tof = front_tof.read(samples=5)
        print(
            f"home: z={home.z_mm:.1f}mm bearing={home.bearing_deg:.2f}deg "
            f"yaw={home.yaw_deg:.2f}deg front_tof_ref={home_tof}",
            flush=True,
        )
        del start

        move_away_pattern(rig, args, millimeters_per_count)
        away = wait_for_marker(
            camera,
            detector,
            timeout_seconds=args.detect_timeout_seconds,
            label="away",
        )
        print(
            f"away_error: dz={away.z_mm - home.z_mm:.1f}mm "
            f"bearing={away.bearing_deg:.2f}deg",
            flush=True,
        )

        final_depth_error, final_bearing, final_tof = return_to_home(
            rig,
            camera,
            detector,
            front_tof,
            args,
            home.z_mm,
            millimeters_per_count,
        )
        passed = (
            abs(final_depth_error) <= args.return_tolerance_mm
            and abs(final_bearing) <= args.bearing_tolerance_deg
        )
        print()
        print("Home return summary")
        print(f"  final_depth_error: {final_depth_error:.1f} mm")
        print(f"  final_bearing:     {final_bearing:.2f} deg")
        print(f"  final_front_tof:   {final_tof} mm (reference only)")
        print(
            "  bad_transitions:   "
            f"L={rig.left.encoder.bad_transitions} R={rig.right.encoder.bad_transitions}"
        )
        print(f"  result:            {'PASS' if passed else 'FAIL'}")
        if not passed:
            raise SystemExit(1)
    finally:
        if rig is not None:
            rig.close()
        front_tof.close()
        camera.close()


if __name__ == "__main__":
    main()
