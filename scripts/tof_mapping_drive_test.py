"""Cautious TOF mapping drive test.

This is an early proof script, not production SLAM. It drives in small pulses,
uses encoder odometry for a rough pose estimate, projects VL53L1X readings into
a 2D point cloud, and writes JSON/CSV artifacts for inspection.

Behavior:
- move forward while the front TOF is clear
- rotate right when blocked
- stop after a duration or step limit
- write map points and robot path to an output directory
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from time import monotonic, sleep

from navibot.robot.encoders import EncoderPins, QuadratureEncoder
from navibot.robot.motors import DifferentialDrive, DriverMotor, MotorPins, validate_motor_voltage
from navibot.sensors.vl53l1x_array import Vl53l1xArray


@dataclass(frozen=True)
class WheelPins:
    motor: MotorPins
    encoder: EncoderPins


@dataclass(frozen=True)
class MapConfig:
    left: WheelPins
    right: WheelPins
    standby_pin: int
    speed: float
    turn_speed: float
    forward_pulse_seconds: float
    turn_pulse_seconds: float
    loop_settle_seconds: float
    max_steps: int
    max_seconds: float
    obstacle_mm: int
    min_valid_tof_mm: int
    max_valid_tof_mm: int
    wheel_diameter_mm: float
    wheel_track_mm: float
    pulses_per_channel: int
    gear_ratio: float
    supply_voltage: float
    motor_voltage_limit: float
    left_motor_inverted: bool
    right_motor_inverted: bool
    left_encoder_inverted: bool
    right_encoder_inverted: bool
    pull_up: bool
    output_dir: Path


@dataclass
class Pose:
    x_mm: float = 0.0
    y_mm: float = 0.0
    theta_rad: float = 0.0


@dataclass(frozen=True)
class MapPoint:
    x_mm: float
    y_mm: float
    sensor: str
    distance_mm: int
    robot_x_mm: float
    robot_y_mm: float
    robot_theta_rad: float
    t_s: float


TOF_ANGLES_RAD = {
    "front": 0.0,
    "left45": math.radians(45),
    "right45": math.radians(-45),
    "back": math.pi,
}


class Wheel:
    def __init__(
        self,
        pins: WheelPins,
        motor_inverted: bool,
        encoder_inverted: bool,
        pull_up: bool,
    ) -> None:
        self.motor = DriverMotor(pins.motor, inverted=motor_inverted, brake_on_stop=False)
        self.encoder = QuadratureEncoder(pins.encoder, pull_up=pull_up, inverted=encoder_inverted)

    def close(self) -> None:
        self.encoder.close()


class MappingRig:
    def __init__(self, config: MapConfig) -> None:
        self.left = Wheel(
            config.left,
            motor_inverted=config.left_motor_inverted,
            encoder_inverted=config.left_encoder_inverted,
            pull_up=config.pull_up,
        )
        self.right = Wheel(
            config.right,
            motor_inverted=config.right_motor_inverted,
            encoder_inverted=config.right_encoder_inverted,
            pull_up=config.pull_up,
        )
        self.drive = DifferentialDrive(
            left=self.left.motor,
            right=self.right.motor,
            standby_pin=config.standby_pin,
        )
        self.tof = Vl53l1xArray()

    def enable(self) -> None:
        self.drive.enable()
        self.tof.start_ranging()

    def close(self) -> None:
        self.drive.close()
        self.left.encoder.close()
        self.right.encoder.close()
        self.tof.close()


def counts_to_mm(counts: int, config: MapConfig) -> float:
    counts_per_rev = config.pulses_per_channel * 4 * config.gear_ratio
    circumference_mm = math.pi * config.wheel_diameter_mm
    return counts * circumference_mm / counts_per_rev


def update_pose_from_counts(
    pose: Pose,
    previous_left: int,
    previous_right: int,
    current_left: int,
    current_right: int,
    config: MapConfig,
) -> tuple[int, int]:
    dl_mm = counts_to_mm(current_left - previous_left, config)
    dr_mm = counts_to_mm(current_right - previous_right, config)
    dc_mm = (dl_mm + dr_mm) / 2
    dtheta = (dr_mm - dl_mm) / config.wheel_track_mm

    theta_mid = pose.theta_rad + dtheta / 2
    pose.x_mm += dc_mm * math.cos(theta_mid)
    pose.y_mm += dc_mm * math.sin(theta_mid)
    pose.theta_rad = normalize_angle(pose.theta_rad + dtheta)
    return current_left, current_right


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def project_tof_points(pose: Pose, readings: dict[str, int | None], config: MapConfig, t_s: float) -> list[MapPoint]:
    points = []
    for name, distance_mm in readings.items():
        if distance_mm is None:
            continue
        if not config.min_valid_tof_mm <= distance_mm <= config.max_valid_tof_mm:
            continue

        angle = pose.theta_rad + TOF_ANGLES_RAD.get(name, 0.0)
        points.append(
            MapPoint(
                x_mm=pose.x_mm + distance_mm * math.cos(angle),
                y_mm=pose.y_mm + distance_mm * math.sin(angle),
                sensor=name,
                distance_mm=distance_mm,
                robot_x_mm=pose.x_mm,
                robot_y_mm=pose.y_mm,
                robot_theta_rad=pose.theta_rad,
                t_s=t_s,
            )
        )
    return points


def latest_tof_readings(rig: MappingRig, timeout_seconds: float = 0.5) -> dict[str, int | None]:
    deadline = monotonic() + timeout_seconds
    readings: dict[str, int | None] = {}
    while monotonic() < deadline:
        for reading in rig.tof.read_all():
            if reading.ready:
                readings[reading.name] = reading.distance_mm
        if {"front", "left45", "right45", "back"}.issubset(readings):
            break
        sleep(0.02)
    return readings


def run_mapping(config: MapConfig) -> None:
    validate_motor_voltage(config.speed, config.supply_voltage, config.motor_voltage_limit)
    validate_motor_voltage(config.turn_speed, config.supply_voltage, config.motor_voltage_limit)

    rig = MappingRig(config)
    pose = Pose()
    points: list[MapPoint] = []
    path: list[dict[str, float | int | str]] = []
    started_at = monotonic()

    try:
        rig.enable()
        rig.left.encoder.reset()
        rig.right.encoder.reset()
        previous_left = 0
        previous_right = 0

        for step in range(config.max_steps):
            elapsed = monotonic() - started_at
            if elapsed >= config.max_seconds:
                break

            readings = latest_tof_readings(rig)
            front_mm = readings.get("front")
            blocked = front_mm is not None and front_mm <= config.obstacle_mm

            if blocked:
                action = "rotate_right"
                rig.drive.rotate_right(config.turn_speed)
                sleep(config.turn_pulse_seconds)
            else:
                action = "forward"
                rig.drive.forward(config.speed)
                sleep(config.forward_pulse_seconds)
            rig.drive.coast()
            sleep(config.loop_settle_seconds)

            left_sample = rig.left.encoder.sample()
            right_sample = rig.right.encoder.sample()
            previous_left, previous_right = update_pose_from_counts(
                pose,
                previous_left,
                previous_right,
                left_sample.counts,
                right_sample.counts,
                config,
            )

            readings = latest_tof_readings(rig)
            elapsed = monotonic() - started_at
            points.extend(project_tof_points(pose, readings, config, elapsed))
            path.append(
                {
                    "step": step,
                    "t_s": elapsed,
                    "action": action,
                    "x_mm": pose.x_mm,
                    "y_mm": pose.y_mm,
                    "theta_rad": pose.theta_rad,
                    "left_counts": left_sample.counts,
                    "right_counts": right_sample.counts,
                    "front_mm": readings.get("front"),
                    "left45_mm": readings.get("left45"),
                    "right45_mm": readings.get("right45"),
                    "back_mm": readings.get("back"),
                }
            )

            print(
                f"step={step:03d} action={action:<12} "
                f"pose=({pose.x_mm:7.1f},{pose.y_mm:7.1f},{math.degrees(pose.theta_rad):6.1f}deg) "
                f"tof front={readings.get('front')} left45={readings.get('left45')} "
                f"right45={readings.get('right45')} back={readings.get('back')}"
            )
    finally:
        rig.close()

    write_outputs(config.output_dir, config, path, points)


def write_outputs(
    output_dir: Path,
    config: MapConfig,
    path: list[dict[str, float | int | str]],
    points: list[MapPoint],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    metadata = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "config": {
            key: str(value) if isinstance(value, Path) else value
            for key, value in asdict(config).items()
        },
        "path_samples": len(path),
        "map_points": len(points),
    }

    (output_dir / "map.json").write_text(
        json.dumps(
            {
                "metadata": metadata,
                "path": path,
                "points": [asdict(point) for point in points],
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    with (output_dir / "points.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(asdict(points[0]).keys()) if points else ["x_mm", "y_mm"])
        writer.writeheader()
        for point in points:
            writer.writerow(asdict(point))

    with (output_dir / "path.csv").open("w", newline="", encoding="utf-8") as file:
        fieldnames = list(path[0].keys()) if path else ["step", "x_mm", "y_mm", "theta_rad"]
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(path)

    print(f"Wrote map outputs to {output_dir}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build a simple TOF point map while driving cautiously.")
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/maps/latest"))
    parser.add_argument("--speed", type=float, default=0.14)
    parser.add_argument("--turn-speed", type=float, default=0.14)
    parser.add_argument("--forward-pulse-seconds", type=float, default=0.25)
    parser.add_argument("--turn-pulse-seconds", type=float, default=0.18)
    parser.add_argument("--loop-settle-seconds", type=float, default=0.05)
    parser.add_argument("--max-steps", type=int, default=120)
    parser.add_argument("--max-seconds", type=float, default=60.0)
    parser.add_argument("--obstacle-mm", type=int, default=180)
    parser.add_argument("--min-valid-tof-mm", type=int, default=40)
    parser.add_argument("--max-valid-tof-mm", type=int, default=3000)
    parser.add_argument("--wheel-diameter-mm", type=float, default=43.0)
    parser.add_argument("--wheel-track-mm", type=float, default=64.0)
    parser.add_argument("--pulses-per-channel", type=int, default=7)
    parser.add_argument("--gear-ratio", type=float, default=132.0)
    parser.add_argument("--supply-voltage", type=float, default=7.4)
    parser.add_argument("--motor-voltage-limit", type=float, default=6.0)
    parser.add_argument("--pull-up", dest="pull_up", action="store_true", default=True)
    parser.add_argument("--no-pull-up", dest="pull_up", action="store_false")
    parser.add_argument("--left-motor-inverted", dest="left_motor_inverted", action="store_true", default=True)
    parser.add_argument("--left-motor-normal", dest="left_motor_inverted", action="store_false")
    parser.add_argument("--right-motor-inverted", action="store_true")
    parser.add_argument("--left-encoder-inverted", action="store_true")
    parser.add_argument("--right-encoder-inverted", dest="right_encoder_inverted", action="store_true", default=True)
    parser.add_argument("--right-encoder-normal", dest="right_encoder_inverted", action="store_false")
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
    return parser.parse_args()


def build_config(args: argparse.Namespace) -> MapConfig:
    if args.speed <= 0 or args.turn_speed <= 0:
        raise ValueError("--speed and --turn-speed must be greater than zero")
    if args.max_steps <= 0:
        raise ValueError("--max-steps must be greater than zero")
    if args.max_seconds <= 0:
        raise ValueError("--max-seconds must be greater than zero")
    if args.wheel_track_mm <= 0:
        raise ValueError("--wheel-track-mm must be greater than zero")

    return MapConfig(
        left=WheelPins(
            motor=MotorPins(pwm=args.left_pwm, in1=args.left_in1, in2=args.left_in2),
            encoder=EncoderPins(a=args.left_encoder_a, b=args.left_encoder_b),
        ),
        right=WheelPins(
            motor=MotorPins(pwm=args.right_pwm, in1=args.right_in1, in2=args.right_in2),
            encoder=EncoderPins(a=args.right_encoder_a, b=args.right_encoder_b),
        ),
        standby_pin=args.standby,
        speed=args.speed,
        turn_speed=args.turn_speed,
        forward_pulse_seconds=args.forward_pulse_seconds,
        turn_pulse_seconds=args.turn_pulse_seconds,
        loop_settle_seconds=args.loop_settle_seconds,
        max_steps=args.max_steps,
        max_seconds=args.max_seconds,
        obstacle_mm=args.obstacle_mm,
        min_valid_tof_mm=args.min_valid_tof_mm,
        max_valid_tof_mm=args.max_valid_tof_mm,
        wheel_diameter_mm=args.wheel_diameter_mm,
        wheel_track_mm=args.wheel_track_mm,
        pulses_per_channel=args.pulses_per_channel,
        gear_ratio=args.gear_ratio,
        supply_voltage=args.supply_voltage,
        motor_voltage_limit=args.motor_voltage_limit,
        left_motor_inverted=args.left_motor_inverted,
        right_motor_inverted=args.right_motor_inverted,
        left_encoder_inverted=args.left_encoder_inverted,
        right_encoder_inverted=args.right_encoder_inverted,
        pull_up=args.pull_up,
        output_dir=args.output_dir,
    )


def main() -> None:
    args = parse_args()
    config = build_config(args)
    run_mapping(config)


if __name__ == "__main__":
    main()
