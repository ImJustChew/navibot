"""Print unified robot telemetry as JSON lines."""

from __future__ import annotations

import argparse
from time import sleep

from navibot.robot.encoders import EncoderPins
from navibot.robot.hardware import RobotHardware, RobotHardwareConfig
from navibot.robot.pose import DifferentialOdometryConfig
from navibot.robot.safety import SafetyConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print robot status JSON lines.")
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--count", type=int, default=0, help="Number of samples; 0 runs forever.")
    parser.add_argument("--front-stop-mm", type=int, default=180)
    parser.add_argument("--low-battery-v", type=float, default=6.2)
    parser.add_argument("--wheel-diameter-mm", type=float, default=43.0)
    parser.add_argument("--wheel-track-mm", type=float, default=64.0)
    parser.add_argument("--pulses-per-channel", type=int, default=7)
    parser.add_argument("--gear-ratio", type=float, default=132.0)
    parser.add_argument("--left-encoder-a", type=int, default=23)
    parser.add_argument("--left-encoder-b", type=int, default=24)
    parser.add_argument("--right-encoder-a", type=int, default=27)
    parser.add_argument("--right-encoder-b", type=int, default=22)
    parser.add_argument("--left-encoder-inverted", action="store_true")
    parser.add_argument("--right-encoder-inverted", dest="right_encoder_inverted", action="store_true", default=True)
    parser.add_argument("--right-encoder-normal", dest="right_encoder_inverted", action="store_false")
    parser.add_argument("--no-pull-up", dest="encoder_pull_up", action="store_false", default=True)
    parser.add_argument("--ina219-address", type=lambda value: int(value, 0), default=0x40)
    parser.add_argument("--pretty", action="store_true")
    return parser.parse_args()


def build_config(args: argparse.Namespace) -> RobotHardwareConfig:
    return RobotHardwareConfig(
        left_encoder=EncoderPins(a=args.left_encoder_a, b=args.left_encoder_b),
        right_encoder=EncoderPins(a=args.right_encoder_a, b=args.right_encoder_b),
        left_encoder_inverted=args.left_encoder_inverted,
        right_encoder_inverted=args.right_encoder_inverted,
        encoder_pull_up=args.encoder_pull_up,
        ina219_address=args.ina219_address,
        odometry=DifferentialOdometryConfig(
            wheel_diameter_mm=args.wheel_diameter_mm,
            wheel_track_mm=args.wheel_track_mm,
            pulses_per_channel=args.pulses_per_channel,
            gear_ratio=args.gear_ratio,
        ),
        safety=SafetyConfig(
            front_stop_mm=args.front_stop_mm,
            low_battery_v=args.low_battery_v,
        ),
    )


def main() -> None:
    args = parse_args()
    hardware = RobotHardware(build_config(args))
    try:
        hardware.start()
        sample = 0
        while args.count <= 0 or sample < args.count:
            state = hardware.read_state()
            if args.pretty:
                print(state.to_dict(), flush=True)
            else:
                print(state.to_json(), flush=True)
            sample += 1
            sleep(args.interval)
    except KeyboardInterrupt:
        print("")
    finally:
        hardware.close()


if __name__ == "__main__":
    main()
