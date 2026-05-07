"""Interactive WASD motor test for the Raspberry Pi console.

Controls:
- W: forward
- S: reverse
- A: rotate left
- D: rotate right
- Space: stop
- Q: quit

Each keypress drives for a short pulse, then stops. Hold or repeatedly press a
key for continuous movement.
"""

from __future__ import annotations

import argparse
import select
import sys
from contextlib import contextmanager
from dataclasses import dataclass
from time import sleep

from navibot.robot.motors import DifferentialDrive, DriverMotor, MotorPins, validate_motor_voltage


@dataclass(frozen=True)
class DriveConfig:
    left: MotorPins
    right: MotorPins
    standby_pin: int
    speed: float
    pulse_seconds: float
    supply_voltage: float
    motor_voltage_limit: float
    left_motor_inverted: bool
    right_motor_inverted: bool


@contextmanager
def raw_terminal() -> object:
    import termios
    import tty

    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def read_key(timeout_seconds: float) -> str | None:
    readable, _, _ = select.select([sys.stdin], [], [], timeout_seconds)
    if not readable:
        return None
    return sys.stdin.read(1).lower()


def run_teleop(config: DriveConfig) -> None:
    rig = DifferentialDrive(
        left=DriverMotor(config.left, inverted=config.left_motor_inverted),
        right=DriverMotor(config.right, inverted=config.right_motor_inverted),
        standby_pin=config.standby_pin,
    )
    try:
        rig.enable()
        print("WASD drive test ready. W/S/A/D move, Space stops, Q quits.")
        print(
            f"speed={config.speed:.0%}, pulse={config.pulse_seconds:g}s, "
            f"effective_voltage={config.speed * config.supply_voltage:.2f}V"
        )
        with raw_terminal():
            while True:
                key = read_key(timeout_seconds=0.1)
                if key is None:
                    continue
                if key == "q":
                    print("\nquit")
                    break
                if key == " ":
                    rig.stop()
                    print("\nstop", flush=True)
                    continue

                label = dispatch_key(rig, key, config.speed)
                if label is None:
                    continue

                print(f"\r{label:<12}", end="", flush=True)
                sleep(config.pulse_seconds)
                rig.stop()
    finally:
        rig.close()


def dispatch_key(rig: DifferentialDrive, key: str, speed: float) -> str | None:
    if key == "w":
        rig.forward(speed)
        return "forward"
    if key == "s":
        rig.reverse(speed)
        return "reverse"
    if key == "a":
        rig.rotate_left(speed)
        return "left"
    if key == "d":
        rig.rotate_right(speed)
        return "right"
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive WASD motor control test.")
    parser.add_argument("--speed", type=float, default=0.18, help="PWM duty cycle from 0.0 to 1.0.")
    parser.add_argument("--pulse-seconds", type=float, default=0.15)
    parser.add_argument("--supply-voltage", type=float, default=7.4)
    parser.add_argument("--motor-voltage-limit", type=float, default=6.0)
    parser.add_argument("--left-motor-inverted", dest="left_motor_inverted", action="store_true", default=True)
    parser.add_argument("--left-motor-normal", dest="left_motor_inverted", action="store_false")
    parser.add_argument("--right-motor-inverted", action="store_true")
    parser.add_argument("--left-pwm", type=int, default=13)
    parser.add_argument("--left-in1", type=int, default=26)
    parser.add_argument("--left-in2", type=int, default=19)
    parser.add_argument("--right-pwm", type=int, default=12)
    parser.add_argument("--right-in1", type=int, default=20)
    parser.add_argument("--right-in2", type=int, default=21)
    parser.add_argument("--standby", type=int, default=16)
    return parser.parse_args()


def build_config(args: argparse.Namespace) -> DriveConfig:
    if not 0.0 <= args.speed <= 1.0:
        raise ValueError("--speed must be between 0.0 and 1.0")
    if args.pulse_seconds <= 0:
        raise ValueError("--pulse-seconds must be greater than zero")
    if args.supply_voltage <= 0:
        raise ValueError("--supply-voltage must be greater than zero")
    if args.motor_voltage_limit <= 0:
        raise ValueError("--motor-voltage-limit must be greater than zero")
    validate_motor_voltage(args.speed, args.supply_voltage, args.motor_voltage_limit)

    return DriveConfig(
        left=MotorPins(pwm=args.left_pwm, in1=args.left_in1, in2=args.left_in2),
        right=MotorPins(pwm=args.right_pwm, in1=args.right_in1, in2=args.right_in2),
        standby_pin=args.standby,
        speed=args.speed,
        pulse_seconds=args.pulse_seconds,
        supply_voltage=args.supply_voltage,
        motor_voltage_limit=args.motor_voltage_limit,
        left_motor_inverted=args.left_motor_inverted,
        right_motor_inverted=args.right_motor_inverted,
    )


def main() -> None:
    args = parse_args()
    config = build_config(args)
    run_teleop(config)


if __name__ == "__main__":
    main()
