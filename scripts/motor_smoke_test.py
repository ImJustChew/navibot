"""Smoke test both drive motors on Raspberry Pi GPIO.

This script assumes a two-motor driver with IN1/IN2 direction pins, one PWM
pin per motor, and a shared standby pin. The defaults match docs/gpio.md.
Run it with the robot lifted off the ground.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from time import sleep


@dataclass(frozen=True)
class MotorPins:
    pwm: int
    in1: int
    in2: int


@dataclass(frozen=True)
class MotorTestConfig:
    left: MotorPins
    right: MotorPins
    standby_pin: int
    speed: float
    step_seconds: float
    settle_seconds: float


class DriverMotor:
    def __init__(self, pins: MotorPins) -> None:
        try:
            from gpiozero import OutputDevice, PWMOutputDevice
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self._pwm = PWMOutputDevice(pins.pwm, frequency=1000, initial_value=0)
        self._in1 = OutputDevice(pins.in1, initial_value=False)
        self._in2 = OutputDevice(pins.in2, initial_value=False)

    def forward(self, speed: float) -> None:
        self._in1.on()
        self._in2.off()
        self._pwm.value = speed

    def reverse(self, speed: float) -> None:
        self._in1.off()
        self._in2.on()
        self._pwm.value = speed

    def stop(self) -> None:
        self._pwm.value = 0
        self._in1.off()
        self._in2.off()

    def close(self) -> None:
        self.stop()
        self._pwm.close()
        self._in1.close()
        self._in2.close()


class MotorRig:
    def __init__(self, config: MotorTestConfig) -> None:
        try:
            from gpiozero import OutputDevice
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self.left = DriverMotor(config.left)
        self.right = DriverMotor(config.right)
        self._standby = OutputDevice(config.standby_pin, initial_value=False)

    def enable(self) -> None:
        self._standby.on()

    def stop(self) -> None:
        self.left.stop()
        self.right.stop()

    def close(self) -> None:
        self.stop()
        self._standby.off()
        self.left.close()
        self.right.close()
        self._standby.close()


@contextmanager
def motor_rig(config: MotorTestConfig) -> Iterator[MotorRig]:
    rig = MotorRig(config)
    try:
        rig.enable()
        yield rig
    finally:
        rig.close()


def pause_between_steps(rig: MotorRig, seconds: float) -> None:
    rig.stop()
    sleep(seconds)


def run_motor_sequence(config: MotorTestConfig) -> None:
    with motor_rig(config) as rig:
        steps = (
            ("left forward", lambda: rig.left.forward(config.speed)),
            ("left reverse", lambda: rig.left.reverse(config.speed)),
            ("right forward", lambda: rig.right.forward(config.speed)),
            ("right reverse", lambda: rig.right.reverse(config.speed)),
            (
                "both forward",
                lambda: (rig.left.forward(config.speed), rig.right.forward(config.speed)),
            ),
            (
                "both reverse",
                lambda: (rig.left.reverse(config.speed), rig.right.reverse(config.speed)),
            ),
            (
                "rotate left",
                lambda: (rig.left.reverse(config.speed), rig.right.forward(config.speed)),
            ),
            (
                "rotate right",
                lambda: (rig.left.forward(config.speed), rig.right.reverse(config.speed)),
            ),
        )

        for label, action in steps:
            print(f"Testing {label} at {config.speed:.0%} PWM")
            action()
            sleep(config.step_seconds)
            pause_between_steps(rig, config.settle_seconds)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Spin Navibot motors in all directions.")
    parser.add_argument("--speed", type=float, default=0.25, help="PWM duty cycle from 0.0 to 1.0.")
    parser.add_argument("--step-seconds", type=float, default=1.0, help="Seconds per movement.")
    parser.add_argument("--settle-seconds", type=float, default=0.5, help="Stop time between movements.")
    parser.add_argument("--yes", action="store_true", help="Skip the safety confirmation prompt.")
    parser.add_argument("--left-pwm", type=int, default=13)
    parser.add_argument("--left-in1", type=int, default=26)
    parser.add_argument("--left-in2", type=int, default=19)
    parser.add_argument("--right-pwm", type=int, default=12)
    parser.add_argument("--right-in1", type=int, default=20)
    parser.add_argument("--right-in2", type=int, default=21)
    parser.add_argument("--standby", type=int, default=16)
    return parser.parse_args()


def build_config(args: argparse.Namespace) -> MotorTestConfig:
    if not 0.0 <= args.speed <= 1.0:
        raise ValueError("--speed must be between 0.0 and 1.0")
    if args.step_seconds <= 0:
        raise ValueError("--step-seconds must be greater than zero")
    if args.settle_seconds < 0:
        raise ValueError("--settle-seconds cannot be negative")

    return MotorTestConfig(
        left=MotorPins(pwm=args.left_pwm, in1=args.left_in1, in2=args.left_in2),
        right=MotorPins(pwm=args.right_pwm, in1=args.right_in1, in2=args.right_in2),
        standby_pin=args.standby,
        speed=args.speed,
        step_seconds=args.step_seconds,
        settle_seconds=args.settle_seconds,
    )


def confirm_or_exit(args: argparse.Namespace, config: MotorTestConfig) -> None:
    if args.yes:
        return

    print("This will move the motors. Lift the robot so wheels cannot drive away.")
    print(f"Left motor: PWM {config.left.pwm}, IN1 {config.left.in1}, IN2 {config.left.in2}")
    print(f"Right motor: PWM {config.right.pwm}, IN1 {config.right.in1}, IN2 {config.right.in2}")
    print(f"Standby: GPIO {config.standby_pin}")
    answer = input("Type RUN to start: ")
    if answer != "RUN":
        raise SystemExit("Motor test cancelled.")


def main() -> None:
    args = parse_args()
    config = build_config(args)
    confirm_or_exit(args, config)
    run_motor_sequence(config)
    print("Motor smoke test complete.")


if __name__ == "__main__":
    main()

