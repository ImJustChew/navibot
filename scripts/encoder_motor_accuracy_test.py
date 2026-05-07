"""Drive each motor to encoder pulse targets.

Defaults match docs/gpio.md:
- left encoder A/B: GPIO 27 / GPIO 22
- right encoder A/B: GPIO 23 / GPIO 24
- left motor PWM/IN1/IN2: GPIO 13 / GPIO 26 / GPIO 19
- right motor PWM/IN1/IN2: GPIO 12 / GPIO 20 / GPIO 21
- motor driver standby: GPIO 16

The encoder default is 7 pulses per pin per wheel rotation. This script uses
encoder A rising edges as the control target and also reports B rising edges.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from threading import Event, Lock
from time import monotonic, sleep


@dataclass(frozen=True)
class MotorPins:
    pwm: int
    in1: int
    in2: int


@dataclass(frozen=True)
class EncoderPins:
    a: int
    b: int


@dataclass(frozen=True)
class WheelPins:
    motor: MotorPins
    encoder: EncoderPins


@dataclass(frozen=True)
class TestConfig:
    left: WheelPins
    right: WheelPins
    standby_pin: int
    speed: float
    rotations: float
    pulses_per_rotation: int
    timeout_seconds: float
    settle_seconds: float
    pull_up: bool


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


class EncoderCounter:
    def __init__(self, pins: EncoderPins, pull_up: bool) -> None:
        try:
            from gpiozero import DigitalInputDevice
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self._lock = Lock()
        self._target_reached = Event()
        self._target_a = 0
        self._count_a = 0
        self._count_b = 0
        self._last_b_state = False
        self._channel_a = DigitalInputDevice(pins.a, pull_up=pull_up)
        self._channel_b = DigitalInputDevice(pins.b, pull_up=pull_up)
        self._channel_a.when_activated = self._on_a_rising
        self._channel_b.when_activated = self._on_b_rising

    def reset(self, target_a: int) -> None:
        with self._lock:
            self._target_a = target_a
            self._count_a = 0
            self._count_b = 0
            self._last_b_state = self._channel_b.is_active
            self._target_reached.clear()

    @property
    def count_a(self) -> int:
        with self._lock:
            return self._count_a

    @property
    def count_b(self) -> int:
        with self._lock:
            return self._count_b

    @property
    def last_b_state(self) -> bool:
        with self._lock:
            return self._last_b_state

    def wait_for_target(self, timeout_seconds: float) -> bool:
        return self._target_reached.wait(timeout=timeout_seconds)

    def close(self) -> None:
        self._channel_a.close()
        self._channel_b.close()

    def _on_a_rising(self) -> None:
        with self._lock:
            self._count_a += 1
            self._last_b_state = self._channel_b.is_active
            if self._count_a >= self._target_a:
                self._target_reached.set()

    def _on_b_rising(self) -> None:
        with self._lock:
            self._count_b += 1


class TestedWheel:
    def __init__(self, pins: WheelPins, pull_up: bool) -> None:
        self.motor = DriverMotor(pins.motor)
        self.encoder = EncoderCounter(pins.encoder, pull_up=pull_up)

    def close(self) -> None:
        self.motor.close()
        self.encoder.close()


class TestRig:
    def __init__(self, config: TestConfig) -> None:
        try:
            from gpiozero import OutputDevice
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self.left = TestedWheel(config.left, pull_up=config.pull_up)
        self.right = TestedWheel(config.right, pull_up=config.pull_up)
        self._standby = OutputDevice(config.standby_pin, initial_value=False)

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


@contextmanager
def test_rig(config: TestConfig) -> Iterator[TestRig]:
    rig = TestRig(config)
    try:
        rig.enable()
        yield rig
    finally:
        rig.close()


def target_pulses(config: TestConfig) -> int:
    return max(1, round(config.rotations * config.pulses_per_rotation))


def move_to_target(
    label: str,
    wheel: TestedWheel,
    direction: str,
    target: int,
    config: TestConfig,
) -> None:
    wheel.encoder.reset(target)
    started_at = monotonic()
    print(f"{label} {direction}: target {target} A pulses at {config.speed:.0%} PWM")

    if direction == "forward":
        wheel.motor.forward(config.speed)
    elif direction == "reverse":
        wheel.motor.reverse(config.speed)
    else:
        raise ValueError(f"Unknown direction: {direction}")

    reached = wheel.encoder.wait_for_target(config.timeout_seconds)
    elapsed = monotonic() - started_at
    wheel.motor.stop()

    status = "OK" if reached else "TIMEOUT"
    print(
        f"{label} {direction}: {status}; "
        f"A={wheel.encoder.count_a}, B={wheel.encoder.count_b}, "
        f"last B active={wheel.encoder.last_b_state}, elapsed={elapsed:.2f}s"
    )
    sleep(config.settle_seconds)


def run_accuracy_test(config: TestConfig) -> None:
    target = target_pulses(config)
    with test_rig(config) as rig:
        for label, wheel in (("left", rig.left), ("right", rig.right)):
            move_to_target(label, wheel, "forward", target, config)
            move_to_target(label, wheel, "reverse", target, config)
    print("Encoder motor accuracy test complete.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Drive motors to encoder pulse targets.")
    parser.add_argument("--speed", type=float, default=0.25, help="PWM duty cycle from 0.0 to 1.0.")
    parser.add_argument("--rotations", type=float, default=1.0, help="Wheel rotations to command.")
    parser.add_argument(
        "--pulses-per-rotation",
        type=int,
        default=7,
        help="A-channel encoder pulses per wheel rotation.",
    )
    parser.add_argument("--timeout-seconds", type=float, default=5.0)
    parser.add_argument("--settle-seconds", type=float, default=0.5)
    parser.add_argument("--pull-up", dest="pull_up", action="store_true", default=True)
    parser.add_argument("--no-pull-up", dest="pull_up", action="store_false")
    parser.add_argument("--yes", action="store_true", help="Skip the safety confirmation prompt.")
    parser.add_argument("--left-pwm", type=int, default=13)
    parser.add_argument("--left-in1", type=int, default=26)
    parser.add_argument("--left-in2", type=int, default=19)
    parser.add_argument("--left-encoder-a", type=int, default=27)
    parser.add_argument("--left-encoder-b", type=int, default=22)
    parser.add_argument("--right-pwm", type=int, default=12)
    parser.add_argument("--right-in1", type=int, default=20)
    parser.add_argument("--right-in2", type=int, default=21)
    parser.add_argument("--right-encoder-a", type=int, default=23)
    parser.add_argument("--right-encoder-b", type=int, default=24)
    parser.add_argument("--standby", type=int, default=16)
    return parser.parse_args()


def build_config(args: argparse.Namespace) -> TestConfig:
    if not 0.0 <= args.speed <= 1.0:
        raise ValueError("--speed must be between 0.0 and 1.0")
    if args.rotations <= 0:
        raise ValueError("--rotations must be greater than zero")
    if args.pulses_per_rotation <= 0:
        raise ValueError("--pulses-per-rotation must be greater than zero")
    if args.timeout_seconds <= 0:
        raise ValueError("--timeout-seconds must be greater than zero")
    if args.settle_seconds < 0:
        raise ValueError("--settle-seconds cannot be negative")

    return TestConfig(
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
        rotations=args.rotations,
        pulses_per_rotation=args.pulses_per_rotation,
        timeout_seconds=args.timeout_seconds,
        settle_seconds=args.settle_seconds,
        pull_up=args.pull_up,
    )


def confirm_or_exit(args: argparse.Namespace, config: TestConfig) -> None:
    if args.yes:
        return

    print("This will move the motors by encoder count. Lift the robot off the ground.")
    print(f"Target: {config.rotations:g} rotations = {target_pulses(config)} A pulses")
    print(f"Left encoder: A GPIO {config.left.encoder.a}, B GPIO {config.left.encoder.b}")
    print(f"Right encoder: A GPIO {config.right.encoder.a}, B GPIO {config.right.encoder.b}")
    answer = input("Type RUN to start: ")
    if answer != "RUN":
        raise SystemExit("Encoder motor test cancelled.")


def main() -> None:
    args = parse_args()
    config = build_config(args)
    confirm_or_exit(args, config)
    run_accuracy_test(config)


if __name__ == "__main__":
    main()

