"""Validate motor and encoder GPIO wiring one wheel at a time.

This is a GPIO diagnostic, not a distance controller. It drives only one motor
at a time for a fixed duration and reports raw encoder activity:

- A rising edges
- A falling edges
- B rising edges
- B falling edges
- signed x4 quadrature count
- invalid quadrature transitions

Defaults match docs/gpio.md. The left motor is inverted by default because this
chassis needs that for a forward command.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from threading import Lock
from time import monotonic, sleep


QUADRATURE_DELTA = {
    (0b00, 0b01): 1,
    (0b01, 0b11): 1,
    (0b11, 0b10): 1,
    (0b10, 0b00): 1,
    (0b00, 0b10): -1,
    (0b10, 0b11): -1,
    (0b11, 0b01): -1,
    (0b01, 0b00): -1,
}


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
    duration_seconds: float
    settle_seconds: float
    pull_up: bool
    left_motor_inverted: bool
    right_motor_inverted: bool
    left_encoder_inverted: bool
    right_encoder_inverted: bool
    wheels: tuple[str, ...]
    directions: tuple[str, ...]


@dataclass(frozen=True)
class EncoderSnapshot:
    a_rising: int
    a_falling: int
    b_rising: int
    b_falling: int
    quadrature_count: int
    bad_transitions: int
    state: int

    @property
    def total_edges(self) -> int:
        return self.a_rising + self.a_falling + self.b_rising + self.b_falling


class DriverMotor:
    def __init__(self, pins: MotorPins, inverted: bool) -> None:
        try:
            from gpiozero import OutputDevice, PWMOutputDevice
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self._pwm = PWMOutputDevice(pins.pwm, frequency=1000, initial_value=0)
        self._in1 = OutputDevice(pins.in1, initial_value=False)
        self._in2 = OutputDevice(pins.in2, initial_value=False)
        self._inverted = inverted

    def forward(self, speed: float) -> None:
        self._drive(speed, forward=True)

    def reverse(self, speed: float) -> None:
        self._drive(speed, forward=False)

    def stop(self) -> None:
        self._pwm.value = 0
        self._in1.off()
        self._in2.off()

    def close(self) -> None:
        self.stop()
        self._pwm.close()
        self._in1.close()
        self._in2.close()

    def _drive(self, speed: float, forward: bool) -> None:
        effective_forward = not forward if self._inverted else forward
        if effective_forward:
            self._in1.on()
            self._in2.off()
        else:
            self._in1.off()
            self._in2.on()
        self._pwm.value = speed


class QuadratureProbe:
    def __init__(self, pins: EncoderPins, pull_up: bool, inverted: bool) -> None:
        try:
            from gpiozero import DigitalInputDevice
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self._lock = Lock()
        self._channel_a = DigitalInputDevice(pins.a, pull_up=pull_up)
        self._channel_b = DigitalInputDevice(pins.b, pull_up=pull_up)
        self._multiplier = -1 if inverted else 1
        self._a_rising = 0
        self._a_falling = 0
        self._b_rising = 0
        self._b_falling = 0
        self._quadrature_count = 0
        self._bad_transitions = 0
        self._state = self._read_state()

        self._channel_a.when_activated = self._on_a_rising
        self._channel_a.when_deactivated = self._on_a_falling
        self._channel_b.when_activated = self._on_b_rising
        self._channel_b.when_deactivated = self._on_b_falling

    def reset(self) -> None:
        with self._lock:
            self._a_rising = 0
            self._a_falling = 0
            self._b_rising = 0
            self._b_falling = 0
            self._quadrature_count = 0
            self._bad_transitions = 0
            self._state = self._read_state()

    def snapshot(self) -> EncoderSnapshot:
        with self._lock:
            return EncoderSnapshot(
                a_rising=self._a_rising,
                a_falling=self._a_falling,
                b_rising=self._b_rising,
                b_falling=self._b_falling,
                quadrature_count=self._quadrature_count,
                bad_transitions=self._bad_transitions,
                state=self._state,
            )

    def close(self) -> None:
        self._channel_a.close()
        self._channel_b.close()

    def _read_state(self) -> int:
        return (int(self._channel_a.is_active) << 1) | int(self._channel_b.is_active)

    def _count_quadrature(self) -> None:
        previous = self._state
        current = self._read_state()
        if previous == current:
            return

        delta = QUADRATURE_DELTA.get((previous, current))
        if delta is None:
            self._bad_transitions += 1
        else:
            self._quadrature_count += delta * self._multiplier
        self._state = current

    def _on_a_rising(self) -> None:
        with self._lock:
            self._a_rising += 1
            self._count_quadrature()

    def _on_a_falling(self) -> None:
        with self._lock:
            self._a_falling += 1
            self._count_quadrature()

    def _on_b_rising(self) -> None:
        with self._lock:
            self._b_rising += 1
            self._count_quadrature()

    def _on_b_falling(self) -> None:
        with self._lock:
            self._b_falling += 1
            self._count_quadrature()


class WheelProbe:
    def __init__(
        self,
        pins: WheelPins,
        motor_inverted: bool,
        encoder_inverted: bool,
        pull_up: bool,
    ) -> None:
        self.motor = DriverMotor(pins.motor, inverted=motor_inverted)
        self.encoder = QuadratureProbe(pins.encoder, pull_up=pull_up, inverted=encoder_inverted)

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

        self.left = WheelProbe(
            config.left,
            motor_inverted=config.left_motor_inverted,
            encoder_inverted=config.left_encoder_inverted,
            pull_up=config.pull_up,
        )
        self.right = WheelProbe(
            config.right,
            motor_inverted=config.right_motor_inverted,
            encoder_inverted=config.right_encoder_inverted,
            pull_up=config.pull_up,
        )
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


def run_wheel_test(
    label: str,
    wheel: WheelProbe,
    other: WheelProbe,
    pins: WheelPins,
    direction: str,
    config: TestConfig,
) -> None:
    other.motor.stop()
    wheel.motor.stop()
    wheel.encoder.reset()
    sleep(config.settle_seconds)

    print("")
    print(f"Testing {label} {direction}")
    print(f"  motor pins: PWM GPIO {pins.motor.pwm}, IN1 GPIO {pins.motor.in1}, IN2 GPIO {pins.motor.in2}")
    print(f"  encoder pins: A GPIO {pins.encoder.a}, B GPIO {pins.encoder.b}")
    print(f"  speed={config.speed:.0%}, duration={config.duration_seconds:g}s")

    started_at = monotonic()
    if direction == "forward":
        wheel.motor.forward(config.speed)
    elif direction == "reverse":
        wheel.motor.reverse(config.speed)
    else:
        raise ValueError(f"Unknown direction: {direction}")

    sleep(config.duration_seconds)
    elapsed = monotonic() - started_at
    wheel.motor.stop()
    sleep(config.settle_seconds)

    snapshot = wheel.encoder.snapshot()
    signed_direction = "positive" if snapshot.quadrature_count > 0 else "negative"
    if snapshot.quadrature_count == 0:
        signed_direction = "none"

    print(
        "  encoder edges: "
        f"A+={snapshot.a_rising}, A-={snapshot.a_falling}, "
        f"B+={snapshot.b_rising}, B-={snapshot.b_falling}, total={snapshot.total_edges}"
    )
    print(
        "  quadrature: "
        f"count={snapshot.quadrature_count}, direction={signed_direction}, "
        f"bad_transitions={snapshot.bad_transitions}, final_state={snapshot.state:02b}"
    )
    print(f"  rate: {abs(snapshot.quadrature_count) / elapsed:.1f} quadrature counts/s")

    warnings = diagnose_snapshot(snapshot)
    for warning in warnings:
        print(f"  WARNING: {warning}")


def diagnose_snapshot(snapshot: EncoderSnapshot) -> list[str]:
    warnings: list[str] = []
    if snapshot.total_edges == 0:
        return ["no encoder edges detected; check encoder VCC/GND/A/B pins"]

    if snapshot.a_rising + snapshot.a_falling == 0:
        warnings.append("channel A has no edges")
    if snapshot.b_rising + snapshot.b_falling == 0:
        warnings.append("channel B has no edges")
    if abs(snapshot.quadrature_count) < snapshot.total_edges * 0.5:
        warnings.append("quadrature count is low relative to raw edges; A/B may be noisy or swapped")
    if snapshot.bad_transitions > snapshot.total_edges * 0.05:
        warnings.append("many invalid quadrature transitions; check signal quality and callback rate")
    return warnings


def run_gpio_test(config: TestConfig) -> None:
    rig = TestRig(config)
    try:
        rig.enable()
        for wheel_name in config.wheels:
            if wheel_name == "left":
                wheel = rig.left
                other = rig.right
                pins = config.left
            elif wheel_name == "right":
                wheel = rig.right
                other = rig.left
                pins = config.right
            else:
                raise ValueError(f"Unknown wheel: {wheel_name}")

            for direction in config.directions:
                run_wheel_test(wheel_name, wheel, other, pins, direction, config)
        print("")
        print("Encoder GPIO validation complete.")
    finally:
        rig.close()


def parse_csv_choices(value: str, allowed: set[str], label: str) -> tuple[str, ...]:
    choices = tuple(part.strip().lower() for part in value.split(",") if part.strip())
    unknown = sorted(set(choices) - allowed)
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown {label}: {', '.join(unknown)}")
    if not choices:
        raise argparse.ArgumentTypeError(f"at least one {label} is required")
    return choices


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate motor and encoder GPIO one wheel at a time.")
    parser.add_argument("--speed", type=float, default=0.20, help="PWM duty cycle from 0.0 to 1.0.")
    parser.add_argument("--duration-seconds", type=float, default=1.0)
    parser.add_argument("--settle-seconds", type=float, default=0.3)
    parser.add_argument(
        "--wheel",
        type=lambda value: parse_csv_choices(value, {"left", "right"}, "wheel"),
        default=("left", "right"),
        help="Wheel to test: left, right, or left,right.",
    )
    parser.add_argument(
        "--direction",
        type=lambda value: parse_csv_choices(value, {"forward", "reverse"}, "direction"),
        default=("forward", "reverse"),
        help="Direction to test: forward, reverse, or forward,reverse.",
    )
    parser.add_argument("--pull-up", dest="pull_up", action="store_true", default=True)
    parser.add_argument("--no-pull-up", dest="pull_up", action="store_false")
    parser.add_argument("--left-motor-inverted", dest="left_motor_inverted", action="store_true", default=True)
    parser.add_argument("--left-motor-normal", dest="left_motor_inverted", action="store_false")
    parser.add_argument("--right-motor-inverted", action="store_true")
    parser.add_argument("--left-encoder-inverted", action="store_true")
    parser.add_argument("--right-encoder-inverted", action="store_true")
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
    if args.duration_seconds <= 0:
        raise ValueError("--duration-seconds must be greater than zero")
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
        duration_seconds=args.duration_seconds,
        settle_seconds=args.settle_seconds,
        pull_up=args.pull_up,
        left_motor_inverted=args.left_motor_inverted,
        right_motor_inverted=args.right_motor_inverted,
        left_encoder_inverted=args.left_encoder_inverted,
        right_encoder_inverted=args.right_encoder_inverted,
        wheels=args.wheel,
        directions=args.direction,
    )


def confirm_or_exit(args: argparse.Namespace, config: TestConfig) -> None:
    if args.yes:
        return

    print("This will run one motor at a time. Lift the robot off the ground.")
    print(f"Wheels: {', '.join(config.wheels)}")
    print(f"Directions: {', '.join(config.directions)}")
    print(f"Left motor inverted for forward: {config.left_motor_inverted}")
    print(f"Right motor inverted for forward: {config.right_motor_inverted}")
    answer = input("Type RUN to start: ")
    if answer != "RUN":
        raise SystemExit("Encoder GPIO validation cancelled.")


def main() -> None:
    args = parse_args()
    config = build_config(args)
    confirm_or_exit(args, config)
    run_gpio_test(config)


if __name__ == "__main__":
    main()
