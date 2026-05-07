"""Drive forward by distance using both quadrature encoders and PID control.

The GA12-N20 encoder is on the motor shaft before the gearbox. Output wheel
counts therefore depend on encoder pulses, quadrature edge mode, and gear ratio:

    wheel_counts_per_rev = pulses_per_channel * 4 * gear_ratio

Defaults assume:
- 7 pulses per encoder channel per motor-shaft revolution
- x4 quadrature decoding, counting rising and falling edges on A and B
- 100:1 gearbox
- 43 mm wheel diameter

Set --gear-ratio to the actual motor gearbox ratio before trusting distances.
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
class PidGains:
    kp: float
    ki: float
    kd: float


@dataclass(frozen=True)
class DriveConfig:
    left: WheelPins
    right: WheelPins
    standby_pin: int
    distance_mm: float
    wheel_diameter_mm: float
    pulses_per_channel: int
    gear_ratio: float
    target_speed_mm_s: float
    min_pwm: float
    max_pwm: float
    loop_seconds: float
    timeout_seconds: float
    slow_zone_mm: float
    sync_gain: float
    left_gains: PidGains
    right_gains: PidGains
    pull_up: bool
    left_motor_inverted: bool
    right_motor_inverted: bool
    left_encoder_inverted: bool
    right_encoder_inverted: bool
    brake_on_stop: bool
    stall_seconds: float
    stall_min_pwm: float
    stall_min_rate: float
    max_skew_counts: int


@dataclass(frozen=True)
class WheelSample:
    counts: int
    abs_counts: int


class DriverMotor:
    def __init__(self, pins: MotorPins, inverted: bool, brake_on_stop: bool) -> None:
        try:
            from gpiozero import OutputDevice, PWMOutputDevice
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self._pwm = PWMOutputDevice(pins.pwm, frequency=1000, initial_value=0)
        self._in1 = OutputDevice(pins.in1, initial_value=False)
        self._in2 = OutputDevice(pins.in2, initial_value=False)
        self._inverted = inverted
        self._brake_on_stop = brake_on_stop

    def forward(self, duty: float) -> None:
        if self._inverted:
            self._in1.off()
            self._in2.on()
        else:
            self._in1.on()
            self._in2.off()
        self._pwm.value = clamp(duty, 0.0, 1.0)

    def stop(self) -> None:
        if self._brake_on_stop:
            self._in1.on()
            self._in2.on()
            self._pwm.value = 1
        else:
            self._pwm.value = 0
            self._in1.off()
            self._in2.off()

    def coast(self) -> None:
        self._pwm.value = 0
        self._in1.off()
        self._in2.off()

    def close(self) -> None:
        self.coast()
        self._pwm.close()
        self._in1.close()
        self._in2.close()


class QuadratureEncoder:
    def __init__(self, pins: EncoderPins, pull_up: bool, inverted: bool) -> None:
        try:
            import lgpio
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self._lgpio = lgpio
        self._lock = Lock()
        self._chip = lgpio.gpiochip_open(0)
        self._pin_a = pins.a
        self._pin_b = pins.b
        self._counts = 0
        self._bad_transitions = 0
        self._multiplier = -1 if inverted else 1
        self._edge_count = 0
        self._last_tick_ns = 0

        line_flags = lgpio.SET_PULL_UP if pull_up else 0
        lgpio.gpio_claim_alert(self._chip, pins.a, lgpio.BOTH_EDGES, line_flags)
        lgpio.gpio_claim_alert(self._chip, pins.b, lgpio.BOTH_EDGES, line_flags)

        self._a_level = lgpio.gpio_read(self._chip, pins.a)
        self._b_level = lgpio.gpio_read(self._chip, pins.b)
        self._state = self._levels_to_state()

        self._callback_a = lgpio.callback(self._chip, pins.a, lgpio.BOTH_EDGES, self._on_edge)
        self._callback_b = lgpio.callback(self._chip, pins.b, lgpio.BOTH_EDGES, self._on_edge)

    def reset(self) -> None:
        with self._lock:
            self._counts = 0
            self._bad_transitions = 0
            self._edge_count = 0
            self._last_tick_ns = 0
            self._a_level = self._lgpio.gpio_read(self._chip, self._pin_a)
            self._b_level = self._lgpio.gpio_read(self._chip, self._pin_b)
            self._state = self._levels_to_state()

    def sample(self) -> WheelSample:
        with self._lock:
            return WheelSample(counts=self._counts, abs_counts=abs(self._counts))

    @property
    def bad_transitions(self) -> int:
        with self._lock:
            return self._bad_transitions

    def close(self) -> None:
        self._callback_a.cancel()
        self._callback_b.cancel()
        self._lgpio.gpio_free(self._chip, self._pin_a)
        self._lgpio.gpio_free(self._chip, self._pin_b)
        self._lgpio.gpiochip_close(self._chip)

    def _levels_to_state(self) -> int:
        return (int(self._a_level) << 1) | int(self._b_level)

    def _on_edge(self, chip: int, gpio: int, level: int, tick_ns: int) -> None:
        del chip
        if level not in (0, 1):
            return

        with self._lock:
            if gpio == self._pin_a:
                self._a_level = level
            elif gpio == self._pin_b:
                self._b_level = level
            else:
                return

            previous = self._state
            current = self._levels_to_state()
            if previous == current:
                return

            delta = QUADRATURE_DELTA.get((previous, current))
            if delta is None:
                self._bad_transitions += 1
            else:
                self._counts += delta * self._multiplier
            self._state = current
            self._edge_count += 1
            self._last_tick_ns = tick_ns


class Wheel:
    def __init__(
        self,
        pins: WheelPins,
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
        self.encoder = QuadratureEncoder(
            pins.encoder,
            pull_up=pull_up,
            inverted=encoder_inverted,
        )

    def close(self) -> None:
        self.motor.close()
        self.encoder.close()


class Rig:
    def __init__(self, config: DriveConfig) -> None:
        try:
            from gpiozero import OutputDevice
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self.left = Wheel(
            config.left,
            pull_up=config.pull_up,
            motor_inverted=config.left_motor_inverted,
            encoder_inverted=config.left_encoder_inverted,
            brake_on_stop=config.brake_on_stop,
        )
        self.right = Wheel(
            config.right,
            pull_up=config.pull_up,
            motor_inverted=config.right_motor_inverted,
            encoder_inverted=config.right_encoder_inverted,
            brake_on_stop=config.brake_on_stop,
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


class PidController:
    def __init__(self, gains: PidGains) -> None:
        self._gains = gains
        self._integral = 0.0
        self._previous_error = 0.0

    def update(self, error: float, dt: float) -> float:
        self._integral += error * dt
        derivative = (error - self._previous_error) / dt if dt > 0 else 0.0
        self._previous_error = error
        return (
            self._gains.kp * error
            + self._gains.ki * self._integral
            + self._gains.kd * derivative
        )


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wheel_counts_per_revolution(config: DriveConfig) -> float:
    return config.pulses_per_channel * 4 * config.gear_ratio


def mm_per_count(config: DriveConfig) -> float:
    circumference_mm = 3.141592653589793 * config.wheel_diameter_mm
    return circumference_mm / wheel_counts_per_revolution(config)


def target_counts(config: DriveConfig) -> int:
    return max(1, round(config.distance_mm / mm_per_count(config)))


def target_counts_per_second(config: DriveConfig) -> float:
    return config.target_speed_mm_s / mm_per_count(config)


def scaled_target_rate(config: DriveConfig, remaining_counts: int) -> float:
    slow_zone_counts = max(1, round(config.slow_zone_mm / mm_per_count(config)))
    scale = clamp(remaining_counts / slow_zone_counts, 0.25, 1.0)
    return target_counts_per_second(config) * scale


def run_drive(config: DriveConfig) -> None:
    target = target_counts(config)
    left_pid = PidController(config.left_gains)
    right_pid = PidController(config.right_gains)

    rig = Rig(config)
    try:
        rig.enable()
        rig.left.encoder.reset()
        rig.right.encoder.reset()

        left_pwm = config.min_pwm
        right_pwm = config.min_pwm
        previous_left = rig.left.encoder.sample()
        previous_right = rig.right.encoder.sample()
        previous_time = monotonic()
        started_at = previous_time
        left_stall_started_at: float | None = None
        right_stall_started_at: float | None = None

        print(
            "Driving forward "
            f"{config.distance_mm:g} mm; target={target} counts; "
            f"counts/rev={wheel_counts_per_revolution(config):.1f}; "
            f"mm/count={mm_per_count(config):.4f}"
        )
        print(
            "Options: "
            f"left_motor_inverted={config.left_motor_inverted}, "
            f"right_motor_inverted={config.right_motor_inverted}, "
            f"left_encoder_inverted={config.left_encoder_inverted}, "
            f"right_encoder_inverted={config.right_encoder_inverted}, "
            f"brake_on_stop={config.brake_on_stop}"
        )

        while True:
            sleep(config.loop_seconds)
            now = monotonic()
            dt = now - previous_time
            elapsed = now - started_at

            left = rig.left.encoder.sample()
            right = rig.right.encoder.sample()
            left_delta = left.abs_counts - previous_left.abs_counts
            right_delta = right.abs_counts - previous_right.abs_counts
            left_rate = left_delta / dt
            right_rate = right_delta / dt
            average_counts = (left.abs_counts + right.abs_counts) / 2
            remaining = max(0, target - round(average_counts))

            if left.abs_counts >= target and right.abs_counts >= target:
                break
            if elapsed >= config.timeout_seconds:
                print("Timed out before target distance.")
                break
            if abs(left.abs_counts - right.abs_counts) >= config.max_skew_counts:
                print(
                    "Aborting: wheel encoder skew exceeded limit "
                    f"({abs(left.abs_counts - right.abs_counts)} >= {config.max_skew_counts})."
                )
                break

            target_rate = scaled_target_rate(config, remaining)
            sync_error = left.abs_counts - right.abs_counts

            left_pwm += left_pid.update(target_rate - left_rate, dt)
            right_pwm += right_pid.update(target_rate - right_rate, dt)

            correction = config.sync_gain * sync_error
            left_pwm -= correction
            right_pwm += correction

            if left.abs_counts >= target:
                left_pwm = 0.0
            if right.abs_counts >= target:
                right_pwm = 0.0

            skew_deadband_counts = 20
            left_low_pwm = (
                0.0
                if left.abs_counts >= target or left.abs_counts > right.abs_counts + skew_deadband_counts
                else config.min_pwm
            )
            right_low_pwm = (
                0.0
                if right.abs_counts >= target or right.abs_counts > left.abs_counts + skew_deadband_counts
                else config.min_pwm
            )
            left_pwm = clamp(left_pwm, left_low_pwm, config.max_pwm)
            right_pwm = clamp(right_pwm, right_low_pwm, config.max_pwm)

            left_should_move = left.abs_counts < target and left_pwm >= config.stall_min_pwm
            right_should_move = right.abs_counts < target and right_pwm >= config.stall_min_pwm
            left_is_stalled = left_should_move and left_rate < config.stall_min_rate
            right_is_stalled = right_should_move and right_rate < config.stall_min_rate

            if left_is_stalled:
                left_stall_started_at = left_stall_started_at or now
            else:
                left_stall_started_at = None

            if right_is_stalled:
                right_stall_started_at = right_stall_started_at or now
            else:
                right_stall_started_at = None

            if left_stall_started_at is not None and now - left_stall_started_at >= config.stall_seconds:
                print("Aborting: left wheel appears stalled while PWM is applied.")
                break
            if right_stall_started_at is not None and now - right_stall_started_at >= config.stall_seconds:
                print("Aborting: right wheel appears stalled while PWM is applied.")
                break

            if left.abs_counts >= target:
                rig.left.motor.stop()
            elif left_pwm <= 0:
                rig.left.motor.coast()
            else:
                rig.left.motor.forward(left_pwm)

            if right.abs_counts >= target:
                rig.right.motor.stop()
            elif right_pwm <= 0:
                rig.right.motor.coast()
            else:
                rig.right.motor.forward(right_pwm)

            print(
                f"t={elapsed:5.2f}s "
                f"L={left.abs_counts:5d} R={right.abs_counts:5d} "
                f"Lrate={left_rate:7.1f}/s Rrate={right_rate:7.1f}/s "
                f"Lpwm={left_pwm:.3f} Rpwm={right_pwm:.3f}"
            )

            previous_left = left
            previous_right = right
            previous_time = now

        rig.stop()
        left = rig.left.encoder.sample()
        right = rig.right.encoder.sample()
        average_mm = ((left.abs_counts + right.abs_counts) / 2) * mm_per_count(config)
        print(
            f"Final: L={left.counts} ({left.abs_counts} abs), "
            f"R={right.counts} ({right.abs_counts} abs), "
            f"estimated distance={average_mm:.1f} mm"
        )
        print(
            f"Bad quadrature transitions: "
            f"L={rig.left.encoder.bad_transitions}, R={rig.right.encoder.bad_transitions}"
        )
    finally:
        rig.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Drive forward by encoder distance using PID.")
    parser.add_argument("--distance-mm", type=float, default=200.0)
    parser.add_argument("--wheel-diameter-mm", type=float, default=43.0)
    parser.add_argument("--pulses-per-channel", type=int, default=7)
    parser.add_argument("--gear-ratio", type=float, default=100.0)
    parser.add_argument("--target-speed-mm-s", type=float, default=80.0)
    parser.add_argument("--min-pwm", type=float, default=0.18)
    parser.add_argument("--max-pwm", type=float, default=0.45)
    parser.add_argument("--loop-seconds", type=float, default=0.05)
    parser.add_argument("--timeout-seconds", type=float, default=10.0)
    parser.add_argument("--slow-zone-mm", type=float, default=80.0)
    parser.add_argument("--sync-gain", type=float, default=0.00008)
    parser.add_argument("--kp", type=float, default=0.00035)
    parser.add_argument("--ki", type=float, default=0.00002)
    parser.add_argument("--kd", type=float, default=0.0)
    parser.add_argument("--pull-up", dest="pull_up", action="store_true", default=True)
    parser.add_argument("--no-pull-up", dest="pull_up", action="store_false")
    parser.add_argument("--left-motor-inverted", dest="left_motor_inverted", action="store_true", default=True)
    parser.add_argument("--left-motor-normal", dest="left_motor_inverted", action="store_false")
    parser.add_argument("--right-motor-inverted", action="store_true")
    parser.add_argument("--left-encoder-inverted", action="store_true")
    parser.add_argument(
        "--right-encoder-inverted",
        dest="right_encoder_inverted",
        action="store_true",
        default=True,
    )
    parser.add_argument("--right-encoder-normal", dest="right_encoder_inverted", action="store_false")
    parser.add_argument("--brake-on-stop", dest="brake_on_stop", action="store_true", default=True)
    parser.add_argument("--coast-on-stop", dest="brake_on_stop", action="store_false")
    parser.add_argument("--stall-seconds", type=float, default=0.75)
    parser.add_argument("--stall-min-pwm", type=float, default=0.25)
    parser.add_argument("--stall-min-rate", type=float, default=3.0)
    parser.add_argument("--max-skew-counts", type=int, default=350)
    parser.add_argument("--yes", action="store_true", help="Skip the safety confirmation prompt.")
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


def build_config(args: argparse.Namespace) -> DriveConfig:
    if args.distance_mm <= 0:
        raise ValueError("--distance-mm must be greater than zero")
    if args.wheel_diameter_mm <= 0:
        raise ValueError("--wheel-diameter-mm must be greater than zero")
    if args.pulses_per_channel <= 0:
        raise ValueError("--pulses-per-channel must be greater than zero")
    if args.gear_ratio <= 0:
        raise ValueError("--gear-ratio must be greater than zero")
    if args.target_speed_mm_s <= 0:
        raise ValueError("--target-speed-mm-s must be greater than zero")
    if not 0 <= args.min_pwm <= args.max_pwm <= 1:
        raise ValueError("--min-pwm and --max-pwm must satisfy 0 <= min <= max <= 1")
    if args.loop_seconds <= 0:
        raise ValueError("--loop-seconds must be greater than zero")
    if args.timeout_seconds <= 0:
        raise ValueError("--timeout-seconds must be greater than zero")
    if args.slow_zone_mm <= 0:
        raise ValueError("--slow-zone-mm must be greater than zero")
    if args.stall_seconds <= 0:
        raise ValueError("--stall-seconds must be greater than zero")
    if not 0 <= args.stall_min_pwm <= 1:
        raise ValueError("--stall-min-pwm must be between 0.0 and 1.0")
    if args.stall_min_rate < 0:
        raise ValueError("--stall-min-rate cannot be negative")
    if args.max_skew_counts <= 0:
        raise ValueError("--max-skew-counts must be greater than zero")

    gains = PidGains(kp=args.kp, ki=args.ki, kd=args.kd)
    return DriveConfig(
        left=WheelPins(
            motor=MotorPins(pwm=args.left_pwm, in1=args.left_in1, in2=args.left_in2),
            encoder=EncoderPins(a=args.left_encoder_a, b=args.left_encoder_b),
        ),
        right=WheelPins(
            motor=MotorPins(pwm=args.right_pwm, in1=args.right_in1, in2=args.right_in2),
            encoder=EncoderPins(a=args.right_encoder_a, b=args.right_encoder_b),
        ),
        standby_pin=args.standby,
        distance_mm=args.distance_mm,
        wheel_diameter_mm=args.wheel_diameter_mm,
        pulses_per_channel=args.pulses_per_channel,
        gear_ratio=args.gear_ratio,
        target_speed_mm_s=args.target_speed_mm_s,
        min_pwm=args.min_pwm,
        max_pwm=args.max_pwm,
        loop_seconds=args.loop_seconds,
        timeout_seconds=args.timeout_seconds,
        slow_zone_mm=args.slow_zone_mm,
        sync_gain=args.sync_gain,
        left_gains=gains,
        right_gains=gains,
        pull_up=args.pull_up,
        left_motor_inverted=args.left_motor_inverted,
        right_motor_inverted=args.right_motor_inverted,
        left_encoder_inverted=args.left_encoder_inverted,
        right_encoder_inverted=args.right_encoder_inverted,
        brake_on_stop=args.brake_on_stop,
        stall_seconds=args.stall_seconds,
        stall_min_pwm=args.stall_min_pwm,
        stall_min_rate=args.stall_min_rate,
        max_skew_counts=args.max_skew_counts,
    )


def confirm_or_exit(args: argparse.Namespace, config: DriveConfig) -> None:
    if args.yes:
        return

    print("This will drive both motors forward. Lift the robot or clear a test lane.")
    print(f"Distance: {config.distance_mm:g} mm")
    print(f"Wheel diameter: {config.wheel_diameter_mm:g} mm")
    print(f"Encoder: {config.pulses_per_channel} pulses/channel, gear ratio {config.gear_ratio:g}:1")
    print(f"Target counts per wheel: {target_counts(config)}")
    print(f"Brake on stop: {config.brake_on_stop}")
    answer = input("Type RUN to start: ")
    if answer != "RUN":
        raise SystemExit("PID drive test cancelled.")


def main() -> None:
    args = parse_args()
    config = build_config(args)
    confirm_or_exit(args, config)
    run_drive(config)


if __name__ == "__main__":
    main()
