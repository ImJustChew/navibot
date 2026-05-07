from dataclasses import dataclass


@dataclass(frozen=True)
class MotorPins:
    pwm: int
    in1: int
    in2: int


class DriverMotor:
    def __init__(self, pins: MotorPins, inverted: bool = False, brake_on_stop: bool = False) -> None:
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
        self._drive(duty, forward=True)

    def reverse(self, duty: float) -> None:
        self._drive(duty, forward=False)

    def stop(self) -> None:
        if self._brake_on_stop:
            self.brake()
        else:
            self.coast()

    def brake(self) -> None:
        self._in1.on()
        self._in2.on()
        self._pwm.value = 1

    def coast(self) -> None:
        self._pwm.value = 0
        self._in1.off()
        self._in2.off()

    def close(self) -> None:
        self.coast()
        self._pwm.close()
        self._in1.close()
        self._in2.close()

    def _drive(self, duty: float, forward: bool) -> None:
        effective_forward = not forward if self._inverted else forward
        if effective_forward:
            self._in1.on()
            self._in2.off()
        else:
            self._in1.off()
            self._in2.on()
        self._pwm.value = clamp(duty, 0.0, 1.0)


class DifferentialDrive:
    def __init__(
        self,
        left: DriverMotor,
        right: DriverMotor,
        standby_pin: int,
    ) -> None:
        try:
            from gpiozero import OutputDevice
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self.left = left
        self.right = right
        self._standby = OutputDevice(standby_pin, initial_value=False)

    def enable(self) -> None:
        self._standby.on()

    def forward(self, duty: float) -> None:
        self.left.forward(duty)
        self.right.forward(duty)

    def reverse(self, duty: float) -> None:
        self.left.reverse(duty)
        self.right.reverse(duty)

    def rotate_left(self, duty: float) -> None:
        self.left.reverse(duty)
        self.right.forward(duty)

    def rotate_right(self, duty: float) -> None:
        self.left.forward(duty)
        self.right.reverse(duty)

    def stop(self) -> None:
        self.left.stop()
        self.right.stop()

    def coast(self) -> None:
        self.left.coast()
        self.right.coast()

    def close(self) -> None:
        self.coast()
        self._standby.off()
        self.left.close()
        self.right.close()
        self._standby.close()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def validate_motor_voltage(speed: float, supply_voltage: float, motor_voltage_limit: float) -> None:
    if speed * supply_voltage > motor_voltage_limit:
        raise ValueError(
            "PWM would exceed the motor voltage limit: "
            f"{speed:.3f} * {supply_voltage:g}V = {speed * supply_voltage:.2f}V, "
            f"limit is {motor_voltage_limit:g}V"
        )

