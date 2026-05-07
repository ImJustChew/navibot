"""Bring up four VL53L1X sensors with XSHUT pins and print readings.

Default Navibot mapping:
- left45: XSHUT GPIO 25 -> I2C 0x30
- front:  XSHUT GPIO 8  -> I2C 0x31
- right45:XSHUT GPIO 7  -> I2C 0x32
- back:   XSHUT GPIO 1  -> I2C 0x33

The VL53L1X default address is 0x29. XSHUT is active low, so the script holds
all sensors off, enables them one at a time, then changes each address.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from time import monotonic, sleep


@dataclass(frozen=True)
class SensorSpec:
    name: str
    xshut_gpio: int
    address: int


@dataclass(frozen=True)
class SensorHandle:
    spec: SensorSpec
    xshut: object
    sensor: object


DEFAULT_SENSORS = (
    SensorSpec(name="left45", xshut_gpio=25, address=0x30),
    SensorSpec(name="front", xshut_gpio=8, address=0x31),
    SensorSpec(name="right45", xshut_gpio=7, address=0x32),
    SensorSpec(name="back", xshut_gpio=1, address=0x33),
)


def parse_sensor_spec(value: str) -> SensorSpec:
    parts = value.split(":")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("sensor spec must be name:xshut_gpio:i2c_address")

    name, gpio_text, address_text = parts
    if not name:
        raise argparse.ArgumentTypeError("sensor name cannot be empty")

    try:
        xshut_gpio = int(gpio_text, 0)
        address = int(address_text, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc

    if not 0x08 <= address <= 0x77:
        raise argparse.ArgumentTypeError("I2C address must be between 0x08 and 0x77")
    return SensorSpec(name=name, xshut_gpio=xshut_gpio, address=address)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Configure VL53L1X sensors with XSHUT pins.")
    parser.add_argument(
        "--sensor",
        action="append",
        type=parse_sensor_spec,
        help=(
            "Sensor mapping as name:xshut_gpio:i2c_address. "
            "Repeat for each sensor. Defaults to Navibot's four sensors."
        ),
    )
    parser.add_argument("--interval", type=float, default=0.2, help="Print interval in seconds.")
    parser.add_argument("--count", type=int, default=0, help="Number of samples to print; 0 runs forever.")
    parser.add_argument("--boot-delay", type=float, default=0.15, help="Delay after XSHUT enable.")
    parser.add_argument("--timing-budget-ms", type=int, default=50)
    parser.add_argument(
        "--distance-mode",
        choices=("short", "long"),
        default="long",
        help="VL53L1X distance mode when supported by the installed driver.",
    )
    return parser.parse_args()


def get_board_pin(board: object, gpio: int) -> object:
    pin_name = f"D{gpio}"
    try:
        return getattr(board, pin_name)
    except AttributeError as exc:
        raise RuntimeError(f"Board pin {pin_name} is not available in Blinka.") from exc


def scan_i2c(i2c: object) -> list[int]:
    if not i2c.try_lock():
        return []
    try:
        return list(i2c.scan())
    finally:
        i2c.unlock()


def configure_sensor(sensor: object, timing_budget_ms: int, distance_mode: str) -> None:
    mode_value = 2 if distance_mode == "long" else 1
    for attr, value in (
        ("distance_mode", mode_value),
        ("timing_budget", timing_budget_ms),
    ):
        if hasattr(sensor, attr):
            try:
                setattr(sensor, attr, value)
            except Exception as exc:
                print(f"warning: could not set {attr}={value}: {exc}")


def bring_up_sensors(args: argparse.Namespace) -> tuple[object, list[SensorHandle]]:
    try:
        import adafruit_vl53l1x
        import board
        import digitalio
    except ImportError as exc:
        msg = (
            "Install VL53L1X dependencies on the Pi with: "
            "python3 -m pip install adafruit-blinka adafruit-circuitpython-vl53l1x"
        )
        raise RuntimeError(msg) from exc

    specs = tuple(args.sensor or DEFAULT_SENSORS)
    addresses = [spec.address for spec in specs]
    if len(addresses) != len(set(addresses)):
        raise ValueError("sensor I2C addresses must be unique")

    xshut_pins = []
    for spec in specs:
        xshut = digitalio.DigitalInOut(get_board_pin(board, spec.xshut_gpio))
        xshut.switch_to_output(value=False)
        xshut_pins.append(xshut)

    sleep(args.boot_delay)
    i2c = board.I2C()
    handles: list[SensorHandle] = []

    try:
        for spec, xshut in zip(specs, xshut_pins, strict=True):
            print(
                f"Enabling {spec.name}: XSHUT GPIO {spec.xshut_gpio}, "
                f"default 0x29 -> {hex(spec.address)}"
            )
            xshut.value = True
            sleep(args.boot_delay)

            sensor = adafruit_vl53l1x.VL53L1X(i2c)
            sensor.set_address(spec.address)
            configure_sensor(sensor, args.timing_budget_ms, args.distance_mode)
            handles.append(SensorHandle(spec=spec, xshut=xshut, sensor=sensor))

        print("I2C scan:", ", ".join(hex(address) for address in scan_i2c(i2c)))
        for handle in handles:
            handle.sensor.start_ranging()
        return i2c, handles
    except Exception:
        for xshut in xshut_pins:
            xshut.value = False
            xshut.deinit()
        raise


def print_readings(handles: list[SensorHandle], interval: float, count: int) -> None:
    sample = 0
    started_at = monotonic()
    while count <= 0 or sample < count:
        fields = [f"t={monotonic() - started_at:7.2f}s"]
        for handle in handles:
            sensor = handle.sensor
            if sensor.data_ready:
                distance_cm = sensor.distance
                sensor.clear_interrupt()
                if distance_cm is None:
                    fields.append(f"{handle.spec.name}=----")
                else:
                    fields.append(f"{handle.spec.name}={distance_cm * 10:5.0f}mm")
            else:
                fields.append(f"{handle.spec.name}=wait")
        print("  ".join(fields), flush=True)
        sample += 1
        sleep(interval)


def shutdown(handles: list[SensorHandle]) -> None:
    for handle in handles:
        try:
            handle.sensor.stop_ranging()
        except Exception:
            pass

    for handle in handles:
        try:
            handle.xshut.value = False
            handle.xshut.deinit()
        except Exception:
            pass


def main() -> None:
    args = parse_args()
    _, handles = bring_up_sensors(args)
    try:
        print_readings(handles, interval=args.interval, count=args.count)
    except KeyboardInterrupt:
        print("")
    finally:
        shutdown(handles)


if __name__ == "__main__":
    main()

