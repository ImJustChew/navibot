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
from time import monotonic, sleep

from navibot.sensors.vl53l1x_array import (
    DEFAULT_VL53L1X_SPECS,
    Vl53l1xArray,
    Vl53l1xSpec,
)


def parse_sensor_spec(value: str) -> Vl53l1xSpec:
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
    return Vl53l1xSpec(name=name, xshut_gpio=xshut_gpio, address=address)


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


def bring_up_sensors(args: argparse.Namespace) -> Vl53l1xArray:
    specs = tuple(args.sensor or DEFAULT_VL53L1X_SPECS)
    for spec in specs:
        print(
            f"Enabling {spec.name}: XSHUT GPIO {spec.xshut_gpio}, "
            f"default 0x29 -> {hex(spec.address)}"
        )
    sensor_array = Vl53l1xArray(
        specs=specs,
        boot_delay=args.boot_delay,
        timing_budget_ms=args.timing_budget_ms,
        distance_mode=args.distance_mode,
    )
    print("I2C scan:", ", ".join(hex(address) for address in sensor_array.scan_i2c()))
    sensor_array.start_ranging()
    return sensor_array


def print_readings(sensor_array: Vl53l1xArray, interval: float, count: int) -> None:
    sample = 0
    started_at = monotonic()
    while count <= 0 or sample < count:
        fields = [f"t={monotonic() - started_at:7.2f}s"]
        for reading in sensor_array.read_all():
            if reading.ready:
                if reading.distance_mm is None:
                    fields.append(f"{reading.name}=----")
                else:
                    fields.append(f"{reading.name}={reading.distance_mm:5d}mm")
            else:
                fields.append(f"{reading.name}=wait")
        print("  ".join(fields), flush=True)
        sample += 1
        sleep(interval)


def main() -> None:
    args = parse_args()
    sensor_array = bring_up_sensors(args)
    try:
        print_readings(sensor_array, interval=args.interval, count=args.count)
    except KeyboardInterrupt:
        print("")
    finally:
        sensor_array.close()


if __name__ == "__main__":
    main()
