"""Read INA219 current sensor values from I2C."""

from __future__ import annotations

import argparse
from time import monotonic, sleep

from navibot.sensors.ina219 import Ina219Sensor, PowerReading


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print INA219 voltage/current/power readings.")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x40)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--count", type=int, default=0, help="Number of samples to print; 0 runs forever.")
    return parser.parse_args()


def print_readings(sensor: Ina219Sensor, interval: float, count: int) -> None:
    sample = 0
    started_at = monotonic()
    while count <= 0 or sample < count:
        reading = sensor.read()

        print(
            "  ".join(
                (
                    f"t={monotonic() - started_at:7.2f}s",
                    format_reading(reading),
                )
            ),
            flush=True,
        )
        sample += 1
        sleep(interval)


def format_value(label: str, value: float | None, unit: str) -> str:
    if value is None:
        return f"{label}=----"
    if unit == "mA":
        return f"{label}={value:8.1f}{unit}"
    return f"{label}={value:8.3f}{unit}"


def format_reading(reading: PowerReading) -> str:
    return "  ".join(
        (
            format_value("bus", reading.bus_voltage_v, "V"),
            format_value("shunt", reading.shunt_voltage_mv, "mV"),
            format_value("load", reading.load_voltage_v, "V"),
            format_value("current", reading.current_ma, "mA"),
            format_value("power", reading.power_w, "W"),
        )
    )


def main() -> None:
    args = parse_args()
    sensor = Ina219Sensor(address=args.address)
    print(f"Reading INA219 at I2C address {hex(args.address)}")
    try:
        print_readings(sensor, interval=args.interval, count=args.count)
    except KeyboardInterrupt:
        print("")


if __name__ == "__main__":
    main()
