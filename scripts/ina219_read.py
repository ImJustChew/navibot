"""Read INA219 current sensor values from I2C."""

from __future__ import annotations

import argparse
from time import monotonic, sleep


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print INA219 voltage/current/power readings.")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x40)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--count", type=int, default=0, help="Number of samples to print; 0 runs forever.")
    return parser.parse_args()


def make_sensor(address: int) -> object:
    try:
        import adafruit_ina219
        import board
    except ImportError as exc:
        msg = (
            "Install INA219 dependencies on the Pi with: "
            "python3 -m pip install adafruit-circuitpython-ina219"
        )
        raise RuntimeError(msg) from exc

    return adafruit_ina219.INA219(board.I2C(), addr=address)


def read_attr(sensor: object, name: str) -> float | None:
    try:
        value = getattr(sensor, name)
    except AttributeError:
        return None
    return float(value)


def print_readings(sensor: object, interval: float, count: int) -> None:
    sample = 0
    started_at = monotonic()
    while count <= 0 or sample < count:
        bus_v = read_attr(sensor, "bus_voltage")
        shunt_v = read_attr(sensor, "shunt_voltage")
        current_ma = read_attr(sensor, "current")
        power_w = read_attr(sensor, "power")
        load_v = None
        if bus_v is not None and shunt_v is not None:
            load_v = bus_v + (shunt_v / 1000.0)

        print(
            "  ".join(
                (
                    f"t={monotonic() - started_at:7.2f}s",
                    format_value("bus", bus_v, "V"),
                    format_value("shunt", shunt_v, "mV"),
                    format_value("load", load_v, "V"),
                    format_value("current", current_ma, "mA"),
                    format_value("power", power_w, "W"),
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


def main() -> None:
    args = parse_args()
    sensor = make_sensor(args.address)
    print(f"Reading INA219 at I2C address {hex(args.address)}")
    try:
        print_readings(sensor, interval=args.interval, count=args.count)
    except KeyboardInterrupt:
        print("")


if __name__ == "__main__":
    main()

