"""Continuously monitor INA219 battery voltage and optionally power off safely."""

from __future__ import annotations

import argparse
import json
import subprocess
from dataclasses import asdict
from time import sleep

from navibot.sensors.battery import BatteryConfig, BatteryMonitor
from navibot.sensors.ina219 import Ina219Sensor


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor battery and shut down on sustained critical voltage.")
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0x40)
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--warning-voltage", type=float, default=6.4)
    parser.add_argument("--critical-voltage", type=float, default=6.2)
    parser.add_argument("--critical-seconds", type=float, default=30.0)
    parser.add_argument("--count", type=int, default=0, help="Number of samples; 0 runs forever.")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Do not power off when shutdown criteria are met.",
    )
    return parser.parse_args()


def make_monitor(args: argparse.Namespace) -> BatteryMonitor:
    return BatteryMonitor(
        sensor=Ina219Sensor(address=args.address),
        config=BatteryConfig(
            warning_voltage_v=args.warning_voltage,
            critical_voltage_v=args.critical_voltage,
            critical_seconds=args.critical_seconds,
        ),
    )


def main() -> None:
    args = parse_args()
    monitor = make_monitor(args)
    sample = 0
    print(
        "Battery guard running "
        f"(warning={args.warning_voltage:g}V, critical={args.critical_voltage:g}V "
        f"for {args.critical_seconds:g}s, shutdown={not args.dry_run})",
        flush=True,
    )

    try:
        while args.count <= 0 or sample < args.count:
            status = monitor.read()
            print(json.dumps(asdict(status), separators=(",", ":")), flush=True)
            if status.should_shutdown:
                if args.dry_run:
                    print("Critical battery threshold sustained; dry run, not powering off.", flush=True)
                else:
                    print("Critical battery threshold sustained; powering off.", flush=True)
                    subprocess.run(["systemctl", "poweroff"], check=False)
                break
            sample += 1
            sleep(args.interval)
    except KeyboardInterrupt:
        print("")


if __name__ == "__main__":
    main()
