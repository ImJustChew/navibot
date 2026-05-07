"""Watch one quadrature encoder and print signed counts.

Spin the wheel by hand:
- one direction should increase the count
- the other direction should decrease the count
"""

from __future__ import annotations

import argparse
from threading import Lock
from time import sleep


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


class Encoder:
    def __init__(self, pin_a: int, pin_b: int, pull_up: bool) -> None:
        try:
            from gpiozero import DigitalInputDevice
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self._lock = Lock()
        self._a = DigitalInputDevice(pin_a, pull_up=pull_up)
        self._b = DigitalInputDevice(pin_b, pull_up=pull_up)
        self._count = 0
        self._bad = 0
        self._state = self._read_state()

        self._a.when_activated = self._on_edge
        self._a.when_deactivated = self._on_edge
        self._b.when_activated = self._on_edge
        self._b.when_deactivated = self._on_edge

    def close(self) -> None:
        self._a.close()
        self._b.close()

    def snapshot(self) -> tuple[int, int, int]:
        with self._lock:
            return self._count, self._bad, self._state

    def _read_state(self) -> int:
        return (int(self._a.is_active) << 1) | int(self._b.is_active)

    def _on_edge(self) -> None:
        with self._lock:
            previous = self._state
            current = self._read_state()
            delta = QUADRATURE_DELTA.get((previous, current))
            if delta is None:
                self._bad += 1
            else:
                self._count += delta
            self._state = current


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print signed quadrature encoder count.")
    parser.add_argument("--a", type=int, required=True, help="BCM GPIO for encoder channel A.")
    parser.add_argument("--b", type=int, required=True, help="BCM GPIO for encoder channel B.")
    parser.add_argument("--interval", type=float, default=0.2, help="Print interval in seconds.")
    parser.add_argument("--pull-up", dest="pull_up", action="store_true", default=True)
    parser.add_argument("--no-pull-up", dest="pull_up", action="store_false")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    encoder = Encoder(pin_a=args.a, pin_b=args.b, pull_up=args.pull_up)
    print(f"Watching encoder A=GPIO {args.a}, B=GPIO {args.b}. Press Ctrl+C to stop.")
    try:
        while True:
            count, bad, state = encoder.snapshot()
            print(f"count={count:+d} bad={bad} state={state:02b}", flush=True)
            sleep(args.interval)
    except KeyboardInterrupt:
        print("")
    finally:
        encoder.close()


if __name__ == "__main__":
    main()

