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
    def __init__(self, pin_a: int, pin_b: int, pull_up: bool, inverted: bool) -> None:
        try:
            from gpiozero import DigitalInputDevice
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self._lock = Lock()
        self._a = DigitalInputDevice(pin_a, pull_up=pull_up)
        self._b = DigitalInputDevice(pin_b, pull_up=pull_up)
        self._multiplier = -1 if inverted else 1
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
                self._count += delta * self._multiplier
            self._state = current


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Print signed quadrature encoder count.")
    parser.add_argument("--wheel", choices=("left", "right"), help="Use known Navibot encoder pins.")
    parser.add_argument("--a", type=int, help="BCM GPIO for encoder channel A.")
    parser.add_argument("--b", type=int, help="BCM GPIO for encoder channel B.")
    parser.add_argument("--invert", action="store_true", help="Invert the signed count direction.")
    parser.add_argument("--no-invert", action="store_true", help="Do not use the wheel default inversion.")
    parser.add_argument("--interval", type=float, default=0.2, help="Print interval in seconds.")
    parser.add_argument("--pull-up", dest="pull_up", action="store_true", default=True)
    parser.add_argument("--no-pull-up", dest="pull_up", action="store_false")
    return parser.parse_args()


def resolve_encoder_args(args: argparse.Namespace) -> tuple[int, int, bool]:
    if args.wheel == "left":
        pin_a = 23 if args.a is None else args.a
        pin_b = 24 if args.b is None else args.b
        default_inverted = False
    elif args.wheel == "right":
        pin_a = 27 if args.a is None else args.a
        pin_b = 22 if args.b is None else args.b
        default_inverted = True
    else:
        if args.a is None or args.b is None:
            raise SystemExit("Use --wheel left/right or provide both --a and --b.")
        pin_a = args.a
        pin_b = args.b
        default_inverted = False

    if args.no_invert:
        inverted = False
    else:
        inverted = default_inverted or args.invert
    return pin_a, pin_b, inverted


def main() -> None:
    args = parse_args()
    pin_a, pin_b, inverted = resolve_encoder_args(args)
    encoder = Encoder(pin_a=pin_a, pin_b=pin_b, pull_up=args.pull_up, inverted=inverted)
    print(
        f"Watching encoder A=GPIO {pin_a}, B=GPIO {pin_b}, inverted={inverted}. "
        "Press Ctrl+C to stop."
    )
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
