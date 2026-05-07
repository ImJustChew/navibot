"""Watch one quadrature encoder and print signed counts with lgpio callbacks.

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
            import lgpio
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self._lgpio = lgpio
        self._lock = Lock()
        self._chip = lgpio.gpiochip_open(0)
        self._pin_a = pin_a
        self._pin_b = pin_b
        self._multiplier = -1 if inverted else 1
        self._count = 0
        self._bad = 0
        self._edge_count = 0
        self._last_tick_ns = 0

        line_flags = lgpio.SET_PULL_UP if pull_up else 0
        lgpio.gpio_claim_alert(self._chip, pin_a, lgpio.BOTH_EDGES, line_flags)
        lgpio.gpio_claim_alert(self._chip, pin_b, lgpio.BOTH_EDGES, line_flags)

        self._a_level = lgpio.gpio_read(self._chip, pin_a)
        self._b_level = lgpio.gpio_read(self._chip, pin_b)
        self._state = self._levels_to_state()

        self._callback_a = lgpio.callback(self._chip, pin_a, lgpio.BOTH_EDGES, self._on_edge)
        self._callback_b = lgpio.callback(self._chip, pin_b, lgpio.BOTH_EDGES, self._on_edge)

    def close(self) -> None:
        self._callback_a.cancel()
        self._callback_b.cancel()
        self._lgpio.gpio_free(self._chip, self._pin_a)
        self._lgpio.gpio_free(self._chip, self._pin_b)
        self._lgpio.gpiochip_close(self._chip)

    def snapshot(self) -> tuple[int, int, int, int, int]:
        with self._lock:
            return self._count, self._bad, self._state, self._edge_count, self._last_tick_ns

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
            delta = QUADRATURE_DELTA.get((previous, current))
            if delta is None:
                self._bad += 1
            else:
                self._count += delta * self._multiplier
            self._state = current
            self._edge_count += 1
            self._last_tick_ns = tick_ns


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
            count, bad, state, edges, tick_ns = encoder.snapshot()
            print(
                f"count={count:+d} edges={edges} bad={bad} state={state:02b} tick_ns={tick_ns}",
                flush=True,
            )
            sleep(args.interval)
    except KeyboardInterrupt:
        print("")
    finally:
        encoder.close()


if __name__ == "__main__":
    main()
