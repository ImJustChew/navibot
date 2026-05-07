"""Watch one quadrature encoder and print signed counts with lgpio callbacks.

Spin the wheel by hand:
- one direction should increase the count
- the other direction should decrease the count
"""

from __future__ import annotations

import argparse
from time import sleep

from navibot.robot.encoders import EncoderPins, QuadratureEncoder


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
    encoder = QuadratureEncoder(
        pins=EncoderPins(a=pin_a, b=pin_b),
        pull_up=args.pull_up,
        inverted=inverted,
    )
    print(
        f"Watching encoder A=GPIO {pin_a}, B=GPIO {pin_b}, inverted={inverted}. "
        "Press Ctrl+C to stop."
    )
    try:
        while True:
            sample = encoder.sample()
            print(
                f"count={sample.counts:+d} edges={sample.edge_count} "
                f"bad={sample.bad_transitions} state={sample.state:02b} "
                f"tick_ns={sample.last_tick_ns}",
                flush=True,
            )
            sleep(args.interval)
    except KeyboardInterrupt:
        print("")
    finally:
        encoder.close()


if __name__ == "__main__":
    main()
