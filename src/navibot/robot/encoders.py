from dataclasses import dataclass
from threading import Lock


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


@dataclass(frozen=True)
class EncoderPins:
    a: int
    b: int


@dataclass(frozen=True)
class EncoderSample:
    counts: int
    abs_counts: int
    bad_transitions: int
    edge_count: int
    state: int
    last_tick_ns: int


class QuadratureEncoder:
    def __init__(self, pins: EncoderPins, pull_up: bool = True, inverted: bool = False) -> None:
        try:
            import lgpio
        except ImportError as exc:
            msg = "Install Raspberry Pi dependencies with: python -m pip install -e '.[rpi]'"
            raise RuntimeError(msg) from exc

        self._lgpio = lgpio
        self._lock = Lock()
        self._chip = lgpio.gpiochip_open(0)
        self._pin_a = pins.a
        self._pin_b = pins.b
        self._counts = 0
        self._bad_transitions = 0
        self._edge_count = 0
        self._last_tick_ns = 0
        self._multiplier = -1 if inverted else 1

        line_flags = lgpio.SET_PULL_UP if pull_up else 0
        lgpio.gpio_claim_alert(self._chip, pins.a, lgpio.BOTH_EDGES, line_flags)
        lgpio.gpio_claim_alert(self._chip, pins.b, lgpio.BOTH_EDGES, line_flags)

        self._a_level = lgpio.gpio_read(self._chip, pins.a)
        self._b_level = lgpio.gpio_read(self._chip, pins.b)
        self._state = self._levels_to_state()

        self._callback_a = lgpio.callback(self._chip, pins.a, lgpio.BOTH_EDGES, self._on_edge)
        self._callback_b = lgpio.callback(self._chip, pins.b, lgpio.BOTH_EDGES, self._on_edge)

    def reset(self) -> None:
        with self._lock:
            self._counts = 0
            self._bad_transitions = 0
            self._edge_count = 0
            self._last_tick_ns = 0
            self._a_level = self._lgpio.gpio_read(self._chip, self._pin_a)
            self._b_level = self._lgpio.gpio_read(self._chip, self._pin_b)
            self._state = self._levels_to_state()

    def sample(self) -> EncoderSample:
        with self._lock:
            return EncoderSample(
                counts=self._counts,
                abs_counts=abs(self._counts),
                bad_transitions=self._bad_transitions,
                edge_count=self._edge_count,
                state=self._state,
                last_tick_ns=self._last_tick_ns,
            )

    @property
    def bad_transitions(self) -> int:
        return self.sample().bad_transitions

    def close(self) -> None:
        self._callback_a.cancel()
        self._callback_b.cancel()
        self._lgpio.gpio_free(self._chip, self._pin_a)
        self._lgpio.gpio_free(self._chip, self._pin_b)
        self._lgpio.gpiochip_close(self._chip)

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
            if previous == current:
                return

            delta = QUADRATURE_DELTA.get((previous, current))
            if delta is None:
                self._bad_transitions += 1
            else:
                self._counts += delta * self._multiplier
            self._state = current
            self._edge_count += 1
            self._last_tick_ns = tick_ns

