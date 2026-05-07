from dataclasses import dataclass
from time import sleep


@dataclass(frozen=True)
class Vl53l1xSpec:
    name: str
    xshut_gpio: int
    address: int


@dataclass(frozen=True)
class Vl53l1xReading:
    name: str
    distance_mm: int | None
    ready: bool


@dataclass(frozen=True)
class Vl53l1xHandle:
    spec: Vl53l1xSpec
    xshut: object
    sensor: object


DEFAULT_VL53L1X_SPECS = (
    Vl53l1xSpec(name="left45", xshut_gpio=25, address=0x30),
    Vl53l1xSpec(name="front", xshut_gpio=8, address=0x31),
    Vl53l1xSpec(name="right45", xshut_gpio=7, address=0x32),
    Vl53l1xSpec(name="back", xshut_gpio=1, address=0x33),
)


class Vl53l1xArray:
    def __init__(
        self,
        specs: tuple[Vl53l1xSpec, ...] = DEFAULT_VL53L1X_SPECS,
        boot_delay: float = 0.15,
        timing_budget_ms: int = 50,
        distance_mode: str = "long",
    ) -> None:
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

        addresses = [spec.address for spec in specs]
        if len(addresses) != len(set(addresses)):
            raise ValueError("sensor I2C addresses must be unique")

        self._i2c = board.I2C()
        self._handles: list[Vl53l1xHandle] = []
        xshut_pins = []

        try:
            for spec in specs:
                xshut = digitalio.DigitalInOut(get_board_pin(board, spec.xshut_gpio))
                xshut.switch_to_output(value=False)
                xshut_pins.append(xshut)

            sleep(boot_delay)

            for spec, xshut in zip(specs, xshut_pins, strict=True):
                xshut.value = True
                sleep(boot_delay)
                sensor = adafruit_vl53l1x.VL53L1X(self._i2c)
                sensor.set_address(spec.address)
                configure_sensor(sensor, timing_budget_ms, distance_mode)
                self._handles.append(Vl53l1xHandle(spec=spec, xshut=xshut, sensor=sensor))
        except Exception:
            for xshut in xshut_pins:
                xshut.value = False
                xshut.deinit()
            raise

    @property
    def handles(self) -> tuple[Vl53l1xHandle, ...]:
        return tuple(self._handles)

    def scan_i2c(self) -> list[int]:
        if not self._i2c.try_lock():
            return []
        try:
            return list(self._i2c.scan())
        finally:
            self._i2c.unlock()

    def start_ranging(self) -> None:
        for handle in self._handles:
            handle.sensor.start_ranging()

    def stop_ranging(self) -> None:
        for handle in self._handles:
            try:
                handle.sensor.stop_ranging()
            except Exception:
                pass

    def read_all(self) -> tuple[Vl53l1xReading, ...]:
        readings = []
        for handle in self._handles:
            sensor = handle.sensor
            if sensor.data_ready:
                distance_cm = sensor.distance
                sensor.clear_interrupt()
                readings.append(
                    Vl53l1xReading(
                        name=handle.spec.name,
                        distance_mm=None if distance_cm is None else int(distance_cm * 10),
                        ready=True,
                    )
                )
            else:
                readings.append(
                    Vl53l1xReading(name=handle.spec.name, distance_mm=None, ready=False)
                )
        return tuple(readings)

    def close(self) -> None:
        self.stop_ranging()
        for handle in self._handles:
            try:
                handle.xshut.value = False
                handle.xshut.deinit()
            except Exception:
                pass


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


def get_board_pin(board: object, gpio: int) -> object:
    pin_name = f"D{gpio}"
    try:
        return getattr(board, pin_name)
    except AttributeError as exc:
        raise RuntimeError(f"Board pin {pin_name} is not available in Blinka.") from exc

