from dataclasses import dataclass


@dataclass(frozen=True)
class PowerReading:
    bus_voltage_v: float | None
    shunt_voltage_mv: float | None
    load_voltage_v: float | None
    current_ma: float | None
    power_w: float | None

    @property
    def is_charging(self) -> bool:
        return self.current_ma is not None and self.current_ma < 0


class Ina219Sensor:
    def __init__(self, address: int = 0x40) -> None:
        try:
            import adafruit_ina219
            import board
        except ImportError as exc:
            msg = (
                "Install INA219 dependencies on the Pi with: "
                "python3 -m pip install adafruit-circuitpython-ina219"
            )
            raise RuntimeError(msg) from exc

        self._sensor = adafruit_ina219.INA219(board.I2C(), addr=address)

    def read(self) -> PowerReading:
        bus_v = read_attr(self._sensor, "bus_voltage")
        shunt_v = read_attr(self._sensor, "shunt_voltage")
        load_v = None
        if bus_v is not None and shunt_v is not None:
            load_v = bus_v + (shunt_v / 1000.0)

        return PowerReading(
            bus_voltage_v=bus_v,
            shunt_voltage_mv=shunt_v,
            load_voltage_v=load_v,
            current_ma=read_attr(self._sensor, "current"),
            power_w=read_attr(self._sensor, "power"),
        )


def read_attr(sensor: object, name: str) -> float | None:
    try:
        value = getattr(sensor, name)
    except AttributeError:
        return None
    return float(value)

