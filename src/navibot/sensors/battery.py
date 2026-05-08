from dataclasses import dataclass
from time import monotonic

from navibot.sensors.ina219 import Ina219Sensor, PowerReading


@dataclass(frozen=True)
class BatteryConfig:
    warning_voltage_v: float = 6.4
    critical_voltage_v: float = 6.2
    critical_seconds: float = 30.0


@dataclass(frozen=True)
class BatteryStatus:
    power: PowerReading
    warning: bool
    critical: bool
    charging: bool
    seconds_below_critical: float
    should_shutdown: bool


class BatteryMonitor:
    def __init__(
        self,
        sensor: Ina219Sensor | None = None,
        config: BatteryConfig = BatteryConfig(),
    ) -> None:
        self._sensor = sensor or Ina219Sensor()
        self._config = config
        self._critical_since: float | None = None

    @property
    def config(self) -> BatteryConfig:
        return self._config

    def read(self) -> BatteryStatus:
        power = self._sensor.read()
        voltage = power.bus_voltage_v
        charging = power.is_charging

        warning = voltage is not None and voltage <= self._config.warning_voltage_v and not charging
        critical = voltage is not None and voltage <= self._config.critical_voltage_v and not charging

        now = monotonic()
        if critical:
            self._critical_since = self._critical_since or now
            seconds_below_critical = now - self._critical_since
        else:
            self._critical_since = None
            seconds_below_critical = 0.0

        return BatteryStatus(
            power=power,
            warning=warning,
            critical=critical,
            charging=charging,
            seconds_below_critical=seconds_below_critical,
            should_shutdown=critical and seconds_below_critical >= self._config.critical_seconds,
        )

