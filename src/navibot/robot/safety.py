from dataclasses import dataclass


@dataclass(frozen=True)
class SafetyConfig:
    front_stop_mm: int = 180
    low_battery_v: float = 6.2


@dataclass(frozen=True)
class SafetyState:
    front_blocked: bool
    low_battery: bool
    estop: bool = False

    @property
    def motion_allowed(self) -> bool:
        return not self.estop and not self.front_blocked and not self.low_battery


def evaluate_safety(
    front_mm: int | None,
    battery_voltage_v: float | None,
    config: SafetyConfig,
    estop: bool = False,
) -> SafetyState:
    return SafetyState(
        front_blocked=front_mm is not None and front_mm <= config.front_stop_mm,
        low_battery=battery_voltage_v is not None and battery_voltage_v <= config.low_battery_v,
        estop=estop,
    )

