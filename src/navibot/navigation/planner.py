from dataclasses import dataclass

from navibot.control.commands import DriveCommand
from navibot.sensors.tof import TofPosition, TofReading


@dataclass(frozen=True)
class NavigationPlanner:
    stop_distance_mm: int = 180
    cruise_speed: float = 0.35

    def plan(self, readings: tuple[TofReading, ...]) -> DriveCommand:
        front = next((r for r in readings if r.position == TofPosition.FRONT), None)
        if front is not None and front.valid and front.distance_mm <= self.stop_distance_mm:
            return DriveCommand(linear=0.0, angular=0.0)
        return DriveCommand(linear=self.cruise_speed, angular=0.0)

