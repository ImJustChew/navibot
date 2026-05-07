from dataclasses import dataclass

from navibot.control.commands import DriveCommand
from navibot.navigation.planner import NavigationPlanner
from navibot.sensors.tof import TofArray


@dataclass
class RobotRuntime:
    tof_array: TofArray
    planner: NavigationPlanner

    def tick(self) -> DriveCommand:
        readings = self.tof_array.read_all()
        return self.planner.plan(readings)

