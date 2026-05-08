from dataclasses import asdict, dataclass
import json

from navibot.robot.safety import SafetyState
from navibot.sensors.battery import BatteryStatus
from navibot.sensors.ina219 import PowerReading


@dataclass(frozen=True)
class EncoderState:
    left_counts: int
    right_counts: int
    left_bad_transitions: int
    right_bad_transitions: int


@dataclass(frozen=True)
class PoseState:
    x_mm: float
    y_mm: float
    theta_rad: float
    theta_deg: float


@dataclass(frozen=True)
class RobotState:
    t_s: float
    power: PowerReading
    battery: BatteryStatus
    tof_mm: dict[str, int | None]
    encoders: EncoderState
    pose: PoseState
    safety: SafetyState

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"))
