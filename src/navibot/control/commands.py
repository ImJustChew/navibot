from dataclasses import dataclass


@dataclass(frozen=True)
class DriveCommand:
    linear: float
    angular: float


@dataclass(frozen=True)
class RobotCommand:
    name: str
    payload: dict[str, object]

