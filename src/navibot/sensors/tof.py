from dataclasses import dataclass
from enum import StrEnum
from typing import Protocol


class TofPosition(StrEnum):
    FRONT = "front"
    LEFT = "left"
    RIGHT = "right"
    REAR = "rear"


@dataclass(frozen=True)
class TofReading:
    position: TofPosition
    distance_mm: int
    valid: bool = True


class TofSensor(Protocol):
    position: TofPosition

    def read(self) -> TofReading:
        """Return the latest distance reading."""


@dataclass
class TofArray:
    sensors: tuple[TofSensor, TofSensor, TofSensor, TofSensor]

    def read_all(self) -> tuple[TofReading, ...]:
        return tuple(sensor.read() for sensor in self.sensors)

