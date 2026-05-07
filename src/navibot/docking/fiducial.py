from dataclasses import dataclass


@dataclass(frozen=True)
class FiducialObservation:
    marker_id: int
    x_offset: float
    y_offset: float
    distance_m: float
    yaw_deg: float


class FiducialDetector:
    def detect(self, frame: object) -> FiducialObservation | None:
        """Detect a docking marker from a camera frame.

        The concrete OpenCV/ArUco implementation belongs here once camera
        hardware and marker family are selected.
        """
        raise NotImplementedError

