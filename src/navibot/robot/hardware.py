from dataclasses import dataclass
from time import monotonic

from navibot.robot.encoders import EncoderPins, QuadratureEncoder
from navibot.robot.pose import DifferentialOdometry, DifferentialOdometryConfig
from navibot.robot.safety import SafetyConfig, evaluate_safety
from navibot.robot.state import EncoderState, PoseState, RobotState
from navibot.sensors.battery import BatteryMonitor
from navibot.sensors.ina219 import Ina219Sensor
from navibot.sensors.vl53l1x_array import DEFAULT_VL53L1X_SPECS, Vl53l1xArray


@dataclass(frozen=True)
class RobotHardwareConfig:
    left_encoder: EncoderPins = EncoderPins(a=23, b=24)
    right_encoder: EncoderPins = EncoderPins(a=27, b=22)
    left_encoder_inverted: bool = False
    right_encoder_inverted: bool = True
    encoder_pull_up: bool = True
    ina219_address: int = 0x40
    odometry: DifferentialOdometryConfig = DifferentialOdometryConfig()
    safety: SafetyConfig = SafetyConfig()


class RobotHardware:
    def __init__(self, config: RobotHardwareConfig) -> None:
        self._config = config
        self._started_at = monotonic()
        self.left_encoder = QuadratureEncoder(
            config.left_encoder,
            pull_up=config.encoder_pull_up,
            inverted=config.left_encoder_inverted,
        )
        self.right_encoder = QuadratureEncoder(
            config.right_encoder,
            pull_up=config.encoder_pull_up,
            inverted=config.right_encoder_inverted,
        )
        self.tof = Vl53l1xArray(specs=DEFAULT_VL53L1X_SPECS)
        self.battery = BatteryMonitor(sensor=Ina219Sensor(address=config.ina219_address))
        self.odometry = DifferentialOdometry(config.odometry)

    def start(self) -> None:
        self.left_encoder.reset()
        self.right_encoder.reset()
        self.tof.start_ranging()
        self.odometry.reset(self.left_encoder.sample(), self.right_encoder.sample())

    def read_state(self) -> RobotState:
        left = self.left_encoder.sample()
        right = self.right_encoder.sample()
        pose = self.odometry.update(left, right)
        battery = self.battery.read()
        power = battery.power
        tof = {reading.name: reading.distance_mm for reading in self.tof.read_all()}
        safety = evaluate_safety(
            front_mm=tof.get("front"),
            battery_voltage_v=power.bus_voltage_v,
            config=self._config.safety,
        )

        return RobotState(
            t_s=monotonic() - self._started_at,
            power=power,
            battery=battery,
            tof_mm=tof,
            encoders=EncoderState(
                left_counts=left.counts,
                right_counts=right.counts,
                left_bad_transitions=left.bad_transitions,
                right_bad_transitions=right.bad_transitions,
            ),
            pose=PoseState(
                x_mm=pose.x_mm,
                y_mm=pose.y_mm,
                theta_rad=pose.theta_rad,
                theta_deg=pose.theta_deg,
            ),
            safety=safety,
        )

    def close(self) -> None:
        self.left_encoder.close()
        self.right_encoder.close()
        self.tof.close()
