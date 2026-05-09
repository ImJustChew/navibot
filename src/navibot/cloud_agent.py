from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
from dataclasses import dataclass
from time import monotonic
from typing import Any

from navibot.robot.hardware import RobotHardware, RobotHardwareConfig
from navibot.robot.motors import DifferentialDrive, DriverMotor, MotorPins, clamp, validate_motor_voltage


@dataclass(frozen=True)
class RobotAgentConfig:
    backend_url: str
    robot_id: str
    shared_secret: str
    telemetry_seconds: float
    mock: bool
    speed_scale: float
    turn_scale: float
    supply_voltage: float
    motor_voltage_limit: float
    left_motor: MotorPins
    right_motor: MotorPins
    standby_pin: int
    left_motor_inverted: bool
    right_motor_inverted: bool


class DriveController:
    def __init__(self, config: RobotAgentConfig) -> None:
        validate_motor_voltage(config.speed_scale + config.turn_scale, config.supply_voltage, config.motor_voltage_limit)
        self._drive = DifferentialDrive(
            left=DriverMotor(config.left_motor, inverted=config.left_motor_inverted),
            right=DriverMotor(config.right_motor, inverted=config.right_motor_inverted),
            standby_pin=config.standby_pin,
        )
        self._speed_scale = config.speed_scale
        self._turn_scale = config.turn_scale
        self._stop_task: asyncio.Task[None] | None = None

    def start(self) -> None:
        self._drive.enable()

    def apply_drive(self, linear: float, angular: float, duration_ms: int) -> None:
        left = clamp(linear * self._speed_scale - angular * self._turn_scale, -1.0, 1.0)
        right = clamp(linear * self._speed_scale + angular * self._turn_scale, -1.0, 1.0)
        self._set_wheel(self._drive.left, left)
        self._set_wheel(self._drive.right, right)

        if self._stop_task:
            self._stop_task.cancel()
        self._stop_task = asyncio.create_task(self._stop_after(duration_ms / 1000))

    def stop(self) -> None:
        self._drive.coast()

    def close(self) -> None:
        if self._stop_task:
            self._stop_task.cancel()
        self._drive.close()

    async def _stop_after(self, seconds: float) -> None:
        try:
            await asyncio.sleep(seconds)
            self.stop()
        except asyncio.CancelledError:
            return

    @staticmethod
    def _set_wheel(motor: DriverMotor, value: float) -> None:
        if value >= 0:
            motor.forward(value)
        else:
            motor.reverse(abs(value))


class TelemetrySource:
    def __init__(self, mock: bool) -> None:
        self._mock = mock
        self._started_at = monotonic()
        self._hardware: RobotHardware | None = None

    def start(self) -> None:
        if self._mock or self._hardware:
            return
        self._hardware = RobotHardware(RobotHardwareConfig())
        self._hardware.start()

    def read(self) -> dict[str, Any]:
        if self._hardware:
            return self._hardware.read_state().to_dict()

        t_s = monotonic() - self._started_at
        return {
            "t_s": t_s,
            "power": {
                "bus_voltage_v": 7.2 + math.sin(t_s / 8) * 0.05,
                "current_ma": 250.0,
                "power_w": 1.8,
            },
            "battery": {
                "level": "healthy",
                "is_charging": False,
            },
            "tof_mm": {
                "front": None,
                "left45": None,
                "right45": None,
                "back": None,
            },
            "pose": {
                "x_mm": 0.0,
                "y_mm": 0.0,
                "theta_deg": 0.0,
            },
            "safety": {
                "ok": True,
                "reasons": [],
            },
        }

    def close(self) -> None:
        if self._hardware:
            self._hardware.close()
            self._hardware = None


async def run_agent(config: RobotAgentConfig) -> None:
    try:
        import websockets
    except ImportError as exc:
        msg = "Install robot websocket dependency with: python -m pip install websockets"
        raise RuntimeError(msg) from exc

    telemetry = TelemetrySource(mock=config.mock)
    drive: DriveController | None = None
    if not config.mock:
        drive = DriveController(config)

    url = websocket_url(config)

    while True:
        try:
            telemetry.start()
            if drive:
                drive.start()
            async with websockets.connect(url, ping_interval=20, ping_timeout=20) as ws:
                await ws.send(
                    json.dumps(
                        {
                            "kind": "hello",
                            "robotId": config.robot_id,
                            "name": config.robot_id,
                            "capabilities": ["drive", "telemetry", "tof", "battery", "mapping"],
                        },
                        separators=(",", ":"),
                    )
                )
                await asyncio.gather(
                    telemetry_loop(ws, telemetry, config),
                    command_loop(ws, drive),
                )
        except Exception as exc:
            print(f"cloud agent disconnected: {exc}; retrying in 3s", flush=True)
            if drive:
                drive.stop()
            await asyncio.sleep(3)
        finally:
            telemetry.close()


async def telemetry_loop(ws: Any, telemetry: TelemetrySource, config: RobotAgentConfig) -> None:
    while True:
        await ws.send(
            json.dumps(
                {
                    "kind": "telemetry",
                    "robotId": config.robot_id,
                    "state": telemetry.read(),
                },
                separators=(",", ":"),
            )
        )
        await asyncio.sleep(config.telemetry_seconds)


async def command_loop(ws: Any, drive: DriveController | None) -> None:
    async for raw in ws:
        message = json.loads(raw)
        command_id = message.get("id")
        command_type = message.get("type")
        payload = message.get("payload") or {}

        if command_type == "drive":
            if drive:
                drive.apply_drive(
                    linear=float(payload.get("linear", 0.0)),
                    angular=float(payload.get("angular", 0.0)),
                    duration_ms=int(payload.get("durationMs", 250)),
                )
        elif command_type == "stop" and drive:
            drive.stop()

        if command_id:
            await ws.send(
                json.dumps(
                    {
                        "kind": "command_ack",
                        "commandId": command_id,
                    },
                    separators=(",", ":"),
                )
            )


def websocket_url(config: RobotAgentConfig) -> str:
    base = config.backend_url.rstrip("/")
    scheme = "wss" if base.startswith("https://") else "ws"
    if base.startswith("ws://") or base.startswith("wss://"):
        url = f"{base}/ws/robot/{config.robot_id}"
    else:
        url = f"{scheme}://{base.removeprefix('http://').removeprefix('https://')}/ws/robot/{config.robot_id}"
    separator = "&" if "?" in url else "?"
    return f"{url}{separator}token={config.shared_secret}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Connect the Raspberry Pi robot to the Navibot cloud backend.")
    parser.add_argument("--backend-url", default=os.getenv("NAVIBOT_BACKEND_URL", "ws://localhost:8787"))
    parser.add_argument("--robot-id", default=os.getenv("NAVIBOT_ROBOT_ID", "devbot"))
    parser.add_argument("--shared-secret", default=os.getenv("NAVIBOT_SHARED_SECRET", "change-me"))
    parser.add_argument("--telemetry-seconds", type=float, default=0.5)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--speed-scale", type=float, default=0.22)
    parser.add_argument("--turn-scale", type=float, default=0.16)
    parser.add_argument("--supply-voltage", type=float, default=7.4)
    parser.add_argument("--motor-voltage-limit", type=float, default=6.0)
    parser.add_argument("--left-pwm", type=int, default=13)
    parser.add_argument("--left-in1", type=int, default=26)
    parser.add_argument("--left-in2", type=int, default=19)
    parser.add_argument("--right-pwm", type=int, default=12)
    parser.add_argument("--right-in1", type=int, default=20)
    parser.add_argument("--right-in2", type=int, default=21)
    parser.add_argument("--standby", type=int, default=16)
    parser.add_argument("--left-motor-inverted", dest="left_motor_inverted", action="store_true", default=True)
    parser.add_argument("--left-motor-normal", dest="left_motor_inverted", action="store_false")
    parser.add_argument("--right-motor-inverted", action="store_true")
    return parser.parse_args()


def build_config(args: argparse.Namespace) -> RobotAgentConfig:
    return RobotAgentConfig(
        backend_url=args.backend_url,
        robot_id=args.robot_id,
        shared_secret=args.shared_secret,
        telemetry_seconds=args.telemetry_seconds,
        mock=args.mock,
        speed_scale=args.speed_scale,
        turn_scale=args.turn_scale,
        supply_voltage=args.supply_voltage,
        motor_voltage_limit=args.motor_voltage_limit,
        left_motor=MotorPins(pwm=args.left_pwm, in1=args.left_in1, in2=args.left_in2),
        right_motor=MotorPins(pwm=args.right_pwm, in1=args.right_in1, in2=args.right_in2),
        standby_pin=args.standby,
        left_motor_inverted=args.left_motor_inverted,
        right_motor_inverted=args.right_motor_inverted,
    )


def main() -> None:
    asyncio.run(run_agent(build_config(parse_args())))


if __name__ == "__main__":
    main()
