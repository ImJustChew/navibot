"""Cautious self-exploration and TOF map generation.

This is a proof-of-capability script, not production SLAM. It uses encoder
odometry plus four VL53L1X range rays to build a simple occupancy grid:

- unknown cells start gray
- free cells are marked along each TOF ray
- occupied cells are marked at range endpoints
- robot path is recorded from wheel encoder odometry

The robot explores with continuous bounded drive segments. It rotates when
blocked by any forward-facing TOF sensor or when it has not discovered new
cells recently, and stops after sustained no-new-coverage.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from time import monotonic, sleep

from navibot.robot.encoders import EncoderPins, QuadratureEncoder
from navibot.robot.motors import DifferentialDrive, DriverMotor, MotorPins, validate_motor_voltage
from navibot.sensors.vl53l1x_array import Vl53l1xArray


TOF_ANGLES_RAD = {
    "front": 0.0,
    "left45": math.radians(45),
    "right45": math.radians(-45),
    "back": math.pi,
}


@dataclass(frozen=True)
class WheelPins:
    motor: MotorPins
    encoder: EncoderPins


@dataclass(frozen=True)
class ExploreConfig:
    left: WheelPins
    right: WheelPins
    standby_pin: int
    output_dir: Path
    speed: float
    turn_speed: float
    forward_segment_seconds: float
    turn_check_seconds: float
    max_turn_seconds: float
    sensor_check_seconds: float
    settle_seconds: float
    max_steps: int
    max_seconds: float
    stop_after_no_new_steps: int
    scan_after_no_new_steps: int
    obstacle_mm: int
    side_obstacle_mm: int
    clear_front_mm: int
    clear_side_mm: int
    min_valid_tof_mm: int
    max_valid_tof_mm: int
    cell_size_mm: int
    ray_step_mm: int
    wheel_diameter_mm: float
    wheel_track_mm: float
    pulses_per_channel: int
    gear_ratio: float
    supply_voltage: float
    motor_voltage_limit: float
    left_motor_inverted: bool
    right_motor_inverted: bool
    left_encoder_inverted: bool
    right_encoder_inverted: bool
    pull_up: bool


@dataclass
class Pose:
    x_mm: float = 0.0
    y_mm: float = 0.0
    theta_rad: float = 0.0


@dataclass(frozen=True)
class MapPoint:
    x_mm: float
    y_mm: float
    sensor: str
    distance_mm: int
    t_s: float


class OccupancyGrid:
    def __init__(self, cell_size_mm: int) -> None:
        self.cell_size_mm = cell_size_mm
        self.free: set[tuple[int, int]] = set()
        self.occupied: set[tuple[int, int]] = set()

    def mark_free(self, x_mm: float, y_mm: float) -> bool:
        cell = self.to_cell(x_mm, y_mm)
        if cell in self.free:
            return False
        self.free.add(cell)
        return True

    def mark_occupied(self, x_mm: float, y_mm: float) -> bool:
        cell = self.to_cell(x_mm, y_mm)
        if cell in self.occupied:
            return False
        self.occupied.add(cell)
        return True

    def to_cell(self, x_mm: float, y_mm: float) -> tuple[int, int]:
        return (math.floor(x_mm / self.cell_size_mm), math.floor(y_mm / self.cell_size_mm))

    def to_dict(self) -> dict[str, object]:
        return {
            "cell_size_mm": self.cell_size_mm,
            "free": [[x, y] for x, y in sorted(self.free)],
            "occupied": [[x, y] for x, y in sorted(self.occupied)],
        }


class Wheel:
    def __init__(
        self,
        pins: WheelPins,
        motor_inverted: bool,
        encoder_inverted: bool,
        pull_up: bool,
    ) -> None:
        self.motor = DriverMotor(pins.motor, inverted=motor_inverted, brake_on_stop=False)
        self.encoder = QuadratureEncoder(pins.encoder, pull_up=pull_up, inverted=encoder_inverted)


class ExploreRig:
    def __init__(self, config: ExploreConfig) -> None:
        self.left = Wheel(
            config.left,
            motor_inverted=config.left_motor_inverted,
            encoder_inverted=config.left_encoder_inverted,
            pull_up=config.pull_up,
        )
        self.right = Wheel(
            config.right,
            motor_inverted=config.right_motor_inverted,
            encoder_inverted=config.right_encoder_inverted,
            pull_up=config.pull_up,
        )
        self.drive = DifferentialDrive(
            left=self.left.motor,
            right=self.right.motor,
            standby_pin=config.standby_pin,
        )
        self.tof = Vl53l1xArray()

    def enable(self) -> None:
        self.drive.enable()
        self.tof.start_ranging()

    def close(self) -> None:
        self.drive.close()
        self.left.encoder.close()
        self.right.encoder.close()
        self.tof.close()


class TofCache:
    def __init__(self) -> None:
        self.readings: dict[str, int | None] = {
            "front": None,
            "left45": None,
            "right45": None,
            "back": None,
        }

    def update(self, rig: ExploreRig) -> dict[str, int | None]:
        for reading in rig.tof.read_all():
            if reading.ready:
                self.readings[reading.name] = reading.distance_mm
        return dict(self.readings)

    def wait_ready(self, rig: ExploreRig, timeout_seconds: float) -> dict[str, int | None]:
        deadline = monotonic() + timeout_seconds
        while monotonic() < deadline:
            self.update(rig)
            if all(value is not None for value in self.readings.values()):
                break
            sleep(0.01)
        return dict(self.readings)


def counts_to_mm(counts: int, config: ExploreConfig) -> float:
    counts_per_rev = config.pulses_per_channel * 4 * config.gear_ratio
    circumference_mm = math.pi * config.wheel_diameter_mm
    return counts * circumference_mm / counts_per_rev


def update_pose(
    pose: Pose,
    previous_left: int,
    previous_right: int,
    current_left: int,
    current_right: int,
    config: ExploreConfig,
) -> tuple[int, int]:
    dl_mm = counts_to_mm(current_left - previous_left, config)
    dr_mm = counts_to_mm(current_right - previous_right, config)
    dc_mm = (dl_mm + dr_mm) / 2
    dtheta = (dr_mm - dl_mm) / config.wheel_track_mm
    theta_mid = pose.theta_rad + dtheta / 2
    pose.x_mm += dc_mm * math.cos(theta_mid)
    pose.y_mm += dc_mm * math.sin(theta_mid)
    pose.theta_rad = normalize_angle(pose.theta_rad + dtheta)
    return current_left, current_right


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def latest_tof_readings(
    rig: ExploreRig,
    timeout_seconds: float = 0.5,
    cache: TofCache | None = None,
) -> dict[str, int | None]:
    if cache is not None:
        return cache.wait_ready(rig, timeout_seconds)

    deadline = monotonic() + timeout_seconds
    readings: dict[str, int | None] = {}
    while monotonic() < deadline:
        for reading in rig.tof.read_all():
            if reading.ready:
                readings[reading.name] = reading.distance_mm
        if {"front", "left45", "right45", "back"}.issubset(readings):
            break
        sleep(0.02)
    return readings


def integrate_readings(
    grid: OccupancyGrid,
    pose: Pose,
    readings: dict[str, int | None],
    config: ExploreConfig,
    points: list[MapPoint],
    t_s: float,
) -> int:
    new_cells = 0
    new_cells += int(grid.mark_free(pose.x_mm, pose.y_mm))

    for name, distance_mm in readings.items():
        if distance_mm is None:
            continue
        if not config.min_valid_tof_mm <= distance_mm <= config.max_valid_tof_mm:
            continue

        angle = pose.theta_rad + TOF_ANGLES_RAD.get(name, 0.0)
        free_until = max(0, distance_mm - config.cell_size_mm)
        ray = config.ray_step_mm
        while ray <= free_until:
            x_mm = pose.x_mm + ray * math.cos(angle)
            y_mm = pose.y_mm + ray * math.sin(angle)
            new_cells += int(grid.mark_free(x_mm, y_mm))
            ray += config.ray_step_mm

        hit_x = pose.x_mm + distance_mm * math.cos(angle)
        hit_y = pose.y_mm + distance_mm * math.sin(angle)
        new_cells += int(grid.mark_occupied(hit_x, hit_y))
        points.append(
            MapPoint(
                x_mm=hit_x,
                y_mm=hit_y,
                sensor=name,
                distance_mm=distance_mm,
                t_s=t_s,
            )
        )
    return new_cells


def choose_action(readings: dict[str, int | None], no_new_steps: int, config: ExploreConfig) -> str:
    left = readings.get("left45") or 0
    right = readings.get("right45") or 0

    if path_blocked(readings, config):
        return choose_turn_direction(readings, "rotate_left")
    if no_new_steps >= config.scan_after_no_new_steps:
        return "rotate_left" if left > right else "rotate_right"
    return "forward"


def choose_turn_direction(readings: dict[str, int | None], last_turn: str) -> str:
    left = readings.get("left45") or 0
    right = readings.get("right45") or 0
    if abs(left - right) < 40:
        return last_turn
    return "rotate_left" if left > right else "rotate_right"


def path_blocked(readings: dict[str, int | None], config: ExploreConfig) -> bool:
    front = readings.get("front")
    left = readings.get("left45")
    right = readings.get("right45")
    return (
        (front is not None and front <= config.obstacle_mm)
        or (left is not None and left <= config.side_obstacle_mm)
        or (right is not None and right <= config.side_obstacle_mm)
    )


def path_clear(readings: dict[str, int | None], config: ExploreConfig) -> bool:
    front = readings.get("front")
    left = readings.get("left45")
    right = readings.get("right45")
    return (
        front is not None
        and front >= config.clear_front_mm
        and (left is None or left >= config.clear_side_mm)
        and (right is None or right >= config.clear_side_mm)
    )


def execute_action(
    rig: ExploreRig,
    action: str,
    config: ExploreConfig,
    last_turn: str,
    cache: TofCache,
) -> tuple[str, str]:
    if action == "forward":
        return drive_forward_segment(rig, config, cache, last_turn)
    elif action == "rotate_left":
        rotate_until_clear(rig, config, cache, "rotate_left")
        return "rotate_left", "rotate_left"
    elif action == "rotate_right":
        rotate_until_clear(rig, config, cache, "rotate_right")
        return "rotate_right", "rotate_right"
    else:
        raise ValueError(f"unknown action: {action}")


def drive_forward_segment(
    rig: ExploreRig,
    config: ExploreConfig,
    cache: TofCache,
    last_turn: str,
) -> tuple[str, str]:
    started = monotonic()
    readings = cache.update(rig)
    if path_blocked(readings, config):
        turn = choose_turn_direction(readings, last_turn)
        rotate_until_clear(rig, config, cache, turn)
        return f"avoid_{turn}", turn

    rig.drive.forward(config.speed)
    try:
        while monotonic() - started < config.forward_segment_seconds:
            sleep(config.sensor_check_seconds)
            readings = cache.update(rig)
            if path_blocked(readings, config):
                rig.drive.coast()
                turn = choose_turn_direction(readings, last_turn)
                rotate_until_clear(rig, config, cache, turn)
                return f"avoid_{turn}", turn
        return "forward", last_turn
    finally:
        rig.drive.coast()


def rotate_until_clear(
    rig: ExploreRig,
    config: ExploreConfig,
    cache: TofCache,
    direction: str,
) -> None:
    started = monotonic()
    if direction == "rotate_left":
        rig.drive.rotate_left(config.turn_speed)
    else:
        rig.drive.rotate_right(config.turn_speed)
    try:
        while monotonic() - started < config.max_turn_seconds:
            sleep(config.turn_check_seconds)
            readings = cache.update(rig)
            if path_clear(readings, config):
                return
    finally:
        rig.drive.coast()
        sleep(config.settle_seconds)


def run_explore(config: ExploreConfig) -> None:
    validate_motor_voltage(config.speed, config.supply_voltage, config.motor_voltage_limit)
    validate_motor_voltage(config.turn_speed, config.supply_voltage, config.motor_voltage_limit)

    rig = ExploreRig(config)
    grid = OccupancyGrid(config.cell_size_mm)
    pose = Pose()
    points: list[MapPoint] = []
    path: list[dict[str, object]] = []
    previous_left = 0
    previous_right = 0
    no_new_steps = 0
    stuck_turns = 0
    last_turn = "rotate_left"
    tof_cache = TofCache()
    stop_reason = "max_steps"
    started_at = monotonic()

    try:
        rig.enable()
        rig.left.encoder.reset()
        rig.right.encoder.reset()
        tof_cache.wait_ready(rig, timeout_seconds=1.0)

        for step in range(config.max_steps):
            elapsed = monotonic() - started_at
            if elapsed >= config.max_seconds:
                stop_reason = "max_seconds"
                break
            if no_new_steps >= config.stop_after_no_new_steps:
                stop_reason = "no_new_cells"
                break

            readings_before = tof_cache.update(rig)
            action = choose_action(readings_before, no_new_steps, config)
            if action.startswith("rotate"):
                action = choose_turn_direction(readings_before, last_turn)
            executed_action, last_turn = execute_action(rig, action, config, last_turn, tof_cache)

            left_sample = rig.left.encoder.sample()
            right_sample = rig.right.encoder.sample()
            previous_left, previous_right = update_pose(
                pose,
                previous_left,
                previous_right,
                left_sample.counts,
                right_sample.counts,
                config,
            )

            elapsed = monotonic() - started_at
            readings = tof_cache.update(rig)
            new_cells = integrate_readings(grid, pose, readings, config, points, elapsed)
            no_new_steps = no_new_steps + 1 if new_cells == 0 else 0
            front_after = readings.get("front")
            if (
                executed_action.startswith("rotate")
                and front_after is not None
                and front_after <= config.obstacle_mm
            ):
                stuck_turns += 1
            else:
                stuck_turns = 0
            if stuck_turns >= 6:
                last_turn = "rotate_right" if last_turn == "rotate_left" else "rotate_left"
                no_new_steps = max(no_new_steps, config.scan_after_no_new_steps)
                stuck_turns = 0
            path.append(
                {
                    "step": step,
                    "t_s": elapsed,
                    "action": executed_action,
                    "x_mm": pose.x_mm,
                    "y_mm": pose.y_mm,
                    "theta_rad": pose.theta_rad,
                    "theta_deg": math.degrees(pose.theta_rad),
                    "left_counts": left_sample.counts,
                    "right_counts": right_sample.counts,
                    "new_cells": new_cells,
                    "free_cells": len(grid.free),
                    "occupied_cells": len(grid.occupied),
                    "front_mm": readings.get("front"),
                    "left45_mm": readings.get("left45"),
                    "right45_mm": readings.get("right45"),
                    "back_mm": readings.get("back"),
                }
            )
            print(
                f"step={step:03d} action={executed_action:<15} new={new_cells:03d} "
                f"no_new={no_new_steps:02d} cells={len(grid.free) + len(grid.occupied):04d} "
                f"pose=({pose.x_mm:7.1f},{pose.y_mm:7.1f},{math.degrees(pose.theta_rad):6.1f}deg) "
                f"front={readings.get('front')} left45={readings.get('left45')} "
                f"right45={readings.get('right45')} back={readings.get('back')}",
                flush=True,
            )
    finally:
        rig.close()

    write_outputs(config, grid, path, points, stop_reason)


def write_outputs(
    config: ExploreConfig,
    grid: OccupancyGrid,
    path: list[dict[str, object]],
    points: list[MapPoint],
    stop_reason: str,
) -> None:
    config.output_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "metadata": {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "stop_reason": stop_reason,
            "path_samples": len(path),
            "map_points": len(points),
            "free_cells": len(grid.free),
            "occupied_cells": len(grid.occupied),
            "config": {
                key: str(value) if isinstance(value, Path) else value
                for key, value in asdict(config).items()
            },
        },
        "grid": grid.to_dict(),
        "path": path,
        "points": [asdict(point) for point in points],
    }
    (config.output_dir / "map.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    (config.output_dir / "map.html").write_text(render_html(payload), encoding="utf-8")
    print(f"Wrote {config.output_dir / 'map.json'}")
    print(f"Wrote {config.output_dir / 'map.html'}")


def render_html(payload: dict[str, object]) -> str:
    data_json = json.dumps(payload, separators=(",", ":"))
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Navibot Exploration Map</title>
  <style>
    :root {{ color-scheme: dark; font-family: Arial, sans-serif; background: #101417; color: #e8edf0; }}
    body {{ margin: 0; display: grid; grid-template-rows: auto 1fr; min-height: 100vh; }}
    header {{ display: flex; gap: 24px; align-items: baseline; padding: 14px 18px; border-bottom: 1px solid #2d363d; background: #151b20; }}
    h1 {{ margin: 0; font-size: 18px; }}
    .meta {{ display: flex; gap: 16px; flex-wrap: wrap; color: #aeb8bf; font-size: 13px; }}
    main {{ display: grid; grid-template-columns: 1fr 300px; min-height: 0; }}
    canvas {{ width: 100%; height: 100%; background: #0b0f12; display: block; }}
    aside {{ border-left: 1px solid #2d363d; padding: 14px; background: #12181d; overflow: auto; }}
    dl {{ display: grid; grid-template-columns: auto 1fr; gap: 8px 12px; font-size: 13px; }}
    dt {{ color: #8d9aa3; }}
    dd {{ margin: 0; text-align: right; }}
    .legend {{ display: grid; gap: 8px; margin-top: 18px; font-size: 13px; }}
    .item {{ display: flex; gap: 8px; align-items: center; }}
    .swatch {{ width: 14px; height: 14px; border-radius: 2px; }}
  </style>
</head>
<body>
  <header>
    <h1>Navibot Exploration Map</h1>
    <div class="meta" id="meta"></div>
  </header>
  <main>
    <canvas id="map"></canvas>
    <aside>
      <dl id="stats"></dl>
      <div class="legend">
        <div class="item"><span class="swatch" style="background:#26323a"></span>free grid</div>
        <div class="item"><span class="swatch" style="background:#f05d4f"></span>TOF hit / occupied</div>
        <div class="item"><span class="swatch" style="background:#56a6ff"></span>robot path</div>
        <div class="item"><span class="swatch" style="background:#f3d36b"></span>robot final pose</div>
      </div>
    </aside>
  </main>
  <script>
    const data = {data_json};
    const canvas = document.getElementById("map");
    const ctx = canvas.getContext("2d");
    const meta = data.metadata;
    document.getElementById("meta").textContent =
      `${{meta.created_at}} · stop=${{meta.stop_reason}} · points=${{meta.map_points}}`;
    const stats = document.getElementById("stats");
    for (const [key, value] of Object.entries({{
      path_samples: meta.path_samples,
      free_cells: meta.free_cells,
      occupied_cells: meta.occupied_cells,
      cell_size_mm: data.grid.cell_size_mm
    }})) {{
      const dt = document.createElement("dt");
      const dd = document.createElement("dd");
      dt.textContent = key;
      dd.textContent = value;
      stats.append(dt, dd);
    }}

    function resize() {{
      canvas.width = canvas.clientWidth * devicePixelRatio;
      canvas.height = canvas.clientHeight * devicePixelRatio;
      draw();
    }}

    function bounds() {{
      const values = [];
      for (const [x, y] of data.grid.free) values.push([x * data.grid.cell_size_mm, y * data.grid.cell_size_mm]);
      for (const [x, y] of data.grid.occupied) values.push([x * data.grid.cell_size_mm, y * data.grid.cell_size_mm]);
      for (const p of data.path) values.push([p.x_mm, p.y_mm]);
      let minX = -500, maxX = 500, minY = -500, maxY = 500;
      for (const [x, y] of values) {{
        minX = Math.min(minX, x); maxX = Math.max(maxX, x);
        minY = Math.min(minY, y); maxY = Math.max(maxY, y);
      }}
      return {{ minX, maxX, minY, maxY }};
    }}

    function draw() {{
      const w = canvas.width, h = canvas.height;
      ctx.clearRect(0, 0, w, h);
      const b = bounds();
      const pad = 60 * devicePixelRatio;
      const scale = Math.min((w - pad * 2) / (b.maxX - b.minX || 1), (h - pad * 2) / (b.maxY - b.minY || 1));
      const tx = pad - b.minX * scale;
      const ty = h - pad + b.minY * scale;
      const cell = Math.max(2, data.grid.cell_size_mm * scale);
      const xy = (x, y) => [tx + x * scale, ty - y * scale];

      ctx.fillStyle = "#26323a";
      for (const [cx, cy] of data.grid.free) {{
        const [x, y] = xy(cx * data.grid.cell_size_mm, cy * data.grid.cell_size_mm);
        ctx.fillRect(x, y - cell, cell, cell);
      }}

      ctx.fillStyle = "#f05d4f";
      for (const [cx, cy] of data.grid.occupied) {{
        const [x, y] = xy(cx * data.grid.cell_size_mm, cy * data.grid.cell_size_mm);
        ctx.fillRect(x, y - cell, cell, cell);
      }}

      ctx.strokeStyle = "#56a6ff";
      ctx.lineWidth = 2 * devicePixelRatio;
      ctx.beginPath();
      data.path.forEach((p, i) => {{
        const [x, y] = xy(p.x_mm, p.y_mm);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }});
      ctx.stroke();

      const last = data.path[data.path.length - 1];
      if (last) {{
        const [x, y] = xy(last.x_mm, last.y_mm);
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(-last.theta_rad);
        ctx.fillStyle = "#f3d36b";
        ctx.beginPath();
        ctx.moveTo(12 * devicePixelRatio, 0);
        ctx.lineTo(-8 * devicePixelRatio, -7 * devicePixelRatio);
        ctx.lineTo(-8 * devicePixelRatio, 7 * devicePixelRatio);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }}
    }}
    addEventListener("resize", resize);
    resize();
  </script>
</body>
</html>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Explore until no new TOF grid cells are discovered.")
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/explore/latest"))
    parser.add_argument("--speed", type=float, default=0.13)
    parser.add_argument("--turn-speed", type=float, default=0.13)
    parser.add_argument("--forward-segment-seconds", type=float, default=3.0)
    parser.add_argument("--turn-check-seconds", type=float, default=0.04)
    parser.add_argument("--max-turn-seconds", type=float, default=1.2)
    parser.add_argument("--sensor-check-seconds", type=float, default=0.03)
    parser.add_argument("--settle-seconds", type=float, default=0.05)
    parser.add_argument("--max-steps", type=int, default=300)
    parser.add_argument("--max-seconds", type=float, default=180.0)
    parser.add_argument("--stop-after-no-new-steps", type=int, default=35)
    parser.add_argument("--scan-after-no-new-steps", type=int, default=8)
    parser.add_argument("--obstacle-mm", type=int, default=40)
    parser.add_argument("--side-obstacle-mm", type=int, default=70)
    parser.add_argument("--clear-front-mm", type=int, default=140)
    parser.add_argument("--clear-side-mm", type=int, default=90)
    parser.add_argument("--min-valid-tof-mm", type=int, default=40)
    parser.add_argument("--max-valid-tof-mm", type=int, default=3000)
    parser.add_argument("--cell-size-mm", type=int, default=50)
    parser.add_argument("--ray-step-mm", type=int, default=50)
    parser.add_argument("--wheel-diameter-mm", type=float, default=43.0)
    parser.add_argument("--wheel-track-mm", type=float, default=105.0)
    parser.add_argument("--pulses-per-channel", type=int, default=7)
    parser.add_argument("--gear-ratio", type=float, default=105.6)
    parser.add_argument("--supply-voltage", type=float, default=7.4)
    parser.add_argument("--motor-voltage-limit", type=float, default=6.0)
    parser.add_argument("--pull-up", dest="pull_up", action="store_true", default=True)
    parser.add_argument("--no-pull-up", dest="pull_up", action="store_false")
    parser.add_argument("--left-motor-inverted", dest="left_motor_inverted", action="store_true", default=True)
    parser.add_argument("--left-motor-normal", dest="left_motor_inverted", action="store_false")
    parser.add_argument("--right-motor-inverted", action="store_true")
    parser.add_argument("--left-encoder-inverted", action="store_true")
    parser.add_argument("--right-encoder-inverted", dest="right_encoder_inverted", action="store_true", default=True)
    parser.add_argument("--right-encoder-normal", dest="right_encoder_inverted", action="store_false")
    parser.add_argument("--left-pwm", type=int, default=13)
    parser.add_argument("--left-in1", type=int, default=26)
    parser.add_argument("--left-in2", type=int, default=19)
    parser.add_argument("--left-encoder-a", type=int, default=23)
    parser.add_argument("--left-encoder-b", type=int, default=24)
    parser.add_argument("--right-pwm", type=int, default=12)
    parser.add_argument("--right-in1", type=int, default=20)
    parser.add_argument("--right-in2", type=int, default=21)
    parser.add_argument("--right-encoder-a", type=int, default=27)
    parser.add_argument("--right-encoder-b", type=int, default=22)
    parser.add_argument("--standby", type=int, default=16)
    return parser.parse_args()


def build_config(args: argparse.Namespace) -> ExploreConfig:
    if args.speed <= 0 or args.turn_speed <= 0:
        raise ValueError("--speed and --turn-speed must be greater than zero")
    if args.cell_size_mm <= 0 or args.ray_step_mm <= 0:
        raise ValueError("--cell-size-mm and --ray-step-mm must be greater than zero")
    if args.stop_after_no_new_steps <= 0:
        raise ValueError("--stop-after-no-new-steps must be greater than zero")

    return ExploreConfig(
        left=WheelPins(
            motor=MotorPins(pwm=args.left_pwm, in1=args.left_in1, in2=args.left_in2),
            encoder=EncoderPins(a=args.left_encoder_a, b=args.left_encoder_b),
        ),
        right=WheelPins(
            motor=MotorPins(pwm=args.right_pwm, in1=args.right_in1, in2=args.right_in2),
            encoder=EncoderPins(a=args.right_encoder_a, b=args.right_encoder_b),
        ),
        standby_pin=args.standby,
        output_dir=args.output_dir,
        speed=args.speed,
        turn_speed=args.turn_speed,
        forward_segment_seconds=args.forward_segment_seconds,
        turn_check_seconds=args.turn_check_seconds,
        max_turn_seconds=args.max_turn_seconds,
        sensor_check_seconds=args.sensor_check_seconds,
        settle_seconds=args.settle_seconds,
        max_steps=args.max_steps,
        max_seconds=args.max_seconds,
        stop_after_no_new_steps=args.stop_after_no_new_steps,
        scan_after_no_new_steps=args.scan_after_no_new_steps,
        obstacle_mm=args.obstacle_mm,
        side_obstacle_mm=args.side_obstacle_mm,
        clear_front_mm=args.clear_front_mm,
        clear_side_mm=args.clear_side_mm,
        min_valid_tof_mm=args.min_valid_tof_mm,
        max_valid_tof_mm=args.max_valid_tof_mm,
        cell_size_mm=args.cell_size_mm,
        ray_step_mm=args.ray_step_mm,
        wheel_diameter_mm=args.wheel_diameter_mm,
        wheel_track_mm=args.wheel_track_mm,
        pulses_per_channel=args.pulses_per_channel,
        gear_ratio=args.gear_ratio,
        supply_voltage=args.supply_voltage,
        motor_voltage_limit=args.motor_voltage_limit,
        left_motor_inverted=args.left_motor_inverted,
        right_motor_inverted=args.right_motor_inverted,
        left_encoder_inverted=args.left_encoder_inverted,
        right_encoder_inverted=args.right_encoder_inverted,
        pull_up=args.pull_up,
    )


def main() -> None:
    run_explore(build_config(parse_args()))


if __name__ == "__main__":
    main()
