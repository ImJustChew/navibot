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
import enum
import json
import math
import random
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from time import monotonic, sleep

from navibot.robot.encoders import EncoderPins, QuadratureEncoder
from navibot.robot.motors import DifferentialDrive, DriverMotor, MotorPins, clamp, validate_motor_voltage
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
    min_forward_pwm: float
    loop_seconds: float
    log_interval_seconds: float
    max_steps: int
    max_seconds: float
    stop_after_no_new_steps: int
    obstacle_mm: int
    side_obstacle_mm: int
    front_turn_mm: int
    wall_side: str
    wall_target_mm: int
    wall_deadband_mm: int
    wall_detect_mm: int
    wall_gain: float
    max_steer: float
    arc_slow_pwm: float
    stuck_window_seconds: float
    stuck_distance_mm: float
    escape_seconds: float
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
    stall_detect_min_pwm: float
    stall_min_counts_per_step: int
    stall_threshold_steps: int
    dead_end_side_mm: int
    min_reverse_mm: float
    back_obstacle_mm: int
    front_clear_mm: int
    reverse_speed: float
    reverse_heading_gain: float
    assess_steps: int
    frontier_update_steps: int
    frontier_gain: float
    random_walk_steps: int


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


class ExploreState(enum.Enum):
    FORWARD = "forward"
    TURNING = "turning"
    REVERSING = "reversing"
    ASSESS = "assess"
    ESCAPE = "escape"
    DONE = "done"


@dataclass
class StateContext:
    state: ExploreState
    # TURNING
    turn_start_theta: float = 0.0
    total_rotated: float = 0.0
    turn_direction: str = "rotate_left"
    # REVERSING
    reverse_start_x: float = 0.0
    reverse_start_y: float = 0.0
    # ASSESS
    stall_triggered: bool = False
    assess_ticks_remaining: int = 0
    post_reversal: bool = False
    # ESCAPE
    escape_until: float = 0.0
    escape_initialized: bool = False
    # RANDOM WALK
    random_walk_remaining: int = 0
    random_walk_heading: float = 0.0


class OccupancyGrid:
    def __init__(self, cell_size_mm: int) -> None:
        self.cell_size_mm = cell_size_mm
        self.free: set[tuple[int, int]] = set()
        self.occupied: set[tuple[int, int]] = set()
        self.penalized: set[tuple[int, int]] = set()

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

    def mark_penalized(self, x_mm: float, y_mm: float) -> None:
        self.penalized.add(self.to_cell(x_mm, y_mm))

    def to_cell(self, x_mm: float, y_mm: float) -> tuple[int, int]:
        return (math.floor(x_mm / self.cell_size_mm), math.floor(y_mm / self.cell_size_mm))

    def to_dict(self) -> dict[str, object]:
        return {
            "cell_size_mm": self.cell_size_mm,
            "free": [[x, y] for x, y in sorted(self.free)],
            "occupied": [[x, y] for x, y in sorted(self.occupied)],
            "penalized": [[x, y] for x, y in sorted(self.penalized)],
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


class StallDetector:
    def __init__(self, min_pwm: float, min_counts: int, threshold_steps: int) -> None:
        self._min_pwm = min_pwm
        self._min_counts = min_counts
        self._threshold = threshold_steps
        self._consecutive = 0

    def update(self, commanded_pwm: float, actual_delta: int) -> bool:
        """Return True if stall threshold reached this step."""
        if commanded_pwm < self._min_pwm:
            self._consecutive = 0
            return False
        if actual_delta < self._min_counts:
            self._consecutive += 1
        else:
            self._consecutive = 0
        return self._consecutive >= self._threshold

    def reset(self) -> None:
        self._consecutive = 0


def detect_dead_end(
    readings: dict[str, int | None],
    obstacle_mm: int,
    dead_end_side_mm: int,
) -> bool:
    front = readings.get("front")
    left = readings.get("left45")
    right = readings.get("right45")
    if front is None or left is None or right is None:
        return False
    return front < obstacle_mm and left < dead_end_side_mm and right < dead_end_side_mm


NEIGHBORS_8: tuple[tuple[int, int], ...] = (
    (-1, -1), (0, -1), (1, -1),
    (-1,  0),          (1,  0),
    (-1,  1), (0,  1), (1,  1),
)


class FrontierCache:
    def __init__(self, update_steps: int) -> None:
        self._update_steps = update_steps
        self._last_update = -1
        self._cached: float | None = None

    def get(self, step: int, grid: OccupancyGrid, pose: Pose) -> float | None:
        if step - self._last_update >= self._update_steps or self._last_update < 0:
            self._cached = self._compute(grid, pose)
            self._last_update = step
        return self._cached

    def invalidate(self) -> None:
        self._last_update = -1

    def _compute(self, grid: OccupancyGrid, pose: Pose) -> float | None:
        cell_size = grid.cell_size_mm
        frontiers: list[tuple[float, float]] = []
        for fx, fy in grid.free:
            if (fx, fy) in grid.penalized:
                continue
            for dx, dy in NEIGHBORS_8:
                nx, ny = fx + dx, fy + dy
                if (nx, ny) not in grid.free and (nx, ny) not in grid.occupied:
                    frontiers.append((fx * cell_size, fy * cell_size))
                    break
        if not frontiers:
            return None
        nearest = min(frontiers, key=lambda c: math.hypot(c[0] - pose.x_mm, c[1] - pose.y_mm))
        return math.atan2(nearest[1] - pose.y_mm, nearest[0] - pose.x_mm)


def choose_turn_biased(
    readings: dict[str, int | None],
    frontier_heading: float | None,
    current_theta: float,
    last_turn: str,
) -> str:
    left = readings.get("left45") or 0
    right = readings.get("right45") or 0
    if abs(left - right) >= 40:
        return "rotate_left" if left > right else "rotate_right"
    if frontier_heading is not None:
        diff = normalize_angle(frontier_heading - current_theta)
        return "rotate_left" if diff > 0 else "rotate_right"
    return last_turn


def tick_assess(
    ctx: StateContext,
    readings: dict[str, int | None],
    config: ExploreConfig,
    actual_delta: int,
) -> None:
    """Mutate ctx to advance ASSESS state."""
    if ctx.assess_ticks_remaining > 0:
        ctx.assess_ticks_remaining -= 1
        return

    if ctx.stall_triggered and actual_delta < config.stall_min_counts_per_step:
        ctx.state = ExploreState.ESCAPE
        ctx.stall_triggered = False
        return

    ctx.stall_triggered = False

    if detect_dead_end(readings, config.obstacle_mm, config.dead_end_side_mm):
        ctx.state = ExploreState.REVERSING
        return

    front = readings.get("front")
    if front is not None and front >= config.front_clear_mm:
        ctx.state = ExploreState.FORWARD
    else:
        ctx.state = ExploreState.TURNING


def tick_reversing(
    ctx: StateContext,
    pose: Pose,
    readings: dict[str, int | None],
    config: ExploreConfig,
) -> tuple[float, float]:
    """Return (left_pwm, right_pwm) as negatives (reversing). Mutate ctx on exit."""
    dist = math.hypot(pose.x_mm - ctx.reverse_start_x, pose.y_mm - ctx.reverse_start_y)
    back = readings.get("back")
    if dist >= config.min_reverse_mm or (back is not None and back < config.back_obstacle_mm):
        ctx.state = ExploreState.ASSESS
        ctx.post_reversal = True
        ctx.assess_ticks_remaining = config.assess_steps
        return 0.0, 0.0

    heading_error = normalize_angle(pose.theta_rad - ctx.turn_start_theta)
    steer = clamp(heading_error * config.reverse_heading_gain, -config.max_steer, config.max_steer)
    left_pwm = -(config.reverse_speed + steer)
    right_pwm = -(config.reverse_speed - steer)
    return left_pwm, right_pwm


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


def set_wheel_speeds(rig: ExploreRig, left_pwm: float, right_pwm: float) -> None:
    if left_pwm >= 0:
        rig.left.motor.forward(left_pwm)
    else:
        rig.left.motor.reverse(abs(left_pwm))

    if right_pwm >= 0:
        rig.right.motor.forward(right_pwm)
    else:
        rig.right.motor.reverse(abs(right_pwm))


def forward_pwm(value: float, config: ExploreConfig) -> float:
    if value <= 0:
        return 0.0
    return clamp(value, config.min_forward_pwm, config.speed + config.max_steer)


def distance_between(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _forward_command(
    readings: dict[str, int | None],
    frontier_heading: float | None,
    pose: Pose,
    last_turn: str,
    config: ExploreConfig,
) -> tuple[float, float, str, str]:
    """Return (left_pwm, right_pwm, action, last_turn) for FORWARD state."""
    front = readings.get("front")
    left = readings.get("left45")
    right = readings.get("right45")

    # Side obstacle avoidance
    if left is not None and left <= config.side_obstacle_mm:
        steer = config.max_steer
        return (
            forward_pwm(config.speed - steer, config),
            forward_pwm(config.speed + steer, config),
            "side_avoid_right", "rotate_right",
        )
    if right is not None and right <= config.side_obstacle_mm:
        steer = config.max_steer
        return (
            forward_pwm(config.speed + steer, config),
            forward_pwm(config.speed - steer, config),
            "side_avoid_left", "rotate_left",
        )

    # Wall following
    follow_side: str | None = None
    if left is not None and left <= config.wall_detect_mm:
        follow_side = "left"
    if right is not None and right <= config.wall_detect_mm:
        if follow_side is None or right < (left or 9999):
            follow_side = "right"

    if follow_side is not None:
        wall_reading = left if follow_side == "left" else right
        if wall_reading is not None:
            error_mm = wall_reading - config.wall_target_mm
            steer = 0.0 if abs(error_mm) <= config.wall_deadband_mm else clamp(
                error_mm * config.wall_gain, -config.max_steer, config.max_steer
            )
            if follow_side == "left":
                lp = forward_pwm(config.speed - steer, config)
                rp = forward_pwm(config.speed + steer, config)
            else:
                lp = forward_pwm(config.speed + steer, config)
                rp = forward_pwm(config.speed - steer, config)
            return lp, rp, f"follow_{follow_side}", last_turn

    # Open space: gentle frontier steer
    if frontier_heading is not None:
        angular_error = normalize_angle(frontier_heading - pose.theta_rad)
        frontier_steer = clamp(config.frontier_gain * angular_error, -config.max_steer, config.max_steer)
        lp = forward_pwm(config.speed - frontier_steer, config)
        rp = forward_pwm(config.speed + frontier_steer, config)
        return lp, rp, "frontier_steer", last_turn

    return config.speed, config.speed, "search_forward", last_turn


def run_explore(config: ExploreConfig) -> None:
    validate_motor_voltage(config.speed + config.max_steer, config.supply_voltage, config.motor_voltage_limit)
    validate_motor_voltage(config.turn_speed, config.supply_voltage, config.motor_voltage_limit)
    validate_motor_voltage(config.reverse_speed, config.supply_voltage, config.motor_voltage_limit)

    rig = ExploreRig(config)
    grid = OccupancyGrid(config.cell_size_mm)
    pose = Pose()
    points: list[MapPoint] = []
    path: list[dict[str, object]] = []
    previous_left = 0
    previous_right = 0
    no_new_steps = 0
    last_turn = "rotate_left"
    tof_cache = TofCache()
    stall_detector = StallDetector(
        min_pwm=config.stall_detect_min_pwm,
        min_counts=config.stall_min_counts_per_step,
        threshold_steps=config.stall_threshold_steps,
    )
    frontier_cache = FrontierCache(update_steps=config.frontier_update_steps)
    ctx = StateContext(
        state=ExploreState.FORWARD,
        assess_ticks_remaining=0,
    )
    stop_reason = "max_steps"
    started_at = monotonic()
    last_log_at = 0.0
    left_pwm = 0.0
    right_pwm = 0.0
    action = "init"

    try:
        rig.enable()
        rig.left.encoder.reset()
        rig.right.encoder.reset()
        tof_cache.wait_ready(rig, timeout_seconds=1.0)

        for step in range(config.max_steps):
            elapsed = monotonic() - started_at

            # --- termination checks ---
            if elapsed >= config.max_seconds:
                stop_reason = "max_seconds"
                break
            if no_new_steps >= config.stop_after_no_new_steps:
                stop_reason = "no_new_cells"
                break
            if ctx.state == ExploreState.DONE:
                stop_reason = "done"
                break

            # --- sensor + odometry ---
            readings = tof_cache.update(rig)
            left_sample = rig.left.encoder.sample()
            right_sample = rig.right.encoder.sample()
            actual_delta = abs(left_sample.counts - previous_left) + abs(right_sample.counts - previous_right)
            previous_left, previous_right = update_pose(
                pose, previous_left, previous_right,
                left_sample.counts, right_sample.counts, config,
            )
            new_cells = integrate_readings(grid, pose, readings, config, points, elapsed)
            no_new_steps = 0 if new_cells > 0 else no_new_steps + 1

            frontier_heading = frontier_cache.get(step, grid, pose)

            # --- state machine ---
            if ctx.state == ExploreState.FORWARD:
                stalled = stall_detector.update(
                    commanded_pwm=max(abs(left_pwm), abs(right_pwm)),
                    actual_delta=actual_delta,
                )
                dead_end = detect_dead_end(readings, config.obstacle_mm, config.dead_end_side_mm)
                if stalled or dead_end:
                    if dead_end:
                        grid.mark_penalized(pose.x_mm, pose.y_mm)
                        frontier_cache.invalidate()
                    ctx.state = ExploreState.ASSESS
                    ctx.stall_triggered = stalled
                    ctx.assess_ticks_remaining = config.assess_steps
                    left_pwm, right_pwm = 0.0, 0.0
                    action = "assess_entry"
                else:
                    front = readings.get("front")
                    if front is not None and front < config.front_turn_mm:
                        ctx.state = ExploreState.ASSESS
                        ctx.assess_ticks_remaining = config.assess_steps
                        left_pwm, right_pwm = 0.0, 0.0
                        action = "assess_entry"
                    else:
                        left_pwm, right_pwm, action, last_turn = _forward_command(
                            readings, frontier_heading, pose, last_turn, config,
                        )

            elif ctx.state == ExploreState.ASSESS:
                tick_assess(ctx, readings, config, actual_delta)
                left_pwm, right_pwm = 0.0, 0.0
                action = f"assess({ctx.state.value})"
                if ctx.state == ExploreState.TURNING:
                    turn_dir = choose_turn_biased(readings, frontier_heading, pose.theta_rad, last_turn)
                    ctx.turn_start_theta = pose.theta_rad
                    ctx.total_rotated = 0.0
                    ctx.turn_direction = turn_dir
                    last_turn = turn_dir
                elif ctx.state == ExploreState.REVERSING:
                    ctx.reverse_start_x = pose.x_mm
                    ctx.reverse_start_y = pose.y_mm
                    ctx.turn_start_theta = pose.theta_rad  # used for heading correction

            elif ctx.state == ExploreState.TURNING:
                dtheta = normalize_angle(pose.theta_rad - ctx.turn_start_theta)
                ctx.total_rotated += abs(dtheta)
                ctx.turn_start_theta = pose.theta_rad
                front = readings.get("front")
                min_turn_rad = math.radians(30)
                if ctx.total_rotated >= 2 * math.pi:
                    # full rotation — no clear path — reverse
                    ctx.state = ExploreState.ASSESS
                    ctx.post_reversal = False
                    ctx.assess_ticks_remaining = config.assess_steps
                    left_pwm, right_pwm = 0.0, 0.0
                    action = "full_rotation_assess"
                elif front is not None and front >= config.front_clear_mm and ctx.total_rotated >= min_turn_rad:
                    ctx.state = ExploreState.FORWARD
                    stall_detector.reset()
                    left_pwm, right_pwm = 0.0, 0.0
                    action = "turn_done"
                else:
                    pwm = config.turn_speed
                    if ctx.turn_direction == "rotate_left":
                        left_pwm, right_pwm = -pwm, pwm
                    else:
                        left_pwm, right_pwm = pwm, -pwm
                    action = f"turning_{ctx.turn_direction}"

            elif ctx.state == ExploreState.REVERSING:
                left_pwm, right_pwm = tick_reversing(ctx, pose, readings, config)
                action = "reversing"

            elif ctx.state == ExploreState.ESCAPE:
                if not ctx.escape_initialized:
                    ctx.escape_until = elapsed + config.escape_seconds
                    ctx.escape_initialized = True
                    stall_detector.reset()
                if elapsed >= ctx.escape_until:
                    ctx.state = ExploreState.ASSESS
                    ctx.assess_ticks_remaining = config.assess_steps
                    ctx.escape_initialized = False
                    left_pwm, right_pwm = 0.0, 0.0
                    action = "escape_done"
                else:
                    left_pwm, right_pwm = -config.reverse_speed, -config.reverse_speed
                    action = "escape_reversing"

            # random walk fallback: no frontiers and not done
            if frontier_heading is None and ctx.state == ExploreState.FORWARD and no_new_steps > 20:
                if ctx.random_walk_remaining > 0:
                    ctx.random_walk_remaining -= 1
                    if ctx.random_walk_remaining == 0:
                        frontier_cache.invalidate()
                        if frontier_cache.get(step, grid, pose) is None:
                            ctx.state = ExploreState.DONE
                            stop_reason = "no_frontiers"
                else:
                    ctx.random_walk_remaining = config.random_walk_steps
                    ctx.random_walk_heading = random.choice([
                        0.0, math.pi / 4, math.pi / 2, 3 * math.pi / 4,
                        math.pi, -3 * math.pi / 4, -math.pi / 2, -math.pi / 4,
                    ])
                    turn_dir = "rotate_left" if normalize_angle(ctx.random_walk_heading - pose.theta_rad) > 0 else "rotate_right"
                    ctx.state = ExploreState.ASSESS
                    ctx.turn_direction = turn_dir
                    ctx.assess_ticks_remaining = config.assess_steps
                    last_turn = turn_dir

            set_wheel_speeds(rig, left_pwm, right_pwm)

            path.append({
                "step": step,
                "t_s": elapsed,
                "action": action,
                "state": ctx.state.value,
                "left_pwm": left_pwm,
                "right_pwm": right_pwm,
                "x_mm": pose.x_mm,
                "y_mm": pose.y_mm,
                "theta_rad": pose.theta_rad,
                "theta_deg": math.degrees(pose.theta_rad),
                "left_counts": left_sample.counts,
                "right_counts": right_sample.counts,
                "new_cells": new_cells,
                "free_cells": len(grid.free),
                "occupied_cells": len(grid.occupied),
                "penalized_cells": len(grid.penalized),
                "front_mm": readings.get("front"),
                "left45_mm": readings.get("left45"),
                "right45_mm": readings.get("right45"),
                "back_mm": readings.get("back"),
            })
            if elapsed - last_log_at >= config.log_interval_seconds:
                last_log_at = elapsed
                print(
                    f"step={step:04d} state={ctx.state.value:<10} action={action:<20} "
                    f"pwm=({left_pwm:.3f},{right_pwm:.3f}) new={new_cells:03d} "
                    f"no_new={no_new_steps:03d} cells={len(grid.free)+len(grid.occupied):04d} "
                    f"pose=({pose.x_mm:7.1f},{pose.y_mm:7.1f},{math.degrees(pose.theta_rad):6.1f}deg) "
                    f"front={readings.get('front')} left45={readings.get('left45')} "
                    f"right45={readings.get('right45')} back={readings.get('back')}",
                    flush=True,
                )
            sleep(config.loop_seconds)
    finally:
        rig.drive.coast()
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
            "penalized_cells": len(grid.penalized),
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
        <div class="item"><span class="swatch" style="background:#b26bff"></span>penalized (dead end)</div>
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
      penalized_cells: meta.penalized_cells || 0,
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

      ctx.fillStyle = "#b26bff";
      for (const [cx, cy] of (data.grid.penalized || [])) {{
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
    parser.add_argument("--speed", type=float, default=0.20)
    parser.add_argument("--turn-speed", type=float, default=0.18)
    parser.add_argument("--min-forward-pwm", type=float, default=0.14)
    parser.add_argument("--loop-seconds", type=float, default=0.05)
    parser.add_argument("--log-interval-seconds", type=float, default=0.5)
    parser.add_argument("--max-steps", type=int, default=3600)
    parser.add_argument("--max-seconds", type=float, default=180.0)
    parser.add_argument("--stop-after-no-new-steps", type=int, default=400)
    parser.add_argument("--obstacle-mm", type=int, default=40)
    parser.add_argument("--side-obstacle-mm", type=int, default=70)
    parser.add_argument("--front-turn-mm", type=int, default=120)
    parser.add_argument("--wall-side", choices=("auto", "left", "right"), default="auto")
    parser.add_argument("--wall-target-mm", type=int, default=190)
    parser.add_argument("--wall-deadband-mm", type=int, default=35)
    parser.add_argument("--wall-detect-mm", type=int, default=700)
    parser.add_argument("--wall-gain", type=float, default=0.0009)
    parser.add_argument("--max-steer", type=float, default=0.045)
    parser.add_argument("--arc-slow-pwm", type=float, default=0.14)
    parser.add_argument("--stuck-window-seconds", type=float, default=4.0)
    parser.add_argument("--stuck-distance-mm", type=float, default=80.0)
    parser.add_argument("--escape-seconds", type=float, default=1.6)
    parser.add_argument("--stall-detect-min-pwm", type=float, default=0.10)
    parser.add_argument("--stall-min-counts-per-step", type=int, default=2)
    parser.add_argument("--stall-threshold-steps", type=int, default=5)
    parser.add_argument("--dead-end-side-mm", type=int, default=150)
    parser.add_argument("--min-reverse-mm", type=float, default=200.0)
    parser.add_argument("--back-obstacle-mm", type=int, default=80)
    parser.add_argument("--front-clear-mm", type=int, default=200)
    parser.add_argument("--reverse-speed", type=float, default=0.15)
    parser.add_argument("--reverse-heading-gain", type=float, default=2.0)
    parser.add_argument("--assess-steps", type=int, default=3)
    parser.add_argument("--frontier-update-steps", type=int, default=50)
    parser.add_argument("--frontier-gain", type=float, default=0.8)
    parser.add_argument("--random-walk-steps", type=int, default=200)
    parser.add_argument("--min-valid-tof-mm", type=int, default=40)
    parser.add_argument("--max-valid-tof-mm", type=int, default=3000)
    parser.add_argument("--cell-size-mm", type=int, default=50)
    parser.add_argument("--ray-step-mm", type=int, default=50)
    parser.add_argument("--wheel-diameter-mm", type=float, default=43.0)
    parser.add_argument("--wheel-track-mm", type=float, default=64.0)
    parser.add_argument("--pulses-per-channel", type=int, default=7)
    parser.add_argument("--gear-ratio", type=float, default=132.0)
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
    if args.min_forward_pwm < 0 or args.arc_slow_pwm < 0:
        raise ValueError("--min-forward-pwm and --arc-slow-pwm cannot be negative")
    if args.loop_seconds <= 0:
        raise ValueError("--loop-seconds must be greater than zero")
    if args.cell_size_mm <= 0 or args.ray_step_mm <= 0:
        raise ValueError("--cell-size-mm and --ray-step-mm must be greater than zero")
    if args.stop_after_no_new_steps <= 0:
        raise ValueError("--stop-after-no-new-steps must be greater than zero")
    if args.max_steer < 0:
        raise ValueError("--max-steer cannot be negative")
    if args.stuck_window_seconds <= 0 or args.escape_seconds <= 0:
        raise ValueError("--stuck-window-seconds and --escape-seconds must be greater than zero")
    if args.stuck_distance_mm < 0:
        raise ValueError("--stuck-distance-mm cannot be negative")

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
        min_forward_pwm=args.min_forward_pwm,
        loop_seconds=args.loop_seconds,
        log_interval_seconds=args.log_interval_seconds,
        max_steps=args.max_steps,
        max_seconds=args.max_seconds,
        stop_after_no_new_steps=args.stop_after_no_new_steps,
        obstacle_mm=args.obstacle_mm,
        side_obstacle_mm=args.side_obstacle_mm,
        front_turn_mm=args.front_turn_mm,
        wall_side=args.wall_side,
        wall_target_mm=args.wall_target_mm,
        wall_deadband_mm=args.wall_deadband_mm,
        wall_detect_mm=args.wall_detect_mm,
        wall_gain=args.wall_gain,
        max_steer=args.max_steer,
        arc_slow_pwm=args.arc_slow_pwm,
        stuck_window_seconds=args.stuck_window_seconds,
        stuck_distance_mm=args.stuck_distance_mm,
        escape_seconds=args.escape_seconds,
        stall_detect_min_pwm=args.stall_detect_min_pwm,
        stall_min_counts_per_step=args.stall_min_counts_per_step,
        stall_threshold_steps=args.stall_threshold_steps,
        dead_end_side_mm=args.dead_end_side_mm,
        min_reverse_mm=args.min_reverse_mm,
        back_obstacle_mm=args.back_obstacle_mm,
        front_clear_mm=args.front_clear_mm,
        reverse_speed=args.reverse_speed,
        reverse_heading_gain=args.reverse_heading_gain,
        assess_steps=args.assess_steps,
        frontier_update_steps=args.frontier_update_steps,
        frontier_gain=args.frontier_gain,
        random_walk_steps=args.random_walk_steps,
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
