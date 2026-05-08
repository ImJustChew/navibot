"""Unit tests for smart explorer logic (no hardware required)."""
from __future__ import annotations
import importlib.util
import math
import pathlib
import sys
from unittest.mock import MagicMock

# Stub hardware modules with MagicMock so attribute access (e.g. "from gpiozero import Device")
# succeeds without raising AttributeError on the dev machine.
for _mod_name in [
    "gpiozero", "lgpio", "board", "busio", "digitalio",
    "adafruit_vl53l1x", "adafruit_blinka", "adafruit_platformdetect",
    "adafruit_platformdetect.board",
]:
    sys.modules[_mod_name] = MagicMock()

# Load the script as a module
_spec = importlib.util.spec_from_file_location(
    "self_explore_room",
    pathlib.Path(__file__).parents[2] / "scripts" / "self_explore_room.py",
)
_mod = importlib.util.module_from_spec(_spec)  # type: ignore[arg-type]
sys.modules["self_explore_room"] = _mod
_spec.loader.exec_module(_mod)  # type: ignore[union-attr]

OccupancyGrid = _mod.OccupancyGrid


# --- OccupancyGrid.penalized ---

def test_occupancy_grid_mark_penalized_adds_cell() -> None:
    grid = OccupancyGrid(cell_size_mm=50)
    grid.mark_penalized(125.0, 75.0)
    assert (2, 1) in grid.penalized


def test_occupancy_grid_penalized_not_in_free_or_occupied() -> None:
    grid = OccupancyGrid(cell_size_mm=50)
    grid.mark_penalized(0.0, 0.0)
    assert len(grid.free) == 0
    assert len(grid.occupied) == 0


def test_occupancy_grid_to_dict_includes_penalized() -> None:
    grid = OccupancyGrid(cell_size_mm=50)
    grid.mark_penalized(0.0, 0.0)
    d = grid.to_dict()
    assert "penalized" in d
    assert [0, 0] in d["penalized"]


ExploreState = _mod.ExploreState
StateContext = _mod.StateContext


def test_explore_state_has_required_values() -> None:
    assert hasattr(ExploreState, "FORWARD")
    assert hasattr(ExploreState, "TURNING")
    assert hasattr(ExploreState, "REVERSING")
    assert hasattr(ExploreState, "ASSESS")
    assert hasattr(ExploreState, "ESCAPE")
    assert hasattr(ExploreState, "DONE")


def test_state_context_defaults() -> None:
    ctx = StateContext(state=ExploreState.FORWARD)
    assert ctx.stall_triggered is False
    assert ctx.post_reversal is False
    assert ctx.total_rotated == 0.0
    assert ctx.assess_ticks_remaining == 0


def test_explore_config_has_new_params() -> None:
    # Just verify the fields exist on the dataclass
    import dataclasses
    fields = {f.name for f in dataclasses.fields(_mod.ExploreConfig)}
    for name in [
        "stall_detect_min_pwm", "stall_min_counts_per_step", "stall_threshold_steps",
        "dead_end_side_mm", "min_reverse_mm", "back_obstacle_mm", "front_clear_mm",
        "reverse_speed", "reverse_heading_gain", "assess_steps",
        "frontier_update_steps", "frontier_gain", "random_walk_steps",
    ]:
        assert name in fields, f"Missing config field: {name}"


StallDetector = _mod.StallDetector
detect_dead_end = _mod.detect_dead_end


def test_stall_detector_no_trigger_below_threshold() -> None:
    sd = StallDetector(min_pwm=0.10, min_counts=2, threshold_steps=5)
    for _ in range(4):
        result = sd.update(commanded_pwm=0.20, actual_delta=1)
    assert result is False


def test_stall_detector_triggers_at_threshold() -> None:
    sd = StallDetector(min_pwm=0.10, min_counts=2, threshold_steps=5)
    for _ in range(5):
        result = sd.update(commanded_pwm=0.20, actual_delta=1)
    assert result is True


def test_stall_detector_resets_on_movement() -> None:
    sd = StallDetector(min_pwm=0.10, min_counts=2, threshold_steps=5)
    for _ in range(4):
        sd.update(commanded_pwm=0.20, actual_delta=1)
    sd.update(commanded_pwm=0.20, actual_delta=10)  # movement resets counter
    for _ in range(4):
        result = sd.update(commanded_pwm=0.20, actual_delta=1)
    assert result is False  # only 4 consecutive again


def test_stall_detector_inactive_below_min_pwm() -> None:
    sd = StallDetector(min_pwm=0.10, min_counts=2, threshold_steps=5)
    for _ in range(10):
        result = sd.update(commanded_pwm=0.05, actual_delta=0)
    assert result is False


def test_detect_dead_end_all_close() -> None:
    readings = {"front": 50, "left45": 100, "right45": 120}
    assert detect_dead_end(readings, obstacle_mm=80, dead_end_side_mm=150) is True


def test_detect_dead_end_front_clear() -> None:
    readings = {"front": 200, "left45": 100, "right45": 120}
    assert detect_dead_end(readings, obstacle_mm=80, dead_end_side_mm=150) is False


def test_detect_dead_end_none_readings() -> None:
    readings = {"front": 50, "left45": None, "right45": 120}
    assert detect_dead_end(readings, obstacle_mm=80, dead_end_side_mm=150) is False


def test_detect_dead_end_one_side_clear() -> None:
    readings = {"front": 50, "left45": 200, "right45": 120}
    assert detect_dead_end(readings, obstacle_mm=80, dead_end_side_mm=150) is False


FrontierCache = _mod.FrontierCache
choose_turn_biased = _mod.choose_turn_biased
Pose = _mod.Pose


def test_frontier_cache_returns_none_on_empty_grid() -> None:
    grid = OccupancyGrid(cell_size_mm=50)
    cache = FrontierCache(update_steps=10)
    pose = Pose(x_mm=0.0, y_mm=0.0, theta_rad=0.0)
    result = cache.get(step=5, grid=grid, pose=pose)
    assert result is None


def test_frontier_cache_finds_frontier_heading() -> None:
    grid = OccupancyGrid(cell_size_mm=50)
    # Robot at origin, one free cell to the east, unknown to its east
    grid.free.add((0, 0))   # robot cell is free
    grid.free.add((1, 0))   # cell to east is free — its neighbor (2,0) is unknown
    pose = Pose(x_mm=25.0, y_mm=25.0, theta_rad=0.0)
    cache = FrontierCache(update_steps=10)
    heading = cache.get(step=0, grid=grid, pose=pose)
    assert heading is not None
    # Should point roughly eastward (angle near 0)
    assert abs(heading) < math.pi / 2


def test_frontier_cache_excludes_penalized() -> None:
    grid = OccupancyGrid(cell_size_mm=50)
    grid.free.add((0, 0))
    grid.occupied.add((1, 0))  # block direct frontier
    grid.penalized.add((0, 0))  # penalize the frontier cell itself
    pose = Pose(x_mm=0.0, y_mm=0.0, theta_rad=0.0)
    cache = FrontierCache(update_steps=10)
    result = cache.get(step=0, grid=grid, pose=pose)
    assert result is None


def test_frontier_cache_respects_update_interval() -> None:
    grid = OccupancyGrid(cell_size_mm=50)
    grid.free.add((0, 0))
    grid.free.add((1, 0))
    pose = Pose(x_mm=0.0, y_mm=0.0, theta_rad=0.0)
    cache = FrontierCache(update_steps=10)
    h1 = cache.get(step=0, grid=grid, pose=pose)
    # Add more free cells — but cache shouldn't update until step 10
    grid.free.add((5, 5))
    h2 = cache.get(step=5, grid=grid, pose=pose)
    assert h1 == h2  # cached value unchanged


def test_choose_turn_biased_prefers_open_side() -> None:
    readings = {"front": 300, "left45": 400, "right45": 100}
    direction = choose_turn_biased(readings, frontier_heading=None,
                                   current_theta=0.0, last_turn="rotate_left")
    assert direction == "rotate_left"  # left is more open


def test_choose_turn_biased_uses_frontier_when_sides_equal() -> None:
    readings = {"front": 300, "left45": 300, "right45": 300}
    # frontier is 90° to the left (pi/2), robot faces 0 → should turn left
    direction = choose_turn_biased(readings, frontier_heading=math.pi / 2,
                                   current_theta=0.0, last_turn="rotate_right")
    assert direction == "rotate_left"


def test_choose_turn_biased_fallback_to_last_turn() -> None:
    readings = {"front": 300, "left45": 300, "right45": 300}
    direction = choose_turn_biased(readings, frontier_heading=None,
                                   current_theta=0.0, last_turn="rotate_right")
    assert direction == "rotate_right"
