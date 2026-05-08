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
