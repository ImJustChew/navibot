from navibot.navigation.planner import NavigationPlanner
from navibot.sensors.tof import TofPosition, TofReading


def test_planner_stops_when_front_obstacle_is_close() -> None:
    planner = NavigationPlanner(stop_distance_mm=200)

    command = planner.plan((TofReading(position=TofPosition.FRONT, distance_mm=120),))

    assert command.linear == 0.0
    assert command.angular == 0.0


def test_planner_cruises_when_front_path_is_clear() -> None:
    planner = NavigationPlanner(stop_distance_mm=200, cruise_speed=0.4)

    command = planner.plan((TofReading(position=TofPosition.FRONT, distance_mm=600),))

    assert command.linear == 0.4
    assert command.angular == 0.0

