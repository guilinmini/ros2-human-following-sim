"""Tests for scripted actor trajectory synchronization."""

import math
from pathlib import Path

from care_robot.actor_box_node import interpolate_trajectory
from care_robot.actor_box_node import load_trajectory
from care_robot.actor_box_node import proxy_center_z
from care_robot.actor_box_node import Waypoint
import pytest


def test_loads_default_world_actor_trajectory():
    """The default simulation world exposes a usable actor path."""
    world_file = (
        Path(__file__).resolve().parents[2]
        / 'care_robot_gazebo'
        / 'worlds'
        / 'maze4.world'
    )

    waypoints, loop, delay = load_trajectory(str(world_file), 'actor')

    assert len(waypoints) == 16
    assert loop is True
    assert delay == pytest.approx(0.0)
    assert waypoints[0].pose[:3] == pytest.approx((-4.0, 2.0, 1.0))
    assert waypoints[0].pose[0] - (-5.5) >= 1.0


def test_interpolates_position_and_loops():
    """Position interpolation follows waypoint time and loop settings."""
    waypoints = [
        Waypoint(0.0, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        Waypoint(10.0, (10.0, 4.0, 2.0, 0.0, 0.0, 1.0)),
    ]

    midpoint = interpolate_trajectory(waypoints, False, 5.0)
    looped = interpolate_trajectory(waypoints, True, 12.0)

    assert midpoint[:3] == pytest.approx((5.0, 2.0, 1.0))
    assert midpoint[5] == pytest.approx(0.5)
    assert looped[:3] == pytest.approx((2.0, 0.8, 0.4))


def test_interpolates_rotation_across_wrap_boundary():
    """Yaw interpolation uses the shortest path around pi."""
    waypoints = [
        Waypoint(0.0, (0.0, 0.0, 0.0, 0.0, 0.0, math.radians(170))),
        Waypoint(1.0, (0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-170))),
    ]

    pose = interpolate_trajectory(waypoints, False, 0.5)

    assert abs(pose[5]) == pytest.approx(math.pi)


def test_maps_actor_origin_to_ground_relative_proxy_height():
    """The one-metre actor origin maps to an 0.85-metre proxy centre."""
    result = proxy_center_z(1.0, 1.0, 0.85)

    assert result == pytest.approx(0.85)
