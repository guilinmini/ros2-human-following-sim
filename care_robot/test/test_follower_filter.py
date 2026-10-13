"""Check that RGB callbacks use filtered positions and reset on target loss."""

from types import SimpleNamespace
from unittest.mock import Mock

from care_robot.follower_node import HumanFollower
from geometry_msgs.msg import PointStamped
import numpy as np
import pytest
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import Image


@pytest.fixture
def follower(monkeypatch):
    """Use a ROS node with deterministic detections, transforms, and time."""
    rclpy.init(args=[
        '--ros-args', '-p', 'detector_backend:=hog',
        '-p', 'send_nav2_goal:=false',
    ])
    node = HumanFollower()
    node._camera_intrinsics = (500.0, 500.0, 1.0, 1.0)
    node._depth_image = np.full((3, 3), 3.0)
    node._last_goal_time = Time(seconds=0)
    node._last_detection_time = Time(seconds=0)
    state = SimpleNamespace(seconds=1.0, point=PointStamped())
    state.point.point.x = 3.0
    clock = SimpleNamespace(now=lambda: Time(seconds=state.seconds))
    monkeypatch.setattr(node, 'get_clock', lambda: clock)
    monkeypatch.setattr(node, '_detect_person_center', lambda frame: (1, 1))
    monkeypatch.setattr(node, '_point_in_global_frame',
                        lambda *args: state.point)
    monkeypatch.setattr(node, '_robot_position_in_global_frame',
                        lambda: (0.0, 0.0))
    monkeypatch.setattr(node, '_dispatch_goal', Mock())
    try:
        yield node, state
    finally:
        node.destroy_node()
        rclpy.shutdown()


def image_at(seconds):
    """Create a small valid RGB frame stamped in simulation time."""
    message = Image()
    message.header.stamp = Time(seconds=seconds).to_msg()
    message.encoding = 'rgb8'
    message.width = 3
    message.height = 3
    message.step = 9
    message.data = bytes(27)
    return message


def test_goal_uses_filtered_position(follower):
    """A noisy detection must be smoothed before applying follow distance."""
    node, state = follower
    node._rgb_callback(image_at(1.0))
    first_goal = node._dispatch_goal.call_args.args[0]
    assert first_goal.pose.position.x == pytest.approx(2.0)

    state.seconds = 2.0
    state.point.point.x = 4.0
    node._rgb_callback(image_at(1.1))
    next_goal = node._dispatch_goal.call_args.args[0]
    assert 2.0 < next_goal.pose.position.x < 3.0
    assert next_goal.pose.position.y == pytest.approx(0.0)


def test_loss_resets_filter_before_reacquisition(follower):
    """Reacquisition must not blend a new position with the old target."""
    node, state = follower
    node._rgb_callback(image_at(1.0))
    state.seconds = 4.0
    node._handle_missing_detection()
    assert node._target_lost
    assert node._dispatch_goal.call_count == 1

    state.seconds = 4.1
    state.point.point.x = 10.0
    node._rgb_callback(image_at(1.1))
    goal = node._dispatch_goal.call_args.args[0]
    assert goal.pose.position.x == pytest.approx(9.0)
    assert not node._target_lost
