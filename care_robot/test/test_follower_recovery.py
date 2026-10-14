"""Regression tests for recovering a person lost during a turn."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

from care_robot.follower_node import HumanFollower
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import pytest
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import Image


@pytest.fixture
def follower(monkeypatch):
    """Provide a real node with controlled time and recorded velocity output."""
    rclpy.init(args=['--ros-args', '-p', 'detector_backend:=hog'])
    node = HumanFollower()
    node._lost_timer.cancel()
    state = SimpleNamespace(seconds=0.0)
    clock = SimpleNamespace(now=lambda: Time(seconds=state.seconds))
    monkeypatch.setattr(node, 'get_clock', lambda: clock)
    monkeypatch.setattr(node, '_cmd_vel_publisher', Mock())
    monkeypatch.setattr(node, '_navigate_client', Mock())
    node._has_seen_target = True
    node._last_detection_time = Time(seconds=0)
    node._last_rgb_time = Time(seconds=0)
    node._last_goal_time = Time(seconds=0)
    node._camera_intrinsics = (500.0, 500.0, 320.0, 240.0)
    try:
        yield node, state
    finally:
        node.destroy_node()
        rclpy.shutdown()


def accept_goal(node, epoch=0):
    """Simulate an accepted Nav2 goal with an independently completed result."""
    result = Future()
    # ClientGoalHandle is unhashable; the stand-in must share that property.
    handle = SimpleNamespace(
        accepted=True, get_result_async=Mock(return_value=result),
        cancel_goal_async=Mock())
    response = Future()
    response.set_result(handle)
    node._goal_response_callback(response, epoch)
    return handle, result


@pytest.mark.parametrize('pixel,expected', [(500, -0.1 / 0.65), (140, 0.1 / 0.65)])
def test_search_turns_toward_last_seen_side(follower, pixel, expected):
    """Right-edge loss requires rightward yaw and vice versa."""
    node, state = follower
    node._remember_search_direction(pixel, 640)
    node._remember_search_direction(321, 640)
    state.seconds = 3.0
    node._last_rgb_time = Time(seconds=3)
    node._check_target_timeout()
    command = node._cmd_vel_publisher.publish.call_args.args[0]
    assert command.angular.z == pytest.approx(expected)
    assert command.linear.x == 0.1
    assert node._target_lost


def test_transient_loss_does_not_interrupt_navigation(follower):
    """Keep the current goal during brief detection gaps."""
    node, state = follower
    handle, _ = accept_goal(node)
    state.seconds = 2.9
    node._check_target_timeout()
    assert not node._target_lost
    handle.cancel_goal_async.assert_not_called()
    node._cmd_vel_publisher.publish.assert_not_called()


def test_search_waits_for_navigation_to_finish(follower):
    """Cancellation request alone must not start competing velocity output."""
    node, state = follower
    handle, result = accept_goal(node)
    state.seconds = 3.0
    node._last_rgb_time = Time(seconds=3)
    node._check_target_timeout()
    handle.cancel_goal_async.assert_called_once()
    node._cmd_vel_publisher.publish.assert_not_called()
    node._dispatch_goal(PoseStamped())
    node._navigate_client.send_goal_async.assert_not_called()

    result.set_result(None)
    node._check_target_timeout()
    assert node._cmd_vel_publisher.publish.call_args.args[0].angular.z == (
        pytest.approx(0.1 / 0.65))


def test_late_goal_acceptance_is_canceled_after_reacquisition(follower):
    """A pending pre-loss request must not revive stale navigation."""
    node, state = follower
    node._goal_request_pending = True
    state.seconds = 3.0
    node._last_rgb_time = Time(seconds=3)
    node._check_target_timeout()
    node._cmd_vel_publisher.publish.assert_not_called()
    node._stop_search()
    handle, result = accept_goal(node, epoch=0)
    handle.cancel_goal_async.assert_called_once()
    assert handle in node._canceling_goals
    node._dispatch_goal(PoseStamped())
    node._navigate_client.send_goal_async.assert_not_called()
    result.set_result(None)
    node._dispatch_goal(PoseStamped())
    node._navigate_client.send_goal_async.assert_called_once()


def test_old_result_does_not_clear_new_goal(follower):
    """Preempted goals must not release ownership of the current goal."""
    node, _ = follower
    _, old_result = accept_goal(node)
    new_handle, _ = accept_goal(node)
    old_result.set_result(None)
    assert node._active_goal_handle is new_handle


def test_camera_dropout_stops_instead_of_searching_blindly(follower):
    """The watchdog must cancel navigation even when frames stop arriving."""
    node, state = follower
    handle, result = accept_goal(node)
    state.seconds = 3.0
    node._check_target_timeout()
    handle.cancel_goal_async.assert_called_once()
    result.set_result(None)
    node._check_target_timeout()
    command = node._cmd_vel_publisher.publish.call_args.args[0]
    assert command.angular.z == 0.0
    assert command.linear.x == 0.0


def test_search_can_still_be_disabled(follower):
    """The explicit hold-position policy remains supported."""
    node, state = follower
    node._search_when_lost = False
    state.seconds = 3.0
    node._last_rgb_time = Time(seconds=3)
    node._check_target_timeout()
    assert node._cmd_vel_publisher.publish.call_args.args[0].angular.z == 0.0


def test_reacquisition_stops_search_and_dispatches_immediately(
        follower, monkeypatch):
    """A valid RGB-D observation resumes following without a period delay."""
    node, state = follower
    node._target_lost = True
    state.seconds = 4.1
    node._last_goal_time = Time(seconds=4.0)
    node._depth_image = np.full((480, 640), 3.0)
    point = PointStamped()
    point.point.x = 3.0
    monkeypatch.setattr(node, '_detect_person_center', lambda frame: (320, 240))
    monkeypatch.setattr(node, '_point_in_global_frame', lambda *args: point)
    monkeypatch.setattr(node, '_robot_position_in_global_frame',
                        lambda: (0.0, 0.0))
    monkeypatch.setattr(node, '_dispatch_goal', Mock())
    image = Image()
    image.encoding = 'rgb8'
    image.width, image.height, image.step = 640, 480, 1920
    image.data = bytes(640 * 480 * 3)
    node._rgb_callback(image)
    assert not node._target_lost
    assert node._cmd_vel_publisher.publish.call_args.args[0].angular.z == 0.0
    node._dispatch_goal.assert_called_once()
    goal = node._dispatch_goal.call_args.args[0]
    assert goal.pose.position.x == pytest.approx(0.8)


def test_search_uses_nav2_velocity_pipeline(follower):
    """Recovery commands must pass through smoothing and collision checks."""
    node, _ = follower
    assert node._cmd_vel_topic == 'cmd_vel_nav'


@pytest.mark.parametrize('seen,seconds', [(False, 3.0), (True, 15.0)])
def test_search_requires_previous_target_and_expires(follower, seen, seconds):
    """Never drive on startup or indefinitely after losing the target."""
    node, state = follower
    node._has_seen_target = seen
    state.seconds = seconds
    node._last_rgb_time = Time(seconds=seconds)
    node._check_target_timeout()
    command = node._cmd_vel_publisher.publish.call_args.args[0]
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0
