"""Unit tests for framework-independent following geometry."""

import math
from types import SimpleNamespace

from care_robot.geometry import compute_follow_goal
from care_robot.geometry import image_message_to_array
from care_robot.geometry import project_pixel_to_optical_frame
from care_robot.geometry import sample_depth
import numpy as np
import pytest


def test_sample_depth_uses_median_of_valid_values():
    """Invalid samples and isolated outliers should not skew the depth."""
    image = np.array([
        [np.nan, 2.0, 2.0],
        [0.0, 100.0, 2.0],
        [2.0, 2.0, np.inf],
    ])
    assert sample_depth(image, 1, 1, window_radius=1) == 2.0


def test_projection_uses_optical_frame_axes():
    """The principal pixel should lie on the optical z axis."""
    point = project_pixel_to_optical_frame(
        320, 240, 2.5, 500.0, 500.0, 320.0, 240.0)
    assert point == (0.0, 0.0, 2.5)


def test_follow_goal_keeps_requested_distance():
    """The generated goal should remain behind the tracked person."""
    goal = compute_follow_goal((0.0, 0.0), (3.0, 4.0), 1.0)
    assert goal is not None
    assert math.isclose(goal[0], 2.4)
    assert math.isclose(goal[1], 3.2)
    assert math.isclose(goal[2], math.atan2(4.0, 3.0))


def test_follow_goal_rejects_overlapping_positions():
    """A direction cannot be computed for coincident positions."""
    assert compute_follow_goal((1.0, 1.0), (1.0, 1.0), 1.0) is None


def test_image_decoder_handles_rgb_rows_with_padding():
    """RGB decoding should discard transport padding at each row end."""
    message = SimpleNamespace(
        encoding='rgb8',
        is_bigendian=0,
        width=2,
        height=1,
        step=8,
        data=bytes([1, 2, 3, 4, 5, 6, 99, 99]),
    )

    image = image_message_to_array(message)

    assert image.shape == (1, 2, 3)
    assert image.tolist() == [[[1, 2, 3], [4, 5, 6]]]


def test_image_decoder_handles_float_depth():
    """Gazebo's 32FC1 depth encoding should retain metric values."""
    depth = np.array([[1.25, 2.5]], dtype='<f4')
    message = SimpleNamespace(
        encoding='32FC1',
        is_bigendian=0,
        width=2,
        height=1,
        step=8,
        data=depth.tobytes(),
    )

    image = image_message_to_array(message)

    assert image.dtype == np.dtype('<f4')
    assert image == pytest.approx(depth)
