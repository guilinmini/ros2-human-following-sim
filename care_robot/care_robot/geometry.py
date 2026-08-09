"""Pure geometry helpers used by the RGB-D following node."""

import math
from typing import Optional, Tuple

import numpy as np


_IMAGE_ENCODINGS = {
    'mono8': ('u1', 1),
    '8uc1': ('u1', 1),
    'rgb8': ('u1', 3),
    'bgr8': ('u1', 3),
    'rgba8': ('u1', 4),
    'bgra8': ('u1', 4),
    '8uc3': ('u1', 3),
    '8uc4': ('u1', 4),
    'mono16': ('u2', 1),
    '16uc1': ('u2', 1),
    '32fc1': ('f4', 1),
}


def image_message_to_array(message) -> np.ndarray:
    """Decode a common sensor_msgs/Image encoding without cv_bridge."""
    encoding = message.encoding.lower()
    if encoding not in _IMAGE_ENCODINGS:
        raise ValueError(f'unsupported image encoding: {message.encoding!r}')
    type_code, channels = _IMAGE_ENCODINGS[encoding]
    byte_order = '>' if message.is_bigendian else '<'
    dtype = np.dtype(type_code if type_code == 'u1' else byte_order + type_code)
    if message.step % dtype.itemsize != 0:
        raise ValueError('image row step is not aligned to its data type')

    row_elements = message.step // dtype.itemsize
    image_elements = message.width * channels
    if row_elements < image_elements:
        raise ValueError('image row step is shorter than its encoded width')

    flat = np.frombuffer(message.data, dtype=dtype)
    required_elements = row_elements * message.height
    if flat.size < required_elements:
        raise ValueError('image data is shorter than height times row step')
    rows = flat[:required_elements].reshape(message.height, row_elements)
    pixels = rows[:, :image_elements]
    if channels == 1:
        return pixels.reshape(message.height, message.width).copy()
    return pixels.reshape(message.height, message.width, channels).copy()


def sample_depth(
        image: np.ndarray, pixel_x: int, pixel_y: int,
        window_radius: int = 2) -> Optional[float]:
    """Return the median finite positive depth around an image pixel."""
    if image.ndim != 2 or window_radius < 0:
        return None
    height, width = image.shape
    if not 0 <= pixel_x < width or not 0 <= pixel_y < height:
        return None

    x_start = max(0, pixel_x - window_radius)
    x_stop = min(width, pixel_x + window_radius + 1)
    y_start = max(0, pixel_y - window_radius)
    y_stop = min(height, pixel_y + window_radius + 1)
    values = image[y_start:y_stop, x_start:x_stop]
    valid_values = values[np.isfinite(values) & (values > 0.0)]
    if valid_values.size == 0:
        return None
    return float(np.median(valid_values))


def project_pixel_to_optical_frame(
        pixel_x: int, pixel_y: int, depth: float,
        focal_x: float, focal_y: float,
        principal_x: float, principal_y: float) -> Tuple[float, float, float]:
    """Project a depth pixel into REP-103 optical-frame coordinates."""
    x = (pixel_x - principal_x) * depth / focal_x
    y = (pixel_y - principal_y) * depth / focal_y
    return x, y, depth


def compute_follow_goal(
        robot_position: Tuple[float, float],
        human_position: Tuple[float, float],
        follow_distance: float) -> Optional[Tuple[float, float, float]]:
    """Return a goal behind the human and a yaw facing the human."""
    delta_x = human_position[0] - robot_position[0]
    delta_y = human_position[1] - robot_position[1]
    distance = math.hypot(delta_x, delta_y)
    if distance < 1e-6:
        return None

    unit_x = delta_x / distance
    unit_y = delta_y / distance
    goal_x = human_position[0] - unit_x * follow_distance
    goal_y = human_position[1] - unit_y * follow_distance
    return goal_x, goal_y, math.atan2(delta_y, delta_x)
