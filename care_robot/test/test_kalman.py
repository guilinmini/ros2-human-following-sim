"""Behavioral tests for filtering noisy human positions."""

from care_robot.kalman import HumanPositionFilter
import numpy as np
import pytest


def test_stationary_measurements_have_less_position_noise():
    """Filtering should reduce steady-state error for a stationary person."""
    rng = np.random.default_rng(42)
    truth = np.array([3.0, -2.0])
    measurements = truth + rng.normal(0.0, 0.15, (300, 2))
    tracker = HumanPositionFilter()
    filtered = np.array([
        tracker.update(point, index / 30.0)
        for index, point in enumerate(measurements)
    ])
    raw_error = np.mean((measurements[30:] - truth) ** 2)
    filtered_error = np.mean((filtered[30:] - truth) ** 2)
    assert filtered_error < raw_error * 0.5


def test_constant_velocity_with_variable_frame_intervals():
    """Motion estimates should track walking without accumulating lag."""
    rng = np.random.default_rng(7)
    stamps = np.cumsum(rng.uniform(0.025, 0.1, 250))
    truth = np.array([1.0, -2.0]) + stamps[:, None] * [0.6, -0.3]
    measurements = truth + rng.normal(0.0, 0.15, truth.shape)
    tracker = HumanPositionFilter()
    filtered = np.array([
        tracker.update(point, stamp)
        for point, stamp in zip(measurements, stamps)
    ])
    assert np.mean((filtered[30:] - truth[30:]) ** 2) < np.mean(
        (measurements[30:] - truth[30:]) ** 2)
    assert np.linalg.norm(np.mean(filtered[30:] - truth[30:], axis=0)) < 0.05


@pytest.mark.parametrize('stamp', [0.5, 4.0, 10.0])
def test_time_reversal_or_long_gap_reinitializes(stamp):
    """Old target state must not bias observations after resets or long gaps."""
    tracker = HumanPositionFilter()
    tracker.update((1.0, 2.0), 0.9)
    tracker.update((1.1, 2.1), 1.0)
    assert tracker.update((20.0, -10.0), stamp) == (20.0, -10.0)


def test_explicit_reset_forgets_lost_target():
    """A reacquired target starts at its measured location."""
    tracker = HumanPositionFilter()
    assert tracker.update((1.0, 2.0), 0.0) == (1.0, 2.0)
    tracker.update((1.1, 2.1), 0.1)
    tracker.reset()
    assert tracker.update((-5.0, 8.0), 0.2) == (-5.0, 8.0)


def test_duplicate_timestamp_does_not_update_estimate():
    """Repeated frames must not create false confidence or velocity."""
    tracker = HumanPositionFilter()
    tracker.update((1.0, 2.0), 0.0)
    position = tracker.update((1.1, 2.1), 0.1)
    assert tracker.update((9.0, 9.0), 0.1) == position


@pytest.mark.parametrize('name', [
    'measurement_std', 'acceleration_std', 'reset_timeout'])
@pytest.mark.parametrize('value', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_configuration_is_rejected(name, value):
    """Invalid noise or timing settings must fail before filtering begins."""
    with pytest.raises(ValueError):
        HumanPositionFilter(**{name: value})


def test_invalid_measurement_does_not_poison_filter():
    """Bad observations should leave the last valid estimate intact."""
    tracker = HumanPositionFilter()
    tracker.update((1.0, 2.0), 0.0)
    with pytest.raises(ValueError):
        tracker.update((float('nan'), 2.0), 0.1)
    with pytest.raises(ValueError):
        tracker.update((1.0, 2.0), float('inf'))
    assert tracker.update((1.0, 2.0), 0.2) == (1.0, 2.0)
