"""Filter planar human positions with a constant-velocity Kalman model."""

import math

import numpy as np


class HumanPositionFilter:
    """Estimate map-frame position and velocity from noisy XY measurements."""

    def __init__(self, measurement_std=0.15, acceleration_std=1.0,
                 reset_timeout=3.0):
        """Configure position noise (m), acceleration noise (m/s²), and gap (s)."""
        for name, value in (
                ('measurement_std', measurement_std),
                ('acceleration_std', acceleration_std),
                ('reset_timeout', reset_timeout)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        self._measurement_covariance = np.eye(2) * measurement_std ** 2
        self._acceleration_variance = acceleration_std ** 2
        self._reset_timeout = reset_timeout
        self.reset()

    def reset(self):
        """Forget the previous target and initialize on the next observation."""
        self._state = None
        self._covariance = None
        self._stamp = None

    def update(self, position, stamp):
        """Return filtered XY using the observation timestamp in seconds."""
        measurement = np.asarray(position, dtype=float)
        if (measurement.shape != (2,)
                or not np.all(np.isfinite(measurement))
                or not math.isfinite(stamp)):
            raise ValueError('position and timestamp must be finite')

        dt = None if self._stamp is None else stamp - self._stamp
        if dt is None or dt < 0.0 or dt >= self._reset_timeout:
            self._state = np.array([*measurement, 0.0, 0.0])
            self._covariance = np.eye(4)
            self._covariance[:2, :2] = self._measurement_covariance
        elif dt > 0.0:
            transition = np.eye(4)
            transition[:2, 2:] = np.eye(2) * dt
            acceleration = np.vstack((
                np.eye(2) * (0.5 * dt ** 2), np.eye(2) * dt))
            process_noise = (
                acceleration @ acceleration.T * self._acceleration_variance)
            predicted = transition @ self._state
            covariance = (
                transition @ self._covariance @ transition.T + process_noise)
            innovation_covariance = (
                covariance[:2, :2] + self._measurement_covariance)
            gain = np.linalg.solve(
                innovation_covariance, covariance[:, :2].T).T
            self._state = predicted + gain @ (measurement - predicted[:2])
            # Joseph form preserves covariance symmetry and positivity.
            residual = np.eye(4)
            residual[:, :2] -= gain
            self._covariance = (
                residual @ covariance @ residual.T
                + gain @ self._measurement_covariance @ gain.T)
        # A duplicate timestamp must not count the same observation twice.
        self._stamp = stamp
        return float(self._state[0]), float(self._state[1])
