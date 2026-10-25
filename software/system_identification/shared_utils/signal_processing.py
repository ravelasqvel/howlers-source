"""
Signal processing utilities including Kalman filtering.

This module provides 1D Kalman filter implementation matching
the ESP32S3 C code used in the robot.
"""

import numpy as np


class Kalman1D:
    """
    1D Kalman filter implementation matching C code behavior.

    This filter provides simple noise reduction for scalar measurements
    using a basic Kalman filter formulation.

    Attributes:
        x: Estimated state value
        P: Estimation error covariance
        Q: Process noise covariance
        R: Measurement noise covariance
    """

    def __init__(self, Q: float, R: float):
        """
        Initialize 1D Kalman filter.

        Args:
            Q: Process noise covariance (how much we trust the model)
            R: Measurement noise covariance (how much we trust measurements)
        """
        self.x = 0.0
        self.P = 1.0
        self.Q = float(Q)
        self.R = float(R)

    def update(self, meas: float) -> float:
        """
        Update filter with new measurement.

        Args:
            meas: New measurement value

        Returns:
            Updated state estimate
        """
        # Prediction step
        self.P += self.Q

        # Measurement update
        K = self.P / (self.P + self.R)
        self.x += K * (meas - self.x)
        self.P *= (1 - K)

        return self.x

    def reset(self):
        """Reset filter to initial conditions."""
        self.x = 0.0
        self.P = 1.0
