"""
Utility functions for data type conversions and transformations.
"""

import numpy as np


def wrap_angle(angle: float) -> float:
    """
    Wrap angle to the interval [-π, π).

    Args:
        angle: Angle in radians

    Returns:
        Wrapped angle in range [-π, π)
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-π, π]. Alias for wrap_angle.

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in range [-π, π]
    """
    return wrap_angle(angle)
