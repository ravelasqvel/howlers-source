"""
Common utilities for state estimation.

Includes angle handling, discretization methods, and residual functions.
"""

from .angles import normalize_angle, wrap_to_pi, angle_diff
from .discretization import euler_discretization, rk4_discretization
from .residuals import residual_state_first, residual_measurement_first, make_residual_fn

__all__ = [
    'normalize_angle',
    'wrap_to_pi',
    'angle_diff',
    'euler_discretization',
    'rk4_discretization',
    'residual_state_first',
    'residual_measurement_first',
    'make_residual_fn',
]
