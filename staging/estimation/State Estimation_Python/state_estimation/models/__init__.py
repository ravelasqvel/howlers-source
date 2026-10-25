"""
Robot models for state estimation.

This module provides example robot models (dynamics and measurement functions)
that can be used with the filters.
"""

from .omnidirectional import OmnidirectionalRobot

__all__ = [
    'OmnidirectionalRobot',
]
