"""
State Estimation Library

A professional, generalized state estimation library for robotics applications.
Implements Extended Kalman Filter (EKF), Unscented Kalman Filter (UKF), 
and Particle Filter (PF) with a FilterPy-inspired API.

Author: State Estimation Team
License: MIT
"""

__version__ = "1.0.0"

from .filters.extended import ExtendedKalmanFilter
from .filters.unscented import UnscentedKalmanFilter, MerweScaledSigmaPoints
from .filters.particle import ParticleFilter

__all__ = [
    'ExtendedKalmanFilter',
    'UnscentedKalmanFilter',
    'MerweScaledSigmaPoints',
    'ParticleFilter',
]
