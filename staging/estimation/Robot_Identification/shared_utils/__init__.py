"""
Shared utilities for Robot Identification and State Estimation projects.

This package contains common functionality used across multiple projects:
- Data loading and processing
- Signal processing (Kalman filters)
- Kinematics calculations
- Utility functions
"""

__version__ = "1.0.0"

from .data_loader import download_files, load_experiment_file
from .signal_processing import Kalman1D
from .kinematics import calc_invkinematics, calc_forwardkinematics
from .conversions import wrap_angle

__all__ = [
    'download_files',
    'load_experiment_file',
    'Kalman1D',
    'calc_invkinematics',
    'calc_forwardkinematics',
    'wrap_angle',
]
