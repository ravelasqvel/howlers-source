"""
Omnidirectional Robot System Identification Framework
======================================================

A modular framework for three-stage parameter identification of omnidirectional
mobile robots with three omni wheels.

Modules:
    - kinematics: Kinematic modeling and validation
    - models: Robot and motor dynamics models
    - identification: Three-stage identification algorithms
    - utils: Utility functions (conversions, helpers)
    - examples: Example usage with synthetic data

Author: Professional implementation with validated kinematics
Version: 2.0.0 (Modular)
"""

__version__ = "2.0.0"
__author__ = "Robot Identification Framework"

# Import key classes for easy access
from src.kinematics.validator import KinematicValidator
from src.models.robot import OmnidirectionalRobot
from src.identification.three_stage import ThreeStageIdentification
from src.utils.conversions import pwm_to_voltage

__all__ = [
    'KinematicValidator',
    'OmnidirectionalRobot',
    'ThreeStageIdentification',
    'pwm_to_voltage'
]
