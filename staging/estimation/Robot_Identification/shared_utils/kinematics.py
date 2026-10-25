"""
Kinematics calculations for omnidirectional robot.

This module provides forward and inverse kinematics functions for a 
3-wheeled omnidirectional robot platform.
"""

import numpy as np


# Robot physical parameters
WHEEL_RADIUS = 0.03  # meters
ROBOT_RADIUS = 0.16  # meters
DELTA = 0.523598     # radians (30 degrees)
TOLERANCE = 1e-5


def calc_invkinematics(vbx: float, vby: float, wb: float):
    """
    Compute wheel angular speed setpoints from desired body velocities.

    Implements inverse kinematics for omnidirectional robot.

    Args:
        vbx: Desired velocity in body x-direction (m/s)
        vby: Desired velocity in body y-direction (m/s)
        wb: Desired angular velocity around z-axis (rad/s)

    Returns:
        Tuple of (w1, w2, w3): Wheel angular speeds in rad/s
    """
    vbx = np.asarray(vbx)
    vby = np.asarray(vby)
    wb = np.asarray(wb)

    w1 = (-vbx * np.sin(DELTA) - vby * np.cos(DELTA) + ROBOT_RADIUS * wb) / WHEEL_RADIUS
    w2 = (vbx + ROBOT_RADIUS * wb) / WHEEL_RADIUS
    w3 = (-vbx * np.sin(DELTA) + vby * np.cos(DELTA) + ROBOT_RADIUS * wb) / WHEEL_RADIUS

    return w1, w2, w3


def calc_forwardkinematics(w1: float, w2: float, w3: float):
    """
    Compute body velocities from wheel angular speeds.

    Implements forward kinematics for omnidirectional robot.

    Args:
        w1: Wheel 1 angular speed (rad/s)
        w2: Wheel 2 angular speed (rad/s)
        w3: Wheel 3 angular speed (rad/s)

    Returns:
        Tuple of (vbx, vby, wb): Body velocities (m/s, m/s, rad/s)
    """
    w1 = np.asarray(w1)
    w2 = np.asarray(w2)
    w3 = np.asarray(w3)

    s = np.sin(DELTA)
    c = np.cos(DELTA)
    denom_xy = 2.0 * (1.0 + s)

    # Linear velocity in body x-direction
    vbx = WHEEL_RADIUS * (-w1 + 2.0 * w2 - w3) / denom_xy

    # Linear velocity in body y-direction
    vby = WHEEL_RADIUS * (-w1 + w3) / (2.0 * c)

    # Angular velocity about body center (positive CCW)
    wb = WHEEL_RADIUS * (w1 + 2.0 * s * w2 + w3) / (2.0 * ROBOT_RADIUS * (1.0 + s))

    return vbx, vby, wb
