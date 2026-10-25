"""
Angle utilities for state estimation.

Functions for normalizing angles, wrapping to [-pi, pi], and computing
angle differences correctly across the discontinuity.
"""

import numpy as np


def normalize_angle(angle):
    """
    Normalize angle to [-pi, pi].
    
    Parameters
    ----------
    angle : float or np.ndarray
        Angle(s) in radians
        
    Returns
    -------
    float or np.ndarray
        Normalized angle(s) in [-pi, pi]
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def wrap_to_pi(angle):
    """
    Wrap angle to [-pi, pi] range.
    
    Alternative implementation using modulo operation.
    
    Parameters
    ----------
    angle : float or np.ndarray
        Angle(s) in radians
        
    Returns
    -------
    float or np.ndarray
        Wrapped angle(s) in [-pi, pi]
    """
    angle = np.asarray(angle)
    wrapped = (angle + np.pi) % (2 * np.pi) - np.pi
    return wrapped


def angle_diff(angle1, angle2):
    """
    Compute the smallest difference between two angles.
    
    Handles the discontinuity at ±pi correctly.
    
    Parameters
    ----------
    angle1 : float or np.ndarray
        First angle(s) in radians
    angle2 : float or np.ndarray
        Second angle(s) in radians
        
    Returns
    -------
    float or np.ndarray
        Smallest angular difference in [-pi, pi]
        
    Examples
    --------
    >>> angle_diff(np.pi, -np.pi)
    0.0
    >>> angle_diff(0.1, -0.1)
    0.2
    """
    diff = angle1 - angle2
    return normalize_angle(diff)


def circular_mean(angles, weights=None):
    """
    Compute the circular mean of angles.
    
    Uses the atan2(sum(sin), sum(cos)) method for correct
    averaging across the ±pi discontinuity.
    
    Parameters
    ----------
    angles : np.ndarray
        Array of angles in radians
    weights : np.ndarray, optional
        Weights for each angle. If None, uniform weights are used.
        
    Returns
    -------
    float
        Circular mean angle in [-pi, pi]
    """
    angles = np.asarray(angles)
    if weights is None:
        weights = np.ones(len(angles))
    else:
        weights = np.asarray(weights)
        
    sin_sum = np.dot(np.sin(angles), weights)
    cos_sum = np.dot(np.cos(angles), weights)
    
    return np.arctan2(sin_sum, cos_sum)
