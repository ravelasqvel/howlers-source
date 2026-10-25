"""
Residual functions for state estimation.

These functions compute residuals (differences) between states or measurements,
handling special cases like angular states that wrap around at ±pi.
"""

import numpy as np
from .angles import normalize_angle


def residual_state_first(a, b, angle_indices=None):
    """
    Compute residual y = a - b for state vectors.
    
    Handles angular states correctly by normalizing angle differences.
    
    Parameters
    ----------
    a : np.ndarray
        First state vector
    b : np.ndarray
        Second state vector
    angle_indices : list of int, optional
        Indices of angular states (in radians) that need normalization
        
    Returns
    -------
    np.ndarray
        Residual vector y = a - b with normalized angles
        
    Examples
    --------
    >>> x1 = np.array([1.0, 2.0, 3.14])
    >>> x2 = np.array([0.5, 1.8, -3.14])
    >>> residual_state_first(x1, x2, angle_indices=[2])
    array([0.5, 0.2, 0.003...])
    """
    y = a - b
    
    if angle_indices is not None:
        for idx in angle_indices:
            if idx < len(y):
                y[idx] = normalize_angle(y[idx])
    
    return y


def residual_measurement_first(a, b, angle_indices=None):
    """
    Compute residual y = a - b for measurement vectors.
    
    Similar to residual_state_first but specifically for measurements.
    
    Parameters
    ----------
    a : np.ndarray
        First measurement vector
    b : np.ndarray
        Second measurement vector
    angle_indices : list of int, optional
        Indices of angular measurements (in radians) that need normalization
        
    Returns
    -------
    np.ndarray
        Residual vector y = a - b with normalized angles
    """
    y = a - b
    
    if angle_indices is not None:
        for idx in angle_indices:
            if idx < len(y):
                y[idx] = normalize_angle(y[idx])
    
    return y


def make_residual_fn(angle_indices=None):
    """
    Factory function to create a residual function with fixed angle indices.
    
    Useful for creating residual functions for UKF that need specific
    angle handling.
    
    Parameters
    ----------
    angle_indices : list of int, optional
        Indices of angular states that need normalization
        
    Returns
    -------
    callable
        Residual function with signature (a, b) -> residual
        
    Examples
    --------
    >>> residual_fn = make_residual_fn(angle_indices=[2])
    >>> x1 = np.array([1.0, 2.0, 3.14])
    >>> x2 = np.array([0.5, 1.8, -3.14])
    >>> residual_fn(x1, x2)
    array([0.5, 0.2, 0.003...])
    """
    def residual(a, b):
        return residual_state_first(a, b, angle_indices=angle_indices)
    
    return residual
