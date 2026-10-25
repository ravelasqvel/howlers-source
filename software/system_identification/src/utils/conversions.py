"""
Utility functions for unit conversions and data preprocessing.
"""

import numpy as np


def pwm_to_voltage(duty_percent, V_battery):
    """
    Convert PWM duty cycle to effective motor voltage for identification.
    
    The motor's electrical time constant (τ_e = L_a/R_a ≈ 1-10 ms) filters 
    the PWM signal (typically 1-20 kHz) into an equivalent DC voltage.
    
    Parameters
    ----------
    duty_percent : float or array
        PWM duty cycle [-100% to 100%]
    V_battery : float
        Battery voltage measured during experiment (V)
    
    Returns
    -------
    V_eff : float or array
        Effective voltage (V)
    
    Example
    -------
    >>> pwm_to_voltage(50, 12.0)
    6.0
    >>> pwm_to_voltage([50, 75, 100], 12.0)
    array([6.0, 9.0, 12.0])
    
    Reference
    ---------
    Mohan, N., et al. (2003). Power Electronics: Converters, Applications, 
    and Design. Wiley. Chapter 7: DC Motor Drives.
    """
    duty_fraction = np.clip(np.asarray(duty_percent) / 100.0, -1.0, 1.0)
    return duty_fraction * V_battery


def voltage_to_pwm(voltage, V_battery):
    """
    Convert voltage to PWM duty cycle (inverse of pwm_to_voltage).
    
    Parameters
    ----------
    voltage : float or array
        Desired voltage (V)
    V_battery : float
        Battery voltage (V)
    
    Returns
    -------
    duty_percent : float or array
        PWM duty cycle [-100% to 100%]
    """
    duty_fraction = np.clip(np.asarray(voltage) / V_battery, -1.0, 1.0)
    return duty_fraction * 100.0


def rad_to_deg(angle_rad):
    """Convert radians to degrees."""
    return np.rad2deg(angle_rad)


def deg_to_rad(angle_deg):
    """Convert degrees to radians."""
    return np.deg2rad(angle_deg)
