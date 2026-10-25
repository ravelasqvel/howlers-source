"""
Discretization methods for continuous-time dynamics.

Provides Euler and Runge-Kutta 4th order (RK4) integration methods
for discretizing continuous-time system dynamics.
"""

import numpy as np


def euler_discretization(f, x, u, dt):
    """
    Euler method for discretizing continuous dynamics.
    
    Simple first-order integration: x_{k+1} = x_k + dt * f(x_k, u_k)
    
    Parameters
    ----------
    f : callable
        Continuous dynamics function f(x, u) returning dx/dt
    x : np.ndarray
        Current state vector
    u : np.ndarray
        Control input vector
    dt : float
        Time step
        
    Returns
    -------
    np.ndarray
        Next state x_{k+1}
    """
    dx = f(x, u)
    return x + dt * dx


def rk4_discretization(f, x, u, dt):
    """
    Runge-Kutta 4th order method for discretizing continuous dynamics.
    
    More accurate fourth-order integration method.
    
    Parameters
    ----------
    f : callable
        Continuous dynamics function f(x, u) returning dx/dt
    x : np.ndarray
        Current state vector
    u : np.ndarray
        Control input vector
    dt : float
        Time step
        
    Returns
    -------
    np.ndarray
        Next state x_{k+1}
        
    Notes
    -----
    The RK4 method evaluates the dynamics at four points:
        k1 = f(x, u)
        k2 = f(x + dt/2 * k1, u)
        k3 = f(x + dt/2 * k2, u)
        k4 = f(x + dt * k3, u)
        x_{k+1} = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    """
    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    
    return x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


def discrete_white_noise(dim, dt, var=1.0, order=1):
    """
    Generate discrete white noise covariance matrix Q.
    
    Useful for creating process noise covariance matrices.
    
    Parameters
    ----------
    dim : int
        Dimension of the state space
    dt : float
        Time step
    var : float, optional
        Variance of the white noise
    order : int, optional
        Order of integration (1 for velocity, 2 for acceleration)
        
    Returns
    -------
    np.ndarray
        Process noise covariance matrix Q
        
    Examples
    --------
    >>> Q = discrete_white_noise(2, dt=0.1, var=0.1, order=1)
    >>> Q.shape
    (2, 2)
    """
    if order == 1:
        Q = np.array([[dt, 0],
                      [0, dt]]) * var
    elif order == 2:
        Q = np.array([[dt**4/4, dt**3/2],
                      [dt**3/2, dt**2]]) * var
    else:
        raise ValueError(f"Order {order} not supported. Use 1 or 2.")
    
    # Extend to full dimension if needed
    if dim > 2:
        Q_full = np.eye(dim) * var * dt
        Q_full[:2, :2] = Q
        return Q_full
    
    return Q
