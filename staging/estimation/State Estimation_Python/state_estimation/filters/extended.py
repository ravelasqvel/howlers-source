"""
Extended Kalman Filter (EKF) implementation.

A generalized EKF that works with any user-defined dynamics f(x, u, dt),
measurement function h(x), and their Jacobians F and H.

Inspired by FilterPy's API design.
"""

import numpy as np
from ..common.angles import normalize_angle


class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for nonlinear systems.
    
    The user must provide:
    - Dynamics function: f(x, u, dt) -> x_next
    - Jacobian of dynamics: F_jacobian(x, u, dt) -> F matrix
    - Measurement function: h(x) -> z
    - Jacobian of measurement: H_jacobian(x) -> H matrix
    
    Attributes
    ----------
    dim_x : int
        Dimension of the state vector
    dim_z : int
        Dimension of the measurement vector
    dim_u : int
        Dimension of the control input vector
    x : np.ndarray
        State estimate vector (dim_x,)
    P : np.ndarray
        State covariance matrix (dim_x, dim_x)
    Q : np.ndarray
        Process noise covariance (dim_x, dim_x)
    R : np.ndarray
        Measurement noise covariance (dim_z, dim_z)
    
    Examples
    --------
    >>> ekf = ExtendedKalmanFilter(dim_x=6, dim_z=4, dim_u=3)
    >>> ekf.x = np.zeros(6)
    >>> ekf.P = np.eye(6)
    >>> ekf.Q = np.eye(6) * 0.01
    >>> ekf.R = np.eye(4) * 0.1
    >>> ekf.predict(u=control_input, f=my_dynamics, F=my_jacobian)
    >>> ekf.update(z=measurement, h=my_measurement_fn, H=my_H_jacobian)
    """
    
    def __init__(self, dim_x, dim_z, dim_u=0):
        """
        Initialize Extended Kalman Filter.
        
        Parameters
        ----------
        dim_x : int
            Dimension of state vector
        dim_z : int
            Dimension of measurement vector
        dim_u : int, optional
            Dimension of control input vector (default: 0)
        """
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u
        
        # State and covariance
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)
        
        # Process and measurement noise
        self.Q = np.eye(dim_x)
        self.R = np.eye(dim_z)
        
        # For storing innovation
        self.y = np.zeros(dim_z)  # Residual (innovation)
        self.S = np.zeros((dim_z, dim_z))  # Innovation covariance
        self.K = np.zeros((dim_x, dim_z))  # Kalman gain
        
        # Residual function (for handling angles)
        self._residual_fn = lambda a, b: a - b
        
        # Angle handling flags (for state normalization)
        self._state_is_angle = None
        
    def predict(self, u=None, f=None, F=None, Q=None):
        """
        Predict step of the EKF.
        
        Propagates the state and covariance forward using the dynamics model.
        
        Parameters
        ----------
        u : np.ndarray, optional
            Control input vector
        f : callable
            Dynamics function: f(x, u) -> x_next
        F : callable or np.ndarray
            Jacobian of f. Can be:
            - Callable: F(x, u) -> F_matrix
            - np.ndarray: Pre-computed Jacobian matrix
        Q : np.ndarray, optional
            Process noise covariance (overrides self.Q if provided)
            
        Returns
        -------
        None
            Updates self.x and self.P in place
        """
        if f is None:
            raise ValueError("Dynamics function f(x, u) must be provided")
        
        if u is None:
            u = np.zeros(self.dim_u)
            
        # Predict state
        self.x = f(self.x, u)
        
        # Predict covariance
        if callable(F):
            F_matrix = F(self.x, u)
        elif F is not None:
            F_matrix = F
        else:
            raise ValueError("Jacobian F must be provided (callable or matrix)")
        
        if Q is None:
            Q = self.Q
            
        self.P = F_matrix @ self.P @ F_matrix.T + Q
        
    def update(self, z, h=None, H=None, R=None, residual_fn=None):
        """
        Update step of the EKF.
        
        Updates the state estimate using a measurement.
        
        Parameters
        ----------
        z : np.ndarray
            Measurement vector
        h : callable
            Measurement function: h(x) -> z_pred
        H : callable or np.ndarray
            Jacobian of h. Can be:
            - Callable: H(x) -> H_matrix
            - np.ndarray: Pre-computed Jacobian matrix
        R : np.ndarray, optional
            Measurement noise covariance (overrides self.R if provided)
        residual_fn : callable, optional
            Function to compute residual: residual_fn(z, z_pred) -> residual
            Useful for handling angular measurements
            
        Returns
        -------
        None
            Updates self.x and self.P in place
        """
        if h is None:
            raise ValueError("Measurement function h(x) must be provided")
        
        # Predicted measurement
        z_pred = h(self.x)
        
        # Compute residual (innovation)
        if residual_fn is not None:
            self.y = residual_fn(z, z_pred)
        else:
            self.y = self._residual_fn(z, z_pred)
        
        # Measurement Jacobian
        if callable(H):
            H_matrix = H(self.x)
        elif H is not None:
            H_matrix = H
        else:
            raise ValueError("Jacobian H must be provided (callable or matrix)")
        
        if R is None:
            R = self.R
        
        # Innovation covariance
        self.S = H_matrix @ self.P @ H_matrix.T + R
        
        # Kalman gain
        self.K = self.P @ H_matrix.T @ np.linalg.inv(self.S)
        
        # Update state
        self.x = self.x + self.K @ self.y
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(self.dim_x) - self.K @ H_matrix
        self.P = I_KH @ self.P @ I_KH.T + self.K @ R @ self.K.T
        
        # Normalize angular states to [-π, π]
        if hasattr(self, '_state_is_angle') and self._state_is_angle is not None:
            for i in range(self.dim_x):
                if self._state_is_angle[i]:
                    self.x[i] = normalize_angle(self.x[i])
    
    def set_residual_fn(self, residual_fn):
        """
        Set custom residual function.
        
        Useful for handling angular states/measurements.
        
        Parameters
        ----------
        residual_fn : callable
            Function with signature: residual_fn(a, b) -> residual
        """
        self._residual_fn = residual_fn
    
    def set_angle_states(self, state_is_angle):
        """
        Set which state components are angular.
        
        Angular states will be normalized to [-π, π] after each update.
        
        Parameters
        ----------
        state_is_angle : list or np.ndarray of bool
            Array indicating which state components are angles (dim_x,)
        
        Examples
        --------
        >>> ekf.set_angle_states([False, False, True])  # 3rd state is angle
        """
        self._state_is_angle = np.asarray(state_is_angle, dtype=bool)
        if len(self._state_is_angle) != self.dim_x:
            raise ValueError(f"state_is_angle must have length {self.dim_x}")
    
    def get_prediction(self, u=None, f=None):
        """
        Get predicted state without updating the filter.
        
        Parameters
        ----------
        u : np.ndarray, optional
            Control input
        f : callable
            Dynamics function
            
        Returns
        -------
        np.ndarray
            Predicted state (does not modify self.x)
        """
        if f is None:
            raise ValueError("Dynamics function f(x, u) must be provided")
        
        if u is None:
            u = np.zeros(self.dim_u)
            
        return f(self.x, u)
    
    def get_innovation(self):
        """
        Get the innovation (residual) from the last update.
        
        Returns
        -------
        np.ndarray
            Innovation vector y
        """
        return self.y
    
    def get_innovation_covariance(self):
        """
        Get the innovation covariance from the last update.
        
        Returns
        -------
        np.ndarray
            Innovation covariance matrix S
        """
        return self.S
