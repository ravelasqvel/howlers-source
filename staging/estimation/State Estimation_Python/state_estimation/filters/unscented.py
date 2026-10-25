"""
Unscented Kalman Filter (UKF) implementation.

A generalized UKF that works with any user-defined dynamics f(x, u, dt)
and measurement function h(x), without requiring Jacobians.

Uses the Unscented Transform with sigma points.
"""

import numpy as np
from scipy.linalg import cholesky


class MerweScaledSigmaPoints:
    """
    Merwe's scaled sigma points.
    
    Generates sigma points and weights for the Unscented Transform.
    
    Parameters
    ----------
    n : int
        Dimensionality of the state
    alpha : float, optional
        Spread of sigma points around mean (typically 1e-3 to 1)
    beta : float, optional
        Incorporate prior knowledge of distribution (2 is optimal for Gaussian)
    kappa : float, optional
        Secondary scaling parameter (typically 0 or 3-n)
    """
    
    def __init__(self, n, alpha, beta, kappa=None):
        self.n = n
        self.alpha = alpha
        self.beta = beta
        
        if kappa is None:
            self.kappa = 3.0 - n
        else:
            self.kappa = kappa
        
        # Compute lambda
        self._lambda = (alpha**2) * (n + self.kappa) - n
        print(f'Alpha: {alpha}, Beta: {beta}, Kappa: {kappa}')
        print(f'Lambda computed: {self._lambda}')
        
        # Compute weights
        self.Wm = np.full(2*n + 1, 0.5 / (n + self._lambda))
        self.Wc = np.copy(self.Wm)
        self.Wm[0] = self._lambda / (n + self._lambda)
        self.Wc[0] = self._lambda / (n + self._lambda) + (1 - alpha**2 + beta)
        
    def sigma_points(self, x, P):
        """
        Generate sigma points around (x, P).
        
        Parameters
        ----------
        x : np.ndarray
            Mean state vector (n,)
        P : np.ndarray
            Covariance matrix (n, n)
            
        Returns
        -------
        np.ndarray
            Sigma points (2n+1, n)
        """
        n = self.n
        lambda_ = self._lambda
        
        # Compute matrix square root
        # scipy.linalg.cholesky returns upper triangular L where L.T @ L = A
        # We need to transpose to access columns as rows for sigma point directions
        try:
            U = cholesky((lambda_ + n) * P).T
        except np.linalg.LinAlgError:
            # If Cholesky fails, use eigendecomposition
            eigval, eigvec = np.linalg.eigh(P)
            eigval = np.maximum(eigval, 0)  # Ensure positive
            U = eigvec @ np.diag(np.sqrt(eigval * (lambda_ + n)))
        
        sigmas = np.zeros((2*n + 1, n))
        sigmas[0] = x
        
        for k in range(n):
            sigmas[k+1]   = x + U[k]
            sigmas[n+k+1] = x - U[k]
            
        return sigmas


class UnscentedKalmanFilter:
    """
    Unscented Kalman Filter for nonlinear systems.
    
    Uses the Unscented Transform instead of linearization, avoiding
    the need to compute Jacobians.
    
    The user must provide:
    - Dynamics function: f(x, u, dt) -> x_next
    - Measurement function: h(x) -> z
    
    Attributes
    ----------
    dim_x : int
        Dimension of the state vector
    dim_z : int
        Dimension of the measurement vector
    dim_u : int
        Dimension of the control input vector
    x : np.ndarray
        State estimate vector
    P : np.ndarray
        State covariance matrix
    Q : np.ndarray
        Process noise covariance
    R : np.ndarray
        Measurement noise covariance
        
    Examples
    --------
    >>> from state_estimation.filters import UnscentedKalmanFilter, MerweScaledSigmaPoints
    >>> points = MerweScaledSigmaPoints(n=6, alpha=0.1, beta=2, kappa=0)
    >>> ukf = UnscentedKalmanFilter(dim_x=6, dim_z=4, dim_u=3, points=points)
    >>> ukf.x = np.zeros(6)
    >>> ukf.P = np.eye(6)
    >>> ukf.Q = np.eye(6) * 0.01
    >>> ukf.R = np.eye(4) * 0.1
    >>> ukf.predict(u=control_input, f=my_dynamics)
    >>> ukf.update(z=measurement, h=my_measurement_fn)
    """
    
    def __init__(self, dim_x, dim_z, dim_u=0, points=None):
        """
        Initialize Unscented Kalman Filter.
        
        Parameters
        ----------
        dim_x : int
            Dimension of state vector
        dim_z : int
            Dimension of measurement vector
        dim_u : int, optional
            Dimension of control input vector
        points : MerweScaledSigmaPoints, optional
            Sigma points generator. If None, uses default parameters.
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
        
        # Sigma points
        if points is None:
            self.points = MerweScaledSigmaPoints(n=dim_x, alpha=0.1, beta=2.0, kappa=0.0)
        else:
            self.points = points
            print(f"Using custom sigma points with n={points.n}, alpha={points.alpha}, beta={points.beta}, kappa={points.kappa}")
        
        # Storage for sigma points
        self.sigmas_f = np.zeros((2*dim_x + 1, dim_x))
        self.sigmas_h = np.zeros((2*dim_x + 1, dim_z))
        
        # Mean and residual functions (for handling angles)
        self._x_mean_fn = None
        self._z_mean_fn = None
        self._residual_x_fn = lambda a, b: a - b
        self._residual_z_fn = lambda a, b: a - b
        
    def predict(self, u=None, f=None, Q=None):
        """
        Predict step of the UKF.
        
        Parameters
        ----------
        u : np.ndarray, optional
            Control input vector
        f : callable
            Dynamics function: f(x, u, dt) -> x_next
        Q : np.ndarray, optional
            Process noise covariance (overrides self.Q if provided)
        """
        if f is None:
            raise ValueError("Dynamics function f(x, u) must be provided")
        
        if u is None:
            u = np.zeros(self.dim_u)
        
        if Q is None:
            Q = self.Q
        
        # Generate sigma points
        sigmas = self.points.sigma_points(self.x, self.P)
        
        # Propagate sigma points through dynamics
        for i, s in enumerate(sigmas):
            self.sigmas_f[i] = f(s, u)
        
        # Compute predicted mean
        if self._x_mean_fn is not None:
            self.x = self._x_mean_fn(self.sigmas_f, self.points.Wm)
        else:
            self.x = np.dot(self.points.Wm, self.sigmas_f)
        
        # Compute predicted covariance
        self.P = self._unscented_transform_cov(
            self.sigmas_f, self.x, self.points.Wc, Q, self._residual_x_fn
        )
    
    def update(self, z, h=None, R=None):
        """
        Update step of the UKF.
        
        Parameters
        ----------
        z : np.ndarray
            Measurement vector
        h : callable
            Measurement function: h(x) -> z
        R : np.ndarray, optional
            Measurement noise covariance (overrides self.R if provided)
        """
        if h is None:
            raise ValueError("Measurement function h(x) must be provided")
        
        if R is None:
            R = self.R
        
        # Propagate sigma points through measurement function
        for i, s in enumerate(self.sigmas_f):
            self.sigmas_h[i] = h(s)
        
        # Compute predicted measurement mean
        if self._z_mean_fn is not None:
            z_pred = self._z_mean_fn(self.sigmas_h, self.points.Wm)
        else:
            z_pred = np.dot(self.points.Wm, self.sigmas_h)
        
        # Innovation covariance
        P_zz = self._unscented_transform_cov(
            self.sigmas_h, z_pred, self.points.Wc, R, self._residual_z_fn
        )
        
        # Cross covariance
        P_xz = self._cross_covariance(
            self.sigmas_f, self.x, self.sigmas_h, z_pred, self.points.Wc
        )
        
        # Kalman gain
        K = P_xz @ np.linalg.inv(P_zz)
        
        # Update state
        residual = self._residual_z_fn(z, z_pred)
        self.x = self.x + K @ residual
        
        # Update covariance
        self.P = self.P - K @ P_zz @ K.T
        
    def _unscented_transform_cov(self, sigmas, mean, Wc, noise_cov, residual_fn):
        """
        Compute covariance from sigma points.
        
        Parameters
        ----------
        sigmas : np.ndarray
            Sigma points
        mean : np.ndarray
            Mean of sigma points
        Wc : np.ndarray
            Covariance weights
        noise_cov : np.ndarray
            Additional noise covariance
        residual_fn : callable
            Function to compute residuals
            
        Returns
        -------
        np.ndarray
            Covariance matrix
        """
        n = sigmas.shape[1]
        P = np.zeros((n, n))
        
        for i, s in enumerate(sigmas):
            y = residual_fn(s, mean)
            P += Wc[i] * np.outer(y, y)
        
        return P + noise_cov
    
    def _cross_covariance(self, sigmas_x, x_mean, sigmas_z, z_mean, Wc):
        """
        Compute cross covariance between state and measurement.
        
        Parameters
        ----------
        sigmas_x : np.ndarray
            State sigma points
        x_mean : np.ndarray
            State mean
        sigmas_z : np.ndarray
            Measurement sigma points
        z_mean : np.ndarray
            Measurement mean
        Wc : np.ndarray
            Covariance weights
            
        Returns
        -------
        np.ndarray
            Cross covariance matrix
        """
        n_x = sigmas_x.shape[1]
        n_z = sigmas_z.shape[1]
        P_xz = np.zeros((n_x, n_z))
        
        for i in range(len(Wc)):
            dx = self._residual_x_fn(sigmas_x[i], x_mean)
            dz = self._residual_z_fn(sigmas_z[i], z_mean)
            P_xz += Wc[i] * np.outer(dx, dz)
        
        return P_xz
    
    def set_mean_fn(self, x_mean_fn=None, z_mean_fn=None):
        """
        Set custom mean functions for state and measurement.
        
        Useful when states/measurements include angles.
        
        Parameters
        ----------
        x_mean_fn : callable, optional
            Function: x_mean_fn(sigmas, Wm) -> mean_x
        z_mean_fn : callable, optional
            Function: z_mean_fn(sigmas, Wm) -> mean_z
        """
        if x_mean_fn is not None:
            self._x_mean_fn = x_mean_fn
        if z_mean_fn is not None:
            self._z_mean_fn = z_mean_fn
    
    def set_residual_fn(self, residual_x_fn=None, residual_z_fn=None):
        """
        Set custom residual functions.
        
        Parameters
        ----------
        residual_x_fn : callable, optional
            Function: residual_x_fn(a, b) -> residual for states
        residual_z_fn : callable, optional
            Function: residual_z_fn(a, b) -> residual for measurements
        """
        if residual_x_fn is not None:
            self._residual_x_fn = residual_x_fn
        if residual_z_fn is not None:
            self._residual_z_fn = residual_z_fn
