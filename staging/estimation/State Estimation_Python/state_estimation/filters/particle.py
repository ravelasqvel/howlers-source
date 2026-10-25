"""
Particle Filter (PF) implementation.

A generalized Sequential Importance Resampling (SIR) particle filter
that works with any user-defined dynamics f(x, u, dt) and measurement
likelihood function.
"""

import numpy as np
from scipy.stats import norm, multivariate_normal


class ParticleFilter:
    """
    Particle Filter for nonlinear, non-Gaussian systems.
    
    Uses Sequential Importance Resampling (SIR) with systematic resampling.
    
    The user must provide:
    - Dynamics function: f(x, u, dt) -> x_next
    - Measurement likelihood: p(z | x) as a function
    
    Attributes
    ----------
    dim_x : int
        Dimension of the state vector
    dim_z : int
        Dimension of the measurement vector
    N : int
        Number of particles
    particles : np.ndarray
        Particle states (N, dim_x)
    weights : np.ndarray
        Particle weights (N,)
    x : np.ndarray
        Estimated state (weighted mean of particles)
    P : np.ndarray
        Estimated covariance
        
    Examples
    --------
    >>> pf = ParticleFilter(dim_x=6, dim_z=4, N=1000)
    >>> pf.initialize_particles(x0=np.zeros(6), P0=np.eye(6))
    >>> pf.predict(u=control_input, f=my_dynamics, process_noise_std=0.1)
    >>> pf.update(z=measurement, likelihood_fn=my_likelihood_fn)
    >>> pf.resample()
    """
    
    def __init__(self, dim_x, dim_z, N=1000, dim_u=0):
        """
        Initialize Particle Filter.
        
        Parameters
        ----------
        dim_x : int
            Dimension of state vector
        dim_z : int
            Dimension of measurement vector
        N : int, optional
            Number of particles (default: 1000)
        dim_u : int, optional
            Dimension of control input vector
        """
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u
        self.N = N
        
        # Particles and weights
        self.particles = np.zeros((N, dim_x))
        self.weights = np.ones(N) / N
        
        # State estimate
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)
        
        # Effective sample size threshold for resampling
        self.resample_threshold = N / 2.0
        
    def initialize_particles(self, x0, P0=None):
        """
        Initialize particles around initial state.
        
        Parameters
        ----------
        x0 : np.ndarray
            Initial state mean (dim_x,)
        P0 : np.ndarray, optional
            Initial state covariance (dim_x, dim_x).
            If None, uses identity matrix.
        """
        if P0 is None:
            P0 = np.eye(self.dim_x)
        
        # Sample particles from multivariate Gaussian
        self.particles = np.random.multivariate_normal(x0, P0, size=self.N)
        self.weights = np.ones(self.N) / self.N
        
        # Compute initial estimate
        self.x = np.average(self.particles, weights=self.weights, axis=0)
        self.P = self._compute_covariance()
        
    def predict(self, u=None, f=None, process_noise_std=None, Q=None):
        """
        Predict step: propagate particles through dynamics.
        
        Parameters
        ----------
        u : np.ndarray, optional
            Control input vector
        f : callable
            Dynamics function: f(x, u) -> x_next
        process_noise_std : float or np.ndarray, optional
            Standard deviation of process noise added to each state dimension.
            Can be scalar or array of shape (dim_x,)
        Q : np.ndarray, optional
            Process noise covariance matrix (dim_x, dim_x).
            If provided, overrides process_noise_std.
        """
        if f is None:
            raise ValueError("Dynamics function f(x, u) must be provided")
        
        if u is None:
            u = np.zeros(self.dim_u)
        
        # Propagate each particle
        for i in range(self.N):
            self.particles[i] = f(self.particles[i], u)
        
        # Add process noise
        if Q is not None:
            # Use full covariance matrix
            noise = np.random.multivariate_normal(
                np.zeros(self.dim_x), Q, size=self.N
            )
            self.particles += noise
        elif process_noise_std is not None:
            # Use diagonal covariance (independent noise per dimension)
            if np.isscalar(process_noise_std):
                noise_std = np.ones(self.dim_x) * process_noise_std
            else:
                noise_std = np.asarray(process_noise_std)
            
            noise = np.random.randn(self.N, self.dim_x) * noise_std
            self.particles += noise
        
        # Update estimate
        self.x = np.average(self.particles, weights=self.weights, axis=0)
        
    def update(self, z, likelihood_fn=None, h=None, R=None):
        """
        Update step: reweight particles based on measurement.
        
        Parameters
        ----------
        z : np.ndarray
            Measurement vector
        likelihood_fn : callable, optional
            Likelihood function: likelihood_fn(z, x) -> probability
            If None, uses Gaussian likelihood with h and R.
        h : callable, optional
            Measurement function: h(x) -> z_pred
            Required if likelihood_fn is None.
        R : np.ndarray, optional
            Measurement noise covariance (dim_z, dim_z).
            Required if likelihood_fn is None.
        """
        if likelihood_fn is None:
            # Use Gaussian likelihood
            if h is None or R is None:
                raise ValueError(
                    "Either provide likelihood_fn or both h and R"
                )
            likelihood_fn = self._make_gaussian_likelihood(h, R)
        
        # Compute likelihood for each particle
        for i in range(self.N):
            self.weights[i] *= likelihood_fn(z, self.particles[i])
        
        # Normalize weights
        weight_sum = np.sum(self.weights)
        if weight_sum < 1e-10:
            # All weights near zero - reset to uniform
            self.weights = np.ones(self.N) / self.N
        else:
            self.weights /= weight_sum
        
        # Update estimate
        self.x = np.average(self.particles, weights=self.weights, axis=0)
        self.P = self._compute_covariance()
        
    def resample(self, scheme='systematic'):
        """
        Resample particles if effective sample size is too low.
        
        Parameters
        ----------
        scheme : str, optional
            Resampling scheme. Options: 'systematic', 'multinomial', 
            'residual', 'stratified'
            Default: 'systematic'
        """
        # Check effective sample size
        N_eff = self.effective_sample_size()
        
        if N_eff < self.resample_threshold:
            if scheme == 'systematic':
                indices = self._systematic_resample()
            elif scheme == 'multinomial':
                indices = self._multinomial_resample()
            elif scheme == 'residual':
                indices = self._residual_resample()
            elif scheme == 'stratified':
                indices = self._stratified_resample()
            else:
                raise ValueError(f"Unknown resampling scheme: {scheme}")
            
            # Resample particles
            self.particles = self.particles[indices]
            self.weights = np.ones(self.N) / self.N
    
    def effective_sample_size(self):
        """
        Compute effective sample size (ESS): Neff.
        
        ESS indicates the quality of the particle approximation.
        Lower values indicate particle degeneracy.
        
        Returns
        -------
        float
            Effective sample size (1 to N)
        """
        return 1.0 / np.sum(self.weights**2)
    
    def _systematic_resample(self):
        """
        Systematic resampling.
        
        More efficient and lower variance than multinomial resampling.
        
        Returns
        -------
        np.ndarray
            Indices of resampled particles
        """
        positions = (np.arange(self.N) + np.random.random()) / self.N
        
        indices = np.zeros(self.N, dtype=int)
        cumsum = np.cumsum(self.weights)
        
        i, j = 0, 0
        while i < self.N:
            if positions[i] < cumsum[j]:
                indices[i] = j
                i += 1
            else:
                j += 1
        
        return indices
    
    def _multinomial_resample(self):
        """
        Multinomial resampling.
        
        Simple but higher variance than systematic resampling.
        
        Returns
        -------
        np.ndarray
            Indices of resampled particles
        """
        return np.random.choice(self.N, size=self.N, p=self.weights)
    
    def _residual_resample(self):
        """
        Residual resampling.
        
        Deterministically keeps floor(N * weight[i]) copies of each particle,
        then resamples the residual stochastically. Lower variance than
        multinomial, similar to systematic.
        
        Returns
        -------
        np.ndarray
            Indices of resampled particles
        """
        indices = np.zeros(self.N, dtype=int)
        
        # Compute number of deterministic copies for each particle
        num_copies = (self.N * self.weights).astype(int)
        
        # Deterministic assignment
        k = 0
        for i in range(self.N):
            for _ in range(num_copies[i]):
                indices[k] = i
                k += 1
        
        # Stochastic assignment for residual
        residual = self.weights - num_copies / self.N
        residual /= np.sum(residual)  # Normalize
        
        cumsum = np.cumsum(residual)
        while k < self.N:
            u = np.random.random()
            # Find index where u < cumsum
            idx = np.searchsorted(cumsum, u)
            indices[k] = idx
            k += 1
        
        return indices
    
    def _stratified_resample(self):
        """
        Stratified resampling.
        
        Divides [0,1] into N strata and samples one particle from each.
        Lower variance than multinomial, similar performance to systematic.
        
        Returns
        -------
        np.ndarray
            Indices of resampled particles
        """
        # Generate random positions in each stratum
        positions = (np.arange(self.N) + np.random.random(self.N)) / self.N
        
        indices = np.zeros(self.N, dtype=int)
        cumsum = np.cumsum(self.weights)
        
        i, j = 0, 0
        while i < self.N:
            if positions[i] < cumsum[j]:
                indices[i] = j
                i += 1
            else:
                j += 1
        
        return indices
    
    def _compute_covariance(self):
        """
        Compute weighted covariance of particles.
        
        Returns
        -------
        np.ndarray
            Covariance matrix (dim_x, dim_x)
        """
        # Weighted mean
        mean = self.x
        
        # Weighted covariance
        diff = self.particles - mean
        P = np.zeros((self.dim_x, self.dim_x))
        
        for i in range(self.N):
            P += self.weights[i] * np.outer(diff[i], diff[i])
        
        return P
    
    def _make_gaussian_likelihood(self, h, R):
        """
        Create Gaussian likelihood function.
        
        Parameters
        ----------
        h : callable
            Measurement function
        R : np.ndarray
            Measurement noise covariance
            
        Returns
        -------
        callable
            Likelihood function
        """
        def likelihood(z, x):
            z_pred = h(x)
            residual = z - z_pred
            
            # Gaussian likelihood
            try:
                prob = multivariate_normal.pdf(residual, mean=np.zeros(self.dim_z), cov=R)
            except:
                # Fallback for numerical issues
                prob = 1e-10
            
            return prob
        
        return likelihood
    
    def get_particles(self):
        """
        Get current particles and weights.
        
        Returns
        -------
        particles : np.ndarray
            Particle states (N, dim_x)
        weights : np.ndarray
            Particle weights (N,)
        """
        return self.particles.copy(), self.weights.copy()
    
    def set_resample_threshold(self, threshold):
        """
        Set the effective sample size threshold for automatic resampling.
        
        Parameters
        ----------
        threshold : float
            ESS threshold (typically N/2)
        """
        self.resample_threshold = threshold
