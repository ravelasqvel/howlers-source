"""
Gaussian Process IMU Acceleration Correction

This module implements a Gaussian Process (GP) to learn and correct systematic
errors in IMU acceleration measurements for omnidirectional mobile robots.

Mathematical Foundation:
------------------------
The IMU accelerations contain systematic errors:
    a_imu = a_true + ε_systematic + ε_noise

where:
- ε_systematic: Learnable bias/drift (learned by GP)
- ε_noise: White noise (captured by Q matrix)

The GP learns the residual function:
    r(x) = a_true - a_imu

by using kinematically-derived accelerations (from wheel encoders) as reference.

Training:
---------
1. Compute reference accelerations from wheel velocities (kinematics)
2. Compute residuals: r = a_ref - a_imu
3. Train independent GPs for x and y accelerations
4. Use state-dependent features: [vx, vy, omega, ax_imu, ay_imu]

Inference:
----------
At runtime, the GP corrects IMU measurements:
    a_corrected = a_imu + GP(state)

This correction can be applied in the prediction step of EKF/UKF/PF.

Author: Robotics Thesis Project
Date: January 2026
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel as C
import pickle


class IMUAccelerationGP:
    """
    Gaussian Process for IMU acceleration correction.
    
    Learns systematic errors in IMU accelerations using wheel encoder
    kinematics as reference. Trains two independent 1D GPs (one for each axis).
    
    Parameters
    ----------
    kernel_x : sklearn kernel, optional
        Kernel for x-acceleration GP (default: RBF + WhiteKernel)
    kernel_y : sklearn kernel, optional
        Kernel for y-acceleration GP (default: RBF + WhiteKernel)
    normalize_inputs : bool, optional
        Whether to normalize input features (default: True)
    
    Attributes
    ----------
    gp_x : GaussianProcessRegressor
        GP model for x-acceleration correction
    gp_y : GaussianProcessRegressor
        GP model for y-acceleration correction
    trained : bool
        Whether the model has been trained
    """
    
    def __init__(self, kernel_x=None, kernel_y=None, normalize_inputs=True):
        
        # Default kernels: RBF (smooth) + WhiteKernel (noise)
        if kernel_x is None:
            kernel_x = C(1.0, (1e-3, 1e3)) * RBF(length_scale=[1.0]*5, length_scale_bounds=(1e-2, 1e3)) + \
                       WhiteKernel(noise_level=0.01, noise_level_bounds=(1e-5, 1e1))
        
        if kernel_y is None:
            kernel_y = C(1.0, (1e-3, 1e3)) * RBF(length_scale=[1.0]*5, length_scale_bounds=(1e-2, 1e3)) + \
                       WhiteKernel(noise_level=0.01, noise_level_bounds=(1e-5, 1e1))
        
        # Initialize GP regressors
        self.gp_x = GaussianProcessRegressor(
            kernel=kernel_x,
            n_restarts_optimizer=10,
            normalize_y=True,
            alpha=1e-6  # Regularization
        )
        
        self.gp_y = GaussianProcessRegressor(
            kernel=kernel_y,
            n_restarts_optimizer=10,
            normalize_y=True,
            alpha=1e-6
        )
        
        self.normalize_inputs = normalize_inputs
        self.trained = False
        
        # Normalization parameters (fitted during training)
        self.X_mean = None
        self.X_std = None
    
    def _extract_features(self, states, imu_accel):
        """
        Extract features for GP from robot state and IMU.
        
        Features: [vx_body, vy_body, omega, ax_imu, ay_imu]
        
        These features capture the robot's kinematic state and raw IMU readings,
        which are informative for predicting systematic IMU errors.
        
        Parameters
        ----------
        states : np.ndarray, shape (N, 6)
            Robot states [x, y, phi, vx, vy, omega]
        imu_accel : np.ndarray, shape (N, 2)
            IMU accelerations [ax_imu, ay_imu]
        
        Returns
        -------
        X : np.ndarray, shape (N, 5)
            Feature matrix
        """
        vx = states[:, 3]
        vy = states[:, 4]
        omega = states[:, 5]
        ax_imu = imu_accel[:, 0]
        ay_imu = imu_accel[:, 1]
        
        X = np.column_stack([vx, vy, omega, ax_imu, ay_imu])
        return X
    
    def _normalize_features(self, X, fit=False):
        """
        Normalize features to zero mean and unit variance.
        
        Parameters
        ----------
        X : np.ndarray, shape (N, D)
            Input features
        fit : bool, optional
            If True, compute normalization parameters from X (default: False)
        
        Returns
        -------
        X_norm : np.ndarray
            Normalized features
        """
        if fit:
            self.X_mean = np.mean(X, axis=0)
            self.X_std = np.std(X, axis=0)
            self.X_std[self.X_std < 1e-8] = 1.0  # Avoid division by zero
        
        if self.normalize_inputs and self.X_mean is not None:
            X_norm = (X - self.X_mean) / self.X_std
        else:
            X_norm = X
        
        return X_norm
    
    def train(self, states, imu_accel, kinematic_accel, verbose=True):
        """
        Train GP models from experimental data.
        
        Parameters
        ----------
        states : np.ndarray, shape (N, 6)
            Robot states [x, y, phi, vx, vy, omega]
        imu_accel : np.ndarray, shape (N, 2)
            IMU accelerations [ax_imu, ay_imu]
        kinematic_accel : np.ndarray, shape (N, 2)
            Reference accelerations from kinematics [ax_ref, ay_ref]
        verbose : bool, optional
            Print training progress (default: True)
        
        Returns
        -------
        training_stats : dict
            Dictionary with training statistics
        """
        if verbose:
            print("="*70)
            print("GAUSSIAN PROCESS TRAINING - IMU ACCELERATION CORRECTION")
            print("="*70)
        
        N = len(states)
        
        # Compute residuals (targets for GP)
        residuals = kinematic_accel - imu_accel
        rx = residuals[:, 0]
        ry = residuals[:, 1]
        
        if verbose:
            print(f"\n1. Training data statistics (N = {N}):")
            print(f"   rx: mean = {np.mean(rx):.4f}, std = {np.std(rx):.4f} m/s^2")
            print(f"   ry: mean = {np.mean(ry):.4f}, std = {np.std(ry):.4f} m/s^2")
        
        # Extract and normalize features
        X = self._extract_features(states, imu_accel)
        X_norm = self._normalize_features(X, fit=True)
        
        if verbose:
            print(f"\n2. Feature extraction:")
            print(f"   Features: [vx, vy, omega, ax_imu, ay_imu]")
            print(f"   Feature matrix shape: {X.shape}")
        
        # Train GP for x-acceleration
        if verbose:
            print(f"\n3. Training GP for x-acceleration...")
        
        self.gp_x.fit(X_norm, rx)
        
        if verbose:
            print(f"   Kernel: {self.gp_x.kernel_}")
            print(f"   Log-likelihood: {self.gp_x.log_marginal_likelihood_value_:.2f}")
        
        # Train GP for y-acceleration
        if verbose:
            print(f"\n4. Training GP for y-acceleration...")
        
        self.gp_y.fit(X_norm, ry)
        
        if verbose:
            print(f"   Kernel: {self.gp_y.kernel_}")
            print(f"   Log-likelihood: {self.gp_y.log_marginal_likelihood_value_:.2f}")
        
        self.trained = True
        
        # Compute training statistics
        rx_pred, rx_std = self.gp_x.predict(X_norm, return_std=True)
        ry_pred, ry_std = self.gp_y.predict(X_norm, return_std=True)
        
        rmse_x = np.sqrt(np.mean((rx - rx_pred)**2))
        rmse_y = np.sqrt(np.mean((ry - ry_pred)**2))
        
        if verbose:
            print(f"\n5. Training performance:")
            print(f"   RMSE(rx) = {rmse_x:.4f} m/s^2")
            print(f"   RMSE(ry) = {rmse_y:.4f} m/s^2")
            print(f"   Mean prediction std(rx) = {np.mean(rx_std):.4f} m/s^2")
            print(f"   Mean prediction std(ry) = {np.mean(ry_std):.4f} m/s^2")
        
        if verbose:
            print("\n" + "="*70)
            print("GP TRAINING COMPLETE")
            print("="*70)
        
        training_stats = {
            'N': N,
            'rmse_x': rmse_x,
            'rmse_y': rmse_y,
            'residuals_mean': [np.mean(rx), np.mean(ry)],
            'residuals_std': [np.std(rx), np.std(ry)],
            'log_likelihood_x': self.gp_x.log_marginal_likelihood_value_,
            'log_likelihood_y': self.gp_y.log_marginal_likelihood_value_,
        }
        
        return training_stats
    
    def predict(self, states, imu_accel, return_std=False):
        """
        Predict acceleration corrections.
        
        Parameters
        ----------
        states : np.ndarray, shape (N, 6) or (6,)
            Robot states [x, y, phi, vx, vy, omega]
        imu_accel : np.ndarray, shape (N, 2) or (2,)
            IMU accelerations [ax_imu, ay_imu]
        return_std : bool, optional
            Return prediction uncertainty (default: False)
        
        Returns
        -------
        corrections : np.ndarray, shape (N, 2) or (2,)
            Predicted residuals [rx, ry]
        std : np.ndarray, shape (N, 2) or (2,), optional
            Prediction standard deviations (if return_std=True)
        """
        if not self.trained:
            raise RuntimeError("GP must be trained before prediction. Call train() first.")
        
        # Handle single sample
        single_sample = False
        if states.ndim == 1:
            states = states.reshape(1, -1)
            imu_accel = imu_accel.reshape(1, -1)
            single_sample = True
        
        # Extract and normalize features
        X = self._extract_features(states, imu_accel)
        X_norm = self._normalize_features(X, fit=False)
        
        # Predict
        if return_std:
            rx_pred, rx_std = self.gp_x.predict(X_norm, return_std=True)
            ry_pred, ry_std = self.gp_y.predict(X_norm, return_std=True)
            
            corrections = np.column_stack([rx_pred, ry_pred])
            std = np.column_stack([rx_std, ry_std])
            
            if single_sample:
                return corrections[0], std[0]
            else:
                return corrections, std
        else:
            rx_pred = self.gp_x.predict(X_norm, return_std=False)
            ry_pred = self.gp_y.predict(X_norm, return_std=False)
            
            corrections = np.column_stack([rx_pred, ry_pred])
            
            if single_sample:
                return corrections[0]
            else:
                return corrections
    
    def correct_imu(self, states, imu_accel):
        """
        Apply GP correction to IMU measurements.
        
        Parameters
        ----------
        states : np.ndarray, shape (N, 6) or (6,)
            Robot states
        imu_accel : np.ndarray, shape (N, 2) or (2,)
            Raw IMU accelerations
        
        Returns
        -------
        corrected_accel : np.ndarray, shape (N, 2) or (2,)
            Corrected accelerations
        """
        corrections = self.predict(states, imu_accel, return_std=False)
        corrected_accel = imu_accel + corrections
        return corrected_accel
    
    def save(self, filepath):
        """
        Save trained GP model to file.
        
        Parameters
        ----------
        filepath : str or Path
            Path to save model
        """
        if not self.trained:
            raise RuntimeError("Cannot save untrained model.")
        
        model_data = {
            'gp_x': self.gp_x,
            'gp_y': self.gp_y,
            'normalize_inputs': self.normalize_inputs,
            'X_mean': self.X_mean,
            'X_std': self.X_std
        }
        
        with open(filepath, 'wb') as f:
            pickle.dump(model_data, f)
        
        print(f"GP model saved to: {filepath}")
    
    @classmethod
    def load(cls, filepath):
        """
        Load trained GP model from file.
        
        Parameters
        ----------
        filepath : str or Path
            Path to model file
        
        Returns
        -------
        model : IMUAccelerationGP
            Loaded GP model
        """
        with open(filepath, 'rb') as f:
            model_data = pickle.load(f)
        
        # Create instance
        model = cls(normalize_inputs=model_data['normalize_inputs'])
        model.gp_x = model_data['gp_x']
        model.gp_y = model_data['gp_y']
        model.X_mean = model_data['X_mean']
        model.X_std = model_data['X_std']
        model.trained = True
        
        print(f"GP model loaded from: {filepath}")
        
        return model


# ============================================================================
# Utility Functions
# ============================================================================

def visualize_gp_predictions(states, imu_accel, kinematic_accel, gp_model, 
                             save_path=None):
    """
    Visualize GP predictions vs. true residuals.
    
    Parameters
    ----------
    states : np.ndarray, shape (N, 6)
        Robot states
    imu_accel : np.ndarray, shape (N, 2)
        IMU accelerations
    kinematic_accel : np.ndarray, shape (N, 2)
        Reference accelerations
    gp_model : IMUAccelerationGP
        Trained GP model
    save_path : str or Path, optional
        Path to save figure
    
    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure handle
    """
    # Compute true residuals
    true_residuals = kinematic_accel - imu_accel
    
    # Predict with GP
    pred_residuals, pred_std = gp_model.predict(states, imu_accel, return_std=True)
    
    # Corrected accelerations
    corrected_accel = gp_model.correct_imu(states, imu_accel)
    
    N = len(states)
    time = np.arange(N) * 0.01  # Assuming 100 Hz
    
    # IEEE formatting
    plt.rcParams.update({
        'font.family': 'serif',
        'font.serif': ['Times New Roman'],
        'font.size': 10,
        'axes.labelsize': 11,
        'axes.titlesize': 11,
        'legend.fontsize': 9,
        'figure.dpi': 300,
        'text.usetex': False
    })
    
    fig, axes = plt.subplots(3, 2, figsize=(7.16, 7))
    
    # Row 1: Residual predictions
    for col, label in enumerate(['X', 'Y']):
        ax = axes[0, col]
        ax.plot(time, true_residuals[:, col], 'k-', linewidth=1, 
                alpha=0.7, label='Residual Real')
        ax.plot(time, pred_residuals[:, col], 'b--', linewidth=1.5, 
                label='Predicción GP')
        ax.fill_between(time, 
                        pred_residuals[:, col] - 2*pred_std[:, col],
                        pred_residuals[:, col] + 2*pred_std[:, col],
                        alpha=0.2, color='blue', label='±2σ GP')
        ax.set_ylabel(f'$r_{{a_{label.lower()}}}$ [m/s²]')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title(f'Predicción Residual {label}')
    
    # Row 2: Accelerations comparison
    for col, label in enumerate(['X', 'Y']):
        ax = axes[1, col]
        ax.plot(time, kinematic_accel[:, col], 'k-', linewidth=1.5, 
                alpha=0.7, label='Referencia (Cinemática)')
        ax.plot(time, imu_accel[:, col], 'r:', linewidth=1.5, 
                alpha=0.7, label='IMU (Sin corregir)')
        ax.plot(time, corrected_accel[:, col], 'b--', linewidth=1.5, 
                label='IMU (Corregido GP)')
        ax.set_ylabel(f'$a_{label.lower()}$ [m/s²]')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title(f'Aceleración {label}')
    
    # Row 3: Error reduction
    error_before = imu_accel - kinematic_accel
    error_after = corrected_accel - kinematic_accel
    
    for col, label in enumerate(['X', 'Y']):
        ax = axes[2, col]
        ax.plot(time, error_before[:, col], 'r-', linewidth=1, 
                alpha=0.7, label='Error antes GP')
        ax.plot(time, error_after[:, col], 'b-', linewidth=1, 
                alpha=0.7, label='Error después GP')
        ax.axhline(0, color='k', linestyle='--', linewidth=0.8)
        ax.set_xlabel('Tiempo [s]')
        ax.set_ylabel(f'Error $a_{label.lower()}$ [m/s²]')
        ax.legend(loc='best', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_title(f'Reducción Error {label}')
    
    plt.tight_layout()
    
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    
    plt.rcParams.update(plt.rcParamsDefault)
    
    return fig


def compute_correction_metrics(imu_accel, kinematic_accel, corrected_accel):
    """
    Compute metrics for GP correction performance.
    
    Parameters
    ----------
    imu_accel : np.ndarray, shape (N, 2)
        Raw IMU accelerations
    kinematic_accel : np.ndarray, shape (N, 2)
        Reference accelerations
    corrected_accel : np.ndarray, shape (N, 2)
        GP-corrected accelerations
    
    Returns
    -------
    metrics : dict
        Dictionary with performance metrics
    """
    # Errors before and after correction
    error_before = imu_accel - kinematic_accel
    error_after = corrected_accel - kinematic_accel
    
    # RMSE
    rmse_before = np.sqrt(np.mean(error_before**2, axis=0))
    rmse_after = np.sqrt(np.mean(error_after**2, axis=0))
    rmse_improvement = (rmse_before - rmse_after) / rmse_before * 100
    
    # MAE
    mae_before = np.mean(np.abs(error_before), axis=0)
    mae_after = np.mean(np.abs(error_after), axis=0)
    mae_improvement = (mae_before - mae_after) / mae_before * 100
    
    # Bias (mean error)
    bias_before = np.mean(error_before, axis=0)
    bias_after = np.mean(error_after, axis=0)
    
    # Standard deviation
    std_before = np.std(error_before, axis=0)
    std_after = np.std(error_after, axis=0)
    
    metrics = {
        'rmse_before': rmse_before,
        'rmse_after': rmse_after,
        'rmse_improvement_pct': rmse_improvement,
        'mae_before': mae_before,
        'mae_after': mae_after,
        'mae_improvement_pct': mae_improvement,
        'bias_before': bias_before,
        'bias_after': bias_after,
        'std_before': std_before,
        'std_after': std_after
    }
    
    return metrics


def print_correction_metrics(metrics):
    """Print GP correction metrics in a formatted table."""
    print("\n" + "="*70)
    print("GP CORRECTION PERFORMANCE METRICS")
    print("="*70)
    
    print("\n{:<20} {:>12} {:>12} {:>12}".format("Metric", "X-axis", "Y-axis", "Unit"))
    print("-"*70)
    
    print("{:<20} {:>12.4f} {:>12.4f} {:>12}".format(
        "RMSE (antes)", metrics['rmse_before'][0], metrics['rmse_before'][1], "m/s^2"))
    print("{:<20} {:>12.4f} {:>12.4f} {:>12}".format(
        "RMSE (despues)", metrics['rmse_after'][0], metrics['rmse_after'][1], "m/s^2"))
    print("{:<20} {:>12.1f} {:>12.1f} {:>12}".format(
        "Mejora RMSE", metrics['rmse_improvement_pct'][0], 
        metrics['rmse_improvement_pct'][1], "%"))
    
    print("-"*70)
    
    print("{:<20} {:>12.4f} {:>12.4f} {:>12}".format(
        "MAE (antes)", metrics['mae_before'][0], metrics['mae_before'][1], "m/s^2"))
    print("{:<20} {:>12.4f} {:>12.4f} {:>12}".format(
        "MAE (despues)", metrics['mae_after'][0], metrics['mae_after'][1], "m/s^2"))
    print("{:<20} {:>12.1f} {:>12.1f} {:>12}".format(
        "Mejora MAE", metrics['mae_improvement_pct'][0], 
        metrics['mae_improvement_pct'][1], "%"))
    
    print("-"*70)
    
    print("{:<20} {:>12.4f} {:>12.4f} {:>12}".format(
        "Bias (antes)", metrics['bias_before'][0], metrics['bias_before'][1], "m/s^2"))
    print("{:<20} {:>12.4f} {:>12.4f} {:>12}".format(
        "Bias (despues)", metrics['bias_after'][0], metrics['bias_after'][1], "m/s^2"))
    
    print("-"*70)
    
    print("{:<20} {:>12.4f} {:>12.4f} {:>12}".format(
        "Std (antes)", metrics['std_before'][0], metrics['std_before'][1], "m/s^2"))
    print("{:<20} {:>12.4f} {:>12.4f} {:>12}".format(
        "Std (despues)", metrics['std_after'][0], metrics['std_after'][1], "m/s^2"))
    
    print("="*70)
