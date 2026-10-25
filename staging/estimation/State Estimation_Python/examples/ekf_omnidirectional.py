"""
EKF Example for Omnidirectional Robot

This example demonstrates how to use the Extended Kalman Filter (EKF)
for state estimation of an omnidirectional (omniwheel) mobile robot.

State: x = [x, y, psi, vx_b, vy_b, omega]
Control: u = [ax_b, ay_b, omega_meas]
Measurement: z = [vx_b, vy_b, omega, psi]
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from state_estimation import ExtendedKalmanFilter
from state_estimation.models import OmnidirectionalRobot
from state_estimation.metrics import compute_all_metrics, print_metrics
from state_estimation.visualization import plot_trajectory, plot_states
from state_estimation.common import make_residual_fn

# Import trajectory generators
from trajectory_generators import (generate_synthetic_data, generate_square_trajectory, 
                                   generate_circular_trajectory, load_experimental_data)

# ============================================================================
# CONFIGURATION - Data Paths
# ============================================================================
USE_EXPERIMENTAL_DATA = True  # Set to True to use real data from experiments
EXPERIMENT_NUMBER = 1  # Which experiment to use (1-10)

# Base directory (State Estimation_Python folder)
BASE_DIR = Path(__file__).parent.parent.parent

# Input data paths
SENSORS_DATA_PATH = BASE_DIR / 'data' / 'sensors' / f'exp{EXPERIMENT_NUMBER}.txt'
TRAJECTORY_DATA_PATH = BASE_DIR / 'data' / 'processed' / 'trajectories' / f'traj_vid_{EXPERIMENT_NUMBER}.csv'

# Output results path
RESULTS_PATH = BASE_DIR / 'results' / 'estimation' / 'ekf'

# Data loading parameters
N_POINTS = 1000  # Number of data points to load
DT = 0.01  # Time step in seconds
# ============================================================================


def run_ekf_example():
    """Run EKF example with synthetic data."""
    
    print("\n" + "="*60)
    print("Extended Kalman Filter Example - Omnidirectional Robot")
    print("="*60 + "\n")
    
    # Generate or load data
    if USE_EXPERIMENTAL_DATA:
        print(f"Using experimental data from experiment {EXPERIMENT_NUMBER}...")
        data = load_experimental_data(
            sensors_path=SENSORS_DATA_PATH,
            trajectory_path=TRAJECTORY_DATA_PATH,
            N=N_POINTS,
            dt=DT, filter_measurements=False
        )
    else:
        print("Using synthetic data...")
        measure_noise_std = np.array([0.025, 0.025, 0.05, 0.035])  # [vx, vy, omega, psi]
        process_noise_std = np.array([0.01, 0.01, 0.005, 0.1, 0.1, 0.05])  # [x, y, psi, vx, vy, omega]
        data = generate_synthetic_data(N=1000, dt=0.01, measurement_noise_std=measure_noise_std, process_noise_std=process_noise_std)
    
    time = data['time']
    controls = data['controls']
    measurements = data['measurements']
    ground_truth = data['ground_truth']
    dt = data['dt']
    
    N = len(time)
    
    # Initialize robot model
    robot = OmnidirectionalRobot(dt=dt)
    
    # Create EKF
    print("Initializing Extended Kalman Filter...")
    ekf = ExtendedKalmanFilter(dim_x=6, dim_z=4, dim_u=2)  # dim_u=2 for [ax, ay]
    
    # Initial state (with some error)
    ekf.x = ground_truth[0]
    
    # Initial covariance
    ekf.P = np.diag([0.5, 0.5, 0.1, 0.2, 0.2, 0.05])
    
    # Process noise covariance (tuned)
    # ekf.Q = np.diag([1e-4, 1e-4, 1e-5, 5e-3, 5e-3, 5e-4])
    # ekf.Q = np.diag([2.605017e-08, 2.368700e-08, 1.000000e-12, 1.041967e-03, 9.474402e-04, 1.000000e-12])
    # ekf.Q = np.diag([ 2.496265e-08,2.477452e-08, 1.000000e-12, 9.984660e-04, 9.909408e-04, 1.000000e-12])
    # ekf.Q = np.diag([5.871770e-06, 5.434666e-06, 1.000000e-12, 2.348707e-01, 2.173866e-01, 1.000000e-12])
    ekf.Q = np.diag([1.228769e-08, 1.241803e-08, 1.000000e-12, 4.914677e-04, 4.966813e-04,  1.000000e-12])
    
    # Measurement noise covariance
    # ekf.R = np.diag([6.72e-4, 6.72e-4, 1.3125e-2, 1.218e-3])  # Scaled for EKF stability
    ekf.R = np.diag([6.72e-4, 6.72e-4, 1.3125e-2, 4.06e-6])  # Mejor
    # ekf.R = np.diag([1.61e-4, 1.61e-4, 4.06e-6, 4.06e-6]) # peor
    
    # Set residual function to handle angle wrapping
    ekf.set_residual_fn(make_residual_fn(angle_indices=[2]))  # psi is at index 2
    
    # Storage for estimates and covariances
    estimates = np.zeros((N, 6))
    covariances = np.zeros((N, 6, 6))
    estimates[0] = ekf.x
    covariances[0] = ekf.P
    
    # Run EKF
    print("Running EKF...")
    for k in range(N - 1):
        # Predict
        ekf.predict(u=controls[k], f=robot.dynamics, F=robot.jacobian_F)
        
        # Update
        ekf.update(z=measurements[k + 1], 
                  h=robot.measurement, 
                  H=robot.jacobian_H)
        
        # Store results
        estimates[k + 1] = ekf.x
        covariances[k + 1] = ekf.P
        
        # Progress
        if (k + 1) % 200 == 0:
            print(f"  Processed {k+1}/{N-1} steps...")
    
    print("EKF complete!\n")
    
    # Compute metrics
    print("Computing performance metrics...")
    metrics = compute_all_metrics(
        estimates=estimates,
        ground_truth=ground_truth,
        covariances=covariances
    )
    
    print_metrics(metrics, filter_name="EKF")
    
    # Visualization
    print("\nGenerating plots...")
    
    # Create results directory
    results_dir = RESULTS_PATH
    results_dir.mkdir(parents=True, exist_ok=True)
    
    # IEEE format settings
    plt.rcParams.update({
        'font.family': 'serif',
        'font.serif': ['Times New Roman'],
        'font.size': 10,
        'axes.labelsize': 11,
        'axes.titlesize': 11,
        'legend.fontsize': 9,
        'xtick.labelsize': 9,
        'ytick.labelsize': 9,
        'figure.dpi': 300,
        'savefig.dpi': 300,
        'text.usetex': False,
        'axes.grid': True,
        'grid.alpha': 0.3,
        'grid.linestyle': '--',
        'axes.linewidth': 0.8,
        'lines.linewidth': 1.5
    })
    
    # Plot trajectory
    fig1, ax1 = plt.subplots(figsize=(7.16, 5.5))
    ax1.plot(ground_truth[:, 0], ground_truth[:, 1], 'k-', 
            linewidth=2, label='Trayectoria Real', alpha=0.7)
    ax1.plot(estimates[:, 0], estimates[:, 1], 'b--',
            linewidth=1.5, label='Estimación EKF', alpha=0.8)
    ax1.plot(ground_truth[0, 0], ground_truth[0, 1], 'go', 
            markersize=10, label='Inicio', zorder=5)
    ax1.plot(ground_truth[-1, 0], ground_truth[-1, 1], 'rs', 
            markersize=10, label='Fin', zorder=5)
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.legend(loc='best', framealpha=0.9)
    ax1.grid(True)
    ax1.axis('equal')
    plt.tight_layout()
    
    fig1_path = results_dir / 'ekf_trayectoria_2d.png'
    fig1.savefig(fig1_path, dpi=300, bbox_inches='tight')
    print(f"  Saved: {fig1_path}")
    
    # Plot all states (2x3 layout: positions/angles on top, velocities on bottom)
    state_labels = ['X [m]', 'Y [m]', '$\\psi$ [rad]', '$v_x$ [m/s]', '$v_y$ [m/s]', '$\\omega$ [rad/s]']
    
    fig2, axes = plt.subplots(2, 3, figsize=(7.16, 5.5))
    
    # Indices for each subplot: [x, y, phi] on top, [vx, vy, omega] on bottom
    state_indices = [0, 1, 2,  # Top row
                     3, 4, 5]  # Bottom row
    
    for plot_idx, state_idx in enumerate(state_indices):
        row = plot_idx // 3
        col = plot_idx % 3
        ax = axes[row, col]
        
        ax.plot(time, ground_truth[:, state_idx], 'k-', linewidth=2, 
                label='Real', alpha=0.7)
        ax.plot(time, estimates[:, state_idx], 'b--', linewidth=1.5, 
                label='EKF', alpha=0.8)
        ax.set_ylabel(state_labels[state_idx])
        
        # X-label only on bottom row
        if row == 1:
            ax.set_xlabel('Tiempo [s]')
        
        ax.legend(loc='best', framealpha=0.9, fontsize=8)
        ax.grid(True)
    
    plt.tight_layout()
    fig2_path = results_dir / 'ekf_estados_temporales.png'
    fig2.savefig(fig2_path, dpi=300, bbox_inches='tight')
    print(f"  Saved: {fig2_path}")
    
    # Plot position error
    fig3, axes = plt.subplots(2, 1, figsize=(7.16, 5.5))
    
    # Position error
    pos_error = np.sqrt((estimates[:, 0] - ground_truth[:, 0])**2 + 
                       (estimates[:, 1] - ground_truth[:, 1])**2)
    axes[0].plot(time, pos_error, 'b-', linewidth=1.5)
    axes[0].set_ylabel('Error de Posición [m]')
    axes[0].grid(True)
    
    # Heading error
    heading_error = np.abs(estimates[:, 2] - ground_truth[:, 2])
    heading_error = np.minimum(heading_error, 2*np.pi - heading_error)  # Wrap
    axes[1].plot(time, heading_error, 'r-', linewidth=1.5)
    axes[1].set_xlabel('Tiempo [s]')
    axes[1].set_ylabel('Error de Orientación [rad]')
    axes[1].grid(True)
    
    plt.tight_layout()
    fig3_path = results_dir / 'ekf_errores.png'
    fig3.savefig(fig3_path, dpi=300, bbox_inches='tight')
    print(f"  Saved: {fig3_path}")
    
    # Export metrics to CSV
    import csv
    metrics_path = results_dir / 'ekf_metricas.csv'
    with open(metrics_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Métrica', 'Valor'])
        writer.writerow(['RMSE Total', f"{metrics['rmse_total']:.6f}"])
        writer.writerow(['RMSE X [m]', f"{metrics['rmse'][0]:.6f}"])
        writer.writerow(['RMSE Y [m]', f"{metrics['rmse'][1]:.6f}"])
        writer.writerow(['RMSE Phi [rad]', f"{metrics['rmse'][2]:.6f}"])
        writer.writerow(['RMSE Vx [m/s]', f"{metrics['rmse'][3]:.6f}"])
        writer.writerow(['RMSE Vy [m/s]', f"{metrics['rmse'][4]:.6f}"])
        writer.writerow(['RMSE Omega [rad/s]', f"{metrics['rmse'][5]:.6f}"])
        writer.writerow(['MAE Total', f"{metrics['mae_total']:.6f}"])
        writer.writerow(['MAE X [m]', f"{metrics['mae'][0]:.6f}"])
        writer.writerow(['MAE Y [m]', f"{metrics['mae'][1]:.6f}"])
        writer.writerow(['MAE Phi [rad]', f"{metrics['mae'][2]:.6f}"])
        writer.writerow(['MAE Vx [m/s]', f"{metrics['mae'][3]:.6f}"])
        writer.writerow(['MAE Vy [m/s]', f"{metrics['mae'][4]:.6f}"])
        writer.writerow(['MAE Omega [rad/s]', f"{metrics['mae'][5]:.6f}"])
        if 'nees_mean' in metrics:
            writer.writerow(['NEES Promedio', f"{metrics['nees_mean']:.6f}"])
            writer.writerow(['NEES Desv. Est.', f"{metrics['nees_std']:.6f}"])
        if 'nis_mean' in metrics:
            writer.writerow(['NIS Promedio', f"{metrics['nis_mean']:.6f}"])
            writer.writerow(['NIS Desv. Est.', f"{metrics['nis_std']:.6f}"])
    print(f"  Saved: {metrics_path}")
    
    # Reset to defaults
    plt.rcParams.update(plt.rcParamsDefault)
    
    print("\n" + "="*60)
    print("EKF Example Complete!")
    print(f"Results saved to '{results_dir}' directory")
    print("="*60)


if __name__ == "__main__":
    run_ekf_example()
