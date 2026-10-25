"""
UKF Example for Omnidirectional Robot

Demonstrates the Unscented Kalman Filter with sigma points.
No Jacobians required!
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from pathlib import Path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from state_estimation import UnscentedKalmanFilter, MerweScaledSigmaPoints
from state_estimation.models import OmnidirectionalRobot
from state_estimation.metrics import compute_all_metrics, print_metrics
from state_estimation.visualization import plot_trajectory, plot_states
from state_estimation.common.angles import circular_mean, normalize_angle


# Import trajectory generators
from trajectory_generators import (generate_synthetic_data, generate_square_trajectory, 
                                   generate_circular_trajectory, load_experimental_data)

# ============================================================================
# CONFIGURATION - Data Paths
# ============================================================================
USE_EXPERIMENTAL_DATA = True  # Set to True to use real data from experiments  
EXPERIMENT_NUMBER = 2  # Which experiment to use (1-10)

# Base directory (State Estimation_Python folder)
BASE_DIR = Path(__file__).parent.parent.parent

# Input data paths
SENSORS_DATA_PATH = BASE_DIR / 'data' / 'sensors' / f'exp{EXPERIMENT_NUMBER}.txt'
TRAJECTORY_DATA_PATH = BASE_DIR / 'data' / 'processed' / 'trajectories' / f'traj_vid_{EXPERIMENT_NUMBER}.csv'

# Output results path
RESULTS_PATH = BASE_DIR / 'results' / 'estimation' / 'ukf'

# Data loading parameters
N_POINTS = 1000  # Number of data points to load
DT = 0.01  # Time step in seconds
# ============================================================================


def state_mean_fn(sigmas, Wm):
    """Custom mean function for states with angles."""
    x = np.zeros(6)
    x[0] = np.dot(Wm, sigmas[:, 0])  # x (linear)
    x[1] = np.dot(Wm, sigmas[:, 1])  # y (linear)
    x[2] = circular_mean(sigmas[:, 2], Wm)  # psi (circular)
    x[3] = np.dot(Wm, sigmas[:, 3])  # vx_b (linear)
    x[4] = np.dot(Wm, sigmas[:, 4])  # vy_b (linear)
    x[5] = np.dot(Wm, sigmas[:, 5])  # omega (linear)
    return x


def z_mean_fn(sigmas, Wm):
    """Custom mean function for measurements with angles."""
    z = np.zeros(4)
    z[0] = np.dot(Wm, sigmas[:, 0])  # vx_b
    z[1] = np.dot(Wm, sigmas[:, 1])  # vy_b
    z[2] = np.dot(Wm, sigmas[:, 2])  # omega
    z[3] = circular_mean(sigmas[:, 3], Wm)  # psi (circular)
    return z


def residual_x(a, b):
    """Residual function for state."""
    y = a - b
    y[2] = normalize_angle(y[2])  # psi
    return y


def residual_z(a, b):
    """Residual function for measurement."""
    y = a - b
    y[3] = normalize_angle(y[3])  # psi
    return y


def run_ukf_example():
    """Run UKF example with synthetic data."""
    
    print("\n" + "="*60)
    print("Unscented Kalman Filter Example - Omnidirectional Robot")
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
        data = generate_synthetic_data(N=1000, dt=0.01)
    
    time = data['time']
    controls = data['controls']
    measurements = data['measurements']
    ground_truth = data['ground_truth']
    dt = data['dt']
    
    N = len(time)
    
    # Initialize robot model
    robot = OmnidirectionalRobot(dt=dt)
    
    # Create sigma points
    print("Creating sigma points...")
    points = MerweScaledSigmaPoints(n=6, alpha=0.5, beta=2.0, kappa=0.0)
    
    # Create UKF
    print("Initializing Unscented Kalman Filter...")
    ukf = UnscentedKalmanFilter(dim_x=6, dim_z=4, dim_u=2, points=points)  # dim_u=2 for [ax, ay]
    
    # Initial state
    ukf.x = ground_truth[0]
    ukf.P = np.diag([0.5, 0.5, 0.1, 0.2, 0.2, 0.05])
    ukf.Q = np.diag([1.228769e-08, 1.241803e-08, 1.000000e-12, 4.914677e-04, 4.966813e-04,  1.000000e-12])*1e4
    ukf.R = np.diag([6.72e-4, 6.72e-4, 1.3125e-2, 4.06e-6])  # Scaled for EKF stability
    # ukf.Q = np.diag([1e-4, 1e-4, 1e-5, 5e-3, 5e-3, 5e-4])

    # ukf.R = np.diag([6.72e-4, 6.72e-4, 1.3125e-2, 1.218e-3])*1e3  # Scaled for EKF stability
    
    # Set custom mean and residual functions
    ukf.set_mean_fn(x_mean_fn=state_mean_fn, z_mean_fn=z_mean_fn)
    ukf.set_residual_fn(residual_x_fn=residual_x, residual_z_fn=residual_z)
    
    # Storage
    estimates = np.zeros((N, 6))
    covariances = np.zeros((N, 6, 6))
    estimates[0] = ukf.x
    covariances[0] = ukf.P
    
    # Run UKF
    print("Running UKF...")
    for k in range(N - 1):
        ukf.predict(u=controls[k], f=robot.dynamics)
        ukf.update(z=measurements[k + 1], h=robot.measurement)
        
        estimates[k + 1] = ukf.x
        covariances[k + 1] = ukf.P
        
        if (k + 1) % 200 == 0:
            print(f"  Processed {k+1}/{N-1} steps...")
    
    print("UKF complete!\n")
    
    # Metrics
    metrics = compute_all_metrics(estimates, ground_truth, covariances)
    print_metrics(metrics, filter_name="UKF")
    
    # Plots
    print("\nGenerating plots...")
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
            linewidth=1.5, label='Estimación UKF', alpha=0.8)
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
    
    fig1_path = results_dir / 'ukf_trayectoria_2d.png'
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
                label='UKF', alpha=0.8)
        ax.set_ylabel(state_labels[state_idx])
        
        # X-label only on bottom row
        if row == 1:
            ax.set_xlabel('Tiempo [s]')
        
        ax.legend(loc='best', framealpha=0.9, fontsize=8)
        ax.grid(True)
    
    plt.tight_layout()
    fig2_path = results_dir / 'ukf_estados_temporales.png'
    fig2.savefig(fig2_path, dpi=300, bbox_inches='tight')
    print(f"  Saved: {fig2_path}")
    
    # Export metrics to CSV
    import csv
    metrics_path = results_dir / 'ukf_metricas.csv'
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
    print("UKF Example Complete!")
    print(f"Results saved to '{results_dir}' directory")
    print("="*60)


if __name__ == "__main__":
    run_ukf_example()
