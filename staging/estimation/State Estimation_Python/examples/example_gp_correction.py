"""
Example: Gaussian Process IMU Correction with State Estimation

This comprehensive example demonstrates:
1. Training a GP to learn IMU systematic errors
2. Applying GP correction in EKF prediction step
3. Comparing filter performance with/without GP correction
4. Integration into the state estimation pipeline

Usage:
------
    python example_gp_correction.py

Author: Robotics Thesis Project
Date: January 2026
"""

import sys
import os
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import csv

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from state_estimation import ExtendedKalmanFilter
from state_estimation.models import OmnidirectionalRobot
from state_estimation.models.gp_imu_correction import (
    IMUAccelerationGP,
    visualize_gp_predictions,
    compute_correction_metrics,
    print_correction_metrics
)
from state_estimation.models.compute_process_noise_Q import (
    compute_kinematic_accelerations,
    load_experimental_data
)
from state_estimation.metrics import compute_all_metrics, print_metrics
from state_estimation.common import make_residual_fn


def load_trajectory_data(trajectory_file, N):
    """
    Load complete state trajectory from video tracking.
    
    Expected format: CSV with columns [time, x, y, phi, vx, vy, omega, u1, u2, u3]
    
    Parameters
    ----------
    trajectory_file : Path
        Path to trajectory CSV file
    N : int
        Number of points to load
    
    Returns
    -------
    states : np.ndarray, shape (N, 6)
        Complete state trajectory [x, y, phi, vx, vy, omega]
    time : np.ndarray, shape (N,)
        Time vector
    """
    data = np.loadtxt(trajectory_file, delimiter=',', skiprows=1)
    
    if N is not None and N < len(data):
        data = data[:N, :]
    
    # Extract: time, x, y, phi, vx, vy, omega
    time = data[:, 0]
    states = data[:, 1:7]  # columns 1-6: x, y, phi, vx, vy, omega
    states[:, 2] = states[:, 2] - states[0, 2]  # Normalize phi to start at 0
    
    return states, time



def run_ekf_with_gp(states, controls, measurements, gp_model, robot, dt, Q, R):
    """
    Run EKF with GP-corrected IMU accelerations.
    
    Parameters
    ----------
    states : np.ndarray, shape (N, 6)
        Ground truth states
    controls : np.ndarray, shape (N, 2)
        Raw IMU accelerations [ax_imu, ay_imu]
    measurements : np.ndarray, shape (N, 4)
        Measurements [vx_b, vy_b, omega, psi]
    gp_model : IMUAccelerationGP
        Trained GP correction model
    robot : OmnidirectionalRobot
        Robot dynamics model
    dt : float
        Time step
    Q : np.ndarray, shape (6, 6)
        Process noise covariance
    R : np.ndarray, shape (4, 4)
        Measurement noise covariance
    
    Returns
    -------
    estimates : np.ndarray, shape (N, 6)
        State estimates
    covariances : np.ndarray, shape (N, 6, 6)
        State covariances
    """
    N = len(states)
    
    # Initialize EKF
    ekf = ExtendedKalmanFilter(dim_x=6, dim_z=4, dim_u=2)
    ekf.x = states[0]
    ekf.P = np.diag([0.5, 0.5, 0.1, 0.2, 0.2, 0.05])
    ekf.Q = Q
    ekf.R = R
    ekf.set_residual_fn(make_residual_fn(angle_indices=[2]))
    
    # Storage
    estimates = np.zeros((N, 6))
    covariances = np.zeros((N, 6, 6))
    estimates[0] = ekf.x
    covariances[0] = ekf.P
    
    # Run filter
    for k in range(N - 1):
        # Apply GP correction to control input
        u_corrected = gp_model.correct_imu(ekf.x.reshape(1, -1), 
                                           controls[k].reshape(1, -1))
        u_corrected = u_corrected.flatten()
        
        # Predict with corrected accelerations
        ekf.predict(u=u_corrected, f=robot.dynamics_2, F=robot.jacobian_F_2)
        
        # Update
        ekf.update(z=measurements[k + 1], h=robot.measurement_2, H=robot.jacobian_H_2)
        
        # Store
        estimates[k + 1] = ekf.x
        covariances[k + 1] = ekf.P
    
    return estimates, covariances


def main():
    """
    Main example: Train GP and evaluate in EKF.
    """
    
    print("\n" + "="*70)
    print("EXAMPLE: GP-BASED IMU CORRECTION FOR STATE ESTIMATION")
    print("="*70 + "\n")
    
    # ========================================================================
    # Configuration
    # ========================================================================
    BASE_DIR = Path(__file__).parent.parent.parent
    SENSORS_DIR = BASE_DIR / 'data' / 'sensors'
    TRAJECTORY_DIR = BASE_DIR / 'data' / 'processed' / 'trajectories'
    RESULTS_DIR = BASE_DIR / 'results' / 'estimation' / 'gp_correction'
    
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    
    # Robot parameters
    robot_params = {
        'wheel_angles': [150, 270, 30],
        'd': 0.08,
        'r': 0.025
    }
    
    # Training experiment
    TRAIN_EXP = 2
    N_TRAIN = 1000
    
    # Test experiment (can be same or different)
    TEST_EXP = 3
    N_TEST = 1000
    
    # ========================================================================
    # PART 1: Train GP from experimental data
    # ========================================================================
    print("PART 1: Train GP from experimental data")
    print("-" * 70)
    
    # Load training data
    print(f"\n1.1 Loading training data (Experiment {TRAIN_EXP})...")
    train_sensor_data = load_experimental_data(
        SENSORS_DIR / f'exp{TRAIN_EXP}.txt', N=N_TRAIN
    )
    train_states, time_train = load_trajectory_data(
        TRAJECTORY_DIR / f'traj_vid_{TRAIN_EXP}.csv', N=N_TRAIN
    )
    
    dt = np.mean(np.diff(time_train))
    print(f"   Loaded {N_TRAIN} samples, dt = {dt:.4f} s")
    
    # Compute kinematic accelerations for GP training
    print("\n1.2 Computing kinematic reference accelerations...")
    wheel_velocities = np.column_stack([
        train_sensor_data['w1'], 
        train_sensor_data['w2'], 
        train_sensor_data['w3']
    ])
    
    ax_kin, ay_kin, _, _ = compute_kinematic_accelerations(
        wheel_velocities, robot_params['wheel_angles'],
        robot_params['d'], robot_params['r'], dt
    )
    
    kinematic_accel = np.column_stack([ax_kin, ay_kin])
    imu_accel = np.column_stack([train_sensor_data['ax_imu'], 
                                 train_sensor_data['ay_imu']])
    
    # Train GP
    print("\n1.3 Training Gaussian Process...")
    gp_model = IMUAccelerationGP(normalize_inputs=True)
    training_stats = gp_model.train(train_states, imu_accel, kinematic_accel, 
                                    verbose=True)
    
    # Save GP model
    gp_path = RESULTS_DIR / f'gp_model_exp{TRAIN_EXP}.pkl'
    gp_model.save(gp_path)
    
    # Visualize training results
    print("\n1.4 Visualizing GP training results...")
    fig_train = visualize_gp_predictions(
        train_states, imu_accel, kinematic_accel, gp_model,
        save_path=RESULTS_DIR / f'gp_training_exp{TRAIN_EXP}.png'
    )
    print(f"   Saved: {RESULTS_DIR / f'gp_training_exp{TRAIN_EXP}.png'}")
    
    # Compute training metrics
    corrected_accel_train = gp_model.correct_imu(train_states, imu_accel)
    metrics_train = compute_correction_metrics(imu_accel, kinematic_accel, 
                                               corrected_accel_train)
    print_correction_metrics(metrics_train)
    
    # ========================================================================
    # PART 2: Test GP on state estimation (EKF)
    # ========================================================================
    print("\n\nPART 2: Evaluate GP correction in EKF")
    print("-" * 70)
    
    # Load test data
    print(f"\n2.1 Loading test data (Experiment {TEST_EXP})...")
    test_sensor_data = load_experimental_data(
        SENSORS_DIR / f'exp{TEST_EXP}.txt', N=N_TEST
    )
    test_states, time_test = load_trajectory_data(
        TRAJECTORY_DIR / f'traj_vid_{TEST_EXP}.csv', N=N_TEST
    )
    
    # Prepare data for EKF
    test_controls = np.column_stack([test_sensor_data['ax_imu'], 
                                     test_sensor_data['ay_imu']])
    
    # Compute body velocities for measurements (from wheel encoders)
    wheel_vel_test = np.column_stack([test_sensor_data['w1'], 
                                      test_sensor_data['w2'], 
                                      test_sensor_data['w3']])
    _, _, vx_b, vy_b = compute_kinematic_accelerations(
        wheel_vel_test, robot_params['wheel_angles'],
        robot_params['d'], robot_params['r'], dt
    )
    
    # Use omega and phi from trajectory states
    omega_meas = test_states[:, 5]  # omega from video tracking
    phi_meas = test_states[:, 2]    # phi from video tracking
    
    test_measurements = np.column_stack([vx_b, vy_b, omega_meas, phi_meas])
    
    # Initialize robot model
    robot = OmnidirectionalRobot(dt=dt, **robot_params)
    
    # Noise covariances (you can use Q from compute_process_noise_Q.py)
    Q = np.diag([1e-4, 1e-4, 1e-5, 5e-3, 5e-3, 5e-4]) * 1e2
    R = np.diag([6.72e-4, 6.72e-4, 1.3125e-2, 1.218e-3]) * 1e3
    
    # ========================================================================
    # Run EKF WITHOUT GP correction (baseline)
    # ========================================================================
    print("\n2.2 Running EKF WITHOUT GP correction (baseline)...")
    
    ekf_baseline = ExtendedKalmanFilter(dim_x=6, dim_z=4, dim_u=2)
    ekf_baseline.x = test_states[0]
    ekf_baseline.P = np.diag([0.5, 0.5, 0.1, 0.2, 0.2, 0.05])
    ekf_baseline.Q = Q
    ekf_baseline.R = R
    ekf_baseline.set_residual_fn(make_residual_fn(angle_indices=[2]))
    
    estimates_baseline = np.zeros((N_TEST, 6))
    covariances_baseline = np.zeros((N_TEST, 6, 6))
    estimates_baseline[0] = ekf_baseline.x
    covariances_baseline[0] = ekf_baseline.P
    
    for k in range(N_TEST - 1):
        ekf_baseline.predict(u=test_controls[k], f=robot.dynamics_2, 
                            F=robot.jacobian_F_2)
        ekf_baseline.update(z=test_measurements[k + 1], h=robot.measurement_2, 
                           H=robot.jacobian_H_2)
        estimates_baseline[k + 1] = ekf_baseline.x
        covariances_baseline[k + 1] = ekf_baseline.P
    
    print("   Baseline EKF complete.")
    
    # ========================================================================
    # Run EKF WITH GP correction
    # ========================================================================
    print("\n2.3 Running EKF WITH GP correction...")
    
    estimates_gp, covariances_gp = run_ekf_with_gp(
        test_states, test_controls, test_measurements, gp_model, 
        robot, dt, Q, R
    )
    
    print("   GP-corrected EKF complete.")
    
    # ========================================================================
    # PART 3: Compare performance
    # ========================================================================
    print("\n\nPART 3: Performance comparison")
    print("-" * 70)
    
    # Compute metrics
    metrics_baseline = compute_all_metrics(
        estimates_baseline, test_states, covariances_baseline
    )
    
    metrics_gp = compute_all_metrics(
        estimates_gp, test_states, covariances_gp
    )
    
    print("\n--- BASELINE EKF (No GP) ---")
    print_metrics(metrics_baseline, filter_name="EKF Baseline")
    
    print("\n--- GP-CORRECTED EKF ---")
    print_metrics(metrics_gp, filter_name="EKF + GP")
    
    # Improvement
    print("\n--- IMPROVEMENT ---")
    # Compute position RMSE manually (x, y are indices 0, 1)
    pos_rmse_baseline = np.sqrt(metrics_baseline['rmse'][0]**2 + metrics_baseline['rmse'][1]**2)
    pos_rmse_gp = np.sqrt(metrics_gp['rmse'][0]**2 + metrics_gp['rmse'][1]**2)
    heading_rmse_baseline = metrics_baseline['rmse'][2]
    heading_rmse_gp = metrics_gp['rmse'][2]
    
    print(f"Position RMSE improvement: {(pos_rmse_baseline - pos_rmse_gp) / pos_rmse_baseline * 100:.1f}%")
    print(f"Heading RMSE improvement: {(heading_rmse_baseline - heading_rmse_gp) / heading_rmse_baseline * 100:.1f}%")
    
    # ========================================================================
    # PART 4: Visualization
    # ========================================================================
    print("\n\nPART 4: Generating comparison plots...")
    print("-" * 70)
    
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
    
    # Trajectory comparison
    fig1, ax = plt.subplots(figsize=(7.16, 5.5))
    ax.plot(test_states[:, 0], test_states[:, 1], 'k-', linewidth=2, 
            label='Trayectoria Real', alpha=0.7)
    ax.plot(estimates_baseline[:, 0], estimates_baseline[:, 1], 'r--', 
            linewidth=1.5, label='EKF (Sin GP)', alpha=0.8)
    ax.plot(estimates_gp[:, 0], estimates_gp[:, 1], 'b--', 
            linewidth=1.5, label='EKF (Con GP)', alpha=0.8)
    ax.plot(test_states[0, 0], test_states[0, 1], 'go', markersize=10, 
            label='Inicio', zorder=5)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.legend(loc='best', framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    plt.tight_layout()
    
    fig1_path = RESULTS_DIR / 'comparison_trajectory.png'
    fig1.savefig(fig1_path, dpi=300, bbox_inches='tight')
    print(f"   Saved: {fig1_path}")
    
    # Position error comparison
    time = np.arange(N_TEST) * dt
    
    pos_error_baseline = np.sqrt((estimates_baseline[:, 0] - test_states[:, 0])**2 + 
                                 (estimates_baseline[:, 1] - test_states[:, 1])**2)
    pos_error_gp = np.sqrt((estimates_gp[:, 0] - test_states[:, 0])**2 + 
                           (estimates_gp[:, 1] - test_states[:, 1])**2)
    
    fig2, axes = plt.subplots(2, 1, figsize=(7.16, 5.5))
    
    axes[0].plot(time, pos_error_baseline, 'r-', linewidth=1.5, 
                label='EKF (Sin GP)', alpha=0.7)
    axes[0].plot(time, pos_error_gp, 'b-', linewidth=1.5, 
                label='EKF (Con GP)', alpha=0.7)
    axes[0].set_ylabel('Error de Posición [m]')
    axes[0].legend(loc='best', framealpha=0.9)
    axes[0].grid(True, alpha=0.3)
    
    heading_error_baseline = np.abs(estimates_baseline[:, 2] - test_states[:, 2])
    heading_error_baseline = np.minimum(heading_error_baseline, 
                                       2*np.pi - heading_error_baseline)
    heading_error_gp = np.abs(estimates_gp[:, 2] - test_states[:, 2])
    heading_error_gp = np.minimum(heading_error_gp, 2*np.pi - heading_error_gp)
    
    axes[1].plot(time, heading_error_baseline, 'r-', linewidth=1.5, 
                label='EKF (Sin GP)', alpha=0.7)
    axes[1].plot(time, heading_error_gp, 'b-', linewidth=1.5, 
                label='EKF (Con GP)', alpha=0.7)
    axes[1].set_xlabel('Tiempo [s]')
    axes[1].set_ylabel('Error de Orientación [rad]')
    axes[1].legend(loc='best', framealpha=0.9)
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    fig2_path = RESULTS_DIR / 'comparison_errors.png'
    fig2.savefig(fig2_path, dpi=300, bbox_inches='tight')
    print(f"   Saved: {fig2_path}")
    
    # Save comparison metrics to CSV
    metrics_csv = RESULTS_DIR / 'comparison_metrics.csv'
    with open(metrics_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Metric', 'Baseline', 'GP-Corrected', 'Improvement %'])
        writer.writerow(['Position RMSE [m]', 
                        f"{pos_rmse_baseline:.4f}",
                        f"{pos_rmse_gp:.4f}",
                        f"{(pos_rmse_baseline - pos_rmse_gp) / pos_rmse_baseline * 100:.1f}"])
        writer.writerow(['Heading RMSE [rad]', 
                        f"{heading_rmse_baseline:.4f}",
                        f"{heading_rmse_gp:.4f}",
                        f"{(heading_rmse_baseline - heading_rmse_gp) / heading_rmse_baseline * 100:.1f}"])
    print(f"   Saved: {metrics_csv}")
    
    # Reset matplotlib
    plt.rcParams.update(plt.rcParamsDefault)
    
    print("\n" + "="*70)
    print("EXAMPLE COMPLETE")
    print(f"Results saved to: {RESULTS_DIR}")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
