"""
Trajectory Generators for State Estimation Testing

This module contains functions to generate different trajectory types for testing
state estimation algorithms (EKF, UKF, PF) with omnidirectional robots.

All generators produce consistent output format:
    - time: array of timestamps
    - controls: array of control inputs [ax_b, ay_b, omega_meas]
    - measurements: array of noisy measurements [vx_b, vy_b, omega, psi]
    - ground_truth: array of true states [x, y, psi, vx_b, vy_b, omega]
    - dt: time step
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys
import os
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from state_estimation.models import OmnidirectionalRobot
from state_estimation.utils.signal_processing import Kalman1D


def load_experimental_data(sensors_path, trajectory_path, N=1000, dt=0.01, filter_measurements=False):
    """
    Load experimental data from sensor file and trajectory ground truth file.
    
    Parameters
    ----------
    sensors_path : str or Path
        Path to sensor data file (expN.txt) containing:
        - controls: ax, ay (columns 1, 2)
        - for measurements: w1, w2, w3 (columns 4, 5, 6) and alpha (column 3)
    trajectory_path : str or Path
        Path to trajectory ground truth file (traj_vid_N.csv) containing:
        - ground_truth: x, y, phi, vx, vy, omega
    N : int, optional
        Number of data points to load (default: 1000)
    dt : float, optional
        Time step in seconds (default: 0.01)
    filter_measurements : bool, optional
        Whether to apply 1D Kalman filtering to raw measurements (default: False)
        Filters: ax, ay, alpha, w1, w2, w3
    
    Returns
    -------
    dict
        Dictionary containing:
        - time: array (N,) - timestamps
        - controls: array (N, 2) - [ax, ay] from IMU
        - measurements: array (N, 4) - [vx_b, vy_b, omega_b, psi] 
        - ground_truth: array (N, 6) - [x, y, psi, vx, vy, omega]
        - dt: float - time step
    """
    print(f"Loading experimental data...")
    print(f"  Sensors: {Path(sensors_path).name}")
    print(f"  Trajectory: {Path(trajectory_path).name}")
    
    # Load sensor data (expN.txt)
    # Columns: t, ax, ay, alpha, w1, w2, w3, u1, u2, u3, vbx_sp, vby_sp, wb_sp
    sensor_data = np.loadtxt(sensors_path, delimiter=',', skiprows=1, max_rows=N)

    # Process alpha measurement: invert direction (wrapped between 0 and 2π)
    # Alpha decreases when it should increase, so we invert it
    sensor_data[:, 3] = 2 * np.pi - sensor_data[:, 3]
    sensor_data[:, 3] = sensor_data[:, 3] - sensor_data[:, 3][0]  # Zero at start
    sensor_data[:, 3] = np.unwrap(sensor_data[:, 3])  # Unwrap angle
    
    # Load trajectory ground truth (traj_vid_N.csv)
    # Columns: time_s, x_m, y_m, phi_rad, vx_m_s, vy_m_s, omega_rad_s, u1_pwm, u2_pwm, u3_pwm
    traj_df = pd.read_csv(trajectory_path, nrows=N)
    
    # Apply Kalman filtering if requested
    if filter_measurements:
        print(f"  Applying 1D Kalman filtering to measurements...")
        
        # Kalman filter parameters
        Q_bno, R_bno = 0.001, 0.04   # IMU (BNO055): ax, ay, alpha
        Q_enc, R_enc = 0.001, 1.12   # Encoders: w1, w2, w3
        
        # Create filters for each measurement
        kf_ax = Kalman1D(Q_bno, R_bno)
        kf_ay = Kalman1D(Q_bno, R_bno)
        kf_alpha = Kalman1D(Q_bno, R_bno)
        kf_w1 = Kalman1D(Q_enc, R_enc)
        kf_w2 = Kalman1D(Q_enc, R_enc)
        kf_w3 = Kalman1D(Q_enc, R_enc)
        
        # Filter data
        for i in range(N):
            sensor_data[i, 1] = kf_ax.update(sensor_data[i, 1])      # ax
            sensor_data[i, 2] = kf_ay.update(sensor_data[i, 2])      # ay
            sensor_data[i, 3] = kf_alpha.update(sensor_data[i, 3])   # alpha
            sensor_data[i, 4] = kf_w1.update(sensor_data[i, 4])      # w1
            sensor_data[i, 5] = kf_w2.update(sensor_data[i, 5])      # w2
            sensor_data[i, 6] = kf_w3.update(sensor_data[i, 6])      # w3
        
        print(f"  Filtering complete.")
    
    # Extract controls: [ax, ay] from sensor data (columns 1, 2)
    controls = sensor_data[:, 1:3]  # ax, ay
    
    # Compute measurements using forward kinematics
    # Need robot model for forward_kinematics
    robot = OmnidirectionalRobot(dt=dt)
    
    measurements = np.zeros((N, 4))
    for i in range(N):
        # Extract wheel speeds from sensor data (columns 4, 5, 6)
        w1 = sensor_data[i, 4]  # wheel 1 angular velocity
        w2 = sensor_data[i, 5]  # wheel 2 angular velocity
        w3 = sensor_data[i, 6]  # wheel 3 angular velocity
        
        # Compute body velocities using forward kinematics
        vx_b, vy_b, omega_b = robot.forward_kinematics(w1, w2, w3)
        
        # Extract orientation (alpha) from sensor data (column 3)
        psi = sensor_data[i, 3]  # alpha (yaw angle)
        
        measurements[i] = [vx_b, vy_b, omega_b, psi]
    
    # Extract ground truth: [x, y, psi, vx, vy, omega]
    ground_truth = np.column_stack([
        traj_df['x_m'].values,
        traj_df['y_m'].values,
        traj_df['phi_rad'].values - traj_df['phi_rad'].values[0],  # Zero at start
        traj_df['vx_m_s'].values,
        traj_df['vy_m_s'].values,
        traj_df['omega_rad_s'].values
    ])
    
    # Generate time vector
    time = np.arange(N) * dt
    
    print(f"  Loaded {N} data points, dt = {dt:.4f} s")
    print(f"  Duration: {time[-1]:.2f} s")
    print(f"  Controls range: ax=[{controls[:, 0].min():.2f}, {controls[:, 0].max():.2f}], ay=[{controls[:, 1].min():.2f}, {controls[:, 1].max():.2f}]")
    print(f"  Measurements computed from wheel speeds using forward kinematics")
    if filter_measurements:
        print(f"  Measurements filtered with 1D Kalman filter")
    
    # Debug: Plot sensor data
    DEBUG_PLOTS = False  # Set to True to see sensor and trajectory plots
    if DEBUG_PLOTS:
        import matplotlib.pyplot as plt
        
        # Figure 1: Sensor data (ax, ay, alpha, w1, w2, w3)
        fig1, axes1 = plt.subplots(2, 3, figsize=(15, 8))
        fig1.suptitle('Sensor Data (Debug)', fontsize=14, fontweight='bold')
        
        sensor_labels = ['ax [m/s²]', 'ay [m/s²]', 'alpha [rad]', 'w1 [rad/s]', 'w2 [rad/s]', 'w3 [rad/s]']
        sensor_indices = [1, 2, 3, 4, 5, 6]
        
        for idx, (ax, label, col_idx) in enumerate(zip(axes1.flatten(), sensor_labels, sensor_indices)):
            ax.plot(time, sensor_data[:, col_idx], 'b-', linewidth=1.5)
            ax.set_xlabel('Tiempo [s]')
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Figure 2: Ground truth trajectory states
        fig2, axes2 = plt.subplots(2, 3, figsize=(15, 8))
        fig2.suptitle('Ground Truth States (Debug)', fontsize=14, fontweight='bold')
        
        state_labels = ['x [m]', 'y [m]', 'phi [rad]', 'vx [m/s]', 'vy [m/s]', 'omega [rad/s]']
        
        for idx, (ax, label) in enumerate(zip(axes2.flatten(), state_labels)):
            ax.plot(time, ground_truth[:, idx], 'r-', linewidth=1.5)
            ax.set_xlabel('Tiempo [s]')
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    return {
        'time': time,
        'controls': controls,
        'measurements': measurements,
        'ground_truth': ground_truth,
        'dt': dt
    }


def generate_synthetic_data(N=1000, dt=0.01, measurement_noise_std=None, 
                           process_noise_std=None):
    """
    Generate synthetic data with a square pattern trajectory.
    
    The robot moves in a square pattern, changing direction every quarter of the trajectory.
    This is a simple trajectory with moderate nonlinearity.
    
    Parameters
    ----------
    N : int
        Number of timesteps
    dt : float
        Time step in seconds
    measurement_noise_std : array_like, optional
        Standard deviation of measurement noise [vx_b, vy_b, omega, psi].
        If None, uses default: [0.0259, 0.0259, 0.1146, 0.0349]
    process_noise_std : array_like, optional
        Standard deviation of process noise [x, y, psi, vx_b, vy_b, omega].
        If None, no process noise is added (perfect dynamics)
    
    Returns
    -------
    dict
        Dictionary containing:
        - time: (N,) array of timestamps
        - controls: (N, 2) array of control inputs [ax, ay]
        - measurements: (N, 4) array of noisy measurements [vx_b, vy_b, omega, psi]
        - ground_truth: (N, 6) array of true states [x, y, psi, vx, vy, omega]
        - dt: float, time step
        - measurement_noise_std: actual noise std used
    """
    # Default measurement noise (from sensor specs)
    if measurement_noise_std is None:
        measurement_noise_std = np.array([0.0259, 0.0259, 0.1146, 0.0349])
    else:
        measurement_noise_std = np.asarray(measurement_noise_std)
    
    # Time vector
    time = np.arange(N) * dt
    
    # Initialize true state
    x_true = np.zeros((N, 6))
    x_true[0] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Control inputs (accelerations in world frame: ax, ay)
    controls = np.zeros((N, 2))
    segment_length = N // 4
    
    for k in range(N):
        if k < segment_length:
            # Accelerate forward in x direction
            controls[k] = [0.5, 0.0]
        elif k < 2 * segment_length:
            # Accelerate in y direction
            controls[k] = [0.0, 0.5]
        elif k < 3 * segment_length:
            # Accelerate in -x direction
            controls[k] = [-0.5, 0.0]
        else:
            # Accelerate in -y direction
            controls[k] = [0.0, -0.5]
    
    # Create robot model
    robot = OmnidirectionalRobot(dt=dt)
    
    # Simulate true dynamics (using dynamics which takes u=[ax, ay])
    for k in range(N - 1):
        x_clean = robot.dynamics(x_true[k], controls[k])
        
        # Add process noise if specified
        if process_noise_std is not None:
            process_noise = np.random.randn(6) * process_noise_std
            x_true[k + 1] = x_clean + process_noise
        else:
            x_true[k + 1] = x_clean
    
    # Generate noisy measurements (using measurement)
    measurements = np.zeros((N, 4))
    R = np.diag(measurement_noise_std**2)  # Covariance matrix
    
    for k in range(N):
        z_clean = robot.measurement(x_true[k])
        noise = np.random.multivariate_normal(np.zeros(4), R)
        measurements[k] = z_clean + noise
    
    return {
        'time': time,
        'controls': controls,
        'measurements': measurements,
        'ground_truth': x_true,
        'dt': dt,
        'measurement_noise_std': measurement_noise_std
    }


def generate_square_trajectory(N=1000, dt=0.01, speed=0.5, speeds=None, 
                               corner_smoothing=0.05, measurement_noise_std=None,
                               process_noise_std=None):
    """
    Generate a square trajectory using omnidirectional motion (robot does not rotate).
    
    The robot moves in a square with 4 equal segments. Corners are smoothed using
    cosine interpolation for realistic velocity transitions. The robot uses
    omnidirectional capabilities to move in different directions without rotating.
    
    Parameters
    ----------
    N : int
        Number of timesteps
    dt : float
        Time step in seconds
    speed : float
        Default linear speed in m/s
    speeds : list of 4 floats, optional
        Speeds for each segment [right, up, left, down]. If None, uses `speed` for all
    corner_smoothing : float
        Fraction of segment duration for corner smoothing (0.0 to 0.5)
    measurement_noise_std : array_like, optional
        Standard deviation of measurement noise [vx_b, vy_b, omega, psi].
        If None, uses default: [0.0259, 0.0259, 0.1146, 0.0349]
    process_noise_std : array_like, optional
        Standard deviation of process noise [x, y, psi, vx_b, vy_b, omega].
        If None, no process noise is added
    
    Returns
    -------
    dict
        Dictionary containing time, controls, measurements, ground_truth, dt, and noise_std
    """
    print("Generating square trajectory...")
    
    # Default measurement noise
    if measurement_noise_std is None:
        measurement_noise_std = np.array([0.0259, 0.0259, 0.1146, 0.0349])
    else:
        measurement_noise_std = np.asarray(measurement_noise_std)
    
    # Time vector
    time = np.arange(N) * dt
    total_time = N * dt
    seg_steps = N // 4
    remainder = N - seg_steps * 4

    # Speeds per segment
    if speeds is None:
        speeds = [speed] * 4
    else:
        if len(speeds) != 4:
            raise ValueError("speeds must be length 4")

    # Direction vectors for square: +x, +y, -x, -y (robot does not rotate)
    dirs = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0], [0.0, -1.0]])

    # Build velocity profile in body frame (since psi=0, body==world)
    vx = np.zeros(N)
    vy = np.zeros(N)

    idx = 0
    prev_v = np.array([0.0, 0.0])
    for seg in range(4):
        # Distribute remainder across first segments
        this_seg_steps = seg_steps + (1 if seg < remainder else 0)
        v_target = np.array(dirs[seg]) * float(speeds[seg])

        # Smoothing window in steps
        smooth_steps = int(max(1, round(corner_smoothing * this_seg_steps))) if corner_smoothing > 0 else 0

        # Fill segment
        for k in range(this_seg_steps):
            global_idx = idx + k
            # Ramp at start of segment
            if k < smooth_steps:
                alpha = 0.5 * (1 - np.cos(np.pi * (k + 1) / (smooth_steps + 1)))
                v = prev_v * (1 - alpha) + v_target * alpha
            # Ramp at end of segment (soften into next segment) - handled when next segment begins
            else:
                v = v_target

            vx[global_idx] = v[0]
            vy[global_idx] = v[1]

        prev_v = v_target.copy()
        idx += this_seg_steps

    # Ensure last index filled
    if idx < N:
        vx[idx:] = prev_v[0]
        vy[idx:] = prev_v[1]

    # Controls: accelerations in world frame [ax, ay] (dim_u=2)
    ax = np.zeros(N)
    ay = np.zeros(N)

    # Use forward difference for acceleration applied at step k
    for k in range(N - 1):
        ax[k] = (vx[k + 1] - vx[k]) / dt
        ay[k] = (vy[k + 1] - vy[k]) / dt
    ax[-1] = ax[-2]
    ay[-1] = ay[-2]

    controls = np.vstack((ax, ay)).T  # Shape: (N, 2)

    # Create robot model and simulate true dynamics using dynamics
    robot = OmnidirectionalRobot(dt=dt)
    x_true = np.zeros((N, 6))
    x_true[0] = np.zeros(6)
    
    # Simulate true dynamics
    for k in range(N - 1):
        x_clean = robot.dynamics(x_true[k], controls[k])
        
        # Add process noise if specified
        if process_noise_std is not None:
            process_noise = np.random.randn(6) * process_noise_std
            x_true[k + 1] = x_clean + process_noise
        else:
            x_true[k + 1] = x_clean

    # Generate noisy measurements using measurement
    measurements = np.zeros((N, 4))
    R = np.diag(measurement_noise_std**2)
    
    for k in range(N):
        z_clean = robot.measurement(x_true[k])
        noise = np.random.multivariate_normal(np.zeros(4), R)
        measurements[k] = z_clean + noise

    print(f"Generated {N} timesteps (duration: {total_time:.2f}s)")

    return {
        'time': time,
        'controls': controls,
        'measurements': measurements,
        'ground_truth': x_true,
        'dt': dt,
        'measurement_noise_std': measurement_noise_std
    }


def generate_circular_trajectory(N=1000, dt=0.01, linear_speed=0.5, 
                                angular_velocity=0.2, rotation_mode='continuous',
                                measurement_noise_std=None, process_noise_std=None):
    """
    Generate a highly nonlinear circular trajectory with continuous rotation.
    
    This trajectory is designed to stress-test nonlinear filters by combining
    circular motion with robot rotation. The radius is automatically computed
    to complete exactly one full circle.
    
    Parameters
    ----------
    N : int
        Number of timesteps
    dt : float
        Time step in seconds
    linear_speed : float
        Forward linear speed in m/s
    angular_velocity : float
        Angular velocity in rad/s (for continuous rotation mode)
    rotation_mode : str
        Rotation behavior:
        - 'continuous': Robot rotates at constant angular_velocity (maximum nonlinearity)
        - 'tangent': Robot orientation follows path tangent (natural motion)
        - 'counter': Robot rotates opposite to path (extreme case)
    measurement_noise_std : array_like, optional
        Standard deviation of measurement noise [vx_b, vy_b, omega, psi].
        If None, uses default: [0.0259, 0.0259, 0.1146, 0.0349]
    process_noise_std : array_like, optional
        Standard deviation of process noise [x, y, psi, vx_b, vy_b, omega].
        If None, no process noise is added
    
    Returns
    -------
    dict
        Dictionary containing time, controls, measurements, ground_truth, dt, noise_std,
        and additional info (radius, angular_velocity_path)
    """
    # Default measurement noise
    if measurement_noise_std is None:
        measurement_noise_std = np.array([0.0259, 0.0259, 0.1146, 0.0349])
    else:
        measurement_noise_std = np.asarray(measurement_noise_std)
    
    # Time vector
    time = np.arange(N) * dt
    total_time = N * dt
    
    # Compute radius to complete exactly one full circle
    # Distance = linear_speed * total_time = 2 * pi * radius
    radius = (linear_speed * total_time) / (2 * np.pi)
    
    # Angular velocity of the PATH (not robot orientation)
    omega_path = linear_speed / radius  # rad/s
    
    # Initialize state
    x_true = np.zeros((N, 6))
    controls = np.zeros((N, 2))  # Only [ax, ay] now (dim_u=2)
    
    # Starting position: top of circle (x=0, y=radius)
    x_true[0] = np.array([0.0, radius, 0.0, 0.0, 0.0, 0.0])
    
    # Generate trajectory
    for k in range(N):
        t = k * dt
        
        # Circular path parametrization
        theta_path = omega_path * t  # Angle along the path
        
        # Global position
        x = radius * np.sin(theta_path)
        y = radius * np.cos(theta_path)
        
        # Global velocity (tangent to circle)
        vx_global = linear_speed * np.cos(theta_path)
        vy_global = -linear_speed * np.sin(theta_path)
        
        # Robot orientation and angular velocity (depends on mode)
        if rotation_mode == 'continuous':
            # Robot rotates at constant rate (independent of path)
            psi = angular_velocity * t
            omega = angular_velocity
        elif rotation_mode == 'tangent':
            # Robot orientation follows path tangent
            psi = theta_path
            omega = omega_path
        elif rotation_mode == 'counter':
            # Robot rotates opposite to path (extreme nonlinearity)
            psi = -theta_path
            omega = -omega_path
        else:
            raise ValueError(f"Unknown rotation_mode: {rotation_mode}")
        
        # Convert global velocity to body frame
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        vx_body = cos_psi * vx_global + sin_psi * vy_global
        vy_body = -sin_psi * vx_global + cos_psi * vy_global
        
        # Store true state
        x_true[k] = np.array([x, y, psi, vx_body, vy_body, omega])
        
        # Compute acceleration in world frame (controls: [ax, ay])
        if k > 0:
            # Acceleration in world frame (centripetal for circular motion)
            ax_world = (vx_global - (linear_speed * np.cos(omega_path * (t - dt)))) / dt
            ay_world = (vy_global - (-linear_speed * np.sin(omega_path * (t - dt)))) / dt
            
            # Control: [ax, ay] in world frame
            controls[k] = np.array([ax_world, ay_world])
        else:
            controls[0] = np.array([0.0, 0.0])
    
    # Add process noise if specified
    if process_noise_std is not None:
        process_noise_std = np.asarray(process_noise_std)
        for k in range(1, N):
            noise = np.random.randn(6) * process_noise_std
            x_true[k] += noise
    
    # Generate noisy measurements using measurement
    robot = OmnidirectionalRobot(dt=dt)
    measurements = np.zeros((N, 4))
    R = np.diag(measurement_noise_std**2)
    
    for k in range(N):
        z_clean = robot.measurement(x_true[k])
        noise = np.random.multivariate_normal(np.zeros(4), R)
        measurements[k] = z_clean + noise
    
    return {
        'time': time,
        'controls': controls,
        'measurements': measurements,
        'ground_truth': x_true,
        'dt': dt,
        'measurement_noise_std': measurement_noise_std,
        'radius': radius,
        'angular_velocity_path': omega_path,
        'rotation_mode': rotation_mode
    }


def plot_trajectory_states(time, states, title="Trajectory Analysis", 
                          save_path=None, show=True):
    """
    Plot all 6 states of the trajectory in a comprehensive 2x3 subplot layout.
    
    Parameters
    ----------
    time : array_like
        Time vector (N,)
    states : array_like
        State trajectory (N, 6) with [x, y, psi, vx_b, vy_b, omega]
    title : str
        Main title for the figure
    save_path : str, optional
        Path to save the figure. If None, figure is not saved
    show : bool
        Whether to display the figure with plt.show()
    
    Returns
    -------
    fig, axes : matplotlib figure and axes
    """
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle(title, fontsize=16, fontweight='bold')
    
    # Subplot 1: XY trajectory
    axes[0, 0].plot(states[:, 0], states[:, 1], 'b-', linewidth=2, label='Path')
    axes[0, 0].scatter(states[0, 0], states[0, 1], c='green', s=150, 
                      marker='o', label='Start', zorder=5, edgecolors='black', linewidths=1.5)
    axes[0, 0].scatter(states[-1, 0], states[-1, 1], c='red', s=150, 
                      marker='^', label='End', zorder=5, edgecolors='black', linewidths=1.5)
    axes[0, 0].set_xlabel('X Position (m)', fontsize=11)
    axes[0, 0].set_ylabel('Y Position (m)', fontsize=11)
    axes[0, 0].set_title('XY Trajectory', fontsize=12, fontweight='bold')
    axes[0, 0].axis('equal')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend(fontsize=9)
    
    # Subplot 2: X position vs time
    axes[0, 1].plot(time, states[:, 0], 'b-', linewidth=1.5, label='X position')
    axes[0, 1].set_xlabel('Time (s)', fontsize=11)
    axes[0, 1].set_ylabel('X Position (m)', fontsize=11)
    axes[0, 1].set_title('X Position vs Time', fontsize=12, fontweight='bold')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend(fontsize=9)
    
    # Subplot 3: Y position vs time
    axes[0, 2].plot(time, states[:, 1], 'g-', linewidth=1.5, label='Y position')
    axes[0, 2].set_xlabel('Time (s)', fontsize=11)
    axes[0, 2].set_ylabel('Y Position (m)', fontsize=11)
    axes[0, 2].set_title('Y Position vs Time', fontsize=12, fontweight='bold')
    axes[0, 2].grid(True, alpha=0.3)
    axes[0, 2].legend(fontsize=9)
    
    # Subplot 4: Orientation vs time
    axes[1, 0].plot(time, states[:, 2], 'r-', linewidth=1.5, label='Orientation (ψ)')
    axes[1, 0].set_xlabel('Time (s)', fontsize=11)
    axes[1, 0].set_ylabel('Orientation (rad)', fontsize=11)
    axes[1, 0].set_title('Robot Orientation vs Time', fontsize=12, fontweight='bold')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend(fontsize=9)
    axes[1, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # Subplot 5: Body velocities vs time
    axes[1, 1].plot(time, states[:, 3], 'c-', linewidth=1.5, label='vx_body')
    axes[1, 1].plot(time, states[:, 4], 'm-', linewidth=1.5, label='vy_body')
    axes[1, 1].set_xlabel('Time (s)', fontsize=11)
    axes[1, 1].set_ylabel('Velocity (m/s)', fontsize=11)
    axes[1, 1].set_title('Body Frame Velocities', fontsize=12, fontweight='bold')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend(fontsize=9)
    axes[1, 1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # Subplot 6: Angular velocity vs time
    axes[1, 2].plot(time, states[:, 5], 'purple', linewidth=1.5, label='ω')
    axes[1, 2].set_xlabel('Time (s)', fontsize=11)
    axes[1, 2].set_ylabel('Angular Velocity (rad/s)', fontsize=11)
    axes[1, 2].set_title('Angular Velocity vs Time', fontsize=12, fontweight='bold')
    axes[1, 2].grid(True, alpha=0.3)
    axes[1, 2].legend(fontsize=9)
    axes[1, 2].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"  Saved trajectory plot: {save_path}")
    
    if show:
        plt.show()
    else:
        plt.close(fig)
    
    return fig, axes


if __name__ == "__main__":
    """Demo: Generate and visualize all trajectory types."""
    
    print("\n" + "="*60)
    print("Trajectory Generators Demo")
    print("="*60 + "\n")
    
    # Create output directory
    os.makedirs('../../results/estimation/trajectories', exist_ok=True)
    
    # Test parameters
    N = 1000
    dt = 0.01
    
    # High noise for visualization
    meas_noise = np.array([0.05, 0.05, 0.2, 0.05])
    
    # 1. Synthetic (square) trajectory
    print("1. Generating synthetic square trajectory...")
    data1 = generate_synthetic_data(N=N, dt=dt, measurement_noise_std=meas_noise)
    plot_trajectory_states(data1['time'], data1['ground_truth'],
                          title="Synthetic Square Trajectory",
                          save_path='../../results/estimation/trajectories/synthetic.png',
                          show=False)
    print(f"   Duration: {N*dt:.2f}s\n")
    
    # 2. Smooth square trajectory
    print("2. Generating smooth square trajectory...")
    data2 = generate_square_trajectory(N=N, dt=dt, speed=0.5, 
                                      corner_smoothing=0.1,
                                      measurement_noise_std=meas_noise)
    plot_trajectory_states(data2['time'], data2['ground_truth'],
                          title="Smooth Square Trajectory (Omnidirectional)",
                          save_path='../../results/estimation/trajectories/square.png',
                          show=False)
    print(f"   Duration: {N*dt:.2f}s\n")
    
    # 3. Circular trajectory (continuous rotation)
    print("3. Generating circular trajectory (continuous rotation)...")
    data3 = generate_circular_trajectory(N=N, dt=dt, linear_speed=0.5,
                                        angular_velocity=0.8,
                                        rotation_mode='continuous',
                                        measurement_noise_std=meas_noise)
    plot_trajectory_states(data3['time'], data3['ground_truth'],
                          title="Circular Trajectory (Continuous Rotation)",
                          save_path='../../results/estimation/trajectories/circular_continuous.png',
                          show=False)
    print(f"   Radius: {data3['radius']:.3f}m")
    print(f"   Path angular velocity: {data3['angular_velocity_path']:.3f} rad/s")
    print(f"   Duration: {N*dt:.2f}s\n")
    
    # 4. Circular trajectory (tangent orientation)
    print("4. Generating circular trajectory (tangent orientation)...")
    data4 = generate_circular_trajectory(N=N, dt=dt, linear_speed=0.5,
                                        rotation_mode='tangent',
                                        measurement_noise_std=meas_noise)
    plot_trajectory_states(data4['time'], data4['ground_truth'],
                          title="Circular Trajectory (Tangent Orientation)",
                          save_path='../../results/estimation/trajectories/circular_tangent.png',
                          show=False)
    print(f"   Radius: {data4['radius']:.3f}m")
    print(f"   Duration: {N*dt:.2f}s\n")
    
    print("="*60)
    print("Demo complete! Figures saved to results/figures/trajectories/")
    print("="*60)
