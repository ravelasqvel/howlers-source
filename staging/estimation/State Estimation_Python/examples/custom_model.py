"""
Custom Model Example

Demonstrates how to use the filters with a custom robot model.
This example shows a simple differential drive robot.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from state_estimation import ExtendedKalmanFilter
from state_estimation.visualization import plot_trajectory


# ============================================================================
# CUSTOM DIFFERENTIAL DRIVE ROBOT MODEL
# ============================================================================

class DifferentialDriveRobot:
    """
    Simple differential drive robot.
    
    State: x = [x, y, theta, v, omega]
    - (x, y): position
    - theta: heading
    - v: linear velocity
    - omega: angular velocity
    
    Control: u = [a, alpha]
    - a: linear acceleration
    - alpha: angular acceleration
    
    Measurement: z = [v, omega, theta]
    """
    
    def __init__(self, dt=0.01):
        self.dt = dt
    
    def dynamics(self, x, u):
        """
        Dynamics: x_next = f(x, u)
        
        Simple kinematic model with acceleration controls.
        """
        x_pos, y_pos, theta, v, omega = x
        a, alpha = u
        
        dt = self.dt
        
        # Update velocities
        v_new = v + a * dt
        omega_new = omega + alpha * dt
        
        # Update heading
        theta_new = theta + omega * dt
        
        # Update position
        x_new = x_pos + v * np.cos(theta) * dt
        y_new = y_pos + v * np.sin(theta) * dt
        
        return np.array([x_new, y_new, theta_new, v_new, omega_new])
    
    def jacobian_F(self, x, u):
        """Jacobian of dynamics."""
        x_pos, y_pos, theta, v, omega = x
        dt = self.dt
        
        F = np.eye(5)
        
        # ∂x/∂theta
        F[0, 2] = -v * np.sin(theta) * dt
        # ∂x/∂v
        F[0, 3] = np.cos(theta) * dt
        
        # ∂y/∂theta
        F[1, 2] = v * np.cos(theta) * dt
        # ∂y/∂v
        F[1, 3] = np.sin(theta) * dt
        
        # ∂theta/∂omega
        F[2, 4] = dt
        
        return F
    
    def measurement(self, x):
        """Measurement: z = h(x) = [v, omega, theta]"""
        return np.array([x[3], x[4], x[2]])
    
    def jacobian_H(self, x):
        """Jacobian of measurement."""
        H = np.array([
            [0, 0, 0, 1, 0],  # v
            [0, 0, 0, 0, 1],  # omega
            [0, 0, 1, 0, 0],  # theta
        ], dtype=float)
        return H


def generate_circle_trajectory(N=500, dt=0.01):
    """Generate circular trajectory."""
    time = np.arange(N) * dt
    
    # Create robot
    robot = DifferentialDriveRobot(dt=dt)
    
    # Initial state
    x_true = np.zeros((N, 5))
    x_true[0] = np.array([0.0, 0.0, np.pi/2, 0.0, 0.0])
    
    # Control: constant velocity, turning
    controls = np.zeros((N, 2))
    for k in range(N):
        controls[k] = [0.5, 0.3]  # Accelerate forward, turn
    
    # Simulate
    for k in range(N - 1):
        x_true[k + 1] = robot.dynamics(x_true[k], controls[k])
    
    # Add noise to measurements
    measurements = np.zeros((N, 3))
    R = np.diag([0.01, 0.02, 0.05])
    
    for k in range(N):
        z_clean = robot.measurement(x_true[k])
        noise = np.random.multivariate_normal(np.zeros(3), R)
        measurements[k] = z_clean + noise
    
    return {
        'time': time,
        'controls': controls,
        'measurements': measurements,
        'ground_truth': x_true,
        'dt': dt
    }


def run_custom_model_example():
    """Run EKF with custom differential drive model."""
    
    print("\n" + "="*60)
    print("Custom Model Example - Differential Drive Robot")
    print("="*60 + "\n")
    
    # Generate data
    print("Generating circular trajectory...")
    data = generate_circle_trajectory(N=500, dt=0.01)
    time = data['time']
    controls = data['controls']
    measurements = data['measurements']
    ground_truth = data['ground_truth']
    dt = data['dt']
    N = len(time)
    
    # Initialize custom robot model
    robot = DifferentialDriveRobot(dt=dt)
    
    # Create EKF
    print("Running EKF with custom model...")
    ekf = ExtendedKalmanFilter(dim_x=5, dim_z=3, dim_u=2)
    
    ekf.x = ground_truth[0] + np.array([0.1, 0.1, 0.05, 0.0, 0.0])
    ekf.P = np.diag([0.2, 0.2, 0.1, 0.1, 0.05])
    ekf.Q = np.diag([1e-4, 1e-4, 1e-5, 1e-3, 1e-4])
    ekf.R = np.diag([0.01, 0.02, 0.05])
    
    # Storage
    estimates = np.zeros((N, 5))
    estimates[0] = ekf.x
    
    # Run filter
    for k in range(N - 1):
        # Use custom dynamics and Jacobian
        ekf.predict(u=controls[k], f=robot.dynamics, F=robot.jacobian_F)
        
        # Use custom measurement function and Jacobian
        ekf.update(z=measurements[k + 1], h=robot.measurement, H=robot.jacobian_H)
        
        estimates[k + 1] = ekf.x
        
        if (k + 1) % 100 == 0:
            print(f"  Processed {k+1}/{N-1} steps...")
    
    print("EKF complete!\n")
    
    # Compute error
    pos_error = np.sqrt((estimates[:, 0] - ground_truth[:, 0])**2 + 
                       (estimates[:, 1] - ground_truth[:, 1])**2)
    rmse = np.sqrt(np.mean(pos_error**2))
    
    print(f"Position RMSE: {rmse:.6f} m\n")
    
    # Plot
    print("Generating plot...")
    os.makedirs('../../results/estimation', exist_ok=True)
    
    plot_trajectory(estimates, ground_truth, 
                   title="Custom Model: Differential Drive Robot",
                   save_path='../../results/estimation/custom_model_trajectory.png',
                   show=False)
    print("  Saved: results/figures/custom_model_trajectory.png")
    
    print("\n" + "="*60)
    print("Custom Model Example Complete!")
    print("This demonstrates how easy it is to use custom dynamics!")
    print("="*60)


if __name__ == "__main__":
    run_custom_model_example()
