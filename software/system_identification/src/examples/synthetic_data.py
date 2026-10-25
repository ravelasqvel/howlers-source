"""
Example usage with synthetic data generation.

This module provides complete examples of the identification workflow
using simulated robot data. Replace with real experimental data for
actual robot identification.
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent.parent))

from src.kinematics.validator import KinematicValidator
from src.models.robot import OmnidirectionalRobot
from src.identification.three_stage import ThreeStageIdentification


def example_kinematic_validation():
    """
    Example 1: Validate kinematic matrix for specific robot configuration.
    
    Returns
    -------
    H : array (3, 3)
        Kinematic matrix
    results : dict
        Validation test results
    """
    print("\n" + "=" * 70)
    print("EXAMPLE 1: KINEMATIC VALIDATION")
    print("=" * 70)
    
    # Robot configuration
    wheel_angles = [150, 270, 30]  # w1, w2, w3 in degrees
    d = 0.099  # meters
    r = 0.0325  # meters
    
    # Visualize geometry
    print("\nVisualizing robot geometry...")
    fig = KinematicValidator.visualize_wheel_geometry(wheel_angles, d)
    
    # Compute and test kinematic matrix
    H = KinematicValidator.compute_H_matrix(wheel_angles, d, r)
    
    print("\nKinematic matrix H:")
    print(H)
    
    results = KinematicValidator.test_kinematic_consistency(H, verbose=True)
    
    return H, results


def example_synthetic_identification():
    """
    Example 2: Complete 3-stage identification with synthetic data.
    
    This demonstrates the full workflow with simulated robot data.
    For real robot identification, replace synthetic data generation
    with actual experimental measurements.
    
    Returns
    -------
    identifier : ThreeStageIdentification
        Identification object with results
    """
    print("\n" + "=" * 70)
    print("EXAMPLE 2: THREE-STAGE IDENTIFICATION (Synthetic Data)")
    print("=" * 70)
    
    # Robot configuration (known parameters)
    wheel_angles = [150, 270, 30]
    d = 0.099
    r = 0.0325
    M = 3.178
    
    # True parameters (unknown in real scenario - used only for simulation)
    true_params = {
        'wheel_angles': wheel_angles,
        'd': d,
        'r': r,
        'M': M,
        'I': 0.0180,
        'K': 0.259,
        'Ra': 1.3111
    }
    
    # Initialize identification framework
    identifier = ThreeStageIdentification(wheel_angles, d, r, M)
    
    # =========================================================================
    # STAGE 1: Motor Parameters
    # =========================================================================
    print("\n[Simulating Stage 1 data: motor bench test]")
    
    # Generate synthetic steady-state data
    pwm_vals = np.linspace(0.2, 1.0, 8)
    friction_coeff = 0.001
    wheel_speeds = true_params['K'] * pwm_vals / \
                   (true_params['Ra'] * friction_coeff + true_params['K']**2)
    wheel_speeds += np.random.randn(len(pwm_vals)) * 0.5  # Add measurement noise
    
    Ra_est, K_est = identifier.stage1_motor_parameters(pwm_vals, wheel_speeds, motor_id=1)
    
    # =========================================================================
    # STAGE 2: Inertia
    # =========================================================================
    print("\n[Simulating Stage 2 data: pure rotation]")
    print("=" * 70)
    
    t_rot = np.linspace(0, 15, 750)
    u_rot = np.column_stack([
        0.3 * np.sin(2*np.pi*0.15*t_rot),
        0.3 * np.sin(2*np.pi*0.15*t_rot),
        0.3 * np.sin(2*np.pi*0.15*t_rot)
    ])
    
    robot_true = OmnidirectionalRobot(true_params)
    y_rot, _ = robot_true.simulate(t_rot, u_rot, np.zeros(6))
    # Add measurement noise
    y_rot += np.random.randn(*y_rot.shape) * np.array([0.005, 0.005, 0.01, 0.01, 0.01, 0.03])
    
    I_est = identifier.stage2_inertia(t_rot, u_rot, y_rot, Ra_est, K_est, plot=True)
    
    # =========================================================================
    # STAGE 3: Compensation
    # =========================================================================
    print("\n[Simulating Stage 3 data: full trajectory - SQUARE PATH]")
    
    # Generate square trajectory: Right → Up → Left → Down
    # Each segment lasts 3.75 seconds, total 15 seconds
    t_full = np.linspace(0, 15, 750)
    dt = t_full[1] - t_full[0]
    
    # Calculate desired velocity for square (in robot's global frame)
    v_desired = np.zeros((len(t_full), 3))  # [vx, vy, omega]
    
    square_speed = 0.20  # m/s for each side
    for i, t in enumerate(t_full):
        if t < 3.75:  # Move right (+X direction)
            v_desired[i] = [square_speed, 0.0, 0.0]
        elif t < 7.5:  # Move up (+Y direction)
            v_desired[i] = [0.0, square_speed, 0.0]
        elif t < 11.25:  # Move left (-X direction)
            v_desired[i] = [-square_speed, 0.0, 0.0]
        else:  # Move down (-Y direction)
            v_desired[i] = [0.0, -square_speed, 0.0]
    
    # Convert desired velocities to wheel speeds using H matrix
    H = KinematicValidator.compute_H_matrix(wheel_angles, d, r)
    wheel_speeds_desired = np.array([H @ v for v in v_desired])
    
    # Convert wheel speeds to motor voltages (inverse of motor model)
    # Approximation: V ≈ ω * K (assuming low friction)
    K_approx = true_params['K']
    u_full = wheel_speeds_desired * K_approx * 1.2  # Scale factor for dynamics
    
    # Add smooth transitions at corners (avoid discontinuities)
    transition_time = 0.5  # seconds for smooth transition
    transition_samples = int(transition_time / dt)
    corner_indices = [int(3.75/dt), int(7.5/dt), int(11.25/dt)]
    
    for corner_idx in corner_indices:
        if corner_idx < len(u_full):
            # Apply smoothing window around corner
            for j in range(3):  # For each motor
                start = max(0, corner_idx - transition_samples//2)
                end = min(len(u_full), corner_idx + transition_samples//2)
                if start < end:
                    # Smooth transition using cosine taper
                    window_len = end - start
                    taper = 0.5 * (1 + np.cos(np.linspace(np.pi, 2*np.pi, window_len)))
                    u_full[start:end, j] = u_full[start:end, j] * taper
    
    # Apply "true" compensation (simulating motor differences)
    true_Cu = np.array([1.05, 0.98, 1.02])
    u_full_true = u_full * true_Cu
    
    y_full, _ = robot_true.simulate(t_full, u_full_true, np.zeros(6))
    # Add measurement noise
    y_full += np.random.randn(*y_full.shape) * np.array([0.01, 0.01, 0.02, 0.02, 0.02, 0.05])
    
    Cu1_est, Cu2_est, Cu3_est = identifier.stage3_compensation(
        t_full, u_full, y_full, Ra_est, K_est, I_est, plot=True
    )
    
    # =========================================================================
    # Summary
    # =========================================================================
    identifier.get_summary()
    
    print("\nComparison with true values:")
    print(f"  Ra:  True={true_params['Ra']:.4f}, Est={Ra_est:.4f}, Error={abs(Ra_est-true_params['Ra'])/true_params['Ra']*100:.1f}%")
    print(f"  K:   True={true_params['K']:.4f}, Est={K_est:.4f}, Error={abs(K_est-true_params['K'])/true_params['K']*100:.1f}%")
    print(f"  I:   True={true_params['I']:.6f}, Est={I_est:.6f}, Error={abs(I_est-true_params['I'])/true_params['I']*100:.1f}%")
    print(f"  Cu1: True={true_Cu[0]:.4f}, Est={Cu1_est:.4f}")
    print(f"  Cu2: True={true_Cu[1]:.4f}, Est={Cu2_est:.4f}")
    print(f"  Cu3: True={true_Cu[2]:.4f}, Est={Cu3_est:.4f}")
    
    plt.show()
    
    return identifier


def run_all_examples():
    """
    Run all examples in sequence.
    
    Returns
    -------
    success : bool
        Whether all examples completed successfully
    """
    print("\n" + "=" * 70)
    print("OMNIDIRECTIONAL ROBOT SYSTEM IDENTIFICATION")
    print("Complete 3-Stage Protocol with Validated Kinematics")
    print("=" * 70)
    
    # Example 1: Validate kinematics
    H, kin_results = example_kinematic_validation()
    
    if kin_results['all_pass']:
        print("\n✓ Kinematics validated - proceeding to identification...")
        
        # Example 2: Full identification
        identifier = example_synthetic_identification()
        
        print("\n" + "=" * 70)
        print("✓ ALL EXAMPLES COMPLETED - Check plots for validation")
        print("=" * 70)
        return True
    else:
        print("\n⚠ Kinematic validation failed - check wheel configuration")
        print("=" * 70)
        return False
