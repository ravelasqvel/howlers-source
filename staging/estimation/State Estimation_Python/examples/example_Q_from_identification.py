"""
Example: Compute Process Noise Q from Experimental Data

This example demonstrates how to compute the process noise covariance matrix Q
for state estimation filters (EKF/UKF/PF) using experimental data from the robot.

The Q matrix is derived from acceleration measurement uncertainty, propagated
through the identified dynamics model.

Methodology:
-----------
1. Load control inputs U = [u1, u2, u3] from experiment
2. Simulate robot dynamics using identified parameters (M, I, K, Ra, Cu1, Cu2, Cu3)
3. Extract model-based accelerations from dynamics
4. Compare with IMU measurements to estimate variance
5. Propagate variance through Jacobian to get Q matrix

Requirements:
------------
- Robot parameters from ThreeStageIdentification:
  * Stage 1: Ra, K (motor electrical parameters)
  * Stage 2: I (moment of inertia)
  * Stage 3: Cu1, Cu2, Cu3 (compensation factors)
- Experimental data with control inputs and IMU measurements

Usage:
------
    python example_Q_from_identification.py

Author: Robotics Thesis Project
Date: January 2026
"""

import sys
import os
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from state_estimation.models.compute_process_noise_Q import (
    compute_Q_from_experiment,
    compute_process_noise_Q,
    make_robot_params
)


def main():
    """
    Main example: Compute Q from multiple experiments and compare methods.
    """
    
    print("\n" + "="*70)
    print("EXAMPLE: PROCESS NOISE Q FROM MODEL IDENTIFICATION")
    print("="*70 + "\n")
    
    # ========================================================================
    # Configuration
    # ========================================================================
    BASE_DIR = Path(__file__).parent.parent.parent
    SENSORS_DIR = BASE_DIR / 'data' / 'sensors'
    RESULTS_DIR = BASE_DIR / 'results' / 'estimation' / 'Q_computation'
    
    # Robot parameters (from ThreeStageIdentification)
    # Replace these with your actual identified values!
    robot_params = make_robot_params(
        # Kinematic (measured)
        wheel_angles=[150.0, 270.0, 30.0],
        d=0.08,
        r=0.025,
        
        # Dynamic (from Stage 1-2 identification)
        M=3.178,    # mass (kg)
        I=0.02,     # moment of inertia (kg·m²)
        K=0.68,     # motor constant (V/(rad/s))
        Ra=1.5,     # armature resistance (Ω)
        
        # Compensation (from Stage 3 identification)
        # Cu1=0.8067,
        # Cu2=0.8150,
        # Cu3=0.8483

        Cu1=0.9930,
        Cu2=1.0066,
        Cu3=1.0072
    )
    
    # Experiment to analyze
    EXPERIMENT_NUM = 2
    N_POINTS = 1000
    
    experiment_file = SENSORS_DIR / f'exp{EXPERIMENT_NUM}.txt'
    
    # ========================================================================
    # Part 1: Compute Q from single experiment
    # ========================================================================
    print("PART 1: Compute Q from single experiment")
    print("-" * 70)
    
    results = compute_Q_from_experiment(
        experiment_file=experiment_file,
        robot_params=robot_params,
        N=N_POINTS,
        method='jacobian',
        variance_method='robust',
        save_dir=RESULTS_DIR / f'exp{EXPERIMENT_NUM}'
    )
    
    Q_jacobian = results['Q']
    var_ax = results['var_ax']
    var_ay = results['var_ay']
    dt = results['dt']
    phi_mean = results['phi_mean']
    # ========================================================================
    # Part 2: Compare different Q computation methods
    # ========================================================================
    print("\n\nPART 2: Compare Q computation methods")
    print("-" * 70)
    
    methods = ['jacobian', 'diagonal', 'physical']
    Q_matrices = {}
    
    for method in methods:
        Q = compute_process_noise_Q(var_ax, var_ay, phi_mean, dt, method=method)
        Q_matrices[method] = Q
        
        print(f"\n{method.upper()} method:")
        print(f"  Q diagonal: {Q.diagonal()}")
    
    # ========================================================================
    # Part 3: Visualize Q comparison
    # ========================================================================
    print("\n\nPART 3: Visualize Q matrices")
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
    
    fig, axes = plt.subplots(1, 3, figsize=(7.16, 2.5))
    state_names = ['x', 'y', r'$\phi$', r'$v_x$', r'$v_y$', r'$\omega$']
    
    for idx, method in enumerate(methods):
        Q = Q_matrices[method]
        Q_diag = Q.diagonal()
        
        # Log scale for better visualization
        axes[idx].bar(range(6), np.log10(Q_diag + 1e-20), 
                      color=['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b'])
        axes[idx].set_xticks(range(6))
        axes[idx].set_xticklabels(state_names, rotation=0)
        axes[idx].set_ylabel(r'$\log_{10}(Q_{ii})$')
        axes[idx].set_title(f'Método: {method.capitalize()}')
        axes[idx].grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    
    fig_path = RESULTS_DIR / 'Q_comparison_methods.png'
    fig_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(fig_path, dpi=300, bbox_inches='tight')
    print(f"Saved: {fig_path}")
    
    # ========================================================================
    # Part 4: Acceleration residuals comparison
    # ========================================================================
    print("\n\nPART 4: Acceleration residuals statistics")
    print("-" * 70)
    
    residuals = results['residuals']
    
    print(f"\nResidual statistics (N = {len(residuals)}):")
    print(f"  ax: mean = {np.mean(residuals[:, 0]):.4f} m/s², "
          f"std = {np.std(residuals[:, 0]):.4f} m/s²")
    print(f"  ay: mean = {np.mean(residuals[:, 1]):.4f} m/s², "
          f"std = {np.std(residuals[:, 1]):.4f} m/s²")
    
    # Normality test (Shapiro-Wilk)
    from scipy.stats import shapiro
    
    stat_x, p_x = shapiro(residuals[:, 0])
    stat_y, p_y = shapiro(residuals[:, 1])
    
    print(f"\nShapiro-Wilk normality test (H0: data is Gaussian):")
    print(f"  ax: W = {stat_x:.4f}, p-value = {p_x:.4f} "
          f"({'Normal' if p_x > 0.05 else 'Non-normal'})")
    print(f"  ay: W = {stat_y:.4f}, p-value = {p_y:.4f} "
          f"({'Normal' if p_y > 0.05 else 'Non-normal'})")
    
    # ========================================================================
    # Part 5: Recommended Q for EKF/UKF/PF
    # ========================================================================
    print("\n\n" + "="*70)
    print("RECOMMENDED Q MATRIX FOR STATE ESTIMATION")
    print("="*70)
    
    Q_recommended = Q_jacobian
    
    print("\nUse the following Q matrix in your filter:")
    print("\n# Process noise covariance Q")
    print("Q = np.array([")
    for row in Q_recommended:
        print(f"    {list(row)},")
    print("])")
    
    print("\n# Or as diagonal (if assuming independence):")
    print(f"Q = np.diag({list(Q_recommended.diagonal())})")
    
    # Save recommended Q as Python code
    code_path = RESULTS_DIR / 'Q_matrix_code.py'
    with open(code_path, 'w') as f:
        f.write('"""Recommended Q matrix from model identification"""\n')
        f.write('import numpy as np\n\n')
        f.write('# Process noise covariance matrix Q\n')
        f.write('# Computed from experimental data\n')
        f.write(f'# Experiment: {EXPERIMENT_NUM}\n')
        f.write(f'# Acceleration variance: sigma_ax = {np.sqrt(var_ax):.4f}, sigma_ay = {np.sqrt(var_ay):.4f}\n')
        f.write(f'# Time step: dt = {dt:.4f} s\n\n')
        f.write('Q = np.diag([\n')
        for i, name in enumerate(['x', 'y', 'phi', 'vx', 'vy', 'omega']):
            f.write(f'    {Q_recommended[i, i]:.6e},  # {name}\n')
        f.write('])\n')
    
    print(f"\nSaved Python code to: {code_path}")
    
    # ========================================================================
    # Part 6: Sensitivity analysis
    # ========================================================================
    print("\n\nPART 6: Sensitivity analysis - Effect of dt on Q")
    print("-" * 70)
    
    dt_values = np.array([0.005, 0.01, 0.02, 0.05, 0.1])
    Q_sensitivity = np.zeros((len(dt_values), 6))
    
    for i, dt_test in enumerate(dt_values):
        Q_test = compute_process_noise_Q(var_ax, var_ay, phi_mean, dt_test, method='jacobian')
        Q_sensitivity[i, :] = Q_test.diagonal()
    
    # Plot sensitivity
    fig2, ax = plt.subplots(figsize=(7.16, 4))
    
    for state_idx, state_name in enumerate(state_names):
        ax.plot(dt_values, Q_sensitivity[:, state_idx], 
                marker='o', label=state_name, linewidth=1.5)
    
    ax.set_xlabel('Paso de tiempo dt [s]')
    ax.set_ylabel('$Q_{ii}$')
    ax.set_title('Sensibilidad de Q al paso de tiempo')
    ax.set_yscale('log')
    ax.set_xscale('log')
    ax.legend(ncol=2)
    ax.grid(True, alpha=0.3, which='both')
    
    plt.tight_layout()
    
    fig2_path = RESULTS_DIR / 'Q_sensitivity_dt.png'
    fig2.savefig(fig2_path, dpi=300, bbox_inches='tight')
    print(f"Saved: {fig2_path}")
    
    # Reset matplotlib defaults
    plt.rcParams.update(plt.rcParamsDefault)
    
    print("\n" + "="*70)
    print("EXAMPLE COMPLETE")
    print(f"Results saved to: {RESULTS_DIR}")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
