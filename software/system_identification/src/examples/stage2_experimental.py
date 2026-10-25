"""
Stage 2 Inertia Identification from Experimental Data.

This script processes experimental rotation test data to identify
the moment of inertia (I) by reconstructing the full state vector
from available measurements.

Experimental Protocol:
- Pure rotation maneuver (robot rotating in place)
- Measurements: orientation (IMU), wheel speeds (encoders)
- State reconstruction:
  * x, y: synthetic Gaussian noise (mean=0, std=0.01m) since robot barely moved
  * phi: directly from IMU (alpha)
  * dx, dy, dphi: computed from wheel velocities via inverse kinematics + rotation matrix
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent.parent))

from src.utils.conversions import pwm_to_voltage
from src.identification.three_stage import ThreeStageIdentification
from shared_utils.signal_processing import Kalman1D
from src.kinematics.validator import KinematicValidator


def load_stage2_data(filepath):
    """
    Load experimental data from stage2 file.
    
    Parameters
    ----------
    filepath : str or Path
        Path to data file
    
    Returns
    -------
    data : dict
        Dictionary with keys: 't', 'alpha', 'w', 'u', 'omega_setpoint'
        - t: time (N,)
        - alpha: orientation from IMU (N,) [rad]
        - w: wheel angular velocities (N, 3) [w1, w2, w3]
        - u: control inputs (N, 3) [u1, u2, u3] (PWM %)
        - omega_setpoint: desired angular velocity (N,)
    """
    # Load data (skip header)
    raw_data = np.loadtxt(filepath, delimiter=',', skiprows=1)
    
    # Extract columns of interest
    # Columns: t, ax, ay, alpha, w1, w2, w3, u1, u2, u3, vbx_sp, vby_sp, wb_sp
    #          0   1   2     3    4   5   6   7   8   9     10       11      12
    
    # Take alpha raw and unwrap
    alpha_raw = raw_data[:, 3]  # theta from IMU
    alpha_unwrapped = np.unwrap(alpha_raw)  # Remove 2π discontinuities
    
    data = {
        't': raw_data[:, 0],           # Time
        'alpha': alpha_unwrapped,      # Orientation (rad, unwrapped)
        'w': raw_data[:, 4:7],         # w1, w2, w3 (wheel angular velocities)
        'u': raw_data[:, 7:10],        # u1, u2, u3 (PWM duty cycles in %)
        'omega_setpoint': raw_data[:, 12] if raw_data.shape[1] > 12 else np.zeros(len(raw_data))
    }
    
    print(f"Loaded {len(data['t'])} data points")
    print(f"Time range: [{data['t'][0]:.3f}, {data['t'][-1]:.3f}] s")
    print(f"Duration: {data['t'][-1] - data['t'][0]:.3f} s")
    
    return data


def compute_body_velocities(w_wheels, H_inv):
    """
    Compute robot body velocities from wheel speeds using inverse kinematics.
    
    Parameters
    ----------
    w_wheels : array (N, 3)
        Wheel angular velocities [w1, w2, w3]
    H_inv : array (3, 3)
        Inverse (pseudoinverse) of kinematic matrix
    
    Returns
    -------
    v_body : array (N, 3)
        Robot body velocities [vx_body, vy_body, omega] in robot frame
    """
    N = len(w_wheels)
    v_body = np.zeros((N, 3))
    
    for i in range(N):
        v_body[i, :] = H_inv @ w_wheels[i, :]  # [vx_body, vy_body, omega]
    
    return v_body


def body_to_global_velocities(v_body, phi):
    """
    Transform body frame velocities to global frame using rotation matrix.
    
    Parameters
    print("=" * 70)
    print("STAGE 2: INERTIA IDENTIFICATION - EXPERIMENTAL DATA")
    print("=" * 70)velocities [vx_body, vy_body, omega]
    phi : array (N,)
        Robot orientation (rad)
    
    Returns
    -------
    v_global : array (N, 3)
        Global frame velocities [dx, dy, dphi]
    """
    N = len(v_body)
    v_global = np.zeros((N, 3))
    
    for i in range(N):
        c = np.cos(phi[i])
        s = np.sin(phi[i])
        
        # Rotation matrix: global = R(phi) @ body
        v_global[i, 0] = c * v_body[i, 0] - s * v_body[i, 1]  # dx
        v_global[i, 1] = s * v_body[i, 0] + c * v_body[i, 1]  # dy
        v_global[i, 2] = v_body[i, 2]                         # dphi (unchanged)
    
    return v_global


def reconstruct_full_state(alpha, w_filtered, H_inv, noise_std=0.01):
    """
    Reconstruct full state vector from experimental measurements.
    
    Parameters
    ----------
    alpha : array (N,)
        Orientation from IMU (rad)
    w_filtered : array (N, 3)
        Filtered wheel velocities
    H_inv : array (3, 3)
        Inverse kinematic matrix
    noise_std : float, optional
        Standard deviation for synthetic position noise (m)
    
    Returns
    -------
    y_measured : array (N, 6)
        Full state vector [x, y, phi, dx, dy, dphi]
    """
    N = len(alpha)
    y_measured = np.zeros((N, 6))
    
    # Position: synthetic Gaussian noise (robot barely moved during rotation)
    y_measured[:, 0] = np.random.normal(0, noise_std, N)  # x
    y_measured[:, 1] = np.random.normal(0, noise_std, N)  # y
    
    # Orientation: directly from IMU (zeroed to start at 0)
    y_measured[:, 2] = alpha - alpha[0]  # phi
    
    # Velocities: computed from kinematics
    v_body = compute_body_velocities(w_filtered, H_inv)
    v_global = body_to_global_velocities(v_body, y_measured[:, 2])
    
    y_measured[:, 3] = v_global[:, 0]  # dx
    y_measured[:, 4] = v_global[:, 1]  # dy
    y_measured[:, 5] = v_global[:, 2]  # dphi
    
    return y_measured


def main():
    """
    Main processing pipeline for Stage 2 experimental identification.
    """
    print("=" * 70)
    print("STAGE 2: INERTIA IDENTIFICATION - EXPERIMENTAL DATA")
    print("=" * 70)
    
    # =========================================================================
    # Configuration
    # =========================================================================
    DATA_FILE = Path(__file__).parent.parent.parent.parent / "data" / "raw" / "rotation_tests" / "stage2_1.txt"
    print(DATA_FILE)
    # Battery configuration (6S LiPo)
    V_BATTERY = 22.2  # Nominal voltage: 3.7V × 6 = 22.2V
    
    # Kalman filter parameters (for encoder and IMU data)
    Q_ENC = 0.001   # Process noise for encoders
    R_ENC = 1.12    # Measurement noise for encoders
    Q_IMU = 0.0001  # Process noise for IMU (orientation)
    R_IMU = 0.05    # Measurement noise for IMU
    
    # Robot geometric parameters
    wheel_angles = [150, 270, 30]  # degrees
    d = 0.08  # m
    r = 0.025  # m
    M = 3.178  # kg
    
    # Motor parameters from Stage 1 (update with your identified values)
    # These should come from running stage1_experimental.py
    Ra = 1.50  # Ω - Update with your value
    K = 0.499   # V/(rad/s) - Update with your value
    
    print(f"\nUsing motor parameters from Stage 1:")
    print(f"  Ra = {Ra:.4f} Ω")
    print(f"  K  = {K:.4f} V/(rad/s)")
    print(f"  (Update these values in the script after running Stage 1)")
    
    # =========================================================================
    # 1. Load Data
    # =========================================================================
    print("\n[1] Loading experimental data...")
    data = load_stage2_data(DATA_FILE)
    
    t = data['t']
    alpha_raw = data['alpha']  # Orientation from IMU (unwrapped)
    w_raw = data['w']          # Wheel speeds from encoders
    u_pwm = data['u']          # PWM duty cycles
    
    # =========================================================================
    # 2. Filter Measurements
    # =========================================================================
    print(f"\n[2] Filtering measurements...")
    
    # Filter wheel velocities only (alpha is already unwrapped, no filtering)
    print(f"    Filtering wheel velocities (Q={Q_ENC}, R={R_ENC})...")
    w_filtered = np.zeros_like(w_raw)
    filters_enc = [Kalman1D(Q_ENC, R_ENC) for _ in range(3)]
    
    for i in range(len(t)):
        for motor_idx in range(3):
            w_filtered[i, motor_idx] = filters_enc[motor_idx].update(w_raw[i, motor_idx])
    
    print(f"    Filtering complete.")
    
    # =========================================================================
    # 3. Reconstruct Full State Vector
    # =========================================================================
    print(f"\n[3] Reconstructing full state vector from measurements...")
    
    # Compute kinematic matrices
    H = KinematicValidator.compute_H_matrix(wheel_angles, d, r)
    H_inv = np.linalg.pinv(H)  # Moore-Penrose pseudoinverse
    
    # Reconstruct y_measured = [x, y, phi, dx, dy, dphi]
    # alpha_raw is already unwrapped
    y_measured = reconstruct_full_state(alpha_raw, w_filtered, H_inv, noise_std=0.02)
    
    # Validation
    omega_rms = np.sqrt(np.mean(y_measured[:, 5]**2))
    vx_rms = np.sqrt(np.mean(y_measured[:, 3]**2))
    vy_rms = np.sqrt(np.mean(y_measured[:, 4]**2))
    
    print(f"    State statistics:")
    print(f"      dx RMS:    {vx_rms:.4f} m/s")
    print(f"      dy RMS:    {vy_rms:.4f} m/s")
    print(f"      dphi RMS:  {omega_rms:.4f} rad/s")
    
    if omega_rms < 0.1:
        print(f"    ⚠ Warning: Low angular velocity - may affect identification quality")
    else:
        print(f"    ✓ Good angular velocity variation")
    
    # =========================================================================
    # 4. Convert PWM to Voltage
    # =========================================================================
    print(f"\n[4] Converting PWM to voltage (V_battery = {V_BATTERY}V)...")
    u_voltage = np.zeros_like(u_pwm)
    for motor_idx in range(3):
        u_voltage[:, motor_idx] = pwm_to_voltage(u_pwm[:, motor_idx], V_BATTERY)
    
    # =========================================================================
    # 5. Run Stage 2 Identification (Angular-Only Mode)
    # =========================================================================
    print(f"\n[5] Running Stage 2 identification...")
    
    # Results directory
    results_dir = Path(__file__).parent.parent.parent.parent / "results" / "identification" / "stage2"
    results_dir.mkdir(parents=True, exist_ok=True)
    
    # Initialize identifier
    identifier = ThreeStageIdentification(wheel_angles, d, r, M)
    
    # Run identification with full state vector
    I = identifier.stage2_inertia(
        t=t,
        u=u_voltage,
        y_measured=y_measured,  # [x, y, phi, dx, dy, dphi]
        Ra=Ra,
        K=K,
        plot=True,
        results_dir=results_dir
    )
    
    # =========================================================================
    # Summary
    # =========================================================================
    print("\n" + "=" * 70)
    print("IDENTIFICATION COMPLETE")
    print("=" * 70)
    print(f"\nIdentified Parameters:")
    print(f"  I (Moment of Inertia): {I:.6f} kg·m²")
    
    # Reference values
    print(f"\nReference values:")
    print(f"  Small robot (< 5 kg):    I ~ 0.005 - 0.03 kg·m²")
    print(f"  Medium robot (5-15 kg):  I ~ 0.03 - 0.15 kg·m²")
    
    print("\n" + "=" * 70)
    print("Next Steps:")
    print("  1. Verify the plots show good agreement between measured and model")
    print("  2. Update the I value in your robot configuration")
    print("  3. Proceed to Stage 3 for motor compensation factors")
    print("=" * 70)
    
    plt.show()
    
    return identifier


if __name__ == "__main__":
    identifier = main()
