"""
Stage 1 Motor Parameter Identification from Experimental Data.

This script processes experimental steady-state motor test data to identify
motor electrical parameters (Ra, K) for an omnidirectional robot.

Experimental Protocol:
- 12 seconds total duration
- 8 segments of 1.5s each
- Each segment applies constant PWM to motors
- Encoder data captures wheel angular velocities
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent.parent))

from src.utils.conversions import pwm_to_voltage
from src.identification.three_stage import ThreeStageIdentification
from shared_utils.signal_processing import Kalman1D




def filter_velocities(w_raw, Q=0.001, R=1.12):
    """
    Filter encoder velocities using 1D Kalman filter.
    
    Parameters
    ----------
    w_raw : array (N, 3)
        Raw angular velocities for 3 motors [w1, w2, w3]
    Q : float
        Process noise covariance
    R : float
        Measurement noise covariance
    
    Returns
    -------
    w_filtered : array (N, 3)
        Filtered angular velocities
    """
    N = len(w_raw)
    w_filtered = np.zeros_like(w_raw)
    
    # Create separate Kalman1D filter for each motor
    filters = [Kalman1D(Q, R) for _ in range(3)]
    
    for i in range(N):
        for motor_idx in range(3):
            w_filtered[i, motor_idx] = filters[motor_idx].update(w_raw[i, motor_idx])
    
    return w_filtered


def extract_steady_state_values(t, w_filtered, u_voltage, segment_duration=1.5, 
                                discard_time=0.5, total_segments=8):
    """
    Extract steady-state values from each segment.
    
    Each segment has constant input. We discard the initial transient period
    (first 0.5s) and average the remaining data (last 1.0s) to get steady-state values.
    
    Parameters
    ----------
    t : array (N,)
        Time vector
    w_filtered : array (N, 3)
        Filtered angular velocities
    u_voltage : array (N, 3)
        Applied voltages
    segment_duration : float
        Duration of each segment (s)
    discard_time : float
        Time to discard from start of each segment (s)
    total_segments : int
        Number of segments
    
    Returns
    -------
    u_steady : array (total_segments, 3)
        Average voltage for each segment and motor
    w_steady : array (total_segments, 3)
        Average angular velocity for each segment and motor
    """
    u_steady = np.zeros((total_segments, 3))
    w_steady = np.zeros((total_segments, 3))
    
    dt = t[1] - t[0]
    
    for seg in range(total_segments):
        # Define segment time range
        t_start = seg * segment_duration
        t_end = (seg + 1) * segment_duration
        
        # Find indices for steady-state portion (last 1.0s = after first 0.5s)
        t_steady_start = t_start + discard_time
        mask = (t >= t_steady_start) & (t < t_end)
        
        if np.sum(mask) > 0:
            # Average over steady-state portion (last 1.0s)
            u_steady[seg, :] = np.mean(u_voltage[mask, :], axis=0)
            w_steady[seg, :] = np.mean(w_filtered[mask, :], axis=0)
        else:
            print(f"Warning: Segment {seg+1} has no data points in steady-state region")
    
    return u_steady, w_steady


def load_experimental_data(filepath):
    """
    Load experimental data from stage1.txt file.
    
    Parameters
    ----------
    filepath : str or Path
        Path to data file
    
    Returns
    -------
    data : dict
        Dictionary with keys: 't', 'w', 'u'
        - t: time (N,)
        - w: angular velocities (N, 3) [w1, w2, w3]
        - u: control inputs (N, 3) [u1, u2, u3]
    """
    # Load data (skip header)
    raw_data = np.loadtxt(filepath, delimiter=',', skiprows=1)
    
    # Extract columns of interest
    # Columns: t, ax, ay, alpha, w1, w2, w3, u1, u2, u3, vbx_sp, vby_sp
    #          0   1   2     3    4   5   6   7   8   9     10       11
    
    data = {
        't': raw_data[:, 0],           # Time
        'w': raw_data[:, 4:7],         # w1, w2, w3 (angular velocities)
        'u': raw_data[:, 7:10]         # u1, u2, u3 (PWM duty cycles in %)
    }
    
    print(f"Loaded {len(data['t'])} data points")
    print(f"Time range: [{data['t'][0]:.3f}, {data['t'][-1]:.3f}] s")
    print(f"Duration: {data['t'][-1] - data['t'][0]:.3f} s")
    
    return data


def main():
    """
    Main processing pipeline for Stage 1 experimental identification.
    """
    print("=" * 70)
    print("STAGE 1: MOTOR PARAMETER IDENTIFICATION - EXPERIMENTAL DATA")
    print("=" * 70)
    
    # =========================================================================
    # Configuration
    # =========================================================================
    DATA_FILE = Path(__file__).parent.parent.parent.parent / "data" / "raw" / "motor_tests" / "stage1_3.txt"
    
    # Battery configuration (6S LiPo)
    V_BATTERY = 22.2  # Nominal voltage: 3.7V × 6 = 22.2V
    
    # Kalman filter parameters (for encoder data)
    Q_ENC = 0.001   # Process noise
    R_ENC = 1.12    # Measurement noise
    
    # Segment parameters
    SEGMENT_DURATION = 1.5  # seconds
    DISCARD_TIME = 1.1      # seconds to discard from start (use last 1.0s for steady-state)
    TOTAL_SEGMENTS = 8
    
    # Motor to identify (1, 2, or 3)
    MOTOR_ID = 2  # Change to 2 or 3 for other motors
    
    # Robot geometric parameters (needed for ThreeStageIdentification)
    wheel_angles = [150, 270, 30]  # degrees
    d = 0.08  # m
    r = 0.025  # m
    M = 3.178  # kg
    
    # =========================================================================
    # 1. Load Data
    # =========================================================================
    print("\n[1] Loading experimental data...")
    data = load_experimental_data(DATA_FILE)
    
    t = data['t']
    w_raw = data['w']  # (N, 3) array
    u_pwm = data['u']  # (N, 3) array - PWM duty cycle (%)
    
    # =========================================================================
    # 2. Filter Velocities
    # =========================================================================
    print(f"\n[2] Filtering velocities with 1D Kalman (Q={Q_ENC}, R={R_ENC})...")
    w_filtered = filter_velocities(w_raw, Q=Q_ENC, R=R_ENC)
    
    print(f"    Filtering complete.")
    print(f"    RMS noise reduction:")
    for motor_idx in range(3):
        noise_raw = np.std(w_raw[:, motor_idx] - w_filtered[:, motor_idx])
        print(f"      Motor {motor_idx+1}: {noise_raw:.2f} rad/s")
    
    # =========================================================================
    # 3. Convert PWM to Voltage
    # =========================================================================
    print(f"\n[3] Converting PWM to voltage (V_battery = {V_BATTERY}V)...")
    u_voltage = np.zeros_like(u_pwm)
    for motor_idx in range(3):
        u_voltage[:, motor_idx] = pwm_to_voltage(u_pwm[:, motor_idx], V_BATTERY)
    
    print(f"    Voltage ranges:")
    for motor_idx in range(3):
        v_min, v_max = u_voltage[:, motor_idx].min(), u_voltage[:, motor_idx].max()
        print(f"      Motor {motor_idx+1}: [{v_min:.2f}, {v_max:.2f}] V")
    
    # =========================================================================
    # 4. Extract Steady-State Values
    # =========================================================================
    print(f"\n[4] Extracting steady-state values from {TOTAL_SEGMENTS} segments...")
    print(f"    Segment duration: {SEGMENT_DURATION}s")
    print(f"    Discarding first {DISCARD_TIME}s of each segment")
    print(f"    Averaging over last {SEGMENT_DURATION - DISCARD_TIME}s (steady-state)")
    
    u_steady, w_steady = extract_steady_state_values(
        t, w_filtered, u_voltage, 
        segment_duration=SEGMENT_DURATION,
        discard_time=DISCARD_TIME,
        total_segments=TOTAL_SEGMENTS
    )
    
    print(f"\n    Extracted {len(u_steady)} steady-state points per motor")
    
    # =========================================================================
    # 5. Visualize Data
    # =========================================================================
    print(f"\n[5] Generating visualizations...")
    
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
    
    results_dir = Path(__file__).parent.parent.parent.parent / "results" / "identification" / "stage1"
    results_dir.mkdir(parents=True, exist_ok=True)
    
    # =========================================================================
    # FIGURE 2: Time series data (Velocidades + Voltajes)
    # =========================================================================
    fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.16, 5.5))  # IEEE double column width
    
    # --- Filtered Velocities ---
    for motor_idx in range(3):
        ax1.plot(t, w_filtered[:, motor_idx], label=f'Motor {motor_idx+1}', alpha=0.8)
    ax1.set_xlabel('Tiempo [s]')
    ax1.set_ylabel('Velocidad Angular [rad/s]')
    ax1.legend(loc='best', framealpha=0.9)
    ax1.grid(True)
    
    # Mark segment boundaries
    for seg in range(TOTAL_SEGMENTS + 1):
        ax1.axvline(seg * SEGMENT_DURATION, color='gray', linestyle='--', alpha=0.5, linewidth=0.8)
    
    # --- Applied Voltages ---
    for motor_idx in range(3):
        ax2.plot(t, u_voltage[:, motor_idx], label=f'Motor {motor_idx+1}', alpha=0.8)
    ax2.set_xlabel('Tiempo [s]')
    ax2.set_ylabel('Voltaje [V]')
    ax2.legend(loc='best', framealpha=0.9)
    ax2.grid(True)
    
    # Mark segment boundaries
    for seg in range(TOTAL_SEGMENTS + 1):
        ax2.axvline(seg * SEGMENT_DURATION, color='gray', linestyle='--', alpha=0.5, linewidth=0.8)
    
    plt.tight_layout()
    fig2_path = results_dir / "stage1_velocidades_filtradas_voltajes_aplicados.png"
    fig2.savefig(fig2_path, dpi=300, bbox_inches='tight')
    print(f"✓ Figura 2 guardada: {fig2_path}")
    
    # =========================================================================
    # FIGURE 3: Steady-State Points (All Motors)
    # =========================================================================
    fig3, ax3 = plt.subplots(1, 1, figsize=(7.16, 4.5))  # IEEE double column width
    
    colors = ['C0', 'C1', 'C2']
    markers = ['o', 's', '^']
    
    for motor_idx in range(3):
        # Filter out near-zero points (motor not active)
        valid = np.abs(w_steady[:, motor_idx]) > 0.5
        ax3.scatter(u_steady[valid, motor_idx], w_steady[valid, motor_idx], 
                   s=100, alpha=0.7, marker=markers[motor_idx],
                   color=colors[motor_idx], label=f'Motor {motor_idx+1}',
                   edgecolors='black', linewidths=1.5)
    
    ax3.set_xlabel('Voltaje [V]')
    ax3.set_ylabel('Velocidad Angular en Estado Estable [rad/s]')
    ax3.legend(loc='best', framealpha=0.9)
    ax3.grid(True)
    ax3.axhline(0, color='k', linestyle='-', linewidth=0.8, alpha=0.3)
    ax3.axvline(0, color='k', linestyle='-', linewidth=0.8, alpha=0.3)
    
    plt.tight_layout()
    fig3_path = results_dir / "stage1_voltaje_vs_velocidad_estado_estable_todos_motores.png"
    fig3.savefig(fig3_path, dpi=300, bbox_inches='tight')
    print(f"✓ Figura 3 guardada: {fig3_path}")
    
    # =========================================================================
    # Prepare data for identification (will generate Figure 1)
    # =========================================================================
    # Prepare data for selected motor
    motor_idx = MOTOR_ID - 1  # Convert to 0-indexed
    
    # Filter out near-zero points
    valid = np.abs(w_steady[:, motor_idx]) > 0.5
    u_motor = u_steady[valid, motor_idx]
    w_motor = w_steady[valid, motor_idx]
    
    print(f"\n[6] Identifying parameters for Motor {MOTOR_ID}...")
    print(f"    Using {len(u_motor)} steady-state points")
    
    # Initialize identifier
    identifier = ThreeStageIdentification(wheel_angles, d, r, M)
    
    # Run Stage 1 identification (this will generate Figure 1)
    identifier.stage1_motor_parameters(u_motor, w_motor, motor_id=MOTOR_ID, results_dir=results_dir)
    
    # Get results for summary
    Ra = identifier.results['stage1']['Ra']
    K = identifier.results['stage1']['K']
    b = identifier.results['stage1']['friction']
    r2 = identifier.results['stage1']['r2']
    
    # =========================================================================
    # Summary
    # =========================================================================
    print("\n" + "=" * 70)
    print("IDENTIFICACIÓN COMPLETADA")
    print("=" * 70)
    print(f"\nParámetros del Motor {MOTOR_ID}:")
    print(f"  Ra (Resistencia del Inducido): {Ra:.4f} Ω")
    print(f"  K  (Constante del Motor):      {K:.4f} V/(rad/s)")
    print(f"  b  (Coeficiente de Fricción):  {b:.6f} Nm/(rad/s)")
    print(f"  R² (Calidad del Modelo):       {r2:.4f}")
    
    if r2 > 0.9:
        print("\n✓ ¡Excelente ajuste! Los parámetros son confiables.")
    elif r2 > 0.8:
        print("\n✓ Buen ajuste. Los parámetros son aceptables.")
    else:
        print("\n⚠ Ajuste pobre. Verifique la calidad de los datos.")
    
    print("\n" + "=" * 70)
    
    # Reset matplotlib to defaults
    plt.rcParams.update(plt.rcParamsDefault)
    
    return identifier


if __name__ == "__main__":
    identifier = main()
