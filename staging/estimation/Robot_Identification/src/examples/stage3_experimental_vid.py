"""
Stage 3 Compensation Factor Identification from Video Trajectory Data.

This script processes experimental trajectory data from video tracking + IMU fusion
to identify motor compensation factors (Cu1, Cu2, Cu3) after Stage 1 (Ra, K) and
Stage 2 (I) have been completed.

Data Source:
- Video tracking provides: x, y positions and linear velocities (vx, vy)
- IMU provides: orientation (phi) and angular velocity (omega)
- Sensor data provides: control inputs (u1, u2, u3) in PWM %
- All data combined by process_imu_data.py in traj_vid_processed/ folder

Experimental Protocol:
- Rich trajectory with translations + rotations
- Mixed speeds and directions
- Minimum 10 seconds of data at 100 Hz
- Data already synchronized and resampled with control inputs included
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent.parent))

from src.identification.three_stage import ThreeStageIdentification
import pandas as pd


def load_stage3_video_data(filepath):
    """
    Load experimental trajectory data from video + IMU processing.
    
    Parameters
    ----------
    filepath : str or Path
        Path to traj_vid_processed CSV file
    
    Returns
    -------
    data : dict
        Dictionary with keys: 't', 'y_measured', 'u_pwm'
        - t: time (N,) [s]
        - y_measured: state vector (N, 6) [x, y, phi, dx, dy, dphi]
        - u_pwm: control inputs (N, 3) [u1, u2, u3] in PWM %
    """
    print(f"Loading trajectory data from {filepath}...")
    
    # Load CSV data
    df = pd.read_csv(filepath)
    
    # Extract state vector: [x, y, phi, vx, vy, omega]
    y_measured = np.column_stack([
        df['x_m'].values,
        df['y_m'].values,
        df['phi_rad'].values,
        df['vx_m_s'].values,
        df['vy_m_s'].values,
        df['omega_rad_s'].values
    ])
    
    # Extract control inputs (u1, u2, u3) in PWM %
    u_pwm = np.column_stack([
        df['u1_pwm'].values,
        df['u2_pwm'].values,
        df['u3_pwm'].values
    ])
    
    t = df['time_s'].values
    
    print(f"  ✓ Loaded {len(t)} data points")
    print(f"  Time range: [{t[0]:.3f}, {t[-1]:.3f}] s")
    print(f"  Duration: {t[-1] - t[0]:.3f} s")
    print(f"  Sampling rate: {1/(t[1]-t[0]):.1f} Hz")
    
    # Verify data quality
    x_range = y_measured[:, 0].max() - y_measured[:, 0].min()
    y_range = y_measured[:, 1].max() - y_measured[:, 1].min()
    phi_range = np.rad2deg(y_measured[:, 2].max() - y_measured[:, 2].min())
    
    print(f"\n  Trajectory characteristics:")
    print(f"    X displacement: {x_range:.3f} m")
    print(f"    Y displacement: {y_range:.3f} m")
    print(f"    Total rotation: {phi_range:.1f}°")
    
    print(f"\n  Control inputs:")
    print(f"    u1 range: [{u_pwm[:, 0].min():.1f}, {u_pwm[:, 0].max():.1f}] %")
    print(f"    u2 range: [{u_pwm[:, 1].min():.1f}, {u_pwm[:, 1].max():.1f}] %")
    print(f"    u3 range: [{u_pwm[:, 2].min():.1f}, {u_pwm[:, 2].max():.1f}] %")
    
    if x_range < 0.1 and y_range < 0.1:
        print(f"    ⚠ Warning: Low linear displacement - may affect identification")
    if abs(phi_range) < 30:
        print(f"    ⚠ Warning: Low rotation - may affect identification")
    
    data = {
        't': t,
        'y_measured': y_measured,
        'u_pwm': u_pwm
    }
    
    return data


def pwm_to_voltage_batch(u_pwm, V_battery):
    """
    Convert PWM duty cycles to voltages.
    
    Parameters
    ----------
    u_pwm : array (N, 3)
        PWM duty cycles in percentage
    V_battery : float
        Battery voltage (V)
    
    Returns
    -------
    u_voltage : array (N, 3)
        Voltages [V]
    """
    return (u_pwm / 100.0) * V_battery


def main():
    """
    Main processing pipeline for Stage 3 video-based identification.
    """
    print("=" * 70)
    print("STAGE 3: COMPENSATION FACTORS - VIDEO TRAJECTORY DATA")
    print("=" * 70)
    
    # =========================================================================
    # Configuration
    # =========================================================================
    
    # Video number to process (1-10)
    VIDEO_NUMBER = 2
    
    # Path to video trajectory data (output from process_imu_data.py)
    DATA_DIR = Path(__file__).parent.parent.parent.parent / "data" / "processed" / "trajectories"
    TRAJ_FILE = DATA_DIR / f"traj_vid_{VIDEO_NUMBER}.csv"
    
    # Battery configuration (6S LiPo)
    V_BATTERY = 22.2  # Nominal voltage: 3.7V × 6 = 22.2V
    
    # Robot geometric parameters
    wheel_angles = [150, 270, 30]  # degrees
    d = 0.08  # m - distance from center to wheels
    r = 0.025  # m - wheel radius
    M = 3.178  # kg - robot mass
    
    # Parameters from Stage 1 and Stage 2 (UPDATE THESE WITH YOUR VALUES!)
    # NOTA: Usando valores razonables porque los datos experimentales tienen problemas
    Ra = 1.510   # Ω - Armature resistance (Stage 1 OK)
    K = 0.60   # V/(rad/s) - Motor constant (ajustado, Stage 1 tenía mucho ruido)
    I = 0.02  # kg·m² - Moment of inertia (ajustado, Stage 2 llegaba a límite)
    
    print(f"\nUsing parameters from previous stages:")
    print(f"  Ra = {Ra:.4f} Ω  (Stage 1)")
    print(f"  K  = {K:.4f} V/(rad/s)  (Stage 1)")
    print(f"  I  = {I:.4f} kg·m²  (Stage 2)")
    print(f"\n⚠ IMPORTANT: Update these values with your actual identified parameters!")
    
    # =========================================================================
    # 1. Load Trajectory Data
    # =========================================================================
    print("\n[1] Loading video trajectory data...")
    
    if not TRAJ_FILE.exists():
        print(f"\n❌ ERROR: Trajectory file not found: {TRAJ_FILE}")
        print(f"\nMake sure you have:")
        print(f"  1. Processed video {VIDEO_NUMBER} with track_simple_robust.py")
        print(f"  2. Run process_imu_data.py to generate traj_vid_processed files")
        print(f"  3. Copied the final CSV to: {TRAJ_FILE}")
        return
    
    data = load_stage3_video_data(TRAJ_FILE)
    t = data['t']
    y_measured = data['y_measured']
    u_pwm = data['u_pwm']
    
    # Adjust phi reference (subtract π/2)
    y_measured[:, 2] -= np.pi / 2
    
    # =========================================================================
    # 2. Convert PWM to Voltage
    # =========================================================================
    print("\n[2] Converting PWM to voltage...")
    print(f"  Battery voltage: {V_BATTERY}V")
    u_voltage = pwm_to_voltage_batch(u_pwm, V_BATTERY)
    
    print(f"  Voltage range:")
    print(f"    Motor 1: [{u_voltage[:, 0].min():.2f}, {u_voltage[:, 0].max():.2f}] V")
    print(f"    Motor 2: [{u_voltage[:, 1].min():.2f}, {u_voltage[:, 1].max():.2f}] V")
    print(f"    Motor 3: [{u_voltage[:, 2].min():.2f}, {u_voltage[:, 2].max():.2f}] V")
    
    # =========================================================================
    # 3. Run Stage 3 Identification
    # =========================================================================
    print(f"\n[3] Running Stage 3 identification...")
    
    # Results directory
    results_dir = Path(__file__).parent.parent.parent.parent / "results" / "identification" / "stage3"
    results_dir.mkdir(parents=True, exist_ok=True)
    
    # Initialize identifier
    identifier = ThreeStageIdentification(wheel_angles, d, r, M)
    
    # Run identification
    identifier.stage3_compensation(
        t=t,
        u=u_voltage,
        y_measured=y_measured,
        Ra=Ra,
        K=K,
        I=I,
        plot=True,
        results_dir=results_dir
    )
    
    # =========================================================================
    # Summary
    # =========================================================================
    print("\n" + "=" * 70)
    print("IDENTIFICATION COMPLETE")
    print("=" * 70)
    
    results = identifier.get_summary()
    
    if 'stage3' in results:
        Cu1 = results['stage3']['Cu1']
        Cu2 = results['stage3']['Cu2']
        Cu3 = results['stage3']['Cu3']
        
        print(f"\nIdentified Compensation Factors:")
        print(f"  Cu1 = {Cu1:.4f}")
        print(f"  Cu2 = {Cu2:.4f}")
        print(f"  Cu3 = {Cu3:.4f}")
        
        print(f"\nInterpretation:")
        if max(abs(Cu1-1), abs(Cu2-1), abs(Cu3-1)) < 0.05:
            print("  ✓ Motors are well-matched (deviations < 5%)")
        elif max(abs(Cu1-1), abs(Cu2-1), abs(Cu3-1)) < 0.15:
            print("  ⚠ Moderate motor asymmetry (compensate in controller)")
        else:
            print("  ⚠ Large motor asymmetry - check mechanical assembly")
    
    print("\n" + "=" * 70)
    print("Next Steps:")
    print("  1. Verify the plots show good agreement between measured and model")
    print("  2. Update Cu1, Cu2, Cu3 in your robot controller configuration")
    print("  3. Test on actual robot with compensated control inputs")
    print("=" * 70)
    
    plt.show()
    
    return identifier


if __name__ == "__main__":
    identifier = main()
