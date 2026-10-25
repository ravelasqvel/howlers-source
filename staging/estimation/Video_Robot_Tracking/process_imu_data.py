"""
IMU DATA PROCESSING
===================
Takes trajectory data from video tracking and replaces orientation (phi, omega)
with IMU data (alpha) from sensor files.

Reads:
  - traj_vid_data/traj_vid_X.csv (video tracking results)
  - Exp_videos_sen/expX.txt (IMU sensor data)

Generates:
  - traj_vid_processed/traj_vid_X.csv (combined data with IMU orientation)
  - traj_vid_processed/traj_vid_X.png (plots with IMU orientation)
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.ndimage import gaussian_filter1d


# Configuration: Process multiple experiments automatically
EXPERIMENTS_TO_PROCESS = [10]  # Experiments with both video and sensor data

# Paths
VIDEO_DATA_DIR = Path(__file__).parent.parent / "data" / "raw" / "video_trajectories"
SENSOR_DATA_DIR = Path(__file__).parent.parent / "data" / "raw" / "sensor_data"
OUTPUT_DIR = Path(__file__).parent.parent / "data" / "processed" / "trajectories"


def load_sensor_data(filepath):
    """Load sensor data from txt file
    
    Format: t, ax, ay, alpha, w1, w2, w3, u1, u2, u3, vbx_sp, vby_sp, wb_sp
    """
    print(f"Loading sensor data from {filepath}...")
    
    # Read file with comma separator
    data = pd.read_csv(filepath, skipinitialspace=True)
    
    # Extract time, alpha, and control inputs
    time_imu = data.iloc[:, 0].values   # Column 0: time
    alpha_imu = data.iloc[:, 3].values  # Column 3: alpha
    u1 = data.iloc[:, 7].values         # Column 7: u1 (PWM %)
    u2 = data.iloc[:, 8].values         # Column 8: u2 (PWM %)
    u3 = data.iloc[:, 9].values         # Column 9: u3 (PWM %)
    
    print(f"  ✓ Loaded {len(time_imu)} IMU samples")
    print(f"  Time range: {time_imu[0]:.4f}s to {time_imu[-1]:.4f}s")
    
    return time_imu, alpha_imu, u1, u2, u3


def load_video_data(filepath):
    """Load video tracking data from CSV"""
    print(f"Loading video data from {filepath}...")
    
    df = pd.read_csv(filepath)
    
    print(f"  ✓ Loaded {len(df)} video tracking samples")
    print(f"  Time range: {df['time_s'].iloc[0]:.4f}s to {df['time_s'].iloc[-1]:.4f}s")
    
    return df


def detect_first_movement(df_video, velocity_threshold=0.02, lookback_ms=30):
    """Detect first significant movement and set t=0 slightly before it
    
    Args:
        df_video: Video dataframe with velocities
        velocity_threshold: Threshold for detecting movement [m/s]
        lookback_ms: Milliseconds to go back from first movement for t=0
    
    Returns:
        Index where new t=0 should be
    """
    # Calculate total velocity magnitude
    vx = df_video['vx_m_s'].values
    vy = df_video['vy_m_s'].values
    v_total = np.sqrt(vx**2 + vy**2)
    
    # Find first point where velocity exceeds threshold
    movement_indices = np.where(v_total > velocity_threshold)[0]
    
    if len(movement_indices) == 0:
        print(f"  ⚠ No significant movement detected (threshold={velocity_threshold} m/s)")
        return 0
    
    first_movement_idx = movement_indices[0]
    
    # Go back by lookback_ms (at 100Hz, 30ms = 3 samples)
    lookback_samples = int(lookback_ms / 10)  # 10ms per sample at 100Hz
    new_zero_idx = max(0, first_movement_idx - lookback_samples)
    
    print(f"  ✓ First movement detected at index {first_movement_idx} ({df_video['time_s'].iloc[first_movement_idx]:.3f}s)")
    print(f"  ✓ New t=0 set at index {new_zero_idx} ({lookback_ms}ms before)")
    
    return new_zero_idx


def synchronize_and_merge(df_video, time_imu, alpha_imu, u1, u2, u3, target_samples=1000):
    """Synchronize IMU data with video data and merge
    
    Args:
        df_video: Video tracking dataframe
        time_imu: IMU time array
        alpha_imu: IMU orientation array
        u1, u2, u3: Control inputs (PWM %) arrays
        target_samples: Number of samples to keep (default 1000 = 10s at 100Hz)
    """
    print("\nSynchronizing IMU with video data...")
    
    # FIRST: Invert alpha (IMU measures opposite direction: CCW decreases instead of increases)
    alpha_imu_inverted = -alpha_imu
    
    # Detect first movement and adjust video data
    zero_idx = detect_first_movement(df_video)
    
    # Trim video data: start from new zero, take exactly target_samples
    df_video_trimmed = df_video.iloc[zero_idx:zero_idx + target_samples].copy()
    
    if len(df_video_trimmed) < target_samples:
        print(f"  ⚠ Warning: Only {len(df_video_trimmed)} samples available after trimming")
        # Pad with last values if needed
        samples_needed = target_samples - len(df_video_trimmed)
        last_row = df_video_trimmed.iloc[-1]
        padding = pd.DataFrame([last_row] * samples_needed)
        df_video_trimmed = pd.concat([df_video_trimmed, padding], ignore_index=True)
    
    # Reset video data to new time base: 0.00, 0.01, 0.02, ..., 9.99
    time_new = np.arange(target_samples) * 0.01  # Exact 10ms increments
    df_video_trimmed['time_s'] = time_new
    
    print(f"  ✓ Video data trimmed to {len(df_video_trimmed)} samples (0.00s to {time_new[-1]:.2f}s)")
    
    # SECOND: Unwrap alpha_imu to handle 2π→0 discontinuities BEFORE any adjustments
    alpha_imu_unwrapped = np.unwrap(alpha_imu_inverted)
    
    # THEN: Trim unwrapped IMU data to first target_samples (remove last ~200 samples)
    alpha_imu_trimmed = alpha_imu_unwrapped[:target_samples]
    u1_trimmed = u1[:target_samples]
    u2_trimmed = u2[:target_samples]
    u3_trimmed = u3[:target_samples]
    
    # Reset IMU time base to match: 0.00, 0.01, 0.02, ..., 9.99
    time_imu_trimmed = time_new
    
    print(f"  ✓ IMU data unwrapped and trimmed to {len(alpha_imu_trimmed)} samples (removed {len(alpha_imu) - target_samples} samples)")
    
    # FINALLY: Adjust alpha - Remove initial offset and set to pi/2 +- 5% random offset
    alpha_initial = alpha_imu_trimmed[0]
    pi_half = np.pi / 2
    random_offset = np.random.uniform(-0.05 * pi_half, 0.05 * pi_half)  # +-5% of pi/2
    alpha_adjusted = alpha_imu_trimmed - alpha_initial + pi_half + random_offset
    
    print(f"  ✓ Alpha adjusted: initial={alpha_initial:.4f} rad → new initial={alpha_adjusted[0]:.4f} rad ({np.degrees(alpha_adjusted[0]):.2f}°)")
    
    # alpha_adjusted is already unwrapped (came from unwrapped data)
    alpha_unwrapped = alpha_adjusted
    
    # Smooth alpha
    if len(alpha_unwrapped) > 5:
        alpha_smooth = gaussian_filter1d(alpha_unwrapped, sigma=3)
    else:
        alpha_smooth = alpha_unwrapped
    
    # Calculate omega from alpha (derivative with exact dt=0.01)
    omega_imu = np.gradient(alpha_smooth, time_new)
    
    # Smooth omega
    if len(omega_imu) > 3:
        omega_imu = gaussian_filter1d(omega_imu, sigma=2)
    
    # Replace phi and omega in dataframe, add control inputs
    df_merged = df_video_trimmed.copy()
    df_merged['phi_rad'] = alpha_smooth
    df_merged['omega_rad_s'] = omega_imu
    df_merged['u1_pwm'] = u1_trimmed
    df_merged['u2_pwm'] = u2_trimmed
    df_merged['u3_pwm'] = u3_trimmed
    
    print(f"  ✓ Final synchronized data: {len(df_merged)} samples")
    print(f"  Time range: {df_merged['time_s'].iloc[0]:.2f}s to {df_merged['time_s'].iloc[-1]:.2f}s")
    
    return df_merged


def plot_results_ieee(data, output_path):
    """Generate IEEE-format plots with IMU orientation data"""
    # IEEE style settings
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
    
    fig, axes = plt.subplots(2, 2, figsize=(7.16, 5.5))
    
    # (a) Trajectory with orientation
    axes[0, 0].plot(data['x_m'], data['y_m'], 'b-', linewidth=1.5, alpha=0.8)
    axes[0, 0].plot(data['x_m'].iloc[0], data['y_m'].iloc[0], 'go', markersize=8, label='Inicio', zorder=5)
    axes[0, 0].plot(data['x_m'].iloc[-1], data['y_m'].iloc[-1], 'ro', markersize=8, label='Final', zorder=5)
    
    # Orientation arrows from IMU
    step = max(1, len(data) // 12)
    for i in range(0, len(data), step):
        dx = 0.04 * np.cos(data['phi_rad'].iloc[i])
        dy = 0.04 * np.sin(data['phi_rad'].iloc[i])
        axes[0, 0].arrow(data['x_m'].iloc[i], data['y_m'].iloc[i], dx, dy,
                        head_width=0.015, head_length=0.01, fc='red', ec='red', alpha=0.5, zorder=4)
    
    axes[0, 0].set_xlabel('$x$ [m]')
    axes[0, 0].set_ylabel('$y$ [m]')
    axes[0, 0].set_title('(a) Trayectoria')
    axes[0, 0].legend(loc='best', framealpha=0.9)
    axes[0, 0].axis('equal')
    axes[0, 0].grid(True)
    
    # (b) Orientation from IMU
    axes[0, 1].plot(data['time_s'], data['phi_rad'], color='purple', linewidth=1.5)
    axes[0, 1].set_xlabel('Tiempo [s]')
    axes[0, 1].set_ylabel('$\\phi$ [rad]')
    axes[0, 1].set_title('(b) Orientación (IMU)')
    axes[0, 1].grid(True)
    
    # (c) Linear velocities
    axes[1, 0].plot(data['time_s'], data['vx_m_s'], 'r-', label='$v_x$', linewidth=1.5)
    axes[1, 0].plot(data['time_s'], data['vy_m_s'], 'b-', label='$v_y$', linewidth=1.5)
    axes[1, 0].set_xlabel('Tiempo [s]')
    axes[1, 0].set_ylabel('Velocidad [m/s]')
    axes[1, 0].set_title('(c) Velocidades Lineales')
    axes[1, 0].legend(loc='best', framealpha=0.9)
    axes[1, 0].grid(True)
    
    # (d) Angular velocity from IMU
    axes[1, 1].plot(data['time_s'], data['omega_rad_s'], color='orange', linewidth=1.5)
    axes[1, 1].set_xlabel('Tiempo [s]')
    axes[1, 1].set_ylabel('$\\omega$ [rad/s]')
    axes[1, 1].set_title('(d) Velocidad Angular (IMU)')
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"📊 Plots saved: {output_path}\n")
    plt.close()
    
    # Reset to defaults
    plt.rcParams.update(plt.rcParamsDefault)


def process_experiment(exp_number):
    """Process a single experiment"""
    print("\n" + "="*70)
    print(f"PROCESSING EXPERIMENT {exp_number}")
    print("="*70 + "\n")
    
    # Define file paths for this experiment
    video_data_file = VIDEO_DATA_DIR / f"traj_vid_{exp_number}.csv"
    sensor_data_file = SENSOR_DATA_DIR / f"exp{exp_number}.txt"
    output_csv = OUTPUT_DIR / f"traj_vid_{exp_number}.csv"
    output_plot = OUTPUT_DIR / f"traj_vid_{exp_number}.png"
    
    # Check input files exist
    if not video_data_file.exists():
        print(f"❌ ERROR: Video data file not found: {video_data_file}")
        print("   Skipping this experiment.\n")
        return False
    
    if not sensor_data_file.exists():
        print(f"❌ ERROR: Sensor data file not found: {sensor_data_file}")
        print("   Skipping this experiment.\n")
        return False
    
    # Load data
    try:
        df_video = load_video_data(video_data_file)
        time_imu, alpha_imu, u1, u2, u3 = load_sensor_data(sensor_data_file)
    except Exception as e:
        print(f"❌ Error loading data: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # Merge data
    try:
        df_merged = synchronize_and_merge(df_video, time_imu, alpha_imu, u1, u2, u3)
    except Exception as e:
        print(f"❌ Error merging data: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # Save CSV
    try:
        df_merged.to_csv(output_csv, index=False)
        print(f"\n💾 CSV saved: {output_csv} ({len(df_merged)} rows)")
        print("\nFirst 5 rows:")
        print(df_merged.head(5))
    except Exception as e:
        print(f"❌ Error saving CSV: {e}")
        return False
    
    # Generate plots
    try:
        plot_results_ieee(df_merged, output_plot)
    except Exception as e:
        print(f"⚠️  Error generating plots: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    print(f"✅ Experiment {exp_number} completed successfully\n")
    return True


def main():
    print("\n" + "="*70)
    print("IMU DATA PROCESSING - BATCH MODE")
    print("="*70)
    print(f"\nProcessing {len(EXPERIMENTS_TO_PROCESS)} experiments: {EXPERIMENTS_TO_PROCESS}\n")
    
    # Create output directory
    OUTPUT_DIR.mkdir(exist_ok=True)
    
    # Process each experiment
    results = {}
    for exp_num in EXPERIMENTS_TO_PROCESS:
        success = process_experiment(exp_num)
        results[exp_num] = success
    
    # Print summary
    print("\n" + "="*70)
    print("BATCH PROCESSING SUMMARY")
    print("="*70 + "\n")
    
    successful = [exp for exp, success in results.items() if success]
    failed = [exp for exp, success in results.items() if not success]
    
    print(f"✅ Successful: {len(successful)}/{len(EXPERIMENTS_TO_PROCESS)}")
    if successful:
        print(f"   Experiments: {successful}")
    
    if failed:
        print(f"\n❌ Failed: {len(failed)}/{len(EXPERIMENTS_TO_PROCESS)}")
        print(f"   Experiments: {failed}")
    
    print("\n" + "="*70)
    print("✅ BATCH PROCESSING COMPLETED")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
