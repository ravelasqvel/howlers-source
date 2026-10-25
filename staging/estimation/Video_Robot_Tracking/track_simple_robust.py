"""
ROBUST ROBOT TRACKING - Optical Flow
======================================
Uses Lucas-Kanade optical flow which is more robust than OpenCV trackers.
Detects dark blue/purple battery for orientation estimation.
"""

import cv2
import numpy as np
import pandas as pd
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt
from pathlib import Path


# Configuration
VIDEO_NUMBER = 10  # Change this to process different videos
VIDEO_DIR = Path("videos")
OUTPUT_DIR = Path(__file__).parent.parent / "data" / "raw" / "video_trajectories"
TILE_SIZE_M = 0.30
TARGET_DT = 0.01  # Target sampling time: 10ms (100 Hz)

# Paths
VIDEO_PATH = VIDEO_DIR / f"vid_{VIDEO_NUMBER}.mp4"
OUTPUT_CSV = OUTPUT_DIR / f"traj_vid_{VIDEO_NUMBER}.csv"
OUTPUT_PLOT = OUTPUT_DIR / f"traj_vid_{VIDEO_NUMBER}.png"

calibration_points = []


def calibrate_perspective(frame, tile_size_m):
    """Interactive perspective calibration"""
    global calibration_points
    calibration_points = []
    
    print("\n" + "="*70)
    print("PERSPECTIVE CALIBRATION")
    print("="*70)
    print("\n   Click on 4 CORNERS of ONE tile:")
    print("   ⬉ Top-Left → ⬈ Top-Right → ⬊ Bottom-Right → ⬋ Bottom-Left")
    print("   Press ESC when done\n")
    
    def mouse_callback(event, x, y, flags, param):
        global calibration_points
        if event == cv2.EVENT_LBUTTONDOWN and len(calibration_points) < 4:
            calibration_points.append([x, y])
            print(f"  ✓ Point {len(calibration_points)}: ({x}, {y})")
            cv2.circle(frame_copy, (x, y), 8, (0, 255, 0), -1)
            if len(calibration_points) == 4:
                pts = np.array(calibration_points, dtype=np.int32)
                cv2.polylines(frame_copy, [pts], True, (0, 255, 0), 3)
            cv2.imshow('Calibration', frame_copy)
    
    frame_copy = frame.copy()
    cv2.namedWindow('Calibration', cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('Calibration', mouse_callback)
    cv2.imshow('Calibration', frame_copy)
    
    while True:
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cv2.destroyAllWindows()
    
    if len(calibration_points) != 4:
        raise ValueError(f"Need 4 points")
    
    src_points = np.float32(calibration_points)
    target_size = 500
    dst_points = np.float32([[0, 0], [target_size, 0], [target_size, target_size], [0, target_size]])
    
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    pixels_per_meter = target_size / tile_size_m
    
    print(f"\n✅ Calibration complete: {pixels_per_meter:.2f} pixels/meter\n")
    return matrix, pixels_per_meter


def select_robot(frame):
    """Interactive robot selection"""
    print("\n" + "="*70)
    print("ROBOT SELECTION")
    print("="*70)
    print("\n   Drag a RECTANGLE around the robot")
    print("   Press SPACE or ENTER\n")
    
    cv2.namedWindow('Select Robot', cv2.WINDOW_NORMAL)
    bbox = cv2.selectROI('Select Robot', frame, fromCenter=False)
    cv2.destroyAllWindows()
    
    if bbox[2] == 0 or bbox[3] == 0:
        raise ValueError("Robot not selected")
    
    print(f"\n✅ Robot selected: {bbox}\n")
    return bbox


def detect_blue_battery_orientation(frame_bgr, cx, cy, search_size=100):
    """Detects orientation using the rectangular dark blue/purple battery"""
    cx, cy = int(cx), int(cy)
    half_size = search_size // 2
    
    # Extract search region around robot
    x1 = max(0, cx - half_size)
    y1 = max(0, cy - half_size)
    x2 = min(frame_bgr.shape[1], cx + half_size)
    y2 = min(frame_bgr.shape[0], cy + half_size)
    
    roi = frame_bgr[y1:y2, x1:x2]
    
    if roi.shape[0] < 20 or roi.shape[1] < 20:
        return None
    
    # Convert to HSV for dark blue/purple detection
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Range for dark blue/purple (adjusted for battery color)
    lower_blue = np.array([100, 80, 30])
    upper_blue = np.array([150, 255, 180])
    
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    # Morphological cleanup
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 0:
        return None
    
    # Find largest contour (battery)
    largest_contour = max(contours, key=cv2.contourArea)
    
    if cv2.contourArea(largest_contour) < 50:  # Too small
        return None
    
    # Get oriented rectangle
    rect = cv2.minAreaRect(largest_contour)
    angle = rect[2]  # Angle in degrees
    
    # cv2.minAreaRect returns angle in range [-90, 0]
    # Adjust based on rectangle dimensions
    width, height = rect[1]
    
    if width < height:
        angle = angle + 90
    
    # Convert to radians in range [0, 2π]
    phi = np.deg2rad(angle) % (2 * np.pi)
    
    return phi


def track_with_optical_flow(video_path, bbox, perspective_matrix, pixels_per_meter):
    """Tracking using Lucas-Kanade Optical Flow + Dark blue battery detection for orientation"""
    print("\n" + "="*70)
    print("PROCESSING VIDEO WITH OPTICAL FLOW")
    print("="*70 + "\n")
    
    cap = cv2.VideoCapture(video_path)
    fps = cap.get(cv2.CAP_PROP_FPS)
    dt = 1.0 / fps if fps > 0 else 0.033
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    # Read first frame
    ret, old_frame = cap.read()
    if not ret:
        raise ValueError("Could not read first frame")
    
    old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
    
    # Create points to track in bbox
    x, y, w, h = [int(v) for v in bbox]
    
    # Create grid of points in bbox
    points_x = np.linspace(x + 5, x + w - 5, 10)
    points_y = np.linspace(y + 5, y + h - 5, 10)
    points = []
    for py in points_y:
        for px in points_x:
            points.append([px, py])
    
    p0 = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
    
    # Optical flow parameters
    lk_params = dict(
        winSize=(15, 15),
        maxLevel=2,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
    )
    
    positions = []
    orientations = []
    timestamps = []
    frame_idx = 1
    last_valid_phi = 0.0
    
    print("Processing frames...")
    print("(Detecting dark blue battery for orientation)\n")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        
        # Select good points
        if p1 is not None:
            good_new = p1[st == 1]
            good_old = p0[st == 1]
            
            if len(good_new) > 5:  # At least 5 valid points
                # Calculate robot center
                cx = np.mean(good_new[:, 0])
                cy = np.mean(good_new[:, 1])
                
                # Detect orientation using dark blue battery
                phi = detect_blue_battery_orientation(frame, cx, cy)
                if phi is not None:
                    last_valid_phi = phi
                else:
                    phi = last_valid_phi
                
                # Transform to corrected perspective
                if perspective_matrix is not None:
                    point = np.array([[[cx, cy]]], dtype=np.float32)
                    transformed = cv2.perspectiveTransform(point, perspective_matrix)
                    cx_t, cy_t = transformed[0][0]
                else:
                    cx_t, cy_t = cx, cy
                    
                # Convert to meters
                x_m = cx_t / pixels_per_meter
                y_m = cy_t / pixels_per_meter
                
                positions.append([x_m, y_m])
                orientations.append(phi)
                timestamps.append(frame_idx * dt)
                
                # Update points for next frame
                p0 = good_new.reshape(-1, 1, 2)
            else:
                # Lost tracking, use last valid position
                positions.append([np.nan, np.nan])
                orientations.append(np.nan)
                timestamps.append(frame_idx * dt)
        else:
            positions.append([np.nan, np.nan])
            orientations.append(np.nan)
            timestamps.append(frame_idx * dt)
        
        old_gray = frame_gray.copy()
        frame_idx += 1
        
        if frame_idx % 30 == 0:
            progress = (frame_idx / frame_count) * 100
            print(f"  {progress:.1f}% ({frame_idx}/{frame_count})")
    
    cap.release()
    print(f"\n✅ Tracking completado: {frame_idx} frames\n")
    
    return np.array(positions), np.array(orientations), np.array(timestamps)


def process_data(positions, orientations, timestamps, target_dt=0.01):
    """Process data and calculate velocities with fixed sampling time"""
    print("Processing data...")
    
    x_vals = positions[:, 0]
    y_vals = positions[:, 1]
    phi_vals = orientations
    
    # Interpolate missing values
    valid_idx = ~np.isnan(x_vals)
    if np.sum(valid_idx) < 2:
        raise ValueError("Too few valid detections")
    
    x_interp = np.interp(timestamps, timestamps[valid_idx], x_vals[valid_idx])
    y_interp = np.interp(timestamps, timestamps[valid_idx], y_vals[valid_idx])
    
    # Resample to fixed dt (10ms = 100 Hz)
    t_start = timestamps[0]
    t_end = timestamps[-1]
    timestamps_fixed = np.arange(t_start, t_end, target_dt)
    
    x_smooth = np.interp(timestamps_fixed, timestamps, x_interp)
    y_smooth = np.interp(timestamps_fixed, timestamps, y_interp)
    
    # Set origin (0,0) to initial robot position and invert Y axis
    x0 = x_smooth[0]
    y0 = y_smooth[0]
    x_smooth = x_smooth - x0
    y_smooth = -(y_smooth - y0)  # Invert Y axis (upward is positive)
    
    timestamps = timestamps_fixed
    
    # For orientation, handle wrapping at ±π
    valid_phi = ~np.isnan(phi_vals)
    if np.sum(valid_phi) < 2:
        phi_smooth = np.zeros_like(timestamps)
    else:
        # First interpolate to original timestamps
        phi_interp_orig = np.interp(timestamps_fixed[:len(phi_vals)], 
                                     np.arange(len(phi_vals)) * (timestamps_fixed[1] - timestamps_fixed[0]), 
                                     phi_vals)
        # Then resample to fixed dt
        phi_interp = np.interp(timestamps, timestamps_fixed[:len(phi_vals)], phi_interp_orig)
        # Unwrap to avoid jumps from -π to +π
        phi_unwrapped = np.unwrap(phi_interp)
        phi_smooth = phi_unwrapped if len(phi_unwrapped) <= 5 else gaussian_filter1d(phi_unwrapped, sigma=3)
    
    # Smooth positions
    if len(x_smooth) > 5:
        x_smooth = gaussian_filter1d(x_smooth, sigma=3)
        y_smooth = gaussian_filter1d(y_smooth, sigma=3)
    
    # Calculate linear velocities (Y already inverted in position)
    vx = np.gradient(x_smooth, timestamps)
    vy = np.gradient(y_smooth, timestamps)
    
    # Calculate angular velocity
    omega = np.gradient(phi_smooth, timestamps)
    
    if len(vx) > 3:
        vx = gaussian_filter1d(vx, sigma=2)
        vy = gaussian_filter1d(vy, sigma=2)
        omega = gaussian_filter1d(omega, sigma=2)
    
    print(f"✅ Data processed\n")
    
    return {
        'time': timestamps,
        'x': x_smooth,
        'y': y_smooth,
        'phi': phi_smooth,
        'vx': vx,
        'vy': vy,
        'omega': omega
    }


def export_csv(data, output_path):
    """Export data to CSV"""
    df = pd.DataFrame({
        'time_s': data['time'],
        'x_m': data['x'],
        'y_m': data['y'],
        'phi_rad': data['phi'],
        'vx_m_s': data['vx'],
        'vy_m_s': data['vy'],
        'omega_rad_s': data['omega']
    })
    
    df.to_csv(output_path, index=False)
    print(f"💾 CSV saved: {output_path} ({len(df)} rows)\n")
    return df


def plot_results(data, output_path):
    """Generate result plots in IEEE format with 4 subplots"""
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
    
    fig, axes = plt.subplots(2, 2, figsize=(7.16, 5.5))  # IEEE double column width
    
    # (a) Trajectory with orientation
    axes[0, 0].plot(data['x'], data['y'], 'b-', linewidth=1.5, alpha=0.8)
    axes[0, 0].plot(data['x'][0], data['y'][0], 'go', markersize=8, label='Inicio', zorder=5)
    axes[0, 0].plot(data['x'][-1], data['y'][-1], 'ro', markersize=8, label='Final', zorder=5)
    # Orientation arrows
    step = max(1, len(data['x']) // 12)
    for i in range(0, len(data['x']), step):
        dx = 0.04 * np.cos(data['phi'][i])
        dy = 0.04 * np.sin(data['phi'][i])
        axes[0, 0].arrow(data['x'][i], data['y'][i], dx, dy,
                        head_width=0.015, head_length=0.01, fc='red', ec='red', alpha=0.5, zorder=4)
    axes[0, 0].set_xlabel('$x$ [m]')
    axes[0, 0].set_ylabel('$y$ [m]')
    axes[0, 0].set_title('(a) Trayectoria')
    axes[0, 0].legend(loc='best', framealpha=0.9)
    axes[0, 0].axis('equal')
    axes[0, 0].grid(True)
    
    # (b) Orientation
    axes[0, 1].plot(data['time'], data['phi'], color='purple', linewidth=1.5)
    axes[0, 1].set_xlabel('Tiempo [s]')
    axes[0, 1].set_ylabel('$\\phi$ [rad]')
    axes[0, 1].set_title('(b) Orientación')
    axes[0, 1].grid(True)
    
    # (c) Linear velocities
    axes[1, 0].plot(data['time'], data['vx'], 'r-', label='$v_x$', linewidth=1.5)
    axes[1, 0].plot(data['time'], data['vy'], 'b-', label='$v_y$', linewidth=1.5)
    axes[1, 0].set_xlabel('Tiempo [s]')
    axes[1, 0].set_ylabel('Velocidad [m/s]')
    axes[1, 0].set_title('(c) Velocidades Lineales')
    axes[1, 0].legend(loc='best', framealpha=0.9)
    axes[1, 0].grid(True)
    
    # (d) Angular velocity
    axes[1, 1].plot(data['time'], data['omega'], color='orange', linewidth=1.5)
    axes[1, 1].set_xlabel('Tiempo [s]')
    axes[1, 1].set_ylabel('$\\omega$ [rad/s]')
    axes[1, 1].set_title('(d) Velocidad Angular')
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"📊 Plots saved: {output_path}\n")
    plt.close()
    
    # Reset to defaults
    plt.rcParams.update(plt.rcParamsDefault)


def main():
    print("\n" + "="*70)
    print("TRACKING WITH OPTICAL FLOW")
    print("="*70)
    
    # Create output directory
    OUTPUT_DIR.mkdir(exist_ok=True)
    
    if not VIDEO_PATH.exists():
        print(f"\n❌ ERROR: Video not found '{VIDEO_PATH}'")
        return
    
    print(f"\n📹 Video: {VIDEO_PATH}")
    print(f"📏 Tile size: {TILE_SIZE_M*100:.0f} cm")
    print(f"⏱️  Target dt: {TARGET_DT*1000:.0f} ms ({1/TARGET_DT:.0f} Hz)\n")
    
    # Read first frame
    cap = cv2.VideoCapture(VIDEO_PATH)
    ret, first_frame = cap.read()
    cap.release()
    
    if not ret:
        print(f"❌ ERROR: Could not read video")
        return
    
    # Select robot
    try:
        bbox = select_robot(first_frame.copy())
    except Exception as e:
        print(f"\n❌ Error selecting robot: {e}")
        return
    
    # Calibrate perspective
    try:
        matrix, ppm = calibrate_perspective(first_frame.copy(), TILE_SIZE_M)
    except Exception as e:
        print(f"\n❌ Error in calibration: {e}")
        return
    
    # Process video
    try:
        positions, orientations, timestamps = track_with_optical_flow(VIDEO_PATH, bbox, matrix, ppm)
    except Exception as e:
        print(f"\n❌ Error during tracking: {e}")
        import traceback
        traceback.print_exc()
        return
    
    # Process data
    try:
        data = process_data(positions, orientations, timestamps, TARGET_DT)
    except Exception as e:
        print(f"❌ Error processing data: {e}")
        return
    
    # Export CSV
    try:
        df = export_csv(data, OUTPUT_CSV)
        print(df.head(10))
    except Exception as e:
        print(f"❌ Error exporting CSV: {e}")
        return
    
    # Plot
    try:
        plot_results(data, OUTPUT_PLOT)
    except Exception as e:
        print(f"⚠️  Error in plots: {e}")
    
    print("="*70)
    print("✅ PROCESS COMPLETED")
    print("="*70 + "\n")


if __name__ == "__main__":
    main()
