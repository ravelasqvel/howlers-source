# Video Tracking System for Omnidirectional Robots

Automatic extraction of complete trajectories `[x, y, φ, vx, vy, ω]` using optical flow and IMU data fusion.

---

## Folder Structure

```
Video_Robot_Tracking/
├── videos/                 # Experiment videos
├── Exp_videos_sen/        # IMU data (.txt)
├── traj_vid_data/         # Intermediate output (video only)
├── traj_vid_processed/    # ✓ Final output (video + IMU)
├── track_simple_robust.py # 1. Video processing
└── process_imu_data.py    # 2. IMU fusion
```

---

## Quick Start

### **Step 1:** Process Video
```bash
python track_simple_robust.py
```
- Configure `VIDEO_NUMBER = 1` (line 16)
- Interactive: select robot and calibrate perspective (30cm tile)
- Output: `traj_vid_data/traj_vid_1.csv`

### **Step 2:** Fuse with IMU (Automatic Batch)
```bash
python process_imu_data.py
```
- Automatically processes experiments 1-10
- **Final output:** `traj_vid_processed/traj_vid_X.csv` (ready for identification)

---

## Data Format

### Final Output (`traj_vid_processed/traj_vid_X.csv`)
| Column | Description | Unit |
|--------|-------------|------|
| `time_s` | Time (100 Hz, dt=0.01s) | s |
| `x_m`, `y_m` | Position (origin at t=0) | m |
| `phi_rad` | Orientation (IMU, inverted and unwrapped) | rad |
| `vx_m_s`, `vy_m_s` | Linear velocities | m/s |
| `omega_rad_s` | Angular velocity (gradient of φ) | rad/s |

**Key Features:**
- ✓ Exactly 1000 samples (10 seconds @ 100 Hz)
- ✓ Automatic synchronization: t=0 starts 30ms before first movement
- ✓ Initial orientation: φ₀ ≈ π/2 ± 5%
- ✓ Coordinate system: origin at initial position, Y pointing upward

---

## Implemented Methods

### Visual Tracking (Lucas-Kanade Optical Flow)
- Grid of 100 points over the robot
- Orientation detection: blue/purple battery (HSV: [100-150, 80-255, 30-180])
- Perspective calibration with 30×30 cm tile

### IMU Processing
- **Inversion correction:** α inverted (`-alpha`) because CCW decreases in sensor
- **Unwrap:** removes 2π→0 discontinuities **before** adjusting offset
- **Smoothing:** Gaussian filter (σ=3 for α, σ=2 for ω)
- **Synchronization:** automatic movement detection (threshold: 0.02 m/s)

### Movement Detection
- Velocity threshold: `√(vx² + vy²) > 0.02 m/s`
- Lookback: t=0 is placed 30ms (3 samples) before first movement
- Ensures capture of initial transient

---

## Visualization (IEEE Format)

4-subplot figures (7.16" × 5.5", 300 DPI, Times New Roman):
- **(a)** X-Y trajectory with orientation vectors
- **(b)** Orientation φ(t)
- **(c)** Linear velocities vx, vy
- **(d)** Angular velocity ω(t)

---

## Requirements

```bash
pip install opencv-contrib-python numpy pandas scipy matplotlib
```

**OpenCV Version:** 4.12.0 (bug in `tracker.init()`, hence optical flow is used)

---

## Important Notes

- **Data for identification:** use only files in `traj_vid_processed/`
- **Orientation φ and ω:** come exclusively from IMU (high precision)
- **Positions x, y:** come from video (perspective-calibrated)
- **Batch processing:** `process_imu_data.py` automatically processes experiments 1-10
