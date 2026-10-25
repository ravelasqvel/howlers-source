"""
Generate Embedded Data for ESP32S3

Converts experimental data to C header file with reduced dataset
for embedded execution on ESP32S3.
"""

import numpy as np
import sys
import os
from pathlib import Path

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from trajectory_generators import load_experimental_data

# Configuration
EXPERIMENT_NUMBER = 1
N_POINTS_EMBEDDED = 200  # Reduced dataset for embedded system
DT = 0.01

BASE_DIR = Path(__file__).parent.parent.parent
SENSORS_DATA_PATH = BASE_DIR / 'data' / 'sensors' / f'exp{EXPERIMENT_NUMBER}.txt'
TRAJECTORY_DATA_PATH = BASE_DIR / 'data' / 'processed' / 'trajectories' / f'traj_vid_{EXPERIMENT_NUMBER}.csv'
OUTPUT_PATH = BASE_DIR / 'State Estimation_C' / 'esp32s3_test' / 'main' / 'robot_data.h'


def generate_c_array(name, data, rows, cols=None):
    """Generate C array declaration from numpy array"""
    if cols is None:
        # 1D array
        lines = [f"const float {name}[{rows}] = {{"]
        for i in range(rows):
            if i % 4 == 0:
                lines.append("    ")
            lines[-1] += f"{data[i]:.6f}f"
            if i < rows - 1:
                lines[-1] += ", "
            if (i + 1) % 4 == 0 or i == rows - 1:
                lines.append("\n")
        lines.append("};\n")
    else:
        # 2D array
        lines = [f"const float {name}[{rows}][{cols}] = {{\n"]
        for i in range(rows):
            lines.append("    {")
            for j in range(cols):
                lines[-1] += f"{data[i, j]:.6f}f"
                if j < cols - 1:
                    lines[-1] += ", "
            lines[-1] += "}"
            if i < rows - 1:
                lines[-1] += ","
            lines[-1] += "\n"
        lines.append("};\n")
    
    return ''.join(lines)


def main():
    print("="*60)
    print("Generating Embedded Data for ESP32S3")
    print("="*60)
    
    # Load experimental data
    print(f"\nLoading experimental data from experiment {EXPERIMENT_NUMBER}...")
    data = load_experimental_data(
        sensors_path=SENSORS_DATA_PATH,
        trajectory_path=TRAJECTORY_DATA_PATH,
        N=N_POINTS_EMBEDDED,
        dt=DT,
        filter_measurements=False
    )
    
    time_array = data['time']
    controls = data['controls']
    measurements = data['measurements']
    ground_truth = data['ground_truth']
    N = len(time_array)
    
    print(f"Loaded {N} data points")
    print(f"  Controls shape: {controls.shape}")
    print(f"  Measurements shape: {measurements.shape}")
    print(f"  Ground truth shape: {ground_truth.shape}")
    
    # Generate header file
    print(f"\nGenerating C header file: {OUTPUT_PATH}")
    
    with open(OUTPUT_PATH, 'w') as f:
        f.write("""/*
 ============================================================================
 Name        : robot_data.h
 Author      : Auto-generated
 Description : Embedded experimental data for ESP32S3 filter comparison
               Generated from Python experimental data
 ============================================================================
*/

#ifndef ROBOT_DATA_H_
#define ROBOT_DATA_H_

#include <stddef.h>

/* ============================================================================
 * DATA CONFIGURATION
 * ============================================================================ */

#define N_DATA_POINTS {N}
#define DT {dt}f
#define STATE_DIM 6
#define MEAS_DIM 4
#define CONTROL_DIM 2

/* ============================================================================
 * EXPERIMENTAL DATA ARRAYS
 * ============================================================================ */

""".format(N=N, dt=DT))
        
        # Time array
        f.write("/* Time array [s] */\n")
        f.write(generate_c_array("time_data", time_array, N))
        f.write("\n")
        
        # Controls array [N x 2]
        f.write("/* Control inputs: [ax, ay] [m/s²] */\n")
        f.write(generate_c_array("controls_data", controls, N, 2))
        f.write("\n")
        
        # Measurements array [N x 4]
        f.write("/* Measurements: [vx_b, vy_b, omega, psi] */\n")
        f.write(generate_c_array("measurements_data", measurements, N, 4))
        f.write("\n")
        
        # Ground truth array [N x 6]
        f.write("/* Ground truth state: [x, y, psi, vx_b, vy_b, omega] */\n")
        f.write(generate_c_array("ground_truth_data", ground_truth, N, 6))
        f.write("\n")
        
        # Noise matrices
        f.write("/* Process noise covariance Q [6x6] - diagonal */\n")
        Q_diag = np.array([1.228769e-08, 1.241803e-08, 1.000000e-12, 
                          4.914677e-04, 4.966813e-04, 1.000000e-12])
        f.write(generate_c_array("Q_diagonal", Q_diag, 6))
        f.write("\n")
        
        f.write("/* Measurement noise covariance R [4x4] - diagonal */\n")
        R_diag = np.array([6.72e-4, 6.72e-4, 1.3125e-2, 4.06e-6])
        f.write(generate_c_array("R_diagonal", R_diag, 4))
        f.write("\n")
        
        # Initial covariance
        f.write("/* Initial state covariance P0 [6x6] - diagonal */\n")
        P0_diag = np.array([0.5, 0.5, 0.1, 0.2, 0.2, 0.05])
        f.write(generate_c_array("P0_diagonal", P0_diag, 6))
        f.write("\n")
        
        # Statistics
        f.write("/* Dataset statistics */\n")
        f.write(f"#define CONTROLS_MIN_AX {controls[:, 0].min():.6f}f\n")
        f.write(f"#define CONTROLS_MAX_AX {controls[:, 0].max():.6f}f\n")
        f.write(f"#define CONTROLS_MIN_AY {controls[:, 1].min():.6f}f\n")
        f.write(f"#define CONTROLS_MAX_AY {controls[:, 1].max():.6f}f\n")
        f.write(f"#define TOTAL_DURATION {time_array[-1]:.6f}f\n")
        f.write("\n")
        
        f.write("#endif /* ROBOT_DATA_H_ */\n")
    
    print(f"✓ Header file generated successfully")
    print(f"  Size: {OUTPUT_PATH.stat().st_size / 1024:.2f} KB")
    print(f"  Data points: {N}")
    print(f"  Duration: {time_array[-1]:.2f} seconds")
    print("\n" + "="*60)
    print("Generation Complete!")
    print("="*60)


if __name__ == "__main__":
    main()
