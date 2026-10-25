"""
Compare All Three Filters

Runs EKF, UKF, and PF on the same data and compares their performance.
Generates comprehensive comparison figures with IEEE formatting.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import sys
import os
import time
import csv
from pathlib import Path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from state_estimation import ExtendedKalmanFilter, UnscentedKalmanFilter, MerweScaledSigmaPoints, ParticleFilter
from state_estimation.models import OmnidirectionalRobot
from state_estimation.metrics import compute_all_metrics, print_metrics
from state_estimation.visualization import plot_comparison
from state_estimation.common import make_residual_fn, normalize_angle

from trajectory_generators import (generate_synthetic_data, generate_square_trajectory, 
                                   generate_circular_trajectory, load_experimental_data)
from ukf_omnidirectional import state_mean_fn, z_mean_fn, residual_x, residual_z

# ============================================================================
# CONFIGURATION - Comparison Parameters
# ============================================================================
USE_EXPERIMENTAL_DATA = True
EXPERIMENT_NUMBER = 1

# Base directory
BASE_DIR = Path(__file__).parent.parent.parent
SENSORS_DATA_PATH = BASE_DIR / 'data' / 'sensors' / f'exp{EXPERIMENT_NUMBER}.txt'
TRAJECTORY_DATA_PATH = BASE_DIR / 'data' / 'processed' / 'trajectories' / f'traj_vid_{EXPERIMENT_NUMBER}.csv'
RESULTS_PATH = BASE_DIR / 'results' / 'estimation' / 'comparison'

# Data parameters
N_POINTS = 1000
DT = 0.01

# Visualization parameters
ENABLE_ZOOM = False  # Enable/disable zoom inset
ZOOM_TIME = 5.0  # Time in seconds for zoom inset
ZOOM_WINDOW = 1.5  # Window size in seconds around zoom time
ZOOM_SIZE = "25%"  # Size of zoom inset (percentage)
SHOW_ORIENTATION_ARROWS = False  # Enable/disable orientation arrows on trajectory plot
ARROW_DECIMATION = 50  # Show orientation arrow every N samples
ARROW_SCALE = 0.05  # Length scale for orientation arrows
# ============================================================================


def run_comparison():
    """Compare EKF, UKF, and PF with comprehensive visualization."""
    
    print("\n" + "="*60)
    print("Filter Comparison: EKF vs UKF vs PF")
    print("="*60 + "\n")
    
    # Generate or load data
    if USE_EXPERIMENTAL_DATA:
        print(f"Using experimental data from experiment {EXPERIMENT_NUMBER}...")
        data = load_experimental_data(
            sensors_path=SENSORS_DATA_PATH,
            trajectory_path=TRAJECTORY_DATA_PATH,
            N=N_POINTS,
            dt=DT,
            filter_measurements=False
        )
    else:
        print("Using synthetic data...")
        data = generate_circular_trajectory(N=1500, dt=0.01)
    
    time_array = data['time']
    controls = data['controls']
    measurements = data['measurements']
    ground_truth = data['ground_truth']
    dt = data['dt']
    N = len(time_array)

    # Define dimensions
    dim_x = 6  # State dimension
    dim_z = 4  # Measurement dimension
    dim_u = 2  # Control dimension (ax, ay)
    
    robot = OmnidirectionalRobot(dt=dt)
    
    # Common parameters (from individual implementations)
    x0 = np.zeros(dim_x)
    P0 = np.diag([0.5, 0.5, 0.1, 0.2, 0.2, 0.05])
    Q = np.diag([1.228769e-08, 1.241803e-08, 1.000000e-12, 4.914677e-04, 4.966813e-04, 1.000000e-12])
    R = np.diag([6.72e-4, 6.72e-4, 1.3125e-2, 4.06e-6])
    
    # ========== EKF ==========
    print("Running EKF...")
    ekf = ExtendedKalmanFilter(dim_x=dim_x, dim_z=dim_z, dim_u=dim_u)
    ekf.x, ekf.P, ekf.Q, ekf.R = x0.copy(), P0.copy(), Q.copy(), R.copy()
    ekf.set_residual_fn(make_residual_fn(angle_indices=[2]))
    
    ekf_estimates = np.zeros((N, dim_x))
    ekf_covariances = np.zeros((N, dim_x, dim_x))
    ekf_estimates[0] = ekf.x
    ekf_covariances[0] = ekf.P
    
    ekf_times = []
    for k in range(N - 1):
        t_start = time.perf_counter()
        ekf.predict(u=controls[k], f=robot.dynamics, F=robot.jacobian_F)
        ekf.update(z=measurements[k + 1], h=robot.measurement, H=robot.jacobian_H)
        t_end = time.perf_counter()
        ekf_times.append((t_end - t_start) * 1000)  # Convert to ms
        
        ekf_estimates[k + 1] = ekf.x
        ekf_covariances[k + 1] = ekf.P
        
        if (k + 1) % 200 == 0:
            print(f"  EKF: Step {k+1}/{N-1}")
    
    ekf_avg_time = np.mean(ekf_times)
    ekf_metrics = compute_all_metrics(ekf_estimates, ground_truth, ekf_covariances)
    print(f"EKF Average Time: {ekf_avg_time:.4f} ms/iteration")
    
    # ========== UKF ==========
    print("\nRunning UKF...")
    points = MerweScaledSigmaPoints(n=dim_x, alpha=0.5, beta=2.0, kappa=0.0)
    ukf = UnscentedKalmanFilter(dim_x=dim_x, dim_z=dim_z, dim_u=dim_u, points=points)
    ukf.x, ukf.P, ukf.Q, ukf.R = x0.copy(), P0.copy(), Q.copy()*1e4, R.copy()
    ukf.set_mean_fn(x_mean_fn=state_mean_fn, z_mean_fn=z_mean_fn)
    ukf.set_residual_fn(residual_x_fn=residual_x, residual_z_fn=residual_z)
    
    ukf_estimates = np.zeros((N, dim_x))
    ukf_covariances = np.zeros((N, dim_x, dim_x))
    ukf_estimates[0] = ukf.x
    ukf_covariances[0] = ukf.P
    
    ukf_times = []
    for k in range(N - 1):
        t_start = time.perf_counter()
        ukf.predict(u=controls[k], f=robot.dynamics)
        ukf.update(z=measurements[k + 1], h=robot.measurement)
        t_end = time.perf_counter()
        ukf_times.append((t_end - t_start) * 1000)  # Convert to ms
        
        ukf_estimates[k + 1] = ukf.x
        ukf_covariances[k + 1] = ukf.P
        
        if (k + 1) % 200 == 0:
            print(f"  UKF: Step {k+1}/{N-1}")
    
    ukf_avg_time = np.mean(ukf_times)
    ukf_metrics = compute_all_metrics(ukf_estimates, ground_truth, ukf_covariances)
    print(f"UKF Average Time: {ukf_avg_time:.4f} ms/iteration")
    
    # ========== PF ==========
    print("\nRunning PF...")
    pf = ParticleFilter(dim_x=dim_x, dim_z=dim_z, N=2000, dim_u=dim_u)
    pf.initialize_particles(x0=x0, P0=P0*1e-5)
    
    pf_estimates = np.zeros((N, dim_x))
    pf_estimates[0] = pf.x
    process_std = np.array([0.01, 0.01, 0.01, 0.05, 0.05, 0.05])
    
    pf_times = []
    for k in range(N - 1):
        t_start = time.perf_counter()
        pf.predict(u=controls[k], f=robot.dynamics, process_noise_std=process_std)
        pf.update(z=measurements[k + 1], h=robot.measurement, R=np.diag([6.72e-4, 6.72e-4, 1.3125e-2, 1.218e-3])*1e2)
        pf.resample(scheme='systematic')
        t_end = time.perf_counter()
        pf_times.append((t_end - t_start) * 1000)  # Convert to ms
        
        pf_estimates[k + 1] = pf.x
        
        if (k + 1) % 200 == 0:
            ess = pf.effective_sample_size()
            print(f"  PF: Step {k+1}/{N-1}, ESS: {ess:.1f}")
    
    pf_avg_time = np.mean(pf_times)
    pf_metrics = compute_all_metrics(pf_estimates, ground_truth)
    print(f"PF Average Time: {pf_avg_time:.4f} ms/iteration")
    
    # Print comparison
    print("\n" + "="*60)
    print("PERFORMANCE COMPARISON")
    print("="*60)
    print_metrics(ekf_metrics, filter_name="EKF")
    print_metrics(ukf_metrics, filter_name="UKF")
    print_metrics(pf_metrics, filter_name="PF")
    
    # Create results directory
    results_dir = RESULTS_PATH
    results_dir.mkdir(parents=True, exist_ok=True)
    print(f"\nSaving results to: {results_dir}")
    
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
    
    # ========== FIGURE 1: XY Trajectory with Zoom Inset and Orientation Arrows ==========
    print("\nGenerating Figure 1: XY Trajectory Comparison...")
    fig1, ax1 = plt.subplots(figsize=(7.16, 5.5))
    
    # Plot trajectories
    ax1.plot(ground_truth[:, 0], ground_truth[:, 1], 'k-', 
            linewidth=2.5, label='Trayectoria Real', alpha=0.8, zorder=1)
    ax1.plot(ekf_estimates[:, 0], ekf_estimates[:, 1], 'b--',
            linewidth=1.5, label='EKF', alpha=0.7, zorder=2)
    ax1.plot(ukf_estimates[:, 0], ukf_estimates[:, 1], 'r-.',
            linewidth=1.5, label='UKF', alpha=0.7, zorder=3)
    ax1.plot(pf_estimates[:, 0], pf_estimates[:, 1], 'g:',
            linewidth=1.8, label='PF', alpha=0.7, zorder=4)
    
    # Start and end markers
    ax1.plot(ground_truth[0, 0], ground_truth[0, 1], 'ko', 
            markersize=10, label='Inicio', zorder=10, markerfacecolor='lime', markeredgewidth=2)
    ax1.plot(ground_truth[-1, 0], ground_truth[-1, 1], 'ks', 
            markersize=10, label='Fin', zorder=10, markerfacecolor='red', markeredgewidth=2)
    
    # Orientation arrows (decimated) for all trajectories - optional
    if SHOW_ORIENTATION_ARROWS:
        arrow_indices = np.arange(0, N, ARROW_DECIMATION)
        for idx in arrow_indices:
            # Ground truth arrow (black)
            psi_gt = ground_truth[idx, 2] + np.pi / 2
            dx_gt = ARROW_SCALE * np.cos(psi_gt)
            dy_gt = ARROW_SCALE * np.sin(psi_gt)
            ax1.arrow(ground_truth[idx, 0], ground_truth[idx, 1], dx_gt, dy_gt,
                     head_width=0.015, head_length=0.01, fc='black', ec='black', 
                     alpha=0.7, linewidth=0.9, zorder=5)
            
            # EKF arrow (blue)
            psi_ekf = ekf_estimates[idx, 2] + np.pi / 2
            dx_ekf = ARROW_SCALE * np.cos(psi_ekf)
            dy_ekf = ARROW_SCALE * np.sin(psi_ekf)
            ax1.arrow(ekf_estimates[idx, 0], ekf_estimates[idx, 1], dx_ekf, dy_ekf,
                     head_width=0.012, head_length=0.008, fc='blue', ec='blue', 
                     alpha=0.5, linewidth=0.7, zorder=6)
            
            # UKF arrow (red)
            psi_ukf = ukf_estimates[idx, 2] + np.pi / 2
            dx_ukf = ARROW_SCALE * np.cos(psi_ukf)
            dy_ukf = ARROW_SCALE * np.sin(psi_ukf)
            ax1.arrow(ukf_estimates[idx, 0], ukf_estimates[idx, 1], dx_ukf, dy_ukf,
                     head_width=0.012, head_length=0.008, fc='red', ec='red', 
                     alpha=0.5, linewidth=0.7, zorder=7)
            
            # PF arrow (green)
            psi_pf = pf_estimates[idx, 2] + np.pi / 2
            dx_pf = ARROW_SCALE * np.cos(psi_pf)
            dy_pf = ARROW_SCALE * np.sin(psi_pf)
            ax1.arrow(pf_estimates[idx, 0], pf_estimates[idx, 1], dx_pf, dy_pf,
                     head_width=0.012, head_length=0.008, fc='green', ec='green', 
                     alpha=0.5, linewidth=0.7, zorder=8)
    
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.legend(loc='upper right', framealpha=0.95, ncol=2)
    ax1.grid(True)
    ax1.axis('equal')
    
    # Zoom inset (optional)
    if ENABLE_ZOOM:
        zoom_idx_center = int(ZOOM_TIME / dt)
        zoom_idx_window = int(ZOOM_WINDOW / dt)
        zoom_start = max(0, zoom_idx_center - zoom_idx_window)
        zoom_end = min(N, zoom_idx_center + zoom_idx_window)
        
        # Create inset axes
        axins = inset_axes(ax1, width=ZOOM_SIZE, height=ZOOM_SIZE, loc='lower left',
                           bbox_to_anchor=(0.05, 0.05, 1, 1), bbox_transform=ax1.transAxes)
        
        axins.plot(ground_truth[zoom_start:zoom_end, 0], ground_truth[zoom_start:zoom_end, 1], 
                  'k-', linewidth=2, alpha=0.8)
        axins.plot(ekf_estimates[zoom_start:zoom_end, 0], ekf_estimates[zoom_start:zoom_end, 1], 
                  'b--', linewidth=1.5, alpha=0.7)
        axins.plot(ukf_estimates[zoom_start:zoom_end, 0], ukf_estimates[zoom_start:zoom_end, 1], 
                  'r-.', linewidth=1.5, alpha=0.7)
        axins.plot(pf_estimates[zoom_start:zoom_end, 0], pf_estimates[zoom_start:zoom_end, 1], 
                  'g:', linewidth=1.8, alpha=0.7)
        
        axins.set_xlim(ground_truth[zoom_start:zoom_end, 0].min() - 0.02,
                       ground_truth[zoom_start:zoom_end, 0].max() + 0.02)
        axins.set_ylim(ground_truth[zoom_start:zoom_end, 1].min() - 0.02,
                       ground_truth[zoom_start:zoom_end, 1].max() + 0.02)
        axins.grid(True, alpha=0.3)
        axins.tick_params(labelsize=7)
        axins.set_title(f'Zoom en t={ZOOM_TIME}s', fontsize=8)
        
        # Mark zoom region on main plot
        x_zoom = ground_truth[zoom_start:zoom_end, 0]
        y_zoom = ground_truth[zoom_start:zoom_end, 1]
        rect = Rectangle((x_zoom.min()-0.02, y_zoom.min()-0.02), 
                         x_zoom.max()-x_zoom.min()+0.04, 
                         y_zoom.max()-y_zoom.min()+0.04,
                         linewidth=1, edgecolor='gray', facecolor='none', 
                         linestyle='--', alpha=0.7)
        ax1.add_patch(rect)
    
    plt.tight_layout()
    fig1_path = results_dir / 'fig1_trayectoria_comparativa.png'
    fig1.savefig(fig1_path, dpi=300, bbox_inches='tight')
    print(f"  Saved: {fig1_path}")
    plt.close(fig1)
    
    # ========== FIGURE 2: Position and Orientation Errors ==========
    print("\nGenerating Figure 2: Position and Orientation Errors...")
    
    # Compute errors
    error_x_ekf = ekf_estimates[:, 0] - ground_truth[:, 0]
    error_y_ekf = ekf_estimates[:, 1] - ground_truth[:, 1]
    error_psi_ekf = np.array([normalize_angle(ekf_estimates[i, 2] - ground_truth[i, 2]) 
                              for i in range(N)])
    
    error_x_ukf = ukf_estimates[:, 0] - ground_truth[:, 0]
    error_y_ukf = ukf_estimates[:, 1] - ground_truth[:, 1]
    error_psi_ukf = np.array([normalize_angle(ukf_estimates[i, 2] - ground_truth[i, 2]) 
                              for i in range(N)])
    
    error_x_pf = pf_estimates[:, 0] - ground_truth[:, 0]
    error_y_pf = pf_estimates[:, 1] - ground_truth[:, 1]
    error_psi_pf = np.array([normalize_angle(pf_estimates[i, 2] - ground_truth[i, 2]) 
                             for i in range(N)])
    
    fig2, axes = plt.subplots(3, 1, figsize=(7.16, 7))
    
    # Error in X
    axes[0].plot(time_array, error_x_ekf * 1000, 'b-', linewidth=1.5, label='EKF', alpha=0.8)
    axes[0].plot(time_array, error_x_ukf * 1000, 'r--', linewidth=1.5, label='UKF', alpha=0.8)
    axes[0].plot(time_array, error_x_pf * 1000, 'g-.', linewidth=1.5, label='PF', alpha=0.8)
    axes[0].axhline(y=0, color='k', linestyle='-', linewidth=0.8, alpha=0.3)
    axes[0].set_ylabel('$e_x$ [mm]')
    axes[0].legend(loc='best', framealpha=0.9, ncol=3)
    axes[0].grid(True)
    axes[0].set_title('Error en Posición X')
    
    # Error in Y
    axes[1].plot(time_array, error_y_ekf * 1000, 'b-', linewidth=1.5, label='EKF', alpha=0.8)
    axes[1].plot(time_array, error_y_ukf * 1000, 'r--', linewidth=1.5, label='UKF', alpha=0.8)
    axes[1].plot(time_array, error_y_pf * 1000, 'g-.', linewidth=1.5, label='PF', alpha=0.8)
    axes[1].axhline(y=0, color='k', linestyle='-', linewidth=0.8, alpha=0.3)
    axes[1].set_ylabel('$e_y$ [mm]')
    axes[1].legend(loc='best', framealpha=0.9, ncol=3)
    axes[1].grid(True)
    axes[1].set_title('Error en Posición Y')
    
    # Error in orientation
    axes[2].plot(time_array, np.rad2deg(error_psi_ekf), 'b-', linewidth=1.5, label='EKF', alpha=0.8)
    axes[2].plot(time_array, np.rad2deg(error_psi_ukf), 'r--', linewidth=1.5, label='UKF', alpha=0.8)
    axes[2].plot(time_array, np.rad2deg(error_psi_pf), 'g-.', linewidth=1.5, label='PF', alpha=0.8)
    axes[2].axhline(y=0, color='k', linestyle='-', linewidth=0.8, alpha=0.3)
    axes[2].set_xlabel('Tiempo [s]')
    axes[2].set_ylabel('$e_\\psi$ [deg]')
    axes[2].legend(loc='best', framealpha=0.9, ncol=3)
    axes[2].grid(True)
    axes[2].set_title('Error en Orientación')
    
    plt.tight_layout()
    fig2_path = results_dir / 'fig2_errores_posicion_orientacion.png'
    fig2.savefig(fig2_path, dpi=300, bbox_inches='tight')
    print(f"  Saved: {fig2_path}")
    plt.close(fig2)
    
    # ========== FIGURE 3: Execution Time Comparison ==========
    print("\nGenerating Figure 3: Execution Time Comparison...")
    
    fig3, ax3 = plt.subplots(figsize=(7.16, 4.5))
    
    filters = ['EKF', 'UKF', 'PF']
    exec_times = [ekf_avg_time, ukf_avg_time, pf_avg_time]
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']
    
    bars = ax3.bar(filters, exec_times, color=colors, alpha=0.7, 
                   edgecolor='black', linewidth=1.5, width=0.6)
    
    ax3.set_ylabel('Tiempo Promedio [ms/iteración]')
    ax3.set_title('Comparación de Tiempos de Ejecución')
    ax3.grid(True, alpha=0.3, axis='y')
    
    # Add value labels on bars
    for bar, val in zip(bars, exec_times):
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height,
                f'{val:.4f}', ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # Add horizontal reference line at minimum time
    min_time = min(exec_times)
    ax3.axhline(y=min_time, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    
    plt.tight_layout()
    fig3_path = results_dir / 'fig3_tiempos_ejecucion.png'
    fig3.savefig(fig3_path, dpi=300, bbox_inches='tight')
    print(f"  Saved: {fig3_path}")
    plt.close(fig3)
    
    # ========== Export Metrics and Timing to CSV ==========
    print("\nExporting metrics to CSV...")
    
    # Comprehensive metrics CSV with all states
    metrics_path = results_dir / 'comparison_metrics.csv'
    with open(metrics_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        
        # Header
        writer.writerow(['Métrica', 'EKF', 'UKF', 'PF'])
        writer.writerow([])  # Blank line
        
        # Execution times
        writer.writerow(['=== TIEMPOS DE EJECUCIÓN ==='])
        writer.writerow(['Tiempo Promedio [ms/iter]', 
                        f"{ekf_avg_time:.6f}",
                        f"{ukf_avg_time:.6f}",
                        f"{pf_avg_time:.6f}"])
        writer.writerow(['Tiempo Total [s]', 
                        f"{sum(ekf_times)/1000:.6f}",
                        f"{sum(ukf_times)/1000:.6f}",
                        f"{sum(pf_times)/1000:.6f}"])
        writer.writerow(['Tiempo Mínimo [ms/iter]', 
                        f"{min(ekf_times):.6f}",
                        f"{min(ukf_times):.6f}",
                        f"{min(pf_times):.6f}"])
        writer.writerow(['Tiempo Máximo [ms/iter]', 
                        f"{max(ekf_times):.6f}",
                        f"{max(ukf_times):.6f}",
                        f"{max(pf_times):.6f}"])
        writer.writerow(['Desv. Est. Tiempo [ms]', 
                        f"{np.std(ekf_times):.6f}",
                        f"{np.std(ukf_times):.6f}",
                        f"{np.std(pf_times):.6f}"])
        writer.writerow([])  # Blank line
        
        # Overall metrics
        writer.writerow(['=== MÉTRICAS GLOBALES ==='])
        writer.writerow(['RMSE Total [m]', 
                        f"{ekf_metrics['rmse_total']:.6f}",
                        f"{ukf_metrics['rmse_total']:.6f}",
                        f"{pf_metrics['rmse_total']:.6f}"])
        writer.writerow(['MAE Total [m]', 
                        f"{ekf_metrics['mae_total']:.6f}",
                        f"{ukf_metrics['mae_total']:.6f}",
                        f"{pf_metrics['mae_total']:.6f}"])
        writer.writerow([])  # Blank line
        
        # RMSE por estado
        writer.writerow(['=== RMSE POR ESTADO ==='])
        writer.writerow(['RMSE X [m]', 
                        f"{ekf_metrics['rmse'][0]:.6f}",
                        f"{ukf_metrics['rmse'][0]:.6f}",
                        f"{pf_metrics['rmse'][0]:.6f}"])
        writer.writerow(['RMSE Y [m]', 
                        f"{ekf_metrics['rmse'][1]:.6f}",
                        f"{ukf_metrics['rmse'][1]:.6f}",
                        f"{pf_metrics['rmse'][1]:.6f}"])
        writer.writerow(['RMSE ψ [rad]', 
                        f"{ekf_metrics['rmse'][2]:.6f}",
                        f"{ukf_metrics['rmse'][2]:.6f}",
                        f"{pf_metrics['rmse'][2]:.6f}"])
        writer.writerow(['RMSE Vx [m/s]', 
                        f"{ekf_metrics['rmse'][3]:.6f}",
                        f"{ukf_metrics['rmse'][3]:.6f}",
                        f"{pf_metrics['rmse'][3]:.6f}"])
        writer.writerow(['RMSE Vy [m/s]', 
                        f"{ekf_metrics['rmse'][4]:.6f}",
                        f"{ukf_metrics['rmse'][4]:.6f}",
                        f"{pf_metrics['rmse'][4]:.6f}"])
        writer.writerow(['RMSE ω [rad/s]', 
                        f"{ekf_metrics['rmse'][5]:.6f}",
                        f"{ukf_metrics['rmse'][5]:.6f}",
                        f"{pf_metrics['rmse'][5]:.6f}"])
        writer.writerow([])  # Blank line
        
        # MAE por estado
        writer.writerow(['=== MAE POR ESTADO ==='])
        writer.writerow(['MAE X [m]', 
                        f"{ekf_metrics['mae'][0]:.6f}",
                        f"{ukf_metrics['mae'][0]:.6f}",
                        f"{pf_metrics['mae'][0]:.6f}"])
        writer.writerow(['MAE Y [m]', 
                        f"{ekf_metrics['mae'][1]:.6f}",
                        f"{ukf_metrics['mae'][1]:.6f}",
                        f"{pf_metrics['mae'][1]:.6f}"])
        writer.writerow(['MAE ψ [rad]', 
                        f"{ekf_metrics['mae'][2]:.6f}",
                        f"{ukf_metrics['mae'][2]:.6f}",
                        f"{pf_metrics['mae'][2]:.6f}"])
        writer.writerow(['MAE Vx [m/s]', 
                        f"{ekf_metrics['mae'][3]:.6f}",
                        f"{ukf_metrics['mae'][3]:.6f}",
                        f"{pf_metrics['mae'][3]:.6f}"])
        writer.writerow(['MAE Vy [m/s]', 
                        f"{ekf_metrics['mae'][4]:.6f}",
                        f"{ukf_metrics['mae'][4]:.6f}",
                        f"{pf_metrics['mae'][4]:.6f}"])
        writer.writerow(['MAE ω [rad/s]', 
                        f"{ekf_metrics['mae'][5]:.6f}",
                        f"{ukf_metrics['mae'][5]:.6f}",
                        f"{pf_metrics['mae'][5]:.6f}"])
        writer.writerow([])  # Blank line
        
        # NEES and NIS if available (EKF and UKF only)
        if 'nees_mean' in ekf_metrics:
            writer.writerow(['=== MÉTRICAS DE CONSISTENCIA ==='])
            writer.writerow(['NEES Promedio', 
                            f"{ekf_metrics['nees_mean']:.6f}",
                            f"{ukf_metrics.get('nees_mean', 'N/A')}",
                            'N/A'])
            writer.writerow(['NEES Desv. Est.', 
                            f"{ekf_metrics['nees_std']:.6f}",
                            f"{ukf_metrics.get('nees_std', 'N/A')}",
                            'N/A'])
        if 'nis_mean' in ekf_metrics:
            writer.writerow(['NIS Promedio', 
                            f"{ekf_metrics['nis_mean']:.6f}",
                            f"{ukf_metrics.get('nis_mean', 'N/A')}",
                            'N/A'])
            writer.writerow(['NIS Desv. Est.', 
                            f"{ekf_metrics['nis_std']:.6f}",
                            f"{ukf_metrics.get('nis_std', 'N/A')}",
                            'N/A'])
    
    print(f"  Saved: {metrics_path}")
    
    # Reset matplotlib to defaults
    plt.rcParams.update(plt.rcParamsDefault)
    
    print("\n" + "="*60)
    print("Comparison Complete!")
    print(f"All results saved to: {results_dir}")
    print("="*60)


if __name__ == "__main__":
    run_comparison()
