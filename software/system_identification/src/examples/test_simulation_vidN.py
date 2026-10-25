"""
Test Simulation with Experimental Video Trajectory Data.

Purpose: Verify if the robot model with estimated parameters can replicate
the measured trajectory using only control inputs.

This script loads experimental data from video tracking + IMU fusion and
simulates the robot model to compare against measured states.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# Add parent directory to path
sys.path.append(str(Path(__file__).parent.parent.parent))

from src.models.robot import OmnidirectionalRobot

def main():
    # =========================================================================
    # Configuration
    # =========================================================================
    
    # Video number to test (1-10)
    VIDEO_NUMBER = 2
    
    print("=" * 70)
    print(f"PRUEBA DE SIMULACION - DATOS EXPERIMENTALES VIDEO {VIDEO_NUMBER}")
    print("=" * 70)
    
    # =========================================================================
    # 1. Cargar datos experimentales
    # =========================================================================
    DATA_DIR = Path(__file__).parent.parent.parent.parent / "data" / "processed" / "trajectories"
    data_file = DATA_DIR / f"traj_vid_{VIDEO_NUMBER}.csv"
    
    if not data_file.exists():
        print(f"\n❌ ERROR: Trajectory file not found: {data_file}")
        print(f"\nMake sure you have:")
        print(f"  1. Processed video {VIDEO_NUMBER} with track_simple_robust.py")
        print(f"  2. Run process_imu_data.py to generate traj_vid_processed files")
        return
    
    df = pd.read_csv(data_file)
    
    print(f"\n[1] Datos cargados:")
    print(f"    Filas: {len(df)}")
    print(f"    Duración: {df['time_s'].iloc[-1]:.2f} s")
    print(f"    Frecuencia: {1/(df['time_s'].iloc[1] - df['time_s'].iloc[0]):.1f} Hz")
    
    # Extraer tiempo
    t = df['time_s'].values
    
    # Extraer estados medidos: [x, y, phi, vx, vy, omega]
    y_measured = np.column_stack([
        df['x_m'].values,
        df['y_m'].values,
        df['phi_rad'].values,
        df['vx_m_s'].values,
        df['vy_m_s'].values,
        df['omega_rad_s'].values
    ])
    
    # Ajustar referencia de phi (restar π/2 para alinear con referencia del modelo)
    y_measured[:, 2] -= np.pi / 2
    
    # Extraer controles PWM y convertir a voltaje
    u_pwm = np.column_stack([
        df['u1_pwm'].values,
        df['u2_pwm'].values,
        df['u3_pwm'].values
    ])
    
    # Configuración de batería (6S LiPo)
    V_BATTERY = 22.2  # Nominal: 3.7V × 6 = 22.2V
    u_voltage = (u_pwm / 100.0) * V_BATTERY
    
    print(f"\n[2] Entradas de control:")
    print(f"    PWM → Voltage conversion at {V_BATTERY}V")
    print(f"    Motor 1: [{u_voltage[:, 0].min():.2f}, {u_voltage[:, 0].max():.2f}] V")
    print(f"    Motor 2: [{u_voltage[:, 1].min():.2f}, {u_voltage[:, 1].max():.2f}] V")
    print(f"    Motor 3: [{u_voltage[:, 2].min():.2f}, {u_voltage[:, 2].max():.2f}] V")
    
    # =========================================================================
    # 2. Configurar parámetros del robot
    # =========================================================================
    
    # Parámetros geométricos (conocidos/medidos)
    wheel_angles = [150, 270, 30]  # grados
    d = 0.08   # m - distance from center to wheels
    r = 0.025  # m - wheel radius
    M = 3.178   # kg - robot mass
    
    # Parámetros identificados (from Stage 1, 2, 3)
    # NOTE: Update these values with your identified parameters!
    Ra = 1.50   # Ω - Armature resistance (Stage 1)
    K = 0.60   # V/(rad/s) - Motor constant (Stage 1)
    I = 0.02    # kg·m² - Moment of inertia (Stage 2)
    
    print(f"\n[3] Parámetros del modelo:")
    print(f"    Geometric:")
    print(f"      d = {d:.4f} m")
    print(f"      r = {r:.4f} m")
    print(f"      M = {M:.3f} kg")
    print(f"    Identified:")
    print(f"      Ra = {Ra:.4f} Ω (Stage 1)")
    print(f"      K  = {K:.4f} V/(rad/s) (Stage 1)")
    print(f"      I  = {I:.6f} kg·m² (Stage 2)")
    print(f"\n    ⚠ Update Ra, K, I with your actual identified values!")
    
    # =========================================================================
    # 3. Simular el robot
    # =========================================================================
    print(f"\n[4] Simulando trayectoria...")
    
    params = {
        'wheel_angles': wheel_angles,
        'd': d,
        'r': r,
        'M': M,
        'I': I,
        'K': K,
        'Ra': Ra
    }
    
    robot = OmnidirectionalRobot(params)
    
    # Condición inicial: usar el primer estado medido
    x0 = y_measured[0].copy()
    
    # Simular
    y_sim, success = robot.simulate(t, u_voltage, x0)
    
    if not success:
        print("    ⚠ Warning: Integración tuvo problemas")
    else:
        print("    ✓ Simulación exitosa")
    
    # =========================================================================
    # 4. Calcular errores
    # =========================================================================
    print(f"\n[5] Errores entre simulación y medición:")
    
    labels = ['x [m]', 'y [m]', 'φ [rad]', 'vx [m/s]', 'vy [m/s]', 'ω [rad/s]']
    
    for i in range(6):
        rmse = np.sqrt(np.mean((y_measured[:, i] - y_sim[:, i])**2))
        mae = np.mean(np.abs(y_measured[:, i] - y_sim[:, i]))
        print(f"    {labels[i]:12s}: RMSE = {rmse:.6f}, MAE = {mae:.6f}")
    
    # =========================================================================
    # 5. Graficar
    # =========================================================================
    print(f"\n[6] Generando gráficas...")
    
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
    
    # ========== FIGURA 1: Estados temporales (2x3) ==========
    fig, axes = plt.subplots(2, 3, figsize=(7.16, 5.5))
    
    state_labels = ['X [m]', 'Y [m]', '$\\phi$ [rad]', '$v_x$ [m/s]', '$v_y$ [m/s]', '$\\omega$ [rad/s]']
    state_indices = [0, 1, 2, 3, 4, 5]
    
    for i, (label, idx) in enumerate(zip(state_labels, state_indices)):
        row = i // 3
        col = i % 3
        ax = axes[row, col]
        
        ax.plot(t, y_measured[:, idx], 'b-', label='Medido', linewidth=2, alpha=0.7)
        ax.plot(t, y_sim[:, idx], 'r--', label='Simulado', linewidth=1.5)
        
        ax.set_ylabel(label)
        if row == 1:  # Bottom row
            ax.set_xlabel('Tiempo [s]')
        ax.legend(loc='best', framealpha=0.9)
        ax.grid(True)
    
    plt.tight_layout()
    
    # Guardar
    results_dir = Path(__file__).parent.parent.parent.parent / "results" / "identification" / "simulation_test"
    results_dir.mkdir(parents=True, exist_ok=True)
    fig_path = results_dir / f"test_simulation_vid{VIDEO_NUMBER}_estados.png"
    fig.savefig(fig_path, dpi=300, bbox_inches='tight')
    print(f"\n✓ Figura 1 guardada: {fig_path}")
    
    # =========================================================================
    # 6. Gráfica de trayectoria 2D con orientación
    # =========================================================================
    fig2, ax2 = plt.subplots(figsize=(7.16, 5.5))
    
    # Trayectorias
    ax2.plot(y_measured[:, 0], y_measured[:, 1], 'b-', label='Medición', 
            linewidth=2, alpha=0.7)
    ax2.plot(y_sim[:, 0], y_sim[:, 1], 'r--', label='Modelo', linewidth=2)
    
    # Marcadores de inicio y fin
    ax2.plot(y_measured[0, 0], y_measured[0, 1], 'go', markersize=10, 
            label='Inicio', zorder=5)
    ax2.plot(y_measured[-1, 0], y_measured[-1, 1], 'rs', markersize=10, 
            label='Fin', zorder=5)
    
    # Flechas de orientación (espaciadas cada N puntos)
    N_arrows = 15
    indices = np.linspace(0, len(t)-1, N_arrows, dtype=int)
    arrow_scale = 0.03
    
    for idx in indices:
        # Medición
        x, y, phi = y_measured[idx, 0], y_measured[idx, 1], y_measured[idx, 2]
        dx, dy = arrow_scale * np.cos(phi), arrow_scale * np.sin(phi)
        ax2.arrow(x, y, dx, dy, head_width=0.01, head_length=0.015, 
                 fc='blue', ec='blue', alpha=0.5, linewidth=0.8)
        
        # Modelo
        x, y, phi = y_sim[idx, 0], y_sim[idx, 1], y_sim[idx, 2]
        dx, dy = arrow_scale * np.cos(phi), arrow_scale * np.sin(phi)
        ax2.arrow(x, y, dx, dy, head_width=0.01, head_length=0.015, 
                 fc='red', ec='red', alpha=0.4, linewidth=0.8, linestyle='--')
    
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.legend(loc='best', framealpha=0.9)
    ax2.grid(True)
    ax2.axis('equal')
    plt.tight_layout()
    
    fig2_path = results_dir / f"test_simulation_vid{VIDEO_NUMBER}_trayectoria.png"
    fig2.savefig(fig2_path, dpi=300, bbox_inches='tight')
    print(f"✓ Figura 2 guardada: {fig2_path}")
    
    # Reset to defaults
    plt.rcParams.update(plt.rcParamsDefault)
    
    # =========================================================================
    # 7. Mostrar gráficas y resumen
    # =========================================================================
    
    print("\n" + "=" * 70)
    print("PRUEBA COMPLETADA")
    print("=" * 70)
    print(f"\nResultados guardados en: {results_dir}")
    print("\nInterpretación:")
    print("  - Si las curvas coinciden bien → parámetros correctos")
    print("  - Si divergen mucho → parámetros necesitan ajuste")
    print("  - Revisar especialmente φ, vx, vy, ω (más sensibles a K e I)")
    print("\nNotas:")
    print(f"  - Video procesado: {VIDEO_NUMBER}")
    print(f"  - Parámetros usados: Ra={Ra:.3f}Ω, K={K:.3f}V/(rad/s), I={I:.4f}kg·m²")
    print("  - Actualiza los parámetros Ra, K, I con tus valores identificados")
    print("=" * 70)
    
    # plt.show()

if __name__ == "__main__":
    main()
