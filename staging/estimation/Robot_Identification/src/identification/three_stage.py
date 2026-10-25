"""
Three-stage identification protocol for omnidirectional robots.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from sklearn.metrics import mean_squared_error, r2_score
from src.models.robot import OmnidirectionalRobot


class ThreeStageIdentification:
    """
    Robust 3-stage parameter identification protocol.
    
    Stage 1: Electrical parameters (Ra, K) from steady-state motor tests
    Stage 2: Mechanical inertia (I) from pure rotation trajectories
    Stage 3: Compensation factors (Cu1, Cu2, Cu3) from full trajectories
    
    This staged approach is more robust than simultaneous optimization because:
    - Each stage focuses on parameters that affect specific dynamics
    - Reduces parameter correlation and non-identifiability
    - Provides better initial guesses for subsequent stages
    - Allows validation at each step
    """
    
    def __init__(self, wheel_angles, d, r, M):
        """
        Initialize with known/measured geometric parameters.
        
        Parameters
        ----------
        wheel_angles : array (3,)
            Wheel angular positions in degrees
        d : float
            Distance from center to wheels (m)
        r : float
            Wheel radius (m)
        M : float
            Robot mass (kg) - can be measured with scale
        """
        self.wheel_angles = wheel_angles
        self.d = d
        self.r = r
        self.M = M
        
        self.results = {}
    
    def stage1_motor_parameters(self, pwm_data, speed_data, motor_id=None, results_dir=None):
        """
        Stage 1: Estimate Ra and K from single-motor steady-state test.
        
        EXPERIMENTAL SETUP
        ------------------
        1. Lift robot so wheels don't touch ground (or test motor on bench)
        2. Apply constant PWM to ONE motor
        3. Wait for steady state (~2 seconds)
        4. Measure wheel angular velocity
        5. Repeat for 5-10 different PWM values
        
        PHYSICAL MODEL
        --------------
        At steady state (acceleration = 0):
        - Electrical: V = Ra*i + K*ω
        - Mechanical: τ_motor = τ_friction
        - Motor torque: τ = K*i
        - Friction: τ_friction ≈ b*ω (viscous) + τ_static (Coulomb)
        
        Combining: V = (Ra*b/K + K)*ω + (Ra*τ_static/K)
        
        This is linear: V = a*ω + c
        where: a = Ra*b/K + K, c = Ra*τ_static/K
        
        If friction is small (b ≈ 0, τ_static ≈ 0):
        ω ≈ V/K  →  K ≈ V/ω
        
        Parameters
        ----------
        pwm_data : array (N,)
            Applied PWM/voltage values
        speed_data : array (N,)
            Measured steady-state wheel speeds (rad/s)
        motor_id : int, optional
            Which motor (1, 2, or 3) for logging
        
        Returns
        -------
        Ra : float
            Armature resistance (Ω)
        K : float
            Motor constant (V/(rad/s) = Nm/A)
        """
        print("\n" + "=" * 70)
        print("STAGE 1: Motor Electrical Parameter Estimation")
        if motor_id is not None:
            print(f"Testing motor: w{motor_id}")
        print("=" * 70)
        
        V = np.array(pwm_data)
        w = np.array(speed_data)
        
        print(f"\nData summary:")
        print(f"  Number of points: {len(V)}")
        print(f"  Voltage range:    [{V.min():.3f}, {V.max():.3f}]")
        print(f"  Speed range:      [{w.min():.3f}, {w.max():.3f}] rad/s")
        
        # Remove zero-speed points (motor not moving)
        valid = w > 0.1
        V_valid = V[valid]
        w_valid = w[valid]
        
        if len(V_valid) < 3:
            print("  ⚠ Error: Need at least 3 valid data points")
            return None, None
        
        print(f"  Valid points:     {len(V_valid)}")
        
        # === Method 1: Simple linear fit (assuming low friction) ===
        # Model: w = V/K
        K_simple = np.mean(V_valid / w_valid)
        
        # === Method 2: Nonlinear fit with friction ===
        # Model: w = K*V / (Ra*b + K²) where b is friction coefficient
        def model(params, V):
            Ra, K, b = params
            return K * V / (Ra * b + K**2)
        
        def residual(params):
            w_pred = model(params, V_valid)
            return np.sum((w_valid - w_pred)**2)
        
        # Optimize
        result = minimize(
            residual,
            x0=[1.5, K_simple, 0.001],  # Initial guess
            bounds=[(0.1, 20.0), (0.01, 5.0), (0.0, 0.1)],
            method='L-BFGS-B'
        )
        
        Ra, K, b_estimated = result.x
        
        print(f"\n=== RESULTS ===")
        print(f"Simple estimate (no friction): K = {K_simple:.4f} V/(rad/s)")
        print(f"\nOptimized with friction model:")
        print(f"  Ra = {Ra:.4f} Ω")
        print(f"  K  = {K:.4f} V/(rad/s)")
        print(f"  b  = {b_estimated:.6f} Nm/(rad/s) [friction coefficient]")
        
        # Validation
        w_pred = model([Ra, K, b_estimated], V_valid)
        rmse = np.sqrt(mean_squared_error(w_valid, w_pred))
        r2 = r2_score(w_valid, w_pred)
        
        print(f"\nModel quality:")
        print(f"  RMSE = {rmse:.4f} rad/s")
        print(f"  R²   = {r2:.4f}")
        
        if r2 < 0.8:
            print("  ⚠ Warning: Low R² suggests model mismatch or noisy data")
        else:
            print("  ✓ Good fit")
        
        # Visualization - IEEE format with Spanish text
        import matplotlib.pyplot as plt
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
        
        fig, ax = plt.subplots(figsize=(7.16, 4.5))  # IEEE double column width
        ax.plot(V_valid, w_valid, 'bo', label='Datos Experimentales', markersize=8, alpha=0.7)
        
        V_plot = np.linspace(V_valid.min(), V_valid.max(), 100)
        w_fit = model([Ra, K, b_estimated], V_plot)
        ax.plot(V_plot, w_fit, 'r-', label='Modelo Ajustado', linewidth=2)
        
        ax.set_xlabel('Voltaje Aplicado [V]')
        ax.set_ylabel('Velocidad Angular [rad/s]')
        ax.legend(loc='best', framealpha=0.9)
        ax.grid(True)
        plt.tight_layout()
        
        # Save figure
        from pathlib import Path
        if results_dir is None:
            results_dir = Path(__file__).parent.parent.parent.parent / "results" / "identification" / "stage1"
        results_dir.mkdir(parents=True, exist_ok=True)
        fig_path = results_dir / f"stage1_caracterizacion_motor{motor_id or 'unknown'}.png"
        fig.savefig(fig_path, dpi=300, bbox_inches='tight')
        print(f"\n✓ Figura 1 guardada: {fig_path}")
        
        # Save metrics to CSV
        import csv
        csv_path = results_dir / f"stage1_metricas_motor{motor_id or 'unknown'}.csv"
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['Parámetro', 'Valor', 'Unidad'])
            writer.writerow(['Ra', f'{Ra:.4f}', 'Ω'])
            writer.writerow(['K', f'{K:.4f}', 'V/(rad/s)'])
            writer.writerow(['b (fricción)', f'{b_estimated:.6f}', 'Nm/(rad/s)'])
            writer.writerow(['RMSE', f'{rmse:.4f}', 'rad/s'])
            writer.writerow(['R²', f'{r2:.4f}', '-'])
        print(f"✓ Métricas guardadas: {csv_path}")
        
        # Reset to defaults
        plt.rcParams.update(plt.rcParamsDefault)
        
        self.results['stage1'] = {
            'Ra': Ra, 'K': K, 'friction': b_estimated,
            'r2': r2, 'rmse': rmse
        }
        
        return Ra, K
    
    def stage2_inertia(self, t, u, y_measured, Ra, K, plot=True, results_dir=None):
        """
        Stage 2: Estimate moment of inertia I from pure rotation data.
        
        EXPERIMENTAL SETUP
        ------------------
        1. Command robot to rotate in place (no translation)
        2. Vary angular velocity: slow → fast → stop → reverse
        3. Record phi(t) and omega(t) for at least 10 seconds
        4. Ensure vx ≈ 0 and vy ≈ 0 (pure rotation)
        
        WHY THIS WORKS
        --------------
        - Decouples rotational dynamics from translational
        - Inertia I only affects angular acceleration: α = τ/I
        - Less affected by friction and mass uncertainty
        - No wheel slippage issues
        
        Parameters
        ----------
        t : array (N,)
            Time vector (s)
        u : array (N, 3)
            Control inputs [u1, u2, u3] during rotation (in volts)
        y_measured : array (N, 6)
            Measured states [x, y, phi, vx, vy, omega]
        Ra, K : float
            Motor parameters from Stage 1
        plot : bool
            Whether to generate validation plots
        
        Returns
        -------
        I : float
            Estimated moment of inertia (kg·m²)
        """
        print("\n" + "=" * 70)
        print("STAGE 2: Moment of Inertia Estimation")
        print("=" * 70)
        
        # Check data quality
        vx_rms = np.sqrt(np.mean(y_measured[:, 3]**2))
        vy_rms = np.sqrt(np.mean(y_measured[:, 4]**2))
        omega_rms = np.sqrt(np.mean(y_measured[:, 5]**2))
        
        print(f"\nData characteristics:")
        print(f"  vx RMS:    {vx_rms:.4f} m/s")
        print(f"  vy RMS:    {vy_rms:.4f} m/s")
        print(f"  omega RMS: {omega_rms:.4f} rad/s")
        
        if omega_rms < 0.1:
            print("  ⚠ Warning: Very low angular velocity - use more aggressive rotation")
        if vx_rms > 0.2 or vy_rms > 0.2:
            print("  ⚠ Warning: Significant translation detected - this should be pure rotation")
        
        # Define cost function
        def cost_full(I_val):
            # Extract scalar if I_val is an array
            I_scalar = float(I_val[0]) if hasattr(I_val, '__len__') else float(I_val)
            
            params = {
                'wheel_angles': self.wheel_angles,
                'd': self.d,
                'r': self.r,
                'M': self.M,
                'I': I_scalar,
                'K': K,
                'Ra': Ra
            }
            
            robot = OmnidirectionalRobot(params)
            y_model, success = robot.simulate(t, u, y_measured[0])
            
            if not success:
                return 1e10  # Penalize failed integration
            
            # Weight angular states heavily, ignore translations
            weights = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 5.0])
            residuals = (y_measured - y_model) * weights
            
            return np.sum(residuals**2)
        
        # Optimize
        print("\nOptimizing I in range [0.001, 0.5] kg·m²...")
        result = minimize(
            cost_full,
            x0=[0.02],
            bounds=[(0.001, 0.5)],
            method='L-BFGS-B', # COBYLA, TNC or L-BFGS-B can also be used
            options={'maxiter': 50, 'ftol': 1e-4}
        )
        
        I = result.x[0]
        
        print(f"\n=== RESULT ===")
        print(f"  I = {I:.6f} kg·m²")
        
        # Typical values for reference:
        # Small robot (< 5 kg): I ~ 0.005 - 0.03 kg·m²
        # Medium robot (5-15 kg): I ~ 0.03 - 0.15 kg·m²
        
        if I < 0.002:
            print("  ⚠ Warning: Very small inertia - check if robot mass is correct")
        elif I > 0.2:
            print("  ⚠ Warning: Very large inertia - check data quality")
        else:
            print("  ✓ Reasonable value for robot size")
        
        # Validate model
        params = {
            'wheel_angles': self.wheel_angles,
            'd': self.d,
            'r': self.r,
            'M': self.M,
            'I': I,
            'K': K,
            'Ra': Ra
        }
        robot = OmnidirectionalRobot(params)
        y_model, _ = robot.simulate(t, u, y_measured[0])
        
        # Compute metrics for angular states
        phi_rmse = np.sqrt(mean_squared_error(y_measured[:, 2], y_model[:, 2]))
        omega_rmse = np.sqrt(mean_squared_error(y_measured[:, 5], y_model[:, 5]))
        phi_r2 = r2_score(y_measured[:, 2], y_model[:, 2])
        
        print(f"\nModel validation (angular states):")
        print(f"  phi:   RMSE = {phi_rmse:.4f} rad,  R² = {phi_r2:.4f}")
        print(f"  omega: RMSE = {omega_rmse:.4f} rad/s")
        
        # Plotting
        if plot:
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
            
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.16, 5.5))  # IEEE double column width
            
            # Posición Angular (phi)
            ax1.plot(t, y_measured[:, 2], 'b-', label='Medición', linewidth=2)
            ax1.plot(t, y_model[:, 2], 'r--', label='Modelo', linewidth=1.5)
            ax1.set_xlabel('Tiempo [s]')
            ax1.set_ylabel('$\\phi$ [rad]')
            ax1.legend(loc='best', framealpha=0.9)
            ax1.grid(True)
            
            # Velocidad Angular (omega)
            ax2.plot(t, y_measured[:, 5], 'b-', label='Medición', linewidth=2)
            ax2.plot(t, y_model[:, 5], 'r--', label='Modelo', linewidth=1.5)
            ax2.set_xlabel('Tiempo [s]')
            ax2.set_ylabel('$\\omega$ [rad/s]')
            ax2.legend(loc='best', framealpha=0.9)
            ax2.grid(True)
            
            plt.tight_layout()
            
            # Save figure
            from pathlib import Path
            if results_dir is None:
                results_dir = Path(__file__).parent.parent.parent.parent / "results" / "identification" / "stage2"
            results_dir.mkdir(parents=True, exist_ok=True)
            fig_path = results_dir / "stage2_posicion_velocidad_angular.png"
            fig.savefig(fig_path, dpi=300, bbox_inches='tight')
            print(f"\n✓ Figura guardada: {fig_path}")
            
            # Save metrics to CSV
            import csv
            csv_path = results_dir / "stage2_metricas_inercia.csv"
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['Parámetro', 'Valor', 'Unidad'])
                writer.writerow(['I (Inercia)', f'{I:.6f}', 'kg·m²'])
                writer.writerow(['RMSE phi', f'{phi_rmse:.4f}', 'rad'])
                writer.writerow(['RMSE omega', f'{omega_rmse:.4f}', 'rad/s'])
                writer.writerow(['R² phi', f'{phi_r2:.4f}', '-'])
            print(f"✓ Métricas guardadas: {csv_path}")
            
            # Reset to defaults
            plt.rcParams.update(plt.rcParamsDefault)
        
        self.results['stage2'] = {
            'I': I,
            'phi_rmse': phi_rmse,
            'omega_rmse': omega_rmse,
            'phi_r2': phi_r2
        }
        
        return I
    
    def stage3_compensation(self, t, u, y_measured, Ra, K, I, plot=True, results_dir=None):
        """
        Stage 3: Fine-tune motor compensation factors Cu1, Cu2, Cu3.
        
        PURPOSE
        -------
        Even with identical motors, manufacturing tolerances cause differences:
        - Slight variations in resistance
        - Bearing friction differences
        - Magnetic field asymmetries
        - Gear backlash
        
        Compensation factors account for these by scaling the PWM/voltage:
        u_actual = Cu * u_commanded
        
        EXPERIMENTAL SETUP
        ------------------
        1. Use a rich trajectory: translations + rotations
        2. Include different speeds and directions
        3. Minimum 15-20 seconds of data
        4. Avoid extreme accelerations (saturation)
        
        Parameters
        ----------
        t : array (N,)
            Time vector
        u : array (N, 3)
            Control inputs [u1, u2, u3]
        y_measured : array (N, 6)
            Measured states [x, y, phi, vx, vy, omega]
        Ra, K, I : float
            Parameters from Stages 1 and 2
        plot : bool
            Generate validation plots
        
        Returns
        -------
        Cu1, Cu2, Cu3 : float
            Compensation factors for each motor
        """
        print("\n" + "=" * 70)
        print("STAGE 3: Motor Compensation Factor Tuning")
        print("=" * 70)
        
        # Check data richness
        print(f"\nData characteristics:")
        print(f"  Duration: {t[-1] - t[0]:.2f} s")
        print(f"  X range:  [{y_measured[:, 0].min():.3f}, {y_measured[:, 0].max():.3f}] m")
        print(f"  Y range:  [{y_measured[:, 1].min():.3f}, {y_measured[:, 1].max():.3f}] m")
        print(f"  Phi range: [{np.rad2deg(y_measured[:, 2].min()):.1f}, {np.rad2deg(y_measured[:, 2].max()):.1f}]°")
        
        # Define cost function
        def cost(Cu):
            Cu1, Cu2, Cu3 = Cu
            
            # Apply compensation
            u_comp = u * np.array([Cu1, Cu2, Cu3])
            
            # Simulate
            params = {
                'wheel_angles': self.wheel_angles,
                'd': self.d,
                'r': self.r,
                'M': self.M,
                'I': I,
                'K': K,
                'Ra': Ra
            }
            robot = OmnidirectionalRobot(params)
            y_model, success = robot.simulate(t, u_comp, y_measured[0])
            
            if not success:
                return 1e10
            
            # Weight positions more than velocities
            weights = np.array([3.0, 3.0, 2.0, 0.5, 0.5, 0.5])
            residuals = (y_measured - y_model) * weights
            
            return np.sum(residuals**2)
        
        # Optimize
        print("\nOptimizing compensation factors...")
        result = minimize(
            cost,
            x0=[1.0, 1.0, 1.0],
            bounds=[(0.7, 1.3), (0.7, 1.3), (0.7, 1.3)],
            method='L-BFGS-B',
            options={'maxiter': 50, 'ftol': 1e-5}  # Faster convergence
        )
        
        Cu1, Cu2, Cu3 = result.x
        
        print(f"\n=== RESULTS ===")
        print(f"  Cu1 = {Cu1:.4f}")
        print(f"  Cu2 = {Cu2:.4f}")
        print(f"  Cu3 = {Cu3:.4f}")
        
        # Interpret results
        deviations = np.abs(np.array([Cu1, Cu2, Cu3]) - 1.0)
        max_dev = np.max(deviations)
        
        if max_dev < 0.05:
            print(f"  ✓ All motors well-matched (max deviation: {max_dev*100:.1f}%)")
        elif max_dev < 0.15:
            print(f"  ✓ Acceptable motor variation (max deviation: {max_dev*100:.1f}%)")
        else:
            print(f"  ⚠ Large compensation needed (max deviation: {max_dev*100:.1f}%)")
            print(f"    Consider recalibrating motors or checking mechanical issues")
        
        # Validate
        u_comp = u * np.array([Cu1, Cu2, Cu3])
        params = {
            'wheel_angles': self.wheel_angles,
            'd': self.d,
            'r': self.r,
            'M': self.M,
            'I': I,
            'K': K,
            'Ra': Ra
        }
        robot = OmnidirectionalRobot(params)
        y_model, _ = robot.simulate(t, u_comp, y_measured[0])
        
        # Compute comprehensive metrics
        print(f"\nModel validation (all states):")
        state_names = ['x', 'y', 'phi', 'vx', 'vy', 'omega']
        for i, name in enumerate(state_names):
            rmse = np.sqrt(mean_squared_error(y_measured[:, i], y_model[:, i]))
            r2 = r2_score(y_measured[:, i], y_model[:, i])
            print(f"  {name:5s}: RMSE = {rmse:.5f},  R² = {r2:.4f}")
        
        # Plotting
        if plot:
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
            
            from pathlib import Path
            if results_dir is None:
                results_dir = Path(__file__).parent.parent.parent.parent / "results" / "identification" / "stage3"
            results_dir.mkdir(parents=True, exist_ok=True)
            
            # ========== FIGURA 1: Trayectoria 2D con orientación ==========
            fig1, ax1 = plt.subplots(figsize=(7.16, 5.5))
            
            # Trayectorias
            ax1.plot(y_measured[:, 0], y_measured[:, 1], 'b-', label='Medición', 
                    linewidth=2, alpha=0.7)
            ax1.plot(y_model[:, 0], y_model[:, 1], 'r--', label='Modelo', linewidth=2)
            
            # Marcadores de inicio y fin
            ax1.plot(y_measured[0, 0], y_measured[0, 1], 'go', markersize=10, 
                    label='Inicio', zorder=5)
            ax1.plot(y_measured[-1, 0], y_measured[-1, 1], 'rs', markersize=10, 
                    label='Fin', zorder=5)
            
            # Flechas de orientación (espaciadas cada N puntos)
            N_arrows = 15  # Número de flechas a lo largo de la trayectoria
            indices = np.linspace(0, len(t)-1, N_arrows, dtype=int)
            arrow_scale = 0.03  # Escala de las flechas
            
            for idx in indices:
                # Medición
                x, y, phi = y_measured[idx, 0], y_measured[idx, 1], y_measured[idx, 2]
                dx, dy = arrow_scale * np.cos(phi), arrow_scale * np.sin(phi)
                ax1.arrow(x, y, dx, dy, head_width=0.015, head_length=0.02, 
                         fc='blue', ec='blue', alpha=0.5, linewidth=0.8)
                
                # Modelo
                x, y, phi = y_model[idx, 0], y_model[idx, 1], y_model[idx, 2]
                dx, dy = arrow_scale * np.cos(phi), arrow_scale * np.sin(phi)
                ax1.arrow(x, y, dx, dy, head_width=0.015, head_length=0.02, 
                         fc='red', ec='red', alpha=0.4, linewidth=0.8, linestyle='--')
            
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Y [m]')
            ax1.legend(loc='best', framealpha=0.9)
            ax1.grid(True)
            ax1.axis('equal')
            plt.tight_layout()
            
            fig1_path = results_dir / "stage3_trayectoria_2d.png"
            fig1.savefig(fig1_path, dpi=300, bbox_inches='tight')
            print(f"\n✓ Figura 1 guardada: {fig1_path}")
            
            # ========== FIGURA 2: Estados lineales (x, y, vx, vy) ==========
            fig2, axes = plt.subplots(2, 2, figsize=(7.16, 5.5))
            
            state_labels = ['X [m]', 'Y [m]', '$v_x$ [m/s]', '$v_y$ [m/s]']
            state_indices = [0, 1, 3, 4]  # x, y, vx, vy
            positions = [(0, 0), (0, 1), (1, 0), (1, 1)]  # (row, col)
            
            for i, (label, idx, pos) in enumerate(zip(state_labels, state_indices, positions)):
                ax = axes[pos]
                ax.plot(t, y_measured[:, idx], 'b-', label='Medición', linewidth=2, alpha=0.7)
                ax.plot(t, y_model[:, idx], 'r--', label='Modelo', linewidth=1.5)
                ax.set_ylabel(label)
                if pos[0] == 1:  # Bottom row
                    ax.set_xlabel('Tiempo [s]')
                ax.legend(loc='best', framealpha=0.9)
                ax.grid(True)
            
            plt.tight_layout()
            
            fig2_path = results_dir / "stage3_estados_lineales.png"
            fig2.savefig(fig2_path, dpi=300, bbox_inches='tight')
            print(f"✓ Figura 2 guardada: {fig2_path}")
            
            # Reset to defaults
            plt.rcParams.update(plt.rcParamsDefault)
            
            # Save metrics to CSV
            import csv
            csv_path = results_dir / "stage3_metricas_compensacion.csv"
            with open(csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(['Parámetro', 'Valor', 'Unidad'])
                writer.writerow(['Cu1', f'{Cu1:.4f}', '-'])
                writer.writerow(['Cu2', f'{Cu2:.4f}', '-'])
                writer.writerow(['Cu3', f'{Cu3:.4f}', '-'])
                writer.writerow(['Desviación máxima', f'{max_dev*100:.2f}', '%'])
                writer.writerow(['', '', ''])
                writer.writerow(['Estado', 'RMSE', 'R²'])
                for i, name in enumerate(['x', 'y', 'phi', 'vx', 'vy', 'omega']):
                    rmse = np.sqrt(mean_squared_error(y_measured[:, i], y_model[:, i]))
                    r2 = r2_score(y_measured[:, i], y_model[:, i])
                    writer.writerow([name, f'{rmse:.5f}', f'{r2:.4f}'])
            print(f"✓ Métricas guardadas: {csv_path}")
        
        self.results['stage3'] = {
            'Cu1': Cu1, 'Cu2': Cu2, 'Cu3': Cu3,
            'max_deviation': max_dev
        }
        
        return Cu1, Cu2, Cu3
    
    def get_summary(self):
        """
        Print summary of all identification results.
        
        Returns
        -------
        results : dict
            Dictionary with all identification results
        """
        print("\n" + "=" * 70)
        print("IDENTIFICATION SUMMARY")
        print("=" * 70)
        
        if 'stage1' in self.results:
            print("\nStage 1 - Motor Parameters:")
            print(f"  Ra = {self.results['stage1']['Ra']:.4f} Ω")
            print(f"  K  = {self.results['stage1']['K']:.4f} V/(rad/s)")
            print(f"  R² = {self.results['stage1']['r2']:.4f}")
        
        if 'stage2' in self.results:
            print("\nStage 2 - Inertia:")
            print(f"  I  = {self.results['stage2']['I']:.6f} kg·m²")
            print(f"  R² = {self.results['stage2']['phi_r2']:.4f}")
        
        if 'stage3' in self.results:
            print("\nStage 3 - Compensation Factors:")
            print(f"  Cu1 = {self.results['stage3']['Cu1']:.4f}")
            print(f"  Cu2 = {self.results['stage3']['Cu2']:.4f}")
            print(f"  Cu3 = {self.results['stage3']['Cu3']:.4f}")
        
        print("=" * 70 + "\n")
        
        return self.results
