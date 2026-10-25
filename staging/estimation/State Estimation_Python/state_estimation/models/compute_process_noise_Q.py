"""
Process Noise Covariance (Q) Computation from Model Identification

This module computes the process noise covariance matrix Q for state estimation
by propagating acceleration measurement uncertainty through the robot dynamics.

Mathematical Foundation:
-----------------------
The process model is:
    x_{k+1} = f(x_k, u_k) + w_k,  w_k ~ N(0, Q)

where u_k = [ax_b, ay_b] are body-frame accelerations from IMU.

The IMU accelerations have measurement uncertainty:
    a_measured = a_true + ε_a,  ε_a ~ N(0, Σ_a)

We estimate Σ_a from residuals between IMU and kinematically-derived accelerations.

Then propagate this uncertainty through the dynamics Jacobian:
    Q ≈ G · Σ_a · G^T

where G = ∂f/∂u is the control input Jacobian.

Author: Robotics Thesis Project
Date: January 2026
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import csv
from scipy.signal import savgol_filter


def compute_model_based_accelerations(controls, robot_params, dt, x0=None):
    """
    Compute accelerations from identified robot dynamics model.
    
    Uses the complete physically-based robot model (motor dynamics, rigid body
    dynamics, Coriolis effects) to simulate the trajectory and extract 
    accelerations directly from the dynamics equations.
    
    This is the rigorous approach: U → dynamics → states → accelerations
    
    Process:
    --------
    1. Initialize robot model with identified parameters
    2. Simulate trajectory using control inputs U
    3. For each time step, evaluate dynamics to get accelerations
    4. Return global-frame accelerations [ax, ay]
    
    Parameters
    ----------
    controls : np.ndarray, shape (N, 3)
        Control inputs [u1, u2, u3] (voltages/PWM) from experiment
    robot_params : dict
        Robot parameters dictionary (from make_robot_params())
    dt : float
        Time step (s)
    x0 : array (6,), optional
        Initial state [x, y, phi, vx, vy, omega]
        Default: zeros (robot starts at rest at origin)
    
    Returns
    -------
    ax_model : np.ndarray, shape (N,)
        Model-based acceleration in global x-direction (m/s²)
    ay_model : np.ndarray, shape (N,)
        Model-based acceleration in global y-direction (m/s²)
    states : np.ndarray, shape (N, 6)
        Simulated state trajectory [x, y, phi, vx, vy, omega]
    
    Notes
    -----
    This function requires the Robot_Identification module to be in the workspace.
    The accelerations are extracted from the state derivatives (dstate[3] and dstate[4])
    which are computed by the full dynamics model including:
    - Motor electrical dynamics (V → i → τ)
    - Force transformation (τ_wheels → F_robot)
    - Rigid body dynamics (F → a)
    - Coriolis effects (rotating reference frame)
    """
    # Import robot model from identification module
    try:
        import sys
        from pathlib import Path
        
        # Add Robot_Identification to path if not already there
        robot_id_path = Path(__file__).parent.parent.parent.parent / 'Robot_Identification'
        if robot_id_path.exists() and str(robot_id_path) not in sys.path:
            sys.path.insert(0, str(robot_id_path))
        
        from src.models.robot import OmnidirectionalRobot
    except ImportError as e:
        raise ImportError(
            f"Could not import OmnidirectionalRobot: {e}\n"
            "Ensure Robot_Identification module is in the workspace at the expected location."
        )
    
    N = len(controls)
    
    # Default initial condition (robot at rest at origin)
    if x0 is None:
        x0 = np.zeros(6)
    
    # Create time vector
    time = np.arange(N) * dt
    
    # Extract compensation factors
    Cu1 = robot_params.get('Cu1', 1.0)
    Cu2 = robot_params.get('Cu2', 1.0)
    Cu3 = robot_params.get('Cu3', 1.0)
    
    # Apply motor compensation to controls
    controls_compensated = controls * np.array([Cu1, Cu2, Cu3])
    
    print(f"   Applying compensation factors: Cu1={Cu1:.4f}, Cu2={Cu2:.4f}, Cu3={Cu3:.4f}")
    
    # Initialize robot model with identified parameters
    robot = OmnidirectionalRobot(robot_params)
    
    # Simulate complete trajectory with compensated controls
    print("   Simulating robot dynamics...")
    states, success = robot.simulate(time, controls, x0)
    
    if not success:
        print("   Warning: Simulation encountered numerical issues")
    
    # Extract accelerations by evaluating dynamics at each point
    print("   Extracting accelerations from dynamics...")
    ax_model = np.zeros(N)
    ay_model = np.zeros(N)
    
    for k in range(N):
        # Control function for this instant
        u_k = controls[k]
        u_func = lambda t: u_k
        
        # Compute state derivative: dstate = [dx/dt, dy/dt, dphi/dt, dvx/dt, dvy/dt, domega/dt]
        dstate = robot.dynamics(time[k], states[k], u_func)
        
        # Extract global-frame accelerations
        ax_model[k] = dstate[3]  # dvx/dt (global frame)
        ay_model[k] = dstate[4]  # dvy/dt (global frame)
    
    # print(" FIRST 20 MODEL-BASED ACCELERATIONS:", ax_model[:20], ay_model[:20], "...")

    return ax_model, ay_model, states


def estimate_acceleration_variance(imu_accel, model_accel, method='robust'):
    """
    Estimate acceleration measurement variance from residuals.
    
    The residuals between IMU and model-based accelerations reveal the
    combined uncertainty from:
    - IMU sensor noise
    - Model identification errors
    - Unmodeled dynamics (friction, slippage, etc.)
    
    Parameters
    ----------
    imu_accel : np.ndarray, shape (N, 2)
        IMU accelerations [ax_imu, ay_imu] (m/s²)
    model_accel : np.ndarray, shape (N, 2)
        Model-based accelerations [ax_model, ay_model] (m/s²)
    method : str, optional
        'sample' - sample variance
        'robust' - robust estimator using MAD (default)
    
    Returns
    -------
    var_ax : float
        Variance of x-acceleration (m²/s⁴)
    var_ay : float
        Variance of y-acceleration (m²/s⁴)
    residuals : np.ndarray, shape (N, 2)
        Acceleration residuals
    """
    # Compute residuals
    residuals = imu_accel - model_accel
    
    if method == 'sample':
        # Sample variance (sensitive to outliers)
        var_ax = np.var(residuals[:, 0], ddof=1)
        var_ay = np.var(residuals[:, 1], ddof=1)
    
    elif method == 'robust':
        # Robust variance estimation using Median Absolute Deviation (MAD)
        # σ ≈ 1.4826 × MAD (for Gaussian data)
        mad_ax = np.median(np.abs(residuals[:, 0] - np.median(residuals[:, 0])))
        mad_ay = np.median(np.abs(residuals[:, 1] - np.median(residuals[:, 1])))
        
        sigma_ax = 1.4826 * mad_ax
        sigma_ay = 1.4826 * mad_ay
        
        var_ax = sigma_ax ** 2
        var_ay = sigma_ay ** 2
    
    else:
        raise ValueError(f"Unknown method: {method}")
    
    return var_ax, var_ay, residuals


def compute_control_jacobian_G(phi, dt):
    """
    Control Jacobian G = ∂f/∂u consistent with EKF/UKF dynamics.

    State: x = [x, y, phi, vx, vy, omega]
    Input: u = [ax_b, ay_b]
    
    For the discrete dynamics x_{k+1} = f(x_k, u_k) with u = [ax_b, ay_b]:
    
    The Jacobian G tells us how input uncertainty propagates to state uncertainty.
    
    Parameters
    ----------
    phi : float
        Heading angle (rad)
    dt : float
        Time step (s)
    
    Returns
    -------
    G : np.ndarray, shape (6, 2)
        Control Jacobian evaluated at nominal conditions
    
    Notes
    -----
    For our kinematic model with IMU inputs:
    - Position (x, y) depends on accelerations through velocity integration
    - Heading (phi) is independent of accelerations (from gyro)
    - Velocities (vx, vy) directly depend on accelerations
    - Angular velocity (omega) is independent of accelerations
    """
    c = np.cos(phi)
    s = np.sin(phi)

    G = np.zeros((6, 2))

    # Position (second-order integration)
    G[0, 0] = 0.5 * c * dt**2
    G[0, 1] = -0.5 * s * dt**2
    G[1, 0] = 0.5 * s * dt**2
    G[1, 1] = 0.5 * c * dt**2

    # Velocity (first-order integration)
    G[3, 0] = c * dt
    G[3, 1] = -s * dt
    G[4, 0] = s * dt
    G[4, 1] = c * dt

    return G


def compute_process_noise_Q(var_ax, var_ay, phi, dt, method='jacobian'):
    """
    Compute process noise covariance matrix Q.
    
    Propagates acceleration uncertainty through dynamics to obtain
    the process noise covariance for state estimation.
    
    Methods:
    --------
    'jacobian' : Q = G · Σ_a · G^T (first-order approximation)
    'diagonal' : Diagonal Q with heuristic scaling
    'physical'  : Physics-based values from model uncertainty
    
    Parameters
    ----------
    var_ax : float
        Variance of x-acceleration (m²/s⁴)
    var_ay : float
        Variance of y-acceleration (m²/s⁴)
    phi : float
        Heading angle (rad)
    dt : float
        Time step (s)
    method : str, optional
        Method for Q computation (default: 'jacobian')
    
    Returns
    -------
    Q : np.ndarray, shape (6, 6)
        Process noise covariance matrix
    
    Examples
    --------
    >>> var_ax, var_ay = 0.01, 0.01  # m²/s⁴
    >>> Q = compute_process_noise_Q(var_ax, var_ay, phi=np.mean(phi_k), dt=0.01)
    >>> print(Q.diagonal())
    """
    if method == 'jacobian':
        # Acceleration covariance
        Sigma_a = np.diag([var_ax, var_ay])
        
        # Control Jacobian
        G = compute_control_jacobian_G(phi, dt)
        
        # Propagate: Q = G · Σ_a · G^T
        Q = G @ Sigma_a @ G.T
        
        # Add small regularization to avoid numerical issues
        Q += np.eye(6) * 1e-12
    
    elif method == 'diagonal':
        # Heuristic diagonal Q based on acceleration variance
        # Scaled by integration time constants
        Q = np.diag([
            var_ax * (0.5 * dt**2)**2,  # x position
            var_ay * (0.5 * dt**2)**2,  # y position
            1e-6,                        # phi (from gyro, very low)
            var_ax * dt**2,              # vx velocity
            var_ay * dt**2,              # vy velocity
            1e-5                         # omega (from gyro, low)
        ])
    
    elif method == 'physical':
        # Physics-based Q from model identification uncertainty
        # Conservative estimates based on typical sensor specs
        sigma_ax = np.sqrt(var_ax)
        sigma_ay = np.sqrt(var_ay)
        
        Q = np.diag([
            (sigma_ax * 0.5 * dt**2)**2,  # x
            (sigma_ay * 0.5 * dt**2)**2,  # y
            (0.001)**2,                    # phi (gyro drift)
            (sigma_ax * dt)**2,            # vx
            (sigma_ay * dt)**2,            # vy
            (0.01)**2                      # omega (gyro noise)
        ])
    
    else:
        raise ValueError(f"Unknown method: {method}")
    
    return Q


def analyze_acceleration_residuals(residuals, save_path=None):
    """
    Analyze and visualize acceleration residuals.
    
    Generates diagnostic plots to assess:
    - Residual distributions (should be Gaussian)
    - Temporal correlation (should be white noise)
    - Outliers and systematic biases
    
    Parameters
    ----------
    residuals : np.ndarray, shape (N, 2)
        Acceleration residuals [rx, ry]
    save_path : str or Path, optional
        Path to save figure
    
    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure handle
    """
    # IEEE formatting
    plt.rcParams.update({
        'font.family': 'serif',
        'font.serif': ['Times New Roman'],
        'font.size': 10,
        'axes.labelsize': 11,
        'axes.titlesize': 11,
        'legend.fontsize': 9,
        'figure.dpi': 300,
        'text.usetex': False
    })
    
    fig, axes = plt.subplots(2, 2, figsize=(7.16, 5.5))
    
    # Time series
    N = len(residuals)
    time = np.arange(N) * 0.01  # Assuming 100 Hz
    
    axes[0, 0].plot(time, residuals[:, 0], 'b-', linewidth=0.8, alpha=0.7)
    axes[0, 0].axhline(0, color='k', linestyle='--', linewidth=1)
    axes[0, 0].set_xlabel('Tiempo [s]')
    axes[0, 0].set_ylabel(r'$r_{a_x}$ [m/s$^2$]')
    axes[0, 0].set_title('Residual Aceleración X')
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(time, residuals[:, 1], 'r-', linewidth=0.8, alpha=0.7)
    axes[0, 1].axhline(0, color='k', linestyle='--', linewidth=1)
    axes[0, 1].set_xlabel('Tiempo [s]')
    axes[0, 1].set_ylabel(r'$r_{a_y}$ [m/s$^2$]')
    axes[0, 1].set_title('Residual Aceleración Y')
    axes[0, 1].grid(True, alpha=0.3)
    
    # Histograms with Gaussian overlay
    for idx, ax in enumerate([axes[1, 0], axes[1, 1]]):
        data = residuals[:, idx]
        mu = np.mean(data)
        sigma = np.std(data)
        
        counts, bins, _ = ax.hist(data, bins=50, density=True, 
                                   alpha=0.7, color=['b', 'r'][idx],
                                   label='Datos')
        
        # Gaussian fit
        x = np.linspace(bins[0], bins[-1], 200)
        gaussian = (1 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((x - mu) / sigma)**2)
        ax.plot(x, gaussian, 'k-', linewidth=2, label=f'N({mu:.3f}, {sigma:.3f}^2)')
        
        ax.set_xlabel([r'$r_{a_x}$ [m/s$^2$]', r'$r_{a_y}$ [m/s$^2$]'][idx])
        ax.set_ylabel('Densidad')
        ax.set_title(['Distribución Residuos X', 'Distribución Residuos Y'][idx])
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
    
    plt.rcParams.update(plt.rcParamsDefault)
    
    return fig


def load_experimental_data(filepath, N=None):
    """
    Load experimental data from text file.
    
    Expected format:
    [t, ax, ay, alpha, w1, w2, w3, u1, u2, u3, vbx_sp, vby_sp, wb_sp]
    
    Parameters
    ----------
    filepath : str or Path
        Path to data file
    N : int, optional
        Number of points to load (default: all)
    
    Returns
    -------
    data : dict
        Dictionary with keys:
        - 'time' : time vector
        - 'ax_imu', 'ay_imu' : IMU accelerations
        - 'w1', 'w2', 'w3' : wheel velocities
        - 'alpha' : yaw angle
        - 'u1', 'u2', 'u3' : motor commands
    """
    raw_data = np.loadtxt(filepath, skiprows=1, delimiter=',')
    
    if N is not None:
        raw_data = raw_data[:N, :]
    
    data = {
        'time': raw_data[:, 0],
        'ax_imu': raw_data[:, 1],
        'ay_imu': raw_data[:, 2],
        'alpha': raw_data[:, 3],  # Normalize to start at 0
        'w1': raw_data[:, 4],
        'w2': raw_data[:, 5],
        'w3': raw_data[:, 6],
        'u1': raw_data[:, 7],
        'u2': raw_data[:, 8],
        'u3': raw_data[:, 9],
    }
    
    return data


def make_robot_params(wheel_angles=[150.0, 270.0, 30.0], d=0.08, r=0.025,
                      M=3.178, I=0.02, K=0.5, Ra=5.0,
                      Cu1=1.0, Cu2=1.0, Cu3=1.0):
    return {
        'wheel_angles': wheel_angles,
        'd': d,
        'r': r,
        'M': M,
        'I': I,
        'K': K,
        'Ra': Ra,
        'Cu1': Cu1,
        'Cu2': Cu2,
        'Cu3': Cu3
    }


# ============================================================================
# Main Pipeline Function
# ============================================================================

def compute_Q_from_experiment(experiment_file,
                              robot_params=None,
                              N=None, 
                              method='jacobian',
                              variance_method='robust', 
                              save_dir=None):
    """
    Complete pipeline: Load data → Simulate dynamics → Compute Q matrix.
    
    This is the main function that orchestrates the entire Q computation
    process from experimental data using the identified robot model.
    
    Steps:
    ------
    1. Load experimental data (IMU + control inputs U)
    2. Simulate robot dynamics using identified model parameters
    3. Extract accelerations from model dynamics
    4. Compute residuals between IMU and model-based accelerations
    5. Estimate acceleration variance
    6. Propagate uncertainty to get Q matrix
    7. Generate diagnostic plots
    
    Parameters
    ----------
    experiment_file : str or Path
        Path to experimental data file
    robot_params : dict, optional
        Robot parameters dictionary (from make_robot_params()).
        If None, uses default parameters.
    N : int, optional
        Number of data points (default: all)
    method : str, optional
        Q computation method: 'jacobian', 'diagonal', 'physical' (default: 'jacobian')
    variance_method : str, optional
        Variance estimation method: 'sample', 'robust' (default: 'robust')
    save_dir : str or Path, optional
        Directory to save results
    
    Returns
    -------
    results : dict
        Dictionary containing:
        - 'Q' : Process noise covariance matrix
        - 'var_ax', 'var_ay' : Acceleration variances
        - 'residuals' : Acceleration residuals
        - 'model_accel' : Model-based accelerations
        - 'imu_accel' : IMU accelerations
        - 'states' : Simulated state trajectory
    
    Examples
    --------
    >>> # Using default parameters
    >>> results = compute_Q_from_experiment('data/sensors/exp2.txt', N=1000)
    >>> 
    >>> # Using identified parameters
    >>> params = make_robot_params(
    ...     M=3.178, I=0.0195, K=0.485, Ra=4.85,
    ...     Cu1=0.952, Cu2=1.018, Cu3=0.983
    ... )
    >>> results = compute_Q_from_experiment(
    ...     'data/sensors/exp2.txt',
    ...     robot_params=params,
    ...     N=1000
    ... )
    >>> Q = results['Q']
    
    Notes
    -----
    Use make_robot_params() with values from ThreeStageIdentification for best accuracy.
    """
    print("="*70)
    print("PROCESS NOISE COVARIANCE (Q) COMPUTATION")
    print("="*70)
    
    # Use default parameters if not provided
    if robot_params is None:
        robot_params = make_robot_params()
        print("\n⚠ Using default robot parameters. For best results, provide identified values.")
    
    # Display parameters
    print(f"\nRobot parameters:")
    print(f"  Kinematic: d={robot_params['d']}m, r={robot_params['r']}m")
    print(f"  Dynamic: M={robot_params['M']}kg, I={robot_params['I']}kg·m², K={robot_params['K']}V/(rad/s), Ra={robot_params['Ra']}Ω")
    print(f"  Compensation: Cu1={robot_params['Cu1']:.3f}, Cu2={robot_params['Cu2']:.3f}, Cu3={robot_params['Cu3']:.3f}")
    
    # Load data
    print(f"\n1. Loading data from {experiment_file}...")
    data = load_experimental_data(experiment_file, N=N)
    N_actual = len(data['time'])
    
    # Compute time step
    dt = np.mean(np.diff(data['time']))
    print(f"   Loaded {N_actual} samples, dt = {dt:.4f} s")
    
    # Extract control inputs (only U, nothing else from sensors) and convert it to voltages
    controls = np.column_stack([data['u1'], data['u2'], data['u3']])
    controls = (controls / 100.0) * 6*3.7  # Assuming 6-cell LiPo battery (22.2V)
    # print(" SOME CONTROL INPUTS (V):", controls[:5], "...")
    
    # Compute model-based accelerations using identified dynamics
    print("\n2. Computing model-based accelerations from dynamics...")
    ax_model, ay_model, states = compute_model_based_accelerations(
        controls, robot_params, dt
    )
    
    # IMU accelerations
    imu_accel = np.column_stack([data['ax_imu'], data['ay_imu']])
    model_accel = np.column_stack([ax_model, ay_model])
    
    # Estimate variance
    print(f"\n3. Estimating acceleration variance (method: {variance_method})...")
    var_ax, var_ay, residuals = estimate_acceleration_variance(
        imu_accel, model_accel, method=variance_method
    )
    
    sigma_ax = np.sqrt(var_ax)
    sigma_ay = np.sqrt(var_ay)
    
    print(f"   sigma_ax = {sigma_ax:.4f} m/s^2  (var = {var_ax:.6f} m^2/s^4)")
    print(f"   sigma_ay = {sigma_ay:.4f} m/s^2  (var = {var_ay:.6f} m^2/s^4)")
    
    # Compute Q
    print(f"\n4. Computing process noise Q (method: {method})...")
    phi_mean = np.mean(data['alpha'])
    Q = compute_process_noise_Q(var_ax, var_ay, phi_mean, dt, method=method)
    
    print("\n   Q diagonal elements:")
    state_names = ['x', 'y', 'phi', 'vx', 'vy', 'omega']
    for i, name in enumerate(state_names):
        print(f"   Q[{name}] = {Q[i, i]:.6e}")
    
    # Analyze residuals
    if save_dir:
        print("\n5. Generating diagnostic plots...")
        save_dir = Path(save_dir)
        save_dir.mkdir(parents=True, exist_ok=True)
        
        fig = analyze_acceleration_residuals(
            residuals, 
            save_path=save_dir / 'acceleration_residuals.png'
        )
        print(f"   Saved: {save_dir / 'acceleration_residuals.png'}")
        
        # Save Q to CSV
        q_path = save_dir / 'Q_matrix.csv'
        np.savetxt(q_path, Q, delimiter=',', 
                   header=','.join(state_names), comments='')
        print(f"   Saved: {q_path}")
        
        # Save summary
        summary_path = save_dir / 'Q_summary.txt'
        with open(summary_path, 'w') as f:
            f.write("Process Noise Covariance (Q) Summary\n")
            f.write("="*50 + "\n\n")
            f.write(f"Experiment: {experiment_file}\n")
            f.write(f"Samples: {N_actual}\n")
            f.write(f"Time step: {dt:.4f} s\n")
            f.write(f"Variance method: {variance_method}\n")
            f.write(f"Q computation method: {method}\n\n")
            f.write(f"Acceleration variances:\n")
            f.write(f"  var_ax = {var_ax:.6e} m^2/s^4\n")
            f.write(f"  var_ay = {var_ay:.6e} m^2/s^4\n")
            f.write(f"  sigma_ax = {sigma_ax:.4f} m/s^2\n")
            f.write(f"  sigma_ay = {sigma_ay:.4f} m/s^2\n\n")
            f.write("Q matrix diagonal:\n")
            for i, name in enumerate(state_names):
                f.write(f"  Q[{name}] = {Q[i, i]:.6e}\n")
        print(f"   Saved: {summary_path}")
    
    print("\n" + "="*70)
    print("Q COMPUTATION COMPLETE")
    print("="*70)
    
    results = {
        'Q': Q,
        'var_ax': var_ax,
        'var_ay': var_ay,
        'sigma_ax': sigma_ax,
        'sigma_ay': sigma_ay,
        'residuals': residuals,
        'model_accel': model_accel,
        'imu_accel': imu_accel,
        'states': states,
        'dt': dt,
        'phi_mean': phi_mean
    }
    
    return results
