"""
Robot dynamics model for omnidirectional mobile robots.
"""

import numpy as np
from scipy.integrate import solve_ivp
from src.kinematics.validator import KinematicValidator


class OmnidirectionalRobot:
    """
    Physically accurate dynamics model for 3-wheel omnidirectional robot.
    
    This model includes:
    - Correct kinematic mapping (wheel velocities ↔ robot velocity)
    - Motor electrical dynamics (back-EMF, armature resistance)
    - Rigid body dynamics (Newton-Euler equations)
    - Coriolis effects from rotating reference frame
    
    The model computes the complete forward dynamics:
    u (voltages) → motor currents → torques → forces → accelerations → velocities
    """
    
    def __init__(self, params):
        """
        Initialize robot model with physical parameters.
        
        Parameters
        ----------
        params : dict with keys:
            wheel_angles : array (3,)
                Wheel positions in degrees [w1, w2, w3]
            d : float
                Distance from center to wheels (m)
            r : float
                Wheel radius (m)
            M : float
                Robot mass (kg)
            I : float
                Robot moment of inertia (kg·m²)
            K : float
                Motor constant (V/(rad/s) = Nm/A)
            Ra : float
                Armature resistance (Ω)
        """
        self.wheel_angles = np.array(params['wheel_angles'])
        self.d = params['d']
        self.r = params['r']
        self.M = params['M']
        self.I = params['I']
        self.K = params['K']
        self.Ra = params['Ra']
        
        # Compute kinematic matrix
        self.H = KinematicValidator.compute_H_matrix(
            self.wheel_angles, self.d, self.r
        )
        self.H_pinv = np.linalg.pinv(self.H)
    
    def dynamics(self, t, state, u_func):
        """
        Continuous-time dynamics: dx/dt = f(x, u, t)
        
        Parameters
        ----------
        t : float
            Current time (s)
        state : array (6,)
            Current state [x, y, phi, vx, vy, omega]
        u_func : callable
            Function that returns control input at time t
        
        State Vector:
            x, y : Position in global frame (m)
            phi : Orientation angle (rad)
            vx, vy : Velocity in global frame (m/s)
            omega : Angular velocity (rad/s)
        
        Control Vector:
            u = [u1, u2, u3] : Voltage/PWM applied to each motor (V)
        
        Returns
        -------
        dstate : array (6,)
            State derivatives [dx/dt, dy/dt, dphi/dt, dvx/dt, dvy/dt, domega/dt]
        """
        x, y, phi, vx, vy, omega = state
        u = np.asarray(u_func(t), dtype=np.float64).flatten()
        
        # === Coordinate Transform: Global → Local ===
        c, s = np.cos(phi), np.sin(phi)
        R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]], dtype=np.float64)
        
        v_global = np.array([vx, vy, omega])
        v_local = R @ v_global
        
        # === Kinematics: Robot Velocity → Wheel Speeds ===
        wheel_speeds = self.H @ v_local  # rad/s
        
        # === Motor Electrical Model ===
        # Voltage equation: V = Ra*i + K*ω
        # Solving for current: i = (V - K*ω) / Ra
        motor_currents = (u - self.K * wheel_speeds) / self.Ra
        
        # Motor torques: τ = K * i
        motor_torques = self.K * motor_currents
        
        # === Force Transformation: Wheel Torques → Robot Forces ===
        # Wheel force: F_wheel = τ / r
        # Robot force/torque: F_robot = H^T @ F_wheels
        forces_local = (1/self.r) * self.H.T @ motor_torques
        
        # === Rigid Body Dynamics (Local Frame) ===
        fx_local = float(forces_local[0])  # Force in local X
        fy_local = float(forces_local[1])  # Force in local Y
        tau_z = float(forces_local[2])     # Torque about Z
        
        ax_local = float(fx_local / self.M)
        ay_local = float(fy_local / self.M)
        alpha = float(tau_z) / float(self.I)
        
        # === Transform Accelerations: Local → Global ===
        R_inv = R.T  # Inverse rotation = transpose
        a_global = R_inv[:2, :2] @ np.array([ax_local, ay_local], dtype=np.float64)
        ax_global = float(a_global[0])
        ay_global = float(a_global[1])
        
        # === Coriolis Terms (from Rotating Frame) ===
        # When transforming between rotating frames:
        # a_global = a_local + ω × v
        coriolis_x = float(omega * vy)
        coriolis_y = float(-omega * vx)
        
        # === State Derivatives ===
        dstate = np.array([
            float(vx),                          # dx/dt
            float(vy),                          # dy/dt  
            float(omega),                       # dφ/dt
            float(ax_global + coriolis_x),      # dvx/dt
            float(ay_global + coriolis_y),      # dvy/dt
            float(alpha)                        # dω/dt
        ], dtype=np.float64)
        
        return dstate
    
    def simulate(self, t, u, x0, method='RK45', rtol=1e-6, atol=1e-8):
        """
        Simulate robot trajectory given control inputs.
        
        Parameters
        ----------
        t : array (N,)
            Time vector (s)
        u : array (N, 3)
            Control inputs [u1, u2, u3] at each time step
        x0 : array (6,)
            Initial state [x0, y0, phi0, vx0, vy0, omega0]
        method : str
            Integration method: 'RK45', 'RK23', 'DOP853', 'LSODA'
        rtol, atol : float
            Relative and absolute tolerances
        
        Returns
        -------
        states : array (N, 6)
            State trajectory over time
        success : bool
            Whether integration succeeded
        """
        # Create interpolated control function
        def u_func(t_val):
            if t_val <= t[0]:
                return np.asarray(u[0], dtype=np.float64)
            elif t_val >= t[-1]:
                return np.asarray(u[-1], dtype=np.float64)
            else:
                return np.array([
                    np.interp(t_val, t, u[:, i]) for i in range(3)
                ], dtype=np.float64)
        
        # Solve ODE
        sol = solve_ivp(
            lambda t_val, x: self.dynamics(t_val, x, u_func),
            (t[0], t[-1]),
            x0,
            t_eval=t,
            method=method,
            rtol=rtol,
            atol=atol,
            max_step=0.1  # Prevent taking too large steps
        )
        
        if not sol.success:
            print(f"⚠ Integration warning: {sol.message}")
        
        return sol.y.T, sol.success
