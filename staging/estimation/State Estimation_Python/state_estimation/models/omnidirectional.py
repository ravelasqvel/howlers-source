"""
Omnidirectional robot model for state estimation.

Provides dynamics f(x, u), measurement h(x), and their Jacobians
for an omnidirectional (omniwheel) mobile robot.

State: x = [x, y, psi, vx_b, vy_b, omega]
- (x, y): position in world frame
- psi: heading angle
- (vx_b, vy_b): body frame velocities
- omega: angular velocity

Control: u = [ax_b, ay_b, omega_meas]
- (ax_b, ay_b): body frame accelerations (from IMU)
- omega_meas: measured angular velocity (from gyro)

Measurement: z = [vx_b, vy_b, omega, psi]
- (vx_b, vy_b): body velocities from encoders
- omega: angular velocity from encoders
- psi: heading from magnetometer/IMU
"""

import numpy as np


class OmnidirectionalRobot:
    """
    Omnidirectional robot model for 3-wheeled omniwheel robots.
    
    Uses the same kinematic model as the identification project for consistency.
    
    Parameters
    ----------
    dt : float, optional
        Time step for discretization (default: 0.01)
    wheel_angles : array_like, optional
        Wheel positions in degrees [w1, w2, w3] (default: [150, 270, 30])
    d : float, optional
        Distance from center to wheels in meters (default: 0.08)
    r : float, optional
        Wheel radius in meters (default: 0.025)
    M : float, optional
        Robot mass in kg (default: 3.178)
    I : float, optional
        Robot moment of inertia in kg·m² (default: 0.02)
    K : float, optional
        Motor constant in V/(rad/s) (default: 0.499)
    Ra : float, optional
        Armature resistance in Ω (default: 1.50)
    """
    
    def __init__(self, dt=0.01, wheel_angles=None, d=0.08, r=0.025, 
                 M=3.178, I=0.02, K=0.499, Ra=1.50):
        self.dt = dt
        
        # Robot geometric parameters
        if wheel_angles is None:
            wheel_angles = [150, 270, 30]  # degrees
        self.wheel_angles = np.array(wheel_angles)
        self.d = d  # distance from center to wheels (m)
        self.r = r  # wheel radius (m)
        
        # Robot physical parameters
        self.M = M   # mass (kg)
        self.I = I   # moment of inertia (kg·m²)
        self.K = K   # motor constant (V/(rad/s))
        self.Ra = Ra # armature resistance (Ω)
        
        # Compute kinematic matrix H
        self._compute_kinematic_matrix()
        
    def _compute_kinematic_matrix(self):
        """Compute the kinematic matrix H from wheel geometry."""
        # Convert angles to radians
        angles_rad = np.deg2rad(self.wheel_angles)
        
        # Build H matrix: [w1, w2, w3]^T = H * [vx_b, vy_b, omega]^T
        # Each row i: [sin(θi), -cos(θi), d] / r
        H = np.zeros((3, 3))
        for i in range(3):
            theta = angles_rad[i]
            H[i, 0] = -np.sin(theta) / self.r
            H[i, 1] =  np.cos(theta) / self.r
            H[i, 2] = -self.d / self.r         # omega contribution (negative for CCW positive)
        
        self.H = H
        self.H_pinv = np.linalg.pinv(H)  # For inverse kinematics


    #======================================================
    # Dynamics and Measurement Models for World-Frame State
    #======================================================
    def dynamics(self, x, u):
        """
        Discrete-time dynamics: x_{k+1} = f(x_k, u_k)

        World-frame state, IMU-driven process model.

        Parameters
        ----------
        x : np.ndarray
            State vector [x, y, phi, vx, vy, omega] (world frame)
        u : np.ndarray
            Input [ax_b, ay_b] (accelerations)

        Returns
        -------
        np.ndarray
            Next state x_{k+1}
        """

        # --- Extract state ---
        x_pos, y_pos, phi, vx_w, vy_w, omega = x
        ax_b, ay_b = u
        dt = self.dt

        c = np.cos(phi)
        s = np.sin(phi)

        # Acceleration in world frame
        ax_w = c * ax_b - s * ay_b
        ay_w = s * ax_b + c * ay_b

        # Velocity update
        vx_w_new = vx_w + ax_w * dt
        vy_w_new = vy_w + ay_w * dt

        # Position update
        x_new = x_pos + vx_w * dt
        y_new = y_pos + vy_w * dt

        # Orientation update
        phi_new = phi + omega * dt

        # Angular velocity (random walk)
        omega_new = omega

        return np.array([
            x_new,
            y_new,
            phi_new,
            vx_w_new,
            vy_w_new,
            omega_new
        ])
    

    def measurement(self, x):
        """
        Measurement model: z = h(x)

        Measurements:
            z = [vx_b, vy_b, omega, psi]

        State:
            x = [x, y, phi, vx, vy, omega]
        """

        _, _, phi, vx, vy, omega = x

        c = np.cos(phi)
        s = np.sin(phi)

        # World -> body velocity
        vx_b =  c * vx + s * vy
        vy_b = -s * vx + c * vy

        psi = phi  # same physical quantity

        return np.array([
            vx_b,
            vy_b,
            omega,
            psi
        ])


    def jacobian_F(self, x, u):
        """
        Jacobian F = ∂f/∂x for world-frame model
        """
        _, _, phi, vx_w, vy_w, omega = x
        ax_b, ay_b = u
        dt = self.dt

        c = np.cos(phi)
        s = np.sin(phi)

        F = np.eye(6)

        # Position derivatives
        F[0, 3] = dt
        F[1, 4] = dt

        # Orientation
        F[2, 5] = dt

        # Velocity derivatives w.r.t phi
        F[3, 2] = (-s * ax_b - c * ay_b) * dt
        F[4, 2] = ( c * ax_b - s * ay_b) * dt

        return F

    def jacobian_H(self, x):
        """
        Jacobian H = ∂h/∂x
        """

        _, _, phi, vx, vy, omega = x

        c = np.cos(phi)
        s = np.sin(phi)

        H = np.zeros((4, 6))

        # vx_b
        H[0, 2] = -s * vx + c * vy
        H[0, 3] =  c
        H[0, 4] =  s

        # vy_b
        H[1, 2] = -c * vx - s * vy
        H[1, 3] = -s
        H[1, 4] =  c

        # omega
        H[2, 5] = 1.0

        # psi = phi
        H[3, 2] = 1.0

        return H
  
    
    

    def dynamics_u(self, x, u):
        """
        Continuous-time dynamics: dx/dt = f(x, u)

        Unused in the filters but provided for completeness.
        
        Parameters
        ----------
        x : array (6,)
            Current state [x, y, phi, vx, vy, omega]
        u : array (3,)
            Control input voltages to motors [u1, u2, u3]
        
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
        x, y, phi, vx, vy, omega = x
        
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


    def forward_kinematics(self, w1, w2, w3):
        """
        Compute body velocities from wheel speeds using kinematic matrix.
        
        Parameters
        ----------
        w1, w2, w3 : float
            Wheel angular velocities (rad/s)
            
        Returns
        -------
        tuple
            (vx_b, vy_b, omega_b) - body frame velocities
        """
        # wheel_speeds = H @ [vx_b, vy_b, omega]
        # Therefore: [vx_b, vy_b, omega] = H_pinv @ wheel_speeds
        wheel_speeds = np.array([w1, w2, w3])
        v_body = self.H_pinv @ wheel_speeds
        
        return v_body[0], v_body[1], v_body[2]
    
    def inverse_kinematics(self, vx_b, vy_b, omega_b):
        """
        Compute wheel speeds from body velocities using kinematic matrix.
        
        Parameters
        ----------
        vx_b, vy_b : float
            Body frame linear velocities (m/s)
        omega_b : float
            Angular velocity (rad/s)
            
        Returns
        -------
        tuple
            (w1, w2, w3) - wheel angular velocities (rad/s)
        """
        # wheel_speeds = H @ [vx_b, vy_b, omega]
        v_body = np.array([vx_b, vy_b, omega_b])
        wheel_speeds = self.H @ v_body
        
        return wheel_speeds[0], wheel_speeds[1], wheel_speeds[2]
