# Omnidirectional Robot System Identification

## Overview

This project implements a comprehensive **three-stage parameter identification** framework for omnidirectional mobile robots with three omni wheels. The approach combines rigorous kinematic modeling, dynamic simulation, and sequential optimization to estimate motor electrical parameters, robot inertia, and motor compensation factors.

## Robot Configuration

- **Wheel 1** $(w_1)$: 150° - Back-left
- **Wheel 2** $(w_2)$: 270° - Back-right (pointing downward)
- **Wheel 3** $(w_3)$: 30° - Front-right
- **Roller type:** Standard omni wheels (rollers perpendicular to wheel axis)

```
         Y
         ↑
         |
    w₁   |    w₃
    ●    |    ●
         |
---------+--------→ X
         |    
       w₂●
         |
```

## Mathematical Model Development

### 1. Kinematic Model

#### 1.1 Forward Kinematics

For a three-wheeled omnidirectional robot, the relationship between robot velocity in the body frame and wheel angular velocities is given by the **kinematic matrix:**

$$\mathbf{H}$$

$$
\begin{bmatrix}
\omega_1 \\
\omega_2 \\
\omega_3
\end{bmatrix}
= \mathbf{H} \cdot
\begin{bmatrix}
v_x \\
v_y \\
\Omega
\end{bmatrix}
$$

Where:

- Wheel angular velocities [rad/s]: $$\boldsymbol{\omega} = [\omega_1, \omega_2, \omega_3]^T$$

- Robot velocity in body frame [m/s, m/s, rad/s]: $$\mathbf{v} = [v_x, v_y, \Omega]^T$$

- Kinematic transformation matrix: $$\mathbf{H} \in \mathbb{R}^{3 \times 3}$$

#### 1.2 Kinematic Matrix Derivation

For standard omni wheels (rollers perpendicular to wheel axis), each wheel constraint is:

$$
\omega_i = \frac{1}{r}\left[-\sin(\theta_i) \cdot v_x + \cos(\theta_i) \cdot v_y + d \cdot \Omega\right]
$$

Where:

- Angular position of wheel $i$ from robot's X-axis [radians]: $$\theta_i$$

- Wheel radius [m]: $$r$$

- Distance from robot center to wheel contact point [m]: $$d$$

Thus, the kinematic matrix is:

$$
\mathbf{H} = \frac{1}{r}
\begin{bmatrix}
-\sin(\theta_1) & \cos(\theta_1) & d \\
-\sin(\theta_2) & \cos(\theta_2) & d \\
-\sin(\theta_3) & \cos(\theta_3) & d
\end{bmatrix}
$$

For our configuration:

$$\theta_1 = 150\degree = 5\pi/6 \text{ rad}$$

$$\theta_2 = 270\degree = 3\pi/2 \text{ rad}$$

$$\theta_3 = 30\degree = \pi/6 \text{ rad}$$

**Reference:** Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.

#### 1.3 Inverse Kinematics

To obtain robot velocities from wheel speeds:

$$
\begin{bmatrix}
v_x \\
v_y \\
\Omega
\end{bmatrix}
= \mathbf{H}^+ \cdot
\begin{bmatrix}
\omega_1 \\
\omega_2 \\
\omega_3
\end{bmatrix}
$$

Where the Moore-Penrose pseudoinverse is:

$$\mathbf{H}^+ = (\mathbf{H}^T \mathbf{H})^{-1} \mathbf{H}^T$$

### 2. Motor Electrical Model

Each motor follows a DC motor electrical model with back-EMF:

#### 2.1 Voltage Equation

$$
V_i = R_a \cdot i_i + K \cdot \omega_i
$$

Where:

- Applied voltage to motor $i$ [V]: $$V_i$$

- Armature current [A]: $$i_i$$

- Armature resistance [Ω]: $$R_a$$

- Motor constant [V/(rad/s) = Nm/A]: $$K$$

- Wheel angular velocity [rad/s]: $$\omega_i$$

#### 2.2 Torque Equation

Motor torque is proportional to current:

$$
\tau_i = K \cdot i_i
$$

#### 2.3 Combined Motor Model

Solving the voltage equation for current and substituting into torque:

$$
\tau_i = K \cdot \frac{V_i - K \cdot \omega_i}{R_a} = \frac{K}{R_a} \cdot V_i - \frac{K^2}{R_a} \cdot \omega_i
$$

**Reference:** Hughes, A., & Drury, B. (2019). *Electric Motors and Drives: Fundamentals, Types and Applications* (5th ed.). Elsevier.

#### 2.4 PWM to Voltage Conversion

Experimental data from embedded systems uses **PWM duty cycle** (0-100%) instead of voltage. The effective voltage is:

$$
V_{\text{eff}} = \frac{D}{100} \cdot V_{\text{bat}}
$$

This conversion is necessary because the motor's electrical time constant ($\tau_e = L_a/R_a \approx 1$-10 ms) filters the PWM signal (typically 1-20 kHz) into an equivalent DC voltage.

**Important:** $V_{\text{bat}}$ must be measured during data collection as it varies with battery discharge (e.g., 3S LiPo: 12.6V → 9.0V).

**Reference:** Mohan, N., Undeland, T. M., & Robbins, W. P. (2003). *Power Electronics: Converters, Applications, and Design* (3rd ed.). Wiley. Chapter 7.

### 3. Robot Dynamics Model

#### 3.1 Rigid Body Dynamics in Body Frame

The robot's motion is governed by Newton-Euler equations in the body-fixed frame:

$$
\begin{aligned}
M \cdot \mathbf{a}_{\text{local}} &= \mathbf{F}_{\text{local}} \\
I \cdot \alpha &= \tau_z
\end{aligned}
$$

Where:
- $M$: robot mass (kg)
- $I$: moment of inertia about vertical axis (kg·m²)
- $\mathbf{a}_{\text{local}} = [a_x, a_y]^T$: linear acceleration in body frame (m/s²)
- $\alpha$: angular acceleration (rad/s²)
- $\mathbf{F}_{\text{local}} = [F_x, F_y]^T$: total force in body frame (N)
- $\tau_z$: total torque about vertical axis (Nm)

#### 3.2 Force Transformation

Wheel torques are transformed to robot forces using the transpose of the kinematic matrix:

$$
\begin{bmatrix}
F_x \\
F_y \\
\tau_z
\end{bmatrix}
= \frac{1}{r} \cdot \mathbf{H}^T \cdot
\begin{bmatrix}
\tau_1 \\
\tau_2 \\
\tau_3
\end{bmatrix}
$$

This relationship comes from the principle of virtual work and ensures power conservation:

$$
P_{\text{robot}} = \mathbf{v} \cdot \mathbf{F} = \boldsymbol{\omega} \cdot \boldsymbol{\tau}
$$

**Reference:** Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

#### 3.3 Coordinate Transformation

Accelerations must be transformed from body frame to global frame using the rotation matrix:

$$
\mathbf{R}(\phi) =
\begin{bmatrix}
\cos(\phi) & -\sin(\phi) & 0 \\
\sin(\phi) & \cos(\phi) & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

The transformation is:

$$
\mathbf{a}_{\text{global}} = \mathbf{R}(\phi)^T \cdot \mathbf{a}_{\text{local}}
$$

#### 3.4 Coriolis Terms

When transforming between rotating reference frames, Coriolis effects appear:

$$
\begin{aligned}
a_{\text{global},x} &= a_{\text{local},x} + \Omega \cdot v_y \\
a_{\text{global},y} &= a_{\text{local},y} - \Omega \cdot v_x
\end{aligned}
$$

Where $\Omega$ is the angular velocity of the robot.

**Reference:** Goldstein, H., Poole, C., & Safko, J. (2002). *Classical Mechanics* (3rd ed.). Addison-Wesley.

### 4. Complete State-Space Model

#### 4.1 State Vector

$$
\mathbf{x} = [x, y, \phi, v_x, v_y, \Omega]^T
$$

Where:

- Position in global frame [m]: $$x, y$$

- Orientation angle [rad]: $$\phi$$

- Velocity in global frame [m/s]: $$v_x, v_y$$

- Angular velocity [rad/s]: $$\Omega$$

#### 4.2 State-Space Equations

$$
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}, t)
$$

**Expanded form:**

**Position dynamics:**

$$
\dot{x} = v_x, \quad \dot{y} = v_y, \quad \dot{\phi} = \Omega
$$

**Velocity dynamics (with Coriolis terms):**

$$
\dot{v}_x = a_{\text{global},x} + \Omega \cdot v_y
$$

$$
\dot{v}_y = a_{\text{global},y} - \Omega \cdot v_x
$$

**Angular dynamics:**

$$
\dot{\Omega} = \frac{\tau_z}{I}
$$

Where accelerations are computed through the complete chain:

$$
\mathbf{u} \rightarrow \boldsymbol{\tau}_{\text{motor}} \rightarrow \mathbf{F}_{\text{local}} \rightarrow \mathbf{a}_{\text{local}} \rightarrow \mathbf{a}_{\text{global}}
$$

## Three-Stage Identification Protocol

### Stage 1: Motor Electrical Parameters

**Objective:** Estimate armature resistance and motor constant from *steady-state* tests.

$$R_a, K$$

**Experimental Setup:**
1. Lift robot (wheels free to spin) or mount motor on bench
2. Apply constant voltage to one motor: $$V_i$$
3. Wait for steady state (~2 seconds)
4. Measure steady-state wheel speed: $$\omega_i$$
5. Repeat for 5-10 voltage levels

**Physical Model:** At steady state (angular acceleration = 0), motor torque balances friction torque:

$$\dot{\omega} = 0$$

$$
\tau_{\text{motor}} = \tau_{\text{friction}}
$$

With viscous friction model:

$$\tau_{\text{friction}} = b \cdot \omega$$

the electrical equation becomes:

$$
V = R_a \cdot i + K \cdot \omega
$$

And the mechanical equilibrium:

$$
K \cdot i = b \cdot \omega
$$

Solving for current and substituting into voltage equation:

$$i = \frac{b \cdot \omega}{K}$$

$$
V = R_a \cdot \frac{b \cdot \omega}{K} + K \cdot \omega = \left(\frac{R_a \cdot b}{K} + K\right) \cdot \omega
$$

This is a **linear relationship** between $V$ and $\omega$:

$$
\omega = \frac{K}{R_a \cdot b + K^2} \cdot V
$$

**Optimization Problem:**

$$
\min_{R_a, K, b} \sum_{i=1}^{N} \left(\omega_{\text{meas},i} - \frac{K \cdot V_i}{R_a \cdot b + K^2}\right)^2
$$

**Output:**

- Armature resistance [Ω]: $$R_a$$
- Motor constant [V/(rad/s)]: $$K$$  
- Friction coefficient [Nm/(rad/s)]: $$b$$

### Stage 2: Moment of Inertia

**Objective:** Estimate robot inertia from pure rotation maneuvers.

$$I$$

**Experimental Setup:**
1. Command robot to rotate in place (minimal translation):
   $$v_x \approx 0, \quad v_y \approx 0$$
2. Vary angular velocity: slow → fast → stop → reverse
3. Record orientation and angular velocity for 10-15 seconds:
   $$\phi(t), \quad \Omega(t)$$
4. Ensure minimal translation (wheels don't slip)

**Why This Works:** Decouples rotational dynamics from translational. The rotational equation:

$$
I \cdot \dot{\Omega} = \tau_z
$$

depends only on $I$ and is less affected by friction/mass uncertainties.

**Optimization Problem:** Minimize weighted error in angular states:

$$
\min_{I} \sum_{t} \left[w_\phi \cdot (\phi_{\text{meas}}(t) - \phi_{\text{model}}(t))^2 + w_\Omega \cdot (\Omega_{\text{meas}}(t) - \Omega_{\text{model}}(t))^2\right]
$$

Where:

- Weight for orientation tracking: $$w_\phi = 10.0$$

- Weight for angular velocity tracking: $$w_\Omega = 5.0$$

- Translation states are ignored (weights = 0)

**Output:**

- Moment of inertia [kg·m²]: $$I$$

### Stage 3: Motor Compensation Factors

**Objective:** Account for motor-to-motor variations due to manufacturing tolerances.

$$C_{u_1}, C_{u_2}, C_{u_3}$$

**Experimental Setup:**
1. Execute rich trajectory (e.g., square path: right → up → left → down)
2. Include both translations and rotations
3. Record complete state for 15-20 seconds:
   $$\mathbf{x}(t)$$
4. Use varied speeds and directions

**Why This is Needed:** Even with identical motor models, real motors have variations:
- Slight differences in winding resistance
- Bearing friction asymmetries
- Magnetic field non-uniformities
- Assembly tolerances

**Compensation Model:** Scale commanded voltages individually:

$$
u_{\text{actual},i} = C_{u_i} \cdot u_{\text{commanded},i}, \quad i = 1, 2, 3
$$

**Optimization Problem:** Minimize position and orientation tracking errors:

$$
\min_{C_{u_1}, C_{u_2}, C_{u_3}} \sum_{t} \left[w_{\text{pos}} \cdot \|\mathbf{r}_{\text{meas}}(t) - \mathbf{r}_{\text{model}}(t)\|^2 + w_\phi \cdot (\phi_{\text{meas}}(t) - \phi_{\text{model}}(t))^2\right]
$$

Where:

- Weight for position tracking (x, y): $$w_{\text{pos}} = 3.0$$

- Weight for orientation tracking: $$w_\phi = 2.0$$

- Velocity weights (lower priority): $$w_v = 0.5$$

**Output:**

- Compensation factors [dimensionless, nominal = 1.0]: $$C_{u_1}, C_{u_2}, C_{u_3}$$

**Typical Values:** 0.85 - 1.15 (within ±15% indicates acceptable motor matching)

## Installation

### Requirements
- Python 3.8+
- NumPy ≥ 1.21.0
- SciPy ≥ 1.7.0
- Matplotlib ≥ 3.4.0
- scikit-learn ≥ 0.24.0

### Setup (Windows PowerShell)

```powershell
# Create virtual environment
python -m venv venv

# Activate
.\venv\Scripts\Activate.ps1

# Install dependencies
pip install -r requirements.txt
```

### Setup (Linux/macOS)

```bash
# Create virtual environment
python3 -m venv venv

# Activate
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

## Usage

### Running Examples

All identification examples are executed from the **project root** using the unified execution script `run.ps1`, as described in the [main README](../README.md#execution-system). This script manages the virtual environment and working directories automatically.

From the project root directory:

```powershell
# Three-stage identification examples
.\run.ps1 stage1          # Stage 1: Motor electrical parameters
.\run.ps1 stage2          # Stage 2: Moment of inertia
.\run.ps1 stage3          # Stage 3: Motor compensation factors

# Simulation testing
.\run.ps1 sim_test        # Test robot model vs experimental data

# Show all available commands
.\run.ps1 -Help
```

#### Stage-by-Stage Execution

**Stage 1 - Motor Parameters:**
```powershell
.\run.ps1 stage1
```
Processes experimental steady-state motor tests from `data/raw/motor_tests/stage1_X.txt`:
- Loads encoder and PWM data (12 seconds, 8 segments)
- Applies Kalman filtering to velocities
- Converts PWM to voltage (battery voltage measured during experiment)
- Extracts steady-state values from each segment
- Identifies Ra, K parameters via nonlinear optimization

**Stage 2 - Inertia Identification:**
```powershell
.\run.ps1 stage2
```
Estimates robot inertia from pure rotation tests in `data/raw/rotation_tests/stage2_X.txt`:
- Uses IMU orientation and encoder data
- Reconstructs full state vector via inverse kinematics
- Minimizes weighted angular tracking error

**Stage 3 - Compensation Factors:**
```powershell
.\run.ps1 stage3
```
Calculates motor compensation factors using video+IMU trajectory data:
- Source data: `data/raw/video_trajectories/` (processed by Video_Robot_Tracking)
- Output location: `Video_Robot_Tracking/traj_vid_processed/`
- Rich trajectories with mixed translations and rotations
- Optimizes compensation factors to minimize position/orientation errors

**Simulation Testing:**
```powershell
.\run.ps1 sim_test
```
Validates complete robot model against experimental trajectory data:
- Simulates forward dynamics with identified parameters
- Compares model predictions vs. measurements

#### Configuration

To modify experiment parameters, edit the corresponding example file in `Robot_Identification/src/examples/`:
- `stage1_experimental.py` - Motor selection, battery voltage, segment processing
- `stage2_experimental.py` - Weight factors, optimization settings
- `stage3_experimental_vid.py` - Trajectory selection, compensation bounds
- `test_simulation_vid4.py` - Simulation parameters, integration method

**Note:** Do not run Python scripts directly from the `Robot_Identification/` directory. Always use `.\run.ps1` from the project root to ensure proper environment activation and path configuration.

### Expected Outputs

**Console Output:**
- **Stage 1:** Ra, K, friction coefficient (b), R² goodness-of-fit
- **Stage 2:** Moment of inertia (I), RMSE on angular states
- **Stage 3:** Compensation factors Cu1, Cu2, Cu3, full-state RMSE

**Generated Plots:**
- **Stage 1:** Voltage vs. speed with fitted model
- **Stage 2:** Angular state tracking (φ and ω vs. time)
- **Stage 3:** 2D trajectory comparison and state-by-state validation

**Results Location:** `Robot_Identification/results/identification/`

## Installation

This project uses a unified virtual environment managed from the project root. See the [main README](../README.md#quick-start) for complete setup instructions.

**Quick setup:**
```powershell
# From project root
.\setup.ps1
```

This installs all dependencies for Robot_Identification, State Estimation, and Video Tracking modules.

## Project Structure

```
Robot_Identification/
├── src/
│   ├── kinematics/              # Kinematic matrix computation and validation
│   ├── models/                  # Robot dynamics (OmnidirectionalRobot class)
│   ├── identification/          # ThreeStageIdentification algorithm
│   ├── utils/                   # PWM conversion utilities
│   └── examples/                # Executable scripts (stage1, stage2, stage3)
├── shared_utils/                # Shared utilities (signal processing, data loading)
└── results/                     # Output directory (plots and metrics)
```

**Key modules:**
- `ThreeStageIdentification`: Main identification class
- `OmnidirectionalRobot`: Forward dynamics model
- `KinematicValidator`: Kinematic matrix computation

## Data Requirements

**Stage 1 (Motor Tests):**
- Format: CSV with columns `[t, ax, ay, alpha, w1, w2, w3, u1, u2, u3, ...]`
- Duration: ~12 seconds, 8 constant-input segments
- Required: Wheel velocities (rad/s), PWM inputs (%)

**Stage 2 (Rotation Tests):**
- Format: Same as Stage 1
- Maneuver: Pure rotation in place
- Required: IMU orientation (alpha), wheel velocities, control inputs

**Stage 3 (Video Trajectories):**
- Format: CSV from `process_imu_data.py` with columns `[time_s, x_m, y_m, phi_rad, vx_m_s, vy_m_s, omega_rad_s, u1_pwm, u2_pwm, u3_pwm]`
- Duration: ≥10 seconds @ 100 Hz
- Required: Full state vector + synchronized control inputs

## Theoretical Background and References

### Omnidirectional Kinematics
1. **Lynch, K. M., & Park, F. C.** (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press. Chapter 13: Wheeled Mobile Robots.

2. **Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D.** (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press. Chapter 3: Mobile Robot Kinematics.

3. **Campion, G., Bastin, G., & D'Andréa-Novel, B.** (1996). Structural properties and classification of kinematic and dynamic models of wheeled mobile robots. *IEEE Transactions on Robotics and Automation*, 12(1), 47-62.

### DC Motor Modeling
4. **Hughes, A., & Drury, B.** (2019). *Electric Motors and Drives: Fundamentals, Types and Applications* (5th ed.). Elsevier.

5. **Chapman, S. J.** (2012). *Electric Machinery Fundamentals* (5th ed.). McGraw-Hill.

### System Identification
6. **Ljung, L.** (1999). *System Identification: Theory for the User* (2nd ed.). Prentice Hall. Chapters on nonlinear system identification.

7. **Åström, K. J., & Murray, R. M.** (2008). *Feedback Systems: An Introduction for Scientists and Engineers*. Princeton University Press.

### Rigid Body Dynamics
8. **Goldstein, H., Poole, C., & Safko, J.** (2002). *Classical Mechanics* (3rd ed.). Addison-Wesley. Chapter 4: The Kinematics of Rigid Body Motion.

9. **Murray, R. M., Li, Z., & Sastry, S. S.** (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press.

## License

This project is intended for academic and research purposes.

## Author

Implementation with validated kinematics and three-stage identification protocol.
