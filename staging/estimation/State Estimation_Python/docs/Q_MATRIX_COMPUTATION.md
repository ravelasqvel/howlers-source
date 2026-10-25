# Process Noise Matrix Q Computation Methodology

**Module:** `state_estimation.models.compute_process_noise_Q`  
**Application:** Omnidirectional robots with EKF/UKF/PF filters

---

## 1. Introduction

### The Problem

State estimation filters require the **Q matrix** (process noise covariance), but determining it is challenging:

- **Manual tuning**: Trial-and-error process, time-consuming, not reproducible
- **Weak theoretical foundation**: Difficult to justify parameter choices
- **Inconsistent results**: Performance varies across different trajectories

### Solution

Propagate IMU uncertainty (known) through the identified dynamic model:

```
σ²_accelerometer → Dynamic Model → Q
```

## 2. Theoretical Foundation

### Process Model

```
x_{k+1} = f(x_k, u_k) + w_k,    w_k ~ N(0, Q)
```

- **State:** x = [x, y, φ, v_x, v_y, ω]ᵀ
- **Control:** u = [a_x^b, a_y^b]ᵀ (from IMU)
- **Q:** 6×6 covariance matrix to be determined

### Origin of Process Noise

The noise **w_k** comes from three sources:

1. **IMU:** a_measured = a_real + ε_a, where ε_a ~ N(0, Σ_a)
2. **Model:** Parameters M, I, K, Ra have uncertainty
3. **Unmodeled dynamics:** Friction, slip, discretization

### Uncertainty Propagation

IMU uncertainty propagates through the **control Jacobian**:

```
G = ∂f/∂u,    Q = G · Σ_a · Gᵀ
```

For omnidirectional robot with Euler discretization:

```
       ∂f/∂a_x^b          ∂f/∂a_y^b
    ┌                                ┐
    │ ½Δt²cos(φ)    -½Δt²sin(φ)     │  ← x
    │ ½Δt²sin(φ)     ½Δt²cos(φ)     │  ← y
G = │      0              0          │  ← φ
    │  Δt·cos(φ)     -Δt·sin(φ)     │  ← v_x
    │  Δt·sin(φ)      Δt·cos(φ)     │  ← v_y
    │      0              0          │  ← ω
    └                                ┘
```

**Note:** Position has ½Δt² term (double integral), velocity has Δt (single integral).

---

## 3. Proposed Methodology

### 3.1 Complete Pipeline

The process to obtain Q consists of **6 stages**:

```
Experimental Data → Identification → Simulation → Residuals → Variance → Q
   (real robot)      (parameters)      (model)    (IMU-model)   (σ²)    (6×6)
```

### Stage 1: Data Acquisition

**Objective:** Obtain real robot trajectory with IMU measurements.

- **Record:** IMU (ax, ay, ω), encoders (w₁, w₂, w₃), PWM commands (u₁, u₂, u₃)
- **Frequency:** 100 Hz (minimum 50 Hz)
- **Duration:** 15-30 seconds
- **Trajectory:** Varied omnidirectional motion (circle, zigzag, figure-eight)

### Stage 2: Parameter Identification

**Objective:** Obtain robot dynamic model.

Use **ThreeStageIdentification** to get:
- Stage 1: Ra, K (electrical parameters)
- Stage 2: I (moment of inertia)
- Stage 3: Cu₁, Cu₂, Cu₃ (motor compensation)

Output: `robot_params = {M, I, K, Ra, Cu₁, Cu₂, Cu₃, ...}`

### Stage 3: Simulation with Identified Model

**Objective:** Reproduce trajectory using only the model.

1. Use same PWM inputs from experiment
2. Simulate complete dynamics: Voltages → Currents → Torques → Forces → Accelerations
3. Extract predicted accelerations: `ax_model`, `ay_model`

### Stage 4: Residual Calculation

**Objective:** Quantify difference between real IMU and model.

```
r_ax(t) = ax_IMU(t) - ax_model(t)
r_ay(t) = ay_IMU(t) - ay_model(t)
```

Residuals capture: sensor noise + identification errors + unmodeled dynamics.

### Stage 5: Variance Estimation

**Objective:** Obtain σ²_ax and σ²_ay from residuals.

**Robust method** (recommended):
```
σ²_ax = (1.4826 · MAD(r_ax))²
σ²_ay = (1.4826 · MAD(r_ay))²
```

**Standard method** (alternative):
```
σ²_ax = var(r_ax)
σ²_ay = var(r_ay)
```

MAD = Median Absolute Deviation (robust to outliers).

### Stage 6: Uncertainty Propagation

**Objective:** Convert σ²_a into Q matrix.

1. Build: Σ_a = diag(σ²_ax, σ²_ay)
2. Compute Jacobian: G = G(φ̄, Δt), where φ̄ = mean(φ)
3. Propagate: **Q = G · Σ_a · Gᵀ**

Output: **Q ∈ ℝ⁶ˣ⁶** ready to use in filters.

### Result Validation

**Verify that residuals have:**
- Mean ≈ 0 (no bias)
- Gaussian distribution (Shapiro-Wilk test)
- No temporal autocorrelation

**If fails:** filter data, re-identify parameters, or improve model.

---

## 4. Code Usage

### 4.1 Module Architecture

Module `compute_process_noise_Q.py` implements complete methodology:

```
compute_process_noise_Q.py
│
├── compute_Q_from_experiment()      # Main pipeline
│   ├── load_experimental_data()     # Load data from file
│   ├── compute_model_based_accelerations()  # Simulate dynamics
│   ├── estimate_acceleration_variance()     # Compute σ²
│   ├── compute_process_noise_Q()    # Propagate uncertainty
│   └── analyze_acceleration_residuals()     # Diagnostics
│
├── compute_control_jacobian_G()     # Jacobian ∂f/∂u
├── make_robot_params()              # Parameter structure
└── [auxiliary utilities]
```

### 4.2 Main Function

```python
from state_estimation.models.compute_process_noise_Q import (
    compute_Q_from_experiment, make_robot_params
)

# 1. Robot parameters (from ThreeStageIdentification)
robot_params = make_robot_params(
    M=3.178, I=0.0195, K=0.485, Ra=4.85,
    Cu1=0.952, Cu2=1.018, Cu3=0.983
)

# 2. Compute Q from experiment
results = compute_Q_from_experiment(
    experiment_file='data/sensors/exp2.txt',
    robot_params=robot_params,
    N=1000,
    method='jacobian',           # jacobian/diagonal/physical
    variance_method='robust'     # robust/sample
)

# 3. Use in filter
Q = results['Q']
ekf.Q = Q
```

### 4.3 Available Methods

| Method | Description | When to use |
|--------|-------------|-------------|
| `jacobian` | Propagate σ²_a via G·Σ_a·Gᵀ | **Recommended** (rigorous) |
| `diagonal` | Diagonal Q with heuristic scaling | Quick prototyping |
| `physical` | Based on parameter uncertainty | Without experimental data |

---

## 5. Results

### Computed Q Matrix (Experimental Data)

Results from experiment 2 (exp2.txt), 1000 samples @ 100 Hz:

**Acceleration variances:**
- σ²_ax = 5.32 m²/s⁴ → σ_ax = 2.31 m/s²
- σ²_ay = 4.56 m²/s⁴ → σ_ay = 2.14 m/s²

**Resulting Q matrix (diagonal elements):**

```python
Q_diag = np.diag([
    1.229e-08,  # x   (position, 2nd order integration)
    1.242e-08,  # y   (position, 2nd order integration)
    1.000e-12,  # φ   (orientation, minimal process noise)
    4.915e-04,  # v_x (velocity, directly affected by IMU)
    4.967e-04,  # v_y (velocity, directly affected by IMU)
    1.000e-12   # ω   (angular velocity, minimal process noise)
])
```

**Key observations:**

1. **Magnitude hierarchy:** Q_velocity >> Q_position >> Q_orientation
   - Velocities: ~10⁻⁴ (dominant uncertainty from IMU accelerations)
   - Positions: ~10⁻⁸ (second-order integration reduces noise)
   - Angles: ~10⁻¹² (gyroscope measurements are precise)

2. **Physical interpretation:**
   - Position noise is 4 orders of magnitude smaller than velocity (Δt² scaling)
   - Angular states have minimal process noise (direct measurements)
   - IMU acceleration uncertainty dominates the error budget

### Importance of Jacobian Method

The full Q matrix computed via Jacobian propagation includes off-diagonal coupling terms between position and velocity states. Using only diagonal approximations can lead to inconsistent filter behavior.

**Recommendation:** Always use the complete Jacobian-based Q for accurate uncertainty propagation.

### Dependency on Time Step

**Theoretical scaling:**
- Position noise: Q_position ∝ Δt⁴ (double integration)
- Velocity noise: Q_velocity ∝ Δt² (single integration)

**Practical values for Δt = 0.01s:**
- Doubling Δt → Q_position increases ~16×, Q_velocity increases ~4×
- Reducing Δt → More accurate integration, but higher computational cost

---

## 6. Conclusions

### Main Contributions

This work presents a complete methodology to **automatically estimate the Q matrix** for state filters, with the following contributions:

1. **Rigorous foundation:** Uncertainty propagation from system physics via Jacobian
2. **Automated pipeline:** From experimental data to ready-to-use Q matrix in ~5 minutes
3. **Reproducible:** Same methodology applies to any robot with identified parameters
4. **Physically grounded:** Q values derived from measured IMU noise characteristics
5. **Complete implementation:** Python code with comprehensive documentation

### Limitations and Future Work

**Current limitations:**

1. **Linear approximation:** Jacobian-based propagation assumes small deviations
   - Future: Use Unscented Transform for highly nonlinear systems

2. **Discrete model:** Assumes explicit Euler integration
   - Future: Extend to higher-order methods (RK4, trapezoidal)

3. **Stationary Q:** Assumes constant process noise over time
   - Future: Implement adaptive Q estimation based on filter residuals

4. **Requires identification:** Depends on accurate robot parameter estimates
   - Future: Joint estimation of parameters and Q matrix

### Quick Guide

```python
# 1. Identify parameters (ThreeStageIdentification)
# 2. Run experiment on real robot
# 3. Compute Q
results = compute_Q_from_experiment(
    experiment_file='data/sensors/exp2.txt',
    robot_params=make_robot_params(M=3.178, I=0.0195, ...),
    N=1000
)
# 4. Use in filter
ekf.Q = results['Q']
```

---

## 7. References

**Fundamental bibliography:**

1. Bar-Shalom, Y., et al. (2001). *Estimation with Applications to Tracking and Navigation.* Wiley.
2. Simon, D. (2006). *Optimal State Estimation.* Wiley.
3. Thrun, S., et al. (2005). *Probabilistic Robotics.* MIT Press.
4. Mehra, R. K. (1970). "On the identification of variances and adaptive Kalman filtering." *IEEE Trans. Automatic Control*, 15(2), 175-184.

---

## Appendix: Troubleshooting

### Common Problems

| Symptom | Cause | Solution |
|---------|-------|----------|
| Filter divergence | Q too small | Increase Q (multiply by 2-5) |
| Overly conservative estimates | Q too large | Decrease Q (multiply by 0.2-0.5) |
| Non-Gaussian residuals | Poorly identified model | Re-identify parameters, use robust variance (MAD) |
| Simulation diverges | Wrong parameters | Verify M>0, I>0, Ra>0, K>0 |

### Data Format

CSV with columns: `t, ax, ay, alpha, w1, w2, w3, u1, u2, u3`
- Frequency: 100 Hz
- Units: SI (m, rad, s)
- Duration: 15-30 s

### Complete Example

See: `examples/example_Q_from_identification.py`

```bash
python examples/example_Q_from_identification.py
```

Outputs saved to: `results/estimation/Q_computation/`
