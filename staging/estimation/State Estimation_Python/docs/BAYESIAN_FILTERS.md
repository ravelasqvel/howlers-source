# Bayesian State Estimation Methodology for Omnidirectional Robots

## Executive Summary

This document describes the implementation methodology of three Bayesian filters (**Extended Kalman Filter**, **Unscented Kalman Filter**, and **Particle Filter**) applied to state estimation of a three-wheeled omnidirectional robot. The filters process noisy measurements from inertial sensors (IMU) and encoders to estimate the complete state vector:

$$\mathbf{x} = [x, y, \psi, v_x, v_y, \omega]^T$$

in real-time.

The implementation follows a modular design that separates:

1. **System model** ([`omnidirectional.py`](../state_estimation/models/omnidirectional.py)): Dynamics and measurements

   $$f(\mathbf{x}, \mathbf{u}), \quad h(\mathbf{x})$$

2. **Bayesian filters** ([`filters/`](../state_estimation/filters/)): EKF, UKF, PF with unified API
3. **Execution scripts** ([`examples/`](../examples/)): Configuration, data loading, and validation

---

## Common Theoretical Framework

### State Model

**State vector** (global frame):
$$
\mathbf{x} = \begin{bmatrix} x \\ y \\ \psi \\ v_x \\ v_y \\ \omega \end{bmatrix} \in \mathbb{R}^6
$$

Where:

- Position in global frame [m]: $$x, y$$

- Orientation (heading angle) [rad]: $$\psi$$

- Velocity in global frame [m/s]: $$v_x, v_y$$

- Angular velocity [rad/s]: $$\omega$$

**Control vector** (accelerations in body frame):
$$
\mathbf{u} = \begin{bmatrix} a_{x,b} \\ a_{y,b} \end{bmatrix} \in \mathbb{R}^2
$$

**Measurement vector**:
$$
\mathbf{z} = \begin{bmatrix} v_{x,b} \\ v_{y,b} \\ \omega \\ \psi \end{bmatrix} \in \mathbb{R}^4
$$

Where $(v_{x,b}, v_{y,b})$ are velocities in body frame derived from encoders via inverse kinematics.

### System Dynamics

The model implemented in [`omnidirectional.py`](../state_estimation/models/omnidirectional.py#L90) performs transformation of accelerations from body frame to global frame:

$$
\begin{aligned}
\mathbf{R}(\psi) &= \begin{bmatrix} \cos\psi & -\sin\psi \\ \sin\psi & \cos\psi \end{bmatrix} \\[0.5em]
\begin{bmatrix} a_x \\ a_y \end{bmatrix}_{\text{global}} &= \mathbf{R}(\psi) \begin{bmatrix} a_{x,b} \\ a_{y,b} \end{bmatrix}
\end{aligned}
$$

**Discrete state equations** (Euler with $\Delta t = 0.01$ s):

$$
\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k) = \begin{bmatrix}
x_k + v_{x,k} \cdot \Delta t \\
y_k + v_{y,k} \cdot \Delta t \\
\psi_k + \omega_k \cdot \Delta t \\
v_{x,k} + a_x \cdot \Delta t \\
v_{y,k} + a_y \cdot \Delta t \\
\omega_k
\end{bmatrix}
$$

Where:

$$a_x = \cos\psi \cdot a_{x,b} - \sin\psi \cdot a_{y,b}$$

$$a_y = \sin\psi \cdot a_{x,b} + \cos\psi \cdot a_{y,b}$$

**Note:** Angular velocity follows a random walk model (Wiener process):

$$\omega_{k+1} = \omega_k + \text{noise}$$

assuming smooth changes between time steps.

### Measurement Model

Transformation of global velocities to body frame:

$$
h(\mathbf{x}) = \begin{bmatrix}
\cos\psi \cdot v_x + \sin\psi \cdot v_y \\
-\sin\psi \cdot v_x + \cos\psi \cdot v_y \\
\omega \\
\psi
\end{bmatrix}
$$

Implemented in [`omnidirectional.py`](../state_estimation/models/omnidirectional.py#L152).

---

## 1. Extended Kalman Filter (EKF)

**Implementation file:** [`filters/extended.py`](../state_estimation/filters/extended.py)  
**Execution script:** [`ekf_omnidirectional.py`](../examples/ekf_omnidirectional.py)

### 1.1 Mathematical Foundation

The EKF locally linearizes the nonlinear system via first-order Taylor expansions around the estimated state, using Jacobian matrices:

$$\mathbf{F} = \frac{\partial f}{\partial \mathbf{x}}, \quad \mathbf{H} = \frac{\partial h}{\partial \mathbf{x}}$$

#### Prediction Step

**A priori state:**
$$
\hat{\mathbf{x}}_{k|k-1} = f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k)
$$

**A priori covariance:**
$$
\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^T + \mathbf{Q}_k
$$

#### Update Step

**Residual (innovation):**
$$
\mathbf{y}_k = \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1})
$$

**Innovation covariance:**
$$
\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k
$$

**Kalman gain:**
$$
\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{S}_k^{-1}
$$

**A posteriori state:**
$$
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k
$$

**A posteriori covariance** (Joseph form for numerical stability):
$$
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T
$$

### 1.2 Jacobian Computation

#### Dynamics Jacobian: $\mathbf{F} = \frac{\partial f}{\partial \mathbf{x}}$

Implemented in [`omnidirectional.py`](../state_estimation/models/omnidirectional.py#L193):

$$
\mathbf{F}_k = \begin{bmatrix}
1 & 0 & 0 & \Delta t & 0 & 0 \\
0 & 1 & 0 & 0 & \Delta t & 0 \\
0 & 0 & 1 & 0 & 0 & \Delta t \\
0 & 0 & \frac{\partial a_x}{\partial \psi} \Delta t & 1 & 0 & 0 \\
0 & 0 & \frac{\partial a_y}{\partial \psi} \Delta t & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

Where the rotational coupling terms are:

$$
\begin{aligned}
\frac{\partial a_x}{\partial \psi} &= -\sin\psi \cdot a_{x,b} - \cos\psi \cdot a_{y,b} \\
\frac{\partial a_y}{\partial \psi} &= \cos\psi \cdot a_{x,b} - \sin\psi \cdot a_{y,b}
\end{aligned}
$$

**Physical interpretation:** These terms capture how changes in orientation affect acceleration in the global frame, representing the kinematic coupling between rotation and translation.

#### Measurement Jacobian: $\mathbf{H} = \frac{\partial h}{\partial \mathbf{x}}$

Implemented in [`omnidirectional.py`](../state_estimation/models/omnidirectional.py#L216):

$$
\mathbf{H}_k = \begin{bmatrix}
0 & 0 & -\sin\psi \cdot v_x + \cos\psi \cdot v_y & \cos\psi & \sin\psi & 0 \\
0 & 0 & -\cos\psi \cdot v_x - \sin\psi \cdot v_y & -\sin\psi & \cos\psi & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 1 & 0 & 0 & 0
\end{bmatrix}
$$

**Physical interpretation:** The first and second rows represent the sensitivity of body-frame velocities with respect to global velocities and orientation. These partial derivatives capture the variation in velocity projection when rotating the reference frame.

### 1.3 Angle Handling

**Problem:** The angle has periodic boundaries:

$$\psi \in [-\pi, \pi]$$

This presents discontinuities - for example, the orientations at $+\pi$ and $-\pi$ are identical physically, but their arithmetic difference is $2\pi$, which can cause filter instability.

**Solution implemented:**

In [`ekf_omnidirectional.py`](../examples/ekf_omnidirectional.py#L111):
```python
from state_estimation.common import make_residual_fn

ekf.set_residual_fn(make_residual_fn(angle_indices=[2]))  # psi at index 2
```

The `make_residual_fn` function ([`common/angles.py`](../state_estimation/common/angles.py)) wraps the angular difference to the interval $(-\pi, \pi]$:

$$
\text{residual}_{\psi}(a, b) = \text{atan2}(\sin(a - b), \cos(a - b))
$$

This ensures angular innovations are consistent (e.g., $\psi_{\text{meas}} = -179°$ and $\psi_{\text{pred}} = 179°$ results in residual of $2°$, not $358°$).

### 1.4 Covariance Matrices

**Process noise** $\mathbf{Q}$ (experimentally calibrated):
$$
\mathbf{Q} = \text{diag}([1.23 \times 10^{-8}, 1.24 \times 10^{-8}, 1 \times 10^{-12}, 4.91 \times 10^{-4}, 4.97 \times 10^{-4}, 1 \times 10^{-12}])
$$

- Small values for position ($\sim 10^{-8}$): High confidence in integration model
- Larger values for velocity ($\sim 10^{-4}$): Greater uncertainty in IMU accelerations
- Minimal values for $\psi$ and $\omega$: Confidence in orientation model

**Measurement noise** $\mathbf{R}$:
$$
\mathbf{R} = \text{diag}([6.72 \times 10^{-4}, 6.72 \times 10^{-4}, 1.31 \times 10^{-2}, 4.06 \times 10^{-6}])
$$

Corresponding to $[v_{x,b}, v_{y,b}, \omega, \psi]$. Values derived from experimental characterization of encoders and IMU.

### 1.5 Experimental Configuration

**Initialization** ([`ekf_omnidirectional.py`](../examples/ekf_omnidirectional.py#L95)):
```python
ekf.x = ground_truth[0]  # Initial state (no error in controlled experiments)
ekf.P = np.diag([0.5, 0.5, 0.1, 0.2, 0.2, 0.05])  # Initial covariance
```

**Update frequency:** 100 Hz ($\Delta t = 0.01$ s)

**Execution flow:**
```python
for k in range(N - 1):
    ekf.predict(u=controls[k], f=robot.dynamics, F=robot.jacobian_F)
    ekf.update(z=measurements[k + 1], h=robot.measurement, H=robot.jacobian_H)
    estimates[k + 1] = ekf.x
```

---

## 2. Unscented Kalman Filter (UKF)

**Implementation file:** [`filters/unscented.py`](../state_estimation/filters/unscented.py)  
**Execution script:** [`ukf_omnidirectional.py`](../examples/ukf_omnidirectional.py)

### 2.1 Mathematical Foundation: Unscented Transform

The UKF avoids the Jacobian linearization of the EKF through the **Unscented Transform (UT)**, which propagates a Gaussian distribution through nonlinear transformations using a deterministic set of sigma points.

**Principle:** It's easier to approximate a Gaussian distribution than to approximate arbitrary nonlinear functions. The UT achieves second-order accuracy (Taylor terms up to $\mathcal{O}(\Delta x^2)$) for any nonlinearity, while the EKF is only first-order.

#### Sigma Points Generation

For a state $\mathbf{x} \sim \mathcal{N}(\bar{\mathbf{x}}, \mathbf{P})$ of dimension $n$, generate $2n+1$ sigma points:

$$
\mathcal{X}^{(i)} = \begin{cases}
\bar{\mathbf{x}} & i = 0 \\
\bar{\mathbf{x}} + \left(\sqrt{(n + \lambda)\mathbf{P}}\right)_i & i = 1, \ldots, n \\
\bar{\mathbf{x}} - \left(\sqrt{(n + \lambda)\mathbf{P}}\right)_{i-n} & i = n+1, \ldots, 2n
\end{cases}
$$

Where $\sqrt{\mathbf{P}}$ is the Cholesky decomposition: $\mathbf{L}\mathbf{L}^T = \mathbf{P}$.

**Implementation** ([`filters/unscented.py`](../state_estimation/filters/unscented.py#L54)):
```python
def sigma_points(self, x, P):
    U = cholesky((self.n + self._lambda) * P)  # scipy returns L^T
    sigmas = np.zeros((2*self.n + 1, self.n))
    sigmas[0] = x
    for k in range(self.n):
        sigmas[k + 1] = x + U[k]
        sigmas[n + k + 1] = x - U[k]
    return sigmas
```

**Note:** `scipy.linalg.cholesky` returns the upper triangular matrix $U$ where $U^T U = A$. Rows are accessed as columns to construct sigma point directions.

#### Scaling Parameters: $\alpha$, $\beta$, $\kappa$

**Combined scaling parameter:**
$$
\lambda = \alpha^2 (n + \kappa) - n
$$

**$\alpha \in (0, 1]$:** Controls sigma point spread around the mean.
- Small values ($\alpha \approx 0.001$): Sigma points very close to $\bar{\mathbf{x}}$ → less exploration, less influence of nonlinearities
- Large values ($\alpha \approx 1$): Greater spread → better captures nonlinear function curvature
- **Value used:** $\alpha = 0.5$ (balance between stability and accuracy)

**$\beta \geq 0$:** Incorporates prior knowledge of the distribution.
- $\beta = 2$: Optimal for Gaussian distributions (minimizes fourth-order error in kurtosis)
- **Value used:** $\beta = 2.0$ (assuming Gaussianity)

**$\kappa \in \mathbb{R}$:** Secondary tuning parameter.
- $\kappa = 3 - n$: Guarantees positive semi-definite matrix
- $\kappa = 0$: Common simplification
- **Value used:** $\kappa = 0.0$

**Experimental configuration** ([`ukf_omnidirectional.py`](../examples/ukf_omnidirectional.py#L107)):
```python
points = MerweScaledSigmaPoints(n=6, alpha=0.5, beta=2.0, kappa=0.0)
```

With $n=6$, results in $\lambda = 0.5^2 \cdot 6 - 6 = -4.5$.

#### Weights for Mean and Covariance

**Mean weight:**
$$
W_m^{(i)} = \begin{cases}
\frac{\lambda}{n + \lambda} & i = 0 \\
\frac{1}{2(n + \lambda)} & i = 1, \ldots, 2n
\end{cases}
$$

**Covariance weight:**
$$
W_c^{(0)} = \frac{\lambda}{n + \lambda} + (1 - \alpha^2 + \beta), \quad W_c^{(i)} = W_m^{(i)} \text{ for } i > 0
$$

The term $(1 - \alpha^2 + \beta)$ in $W_c^{(0)}$ corrects higher-order effects in covariance.

**Implementation** ([`filters/unscented.py`](../state_estimation/filters/unscented.py#L45)):
```python
self.Wm = np.full(2*n + 1, 0.5 / (n + self._lambda))
self.Wc = np.copy(self.Wm)
self.Wm[0] = self._lambda / (n + self._lambda)
self.Wc[0] = self._lambda / (n + self._lambda) + (1 - alpha**2 + beta)
```

With the used values:
- $W_m^{(0)} = -4.5 / 1.5 = -3.0$
- $W_m^{(i)} = 0.5 / 1.5 = 0.333$ for $i > 0$
- $W_c^{(0)} = -3.0 + (1 - 0.25 + 2) = -0.25$

**Note:** Negative weights are valid and arise naturally from Merwe's scaling.

### 2.2 Unscented Prediction

**Algorithm:**

1. **Generate sigma points** from previous a posteriori state:
   $$\mathcal{X}_{k-1|k-1}^{(i)} = \text{sigmaPoints}(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{P}_{k-1|k-1})$$

2. **Propagate** each sigma point through dynamics:
   $$\mathcal{X}_{k|k-1}^{(i)} = f(\mathcal{X}_{k-1|k-1}^{(i)}, \mathbf{u}_k)$$

3. **Compute a priori mean**:
   $$\hat{\mathbf{x}}_{k|k-1} = \sum_{i=0}^{2n} W_m^{(i)} \mathcal{X}_{k|k-1}^{(i)}$$

4. **Compute a priori covariance**:
   $$\mathbf{P}_{k|k-1} = \sum_{i=0}^{2n} W_c^{(i)} [\mathcal{X}_{k|k-1}^{(i)} - \hat{\mathbf{x}}_{k|k-1}][\mathcal{X}_{k|k-1}^{(i)} - \hat{\mathbf{x}}_{k|k-1}]^T + \mathbf{Q}_k$$

**Implementation** ([`filters/unscented.py`](../state_estimation/filters/unscented.py#L180)):
```python
def predict(self, u=None, f=None, Q=None):
    sigmas = self.points.sigma_points(self.x, self.P)
    for i, s in enumerate(sigmas):
        self.sigmas_f[i] = f(s, u)
    self.x = np.dot(self.points.Wm, self.sigmas_f)
    self.P = self._unscented_transform_cov(self.sigmas_f, self.x, 
                                            self.points.Wc, Q, self._residual_x_fn)
```

### 2.3 Unscented Update

**Algorithm:**

1. **Propagate** predicted sigma points through measurement:
   $$\mathcal{Z}_{k|k-1}^{(i)} = h(\mathcal{X}_{k|k-1}^{(i)})$$

2. **Predicted measurement**:
   $$\hat{\mathbf{z}}_{k|k-1} = \sum_{i=0}^{2n} W_m^{(i)} \mathcal{Z}_{k|k-1}^{(i)}$$

3. **Innovation covariance**:
   $$\mathbf{S}_k = \sum_{i=0}^{2n} W_c^{(i)} [\mathcal{Z}_{k|k-1}^{(i)} - \hat{\mathbf{z}}_{k|k-1}][\mathcal{Z}_{k|k-1}^{(i)} - \hat{\mathbf{z}}_{k|k-1}]^T + \mathbf{R}_k$$

4. **Cross-covariance** state-measurement:
   $$\mathbf{P}_{xz} = \sum_{i=0}^{2n} W_c^{(i)} [\mathcal{X}_{k|k-1}^{(i)} - \hat{\mathbf{x}}_{k|k-1}][\mathcal{Z}_{k|k-1}^{(i)} - \hat{\mathbf{z}}_{k|k-1}]^T$$

5. **Kalman gain**:
   $$\mathbf{K}_k = \mathbf{P}_{xz} \mathbf{S}_k^{-1}$$

6. **State update**:
   $$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k (\mathbf{z}_k - \hat{\mathbf{z}}_{k|k-1})$$

7. **Covariance update**:
   $$\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{K}_k \mathbf{S}_k \mathbf{K}_k^T$$

**Implementation** ([`filters/unscented.py`](../state_estimation/filters/unscented.py#L220)):
```python
def update(self, z, h=None, R=None):
    for i, s in enumerate(self.sigmas_f):
        self.sigmas_h[i] = h(s)
    z_mean = np.dot(self.points.Wm, self.sigmas_h)
    
    S = self._unscented_transform_cov(self.sigmas_h, z_mean, 
                                       self.points.Wc, R, self._residual_z_fn)
    Pxz = self._cross_covariance(self.sigmas_f, self.x, 
                                   self.sigmas_h, z_mean, self.points.Wc)
    K = Pxz @ np.linalg.inv(S)
    y = self._residual_z_fn(z, z_mean)
    self.x = self.x + K @ y
    self.P = self.P - K @ S @ K.T
```

### 2.4 Custom Functions for Angles

#### Circular Mean Function

For states with angular components, arithmetic mean is inadequate. **Circular mean** is used:

$$
\bar{\psi} = \text{atan2}\left(\sum_{i=0}^{2n} W_m^{(i)} \sin\psi^{(i)}, \sum_{i=0}^{2n} W_m^{(i)} \cos\psi^{(i)}\right)
$$

**Implementation** ([`ukf_omnidirectional.py`](../examples/ukf_omnidirectional.py#L47)):
```python
from state_estimation.common.angles import circular_mean

def state_mean_fn(sigmas, Wm):
    x = np.zeros(6)
    x[0] = np.dot(Wm, sigmas[:, 0])  # x (linear)
    x[1] = np.dot(Wm, sigmas[:, 1])  # y (linear)
    x[2] = circular_mean(sigmas[:, 2], Wm)  # psi (circular)
    x[3] = np.dot(Wm, sigmas[:, 3])  # vx (linear)
    x[4] = np.dot(Wm, sigmas[:, 4])  # vy (linear)
    x[5] = np.dot(Wm, sigmas[:, 5])  # omega (linear)
    return x
```

The `circular_mean` function ([`common/angles.py`](../state_estimation/common/angles.py)) projects angles to unit circle, averages 2D vectors, and recovers the angle:

```python
def circular_mean(angles, weights):
    sin_sum = np.sum(weights * np.sin(angles))
    cos_sum = np.sum(weights * np.cos(angles))
    return np.arctan2(sin_sum, cos_sum)
```

**Justification:** Arithmetic mean of $\psi = [179°, -179°]$ is $0°$ (incorrect, should be $\pm 180°$). Circular mean correctly results in $180°$.

#### Angular Residual Function

For differences in covariance, angles must be normalized:

$$
\text{residual}(\psi_a, \psi_b) = \text{atan2}(\sin(\psi_a - \psi_b), \cos(\psi_a - \psi_b))
$$

**Implementation** ([`ukf_omnidirectional.py`](../examples/ukf_omnidirectional.py#L71)):
```python
from state_estimation.common.angles import normalize_angle

def residual_x(a, b):
    y = a - b
    y[2] = normalize_angle(y[2])  # psi
    return y

def residual_z(a, b):
    y = a - b
    y[3] = normalize_angle(y[3])  # psi in measurement
    return y
```

**UKF configuration** ([`ukf_omnidirectional.py`](../examples/ukf_omnidirectional.py#L131)):
```python
ukf.set_mean_fn(x_mean_fn=state_mean_fn, z_mean_fn=z_mean_fn)
ukf.set_residual_fn(residual_x_fn=residual_x, residual_z_fn=residual_z)
```

### 2.5 Covariance Matrices

**Process noise:**
$$
\mathbf{Q} = \text{diag}([1 \times 10^{-4}, 1 \times 10^{-4}, 1 \times 10^{-5}, 5 \times 10^{-3}, 5 \times 10^{-3}, 5 \times 10^{-4}]) \times 10
$$

**Measurement noise:**
$$
\mathbf{R} = \text{diag}([6.72 \times 10^{-4}, 6.72 \times 10^{-4}, 1.31 \times 10^{-2}, 1.22 \times 10^{-3}]) \times 10^3
$$

Scaled to improve numerical conditioning of the UKF.

---

## 3. Particle Filter (PF)

**Implementation file:** [`filters/particle.py`](../state_estimation/filters/particle.py)  
**Execution script:** [`pf_omnidirectional.py`](../examples/pf_omnidirectional.py)

### 3.1 Foundation: Sequential Importance Sampling

The Particle Filter represents the posterior distribution $p(\mathbf{x}_k | \mathbf{z}_{1:k}, \mathbf{u}_{1:k})$ using a set of **particles** (Monte Carlo samples):

$$
p(\mathbf{x}_k | \mathbf{z}_{1:k}) \approx \sum_{i=1}^{N} w_k^{(i)} \delta(\mathbf{x}_k - \mathbf{x}_k^{(i)})
$$

Where:
- $\mathbf{x}_k^{(i)}$: State of particle $i$
- $w_k^{(i)}$: Normalized weight ($\sum_i w_k^{(i)} = 1$)
- $N$: Number of particles (typically 1000-10000)

**Advantages:**
- Handles arbitrary nonlinearities and multimodal distributions
- Does not assume Gaussianity
- Converges to true distribution as $N \to \infty$

**Disadvantages:**
- Computational complexity $\mathcal{O}(N)$
- Particle degeneracy requires resampling

### 3.2 SIR Algorithm (Sequential Importance Resampling)

Implemented in [`filters/particle.py`](../state_estimation/filters/particle.py).

#### Step 1: Initialization

Sample particles from initial distribution:

$$
\mathbf{x}_0^{(i)} \sim \mathcal{N}(\mathbf{x}_0, \mathbf{P}_0), \quad w_0^{(i)} = \frac{1}{N}
$$

**Implementation** ([`pf_omnidirectional.py`](../examples/pf_omnidirectional.py#L70)):
```python
x0 = ground_truth[0]
P0 = np.diag([0.5, 0.5, 0.1, 0.2, 0.2, 0.05]) * 1e-5
pf.initialize_particles(x0=x0, P0=P0)
```

Generates 2000 particles with very low initial spread (high confidence in initial state).

#### Step 2: Prediction (Propagation)

Each particle propagates through dynamics with process noise:

$$
\mathbf{x}_k^{(i)} = f(\mathbf{x}_{k-1}^{(i)}, \mathbf{u}_k) + \mathbf{w}_k^{(i)}, \quad \mathbf{w}_k^{(i)} \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})
$$

**Implementation** ([`filters/particle.py`](../state_estimation/filters/particle.py#L106)):
```python
def predict(self, u=None, f=None, process_noise_std=None):
    for i in range(self.N):
        self.particles[i] = f(self.particles[i], u)
    
    # Add Gaussian noise
    if process_noise_std is not None:
        noise = np.random.normal(0, process_noise_std, size=self.particles.shape)
        self.particles += noise
    
    self.x = np.average(self.particles, weights=self.weights, axis=0)
```

**Experimental configuration** ([`pf_omnidirectional.py`](../examples/pf_omnidirectional.py#L76)):
```python
process_std = np.array([0.01, 0.01, 0.01, 0.05, 0.05, 0.05])
```

Larger noise for velocities than positions/orientation.

#### Step 3: Update (Reweighting)

Weights update based on measurement likelihood:

$$
w_k^{(i)} \propto w_{k-1}^{(i)} \cdot p(\mathbf{z}_k | \mathbf{x}_k^{(i)})
$$

For Gaussian measurements:

$$
p(\mathbf{z}_k | \mathbf{x}_k^{(i)}) = \frac{1}{(2\pi)^{m/2}|\mathbf{R}|^{1/2}} \exp\left(-\frac{1}{2}[\mathbf{z}_k - h(\mathbf{x}_k^{(i)})]^T \mathbf{R}^{-1} [\mathbf{z}_k - h(\mathbf{x}_k^{(i)})]\right)
$$

**Implementation** ([`filters/particle.py`](../state_estimation/filters/particle.py#L148)):
```python
def update(self, z, h=None, R=None):
    likelihood_fn = self._make_gaussian_likelihood(h, R)
    
    for i in range(self.N):
        self.weights[i] *= likelihood_fn(z, self.particles[i])
    
    # Normalize weights
    self.weights /= np.sum(self.weights)
    
    self.x = np.average(self.particles, weights=self.weights, axis=0)
    self.P = self._compute_covariance()
```

The `_make_gaussian_likelihood` function ([`filters/particle.py`](../state_estimation/filters/particle.py#L364)) implements multivariate Gaussian likelihood using `scipy.stats.multivariate_normal`.

#### Step 4: Systematic Resampling

**Degeneracy problem:** After several iterations, most particles have negligible weights ($w^{(i)} \approx 0$), concentrating probability mass on few particles.

**Metric:** Effective sample size (ESS):

$$
N_{\text{eff}} = \frac{1}{\sum_{i=1}^{N} (w^{(i)})^2}
$$

- $N_{\text{eff}} = N$: Uniform weights (ideal)
- $N_{\text{eff}} = 1$: One weight dominates (total degeneracy)

**Resampling criterion:** If $N_{\text{eff}} < N / 2$, apply resampling.

**Systematic resampling algorithm** ([`filters/particle.py`](../state_estimation/filters/particle.py#L234)):

1. Generate $N$ evenly-spaced positions with random offset:
   $$u_i = \frac{i + \mathcal{U}(0,1)}{N}, \quad i = 0, \ldots, N-1$$

2. Compute weight CDF: $c_j = \sum_{k=0}^{j} w^{(k)}$

3. For each $u_i$, find $j$ such that $c_{j-1} < u_i \leq c_j$ and duplicate particle $j$

**Advantages over multinomial:**
- Lower variance (stratified sampling)
- Complexity $\mathcal{O}(N)$ instead of $\mathcal{O}(N \log N)$

**Implementation:**
```python
def _systematic_resample(self):
    positions = (np.arange(self.N) + np.random.random()) / self.N
    indices = np.zeros(self.N, dtype=int)
    cumsum = np.cumsum(self.weights)
    
    i, j = 0, 0
    while i < self.N:
        if positions[i] < cumsum[j]:
            indices[i] = j
            i += 1
        else:
            j += 1
    return indices
```

**Usage in filter** ([`pf_omnidirectional.py`](../examples/pf_omnidirectional.py#L87)):
```python
for k in range(N - 1):
    pf.predict(u=controls[k], f=robot.dynamics, process_noise_std=process_std)
    pf.update(z=measurements[k + 1], h=robot.measurement, R=R)
    pf.resample(scheme='systematic')  # Resampling at each step
```

**Note:** Current implementation applies resampling every iteration. Alternatively, can be conditioned on ESS:

```python
if pf.effective_sample_size() < pf.resample_threshold:
    pf.resample(scheme='systematic')
```

### 3.3 State Estimation

**Weighted mean:**
$$
\hat{\mathbf{x}}_k = \sum_{i=1}^{N} w_k^{(i)} \mathbf{x}_k^{(i)}
$$

**Covariance:**
$$
\mathbf{P}_k = \sum_{i=1}^{N} w_k^{(i)} (\mathbf{x}_k^{(i)} - \hat{\mathbf{x}}_k)(\mathbf{x}_k^{(i)} - \hat{\mathbf{x}}_k)^T
$$

**Implementation** ([`filters/particle.py`](../state_estimation/filters/particle.py#L343)):
```python
def _compute_covariance(self):
    mean = self.x
    diff = self.particles - mean
    P = np.zeros((self.dim_x, self.dim_x))
    for i in range(self.N):
        d = diff[i].reshape(-1, 1)
        P += self.weights[i] * (d @ d.T)
    return P
```

### 3.4 Experimental Configuration

**Number of particles:** $N = 2000$ (balance between accuracy and computational cost)

**Noise matrices:**
```python
process_std = np.array([0.01, 0.01, 0.01, 0.05, 0.05, 0.05])
R = np.diag([6.72e-4, 6.72e-4, 1.3125e-2, 1.218e-3])
```

**Frequency:** 100 Hz (same as EKF/UKF)

**Monitoring metrics:**
```python
if (k + 1) % 100 == 0:
    ess = pf.effective_sample_size()
    print(f"  Step {k+1}/{N-1}, ESS: {ess:.1f}")
```

Typical ESS values: 800-1500 (out of 2000), indicating healthy weight distribution.

---

## 4. Methodological Comparison

### Comparison Table

| Feature | EKF | UKF | PF |
|---------|-----|-----|-----|
| **Linearization** | Jacobian (1st order) | Unscented Transform (2nd order) | Not required |
| **Distribution** | Gaussian | Gaussian | Arbitrary |
| **Complexity** | $\mathcal{O}(n^3)$ | $\mathcal{O}(n^3)$ | $\mathcal{O}(N \cdot n)$ |
| **Jacobian** | ✅ Required | ❌ Not required | ❌ Not required |
| **Accuracy** | Low (strong nonlinearity) | High (moderate nonlinearity) | Converges to optimal |
| **Numerical stability** | Sensitive | Robust | Very robust |
| **Multimodality** | ❌ | ❌ | ✅ |
| **Execution time** | ~1 ms/iter | ~2 ms/iter | ~20 ms/iter |

### Application Scenarios

**EKF:**
- Weakly nonlinear dynamics
- Limited computational resources
- Well-characterized analytical model

**UKF:**
- Moderate nonlinearities
- Difficult or costly Jacobian derivation
- Higher accuracy without dramatic cost increase

**PF:**
- Severe nonlinearities (e.g., localization with ambiguity)
- Non-Gaussian distributions (e.g., intermittent failures)
- Abundant computational resources

For omnidirectional robot with rigid-body dynamics, **EKF and UKF** are usually sufficient, with PF as a validation case for potentially multimodal orientation distributions.

---

## 5. Experimental Data Sources

### 5.1 File Structure

```
data/
├── sensors/              # Raw IMU + encoder data
│   └── expN.txt         # N = 1-10
└── processed/
    └── trajectories/    # Fused trajectories (video + IMU)
        └── traj_vid_N.csv
```

### 5.2 Sensor Data Format

**File:** `data/sensors/expN.txt`

```
t,ax,ay,alpha,w1,w2,w3,u1,u2,u3,vbx_sp,vby_sp,wb_sp
```

- `t`: Time [s]
- `ax, ay`: IMU acceleration in body frame [m/s²]
- `alpha`: IMU orientation [rad]
- `w1, w2, w3`: Wheel angular velocities [rad/s]
- `u1, u2, u3`: PWM signals to motors [%]
- `*_sp`: Setpoints (not used in estimation)

### 5.3 Ground Truth Trajectory Format

**File:** `data/processed/trajectories/traj_vid_N.csv`

```
time_s,x_m,y_m,phi_rad,vx_m_s,vy_m_s,omega_rad_s,u1_pwm,u2_pwm,u3_pwm
```

- Obtained via optical flow + color detection (see [Robot_Identification documentation](../../../Robot_Identification/docs/))
- Frequency: 100 Hz (resampled)
- Used as reference for error metrics

### 5.4 Data Loading

Implemented in [`trajectory_generators.py`](../examples/trajectory_generators.py#L150):

```python
def load_experimental_data(sensors_path, trajectory_path, N=1000, dt=0.01):
    # Load sensors
    sensors_df = pd.read_csv(sensors_path)
    ax = sensors_df['ax'].values[:N]
    ay = sensors_df['ay'].values[:N]
    
    # Load ground truth
    traj_df = pd.read_csv(trajectory_path)
    x = traj_df['x_m'].values[:N]
    y = traj_df['y_m'].values[:N]
    phi = traj_df['phi_rad'].values[:N]
    
    # Build measurements: [vx_b, vy_b, omega, psi]
    measurements = compute_body_velocities(vx, vy, phi, omega)
    
    # Controls: [ax_b, ay_b]
    controls = np.column_stack([ax, ay])
    
    return {'time': t, 'controls': controls, 'measurements': measurements,
            'ground_truth': ground_truth, 'dt': dt}
```

---

## 6. Performance Metrics

### 6.1 Estimation Errors

**Root Mean Square Error (RMSE):**
$$
\text{RMSE}_i = \sqrt{\frac{1}{N} \sum_{k=1}^{N} (x_k^{(i)} - \hat{x}_k^{(i)})^2}
$$

For each state component $i \in \{x, y, \psi, v_x, v_y, \omega\}$.

**Mean Absolute Error (MAE):**
$$
\text{MAE}_i = \frac{1}{N} \sum_{k=1}^{N} |x_k^{(i)} - \hat{x}_k^{(i)}|
$$

### 6.2 Filter Consistency

**Normalized Estimation Error Squared (NEES):**
$$
\epsilon_k = (\mathbf{x}_k - \hat{\mathbf{x}}_k)^T \mathbf{P}_k^{-1} (\mathbf{x}_k - \hat{\mathbf{x}}_k)
$$

For a consistent filter, $\epsilon_k \sim \chi^2(n)$ with $n$ degrees of freedom. Expected value: $\mathbb{E}[\epsilon_k] = n = 6$.

**Normalized Innovation Squared (NIS):**
$$
\nu_k = \mathbf{y}_k^T \mathbf{S}_k^{-1} \mathbf{y}_k
$$

Where $\mathbf{y}_k$ is the innovation. For consistent filter, $\nu_k \sim \chi^2(m)$ with $m = 4$ (measurement dimension).

**Implementation:** [`metrics.py`](../state_estimation/metrics.py)

### 6.3 Typical Results

**EKF:**
- Position RMSE: 0.03-0.05 m
- Orientation RMSE: 0.05-0.10 rad
- Average NEES: 5-8 (slightly optimistic)

**UKF:**
- Position RMSE: 0.025-0.04 m
- Orientation RMSE: 0.04-0.08 rad
- Average NEES: 6-9 (more consistent)

**PF (N=2000):**
- Position RMSE: 0.02-0.035 m
- Orientation RMSE: 0.03-0.06 rad
- Average ESS: 900-1400

---

## 7. Implementation Considerations

### 7.1 Numerical Stability

**Problem:** Covariance matrices can lose symmetry or positive-definiteness due to floating-point errors.

**Implemented solutions:**

1. **Joseph form for EKF** ([`extended.py`](../state_estimation/filters/extended.py#L195)):
   $$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T$$
   
   Guarantees symmetry and improves numerical conditioning.

2. **Cholesky decomposition with error handling** ([`unscented.py`](../state_estimation/filters/unscented.py#L76)):
   ```python
   try:
       U = cholesky((self.n + self._lambda) * P)
   except np.linalg.LinAlgError:
       # If fails, force symmetry and add regularization
       P = (P + P.T) / 2
       P += np.eye(self.n) * 1e-9
       U = cholesky((self.n + self._lambda) * P)
   ```

3. **Weight normalization in PF** ([`particle.py`](../state_estimation/filters/particle.py#L183)):
   ```python
   weight_sum = np.sum(self.weights)
   if weight_sum < 1e-10:  # Extreme degeneracy
       self.weights = np.ones(self.N) / self.N  # Reset uniformly
   else:
       self.weights /= weight_sum
   ```

### 7.2 Computational Efficiency

**Operation vectorization:**

- EKF/UKF: Matrix operations with NumPy/SciPy (optimized BLAS)
- PF: Particle loops unavoidable, but internal operations vectorized

**Typical profiling (Intel i7, 2.6 GHz):**
- EKF: ~0.8 ms/iteration
- UKF: ~1.5 ms/iteration
- PF (N=2000): ~18 ms/iteration

**Potential parallelization:** PF is embarrassingly parallel (each particle processes independently). Can use `multiprocessing` or Numba JIT for acceleration.

### 7.3 Parameter Tuning

**Q and R calibration process:**

1. **Sensor characterization:**
   - Place robot stationary, measure $\sigma_{\text{sensor}}$ for 1 minute
   - R diagonals: empirical variances

2. **Q optimization:**
   - Start with heuristic values: $\mathbf{Q} = 0.01 \cdot \mathbb{E}[\mathbf{x}\mathbf{x}^T]$
   - Run filter, compute NEES
   - Iteratively adjust seeking NEES $\approx n$
   - Tools: Grid search or CMA-ES

3. **Cross-validation:**
   - Calibrate on experiments 1-5
   - Validate on experiments 6-10
   - Report metrics for both sets

---

## 8. Methodological References

### Fundamental Books

1. **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics*. MIT Press.  
   Chapters 3 (Recursive State Estimation), 7 (Kalman Filters), 8 (EKF), 9 (UKF), 4 (Particle Filters).

2. **Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001).** *Estimation with Applications to Tracking and Navigation*. Wiley.  
   Sections on EKF Jacobian and NEES/NIS validation.

3. **Särkkä, S. (2013).** *Bayesian Filtering and Smoothing*. Cambridge University Press.  
   Rigorous derivations of UKF and Unscented Transform.

### Specific Articles

4. **Julier, S. J., & Uhlmann, J. K. (1997).** "New Extension of the Kalman Filter to Nonlinear Systems". *Signal Processing, Sensor Fusion, and Target Recognition VI*, SPIE.  
   Original UKF introduction.

5. **Van Der Merwe, R., & Wan, E. A. (2001).** "The Square-Root Unscented Kalman Filter for State and Parameter-Estimation". *IEEE International Conference on Acoustics, Speech, and Signal Processing*.  
   Scaled sigma points algorithm (used in this implementation).

6. **Arulampalam, M. S., et al. (2002).** "A Tutorial on Particle Filters for Online Nonlinear/Non-Gaussian Bayesian Tracking". *IEEE Transactions on Signal Processing*, 50(2), 174-188.  
   Comprehensive PF tutorial with resampling method analysis.

### Software and Libraries

7. **FilterPy (Roger R. Labbe Jr.):** Python reference library for Bayesian filters.  
   Available at: https://github.com/rlabbe/filterpy  
   (Inspiration for this implementation's API)

---

## Appendix A: Nomenclature and Conventions

| Symbol | Description | Dimension |
|---------|-------------|-----------|
| $\mathbf{x}_k$ | State vector at time $k$ | $6 \times 1$ |
| $\mathbf{u}_k$ | Control vector | $2 \times 1$ |
| $\mathbf{z}_k$ | Measurement vector | $4 \times 1$ |
| $\mathbf{P}_k$ | State covariance | $6 \times 6$ |
| $\mathbf{Q}$ | Process noise covariance | $6 \times 6$ |
| $\mathbf{R}$ | Measurement noise covariance | $4 \times 4$ |
| $\mathbf{F}_k$ | Dynamics Jacobian | $6 \times 6$ |
| $\mathbf{H}_k$ | Measurement Jacobian | $4 \times 6$ |
| $\mathbf{K}_k$ | Kalman gain | $6 \times 4$ |
| $n$ | State dimension | 6 |
| $m$ | Measurement dimension | 4 |
| $N$ | Number of particles (PF) | 1000-10000 |
| $\Delta t$ | Sampling period | 0.01 s |

---

## Appendix B: Complete Pseudocode

### EKF
```
INITIALIZE: x₀, P₀, Q, R

FOR k = 1 TO N:
    # Predict
    x̄ₖ = f(xₖ₋₁, uₖ)
    Fₖ = ∂f/∂x|(xₖ₋₁, uₖ)
    P̄ₖ = Fₖ Pₖ₋₁ Fₖᵀ + Q
    
    # Update
    yₖ = zₖ - h(x̄ₖ)
    Hₖ = ∂h/∂x|x̄ₖ
    Sₖ = Hₖ P̄ₖ Hₖᵀ + R
    Kₖ = P̄ₖ Hₖᵀ Sₖ⁻¹
    xₖ = x̄ₖ + Kₖ yₖ
    Pₖ = (I - Kₖ Hₖ) P̄ₖ (I - Kₖ Hₖ)ᵀ + Kₖ R Kₖᵀ
END FOR
```

### UKF
```
INITIALIZE: x₀, P₀, Q, R, α, β, κ

FOR k = 1 TO N:
    # Generate sigma points
    𝒳ₖ₋₁ = sigmaPoints(xₖ₋₁, Pₖ₋₁)
    
    # Predict
    FOR i = 0 TO 2n:
        𝒳ₖ|ₖ₋₁⁽ⁱ⁾ = f(𝒳ₖ₋₁⁽ⁱ⁾, uₖ)
    END FOR
    x̄ₖ = Σᵢ Wₘ⁽ⁱ⁾ 𝒳ₖ|ₖ₋₁⁽ⁱ⁾
    P̄ₖ = Σᵢ Wc⁽ⁱ⁾ (𝒳ₖ|ₖ₋₁⁽ⁱ⁾ - x̄ₖ)(𝒳ₖ|ₖ₋₁⁽ⁱ⁾ - x̄ₖ)ᵀ + Q
    
    # Update
    FOR i = 0 TO 2n:
        𝒵ₖ|ₖ₋₁⁽ⁱ⁾ = h(𝒳ₖ|ₖ₋₁⁽ⁱ⁾)
    END FOR
    z̄ₖ = Σᵢ Wₘ⁽ⁱ⁾ 𝒵ₖ|ₖ₋₁⁽ⁱ⁾
    Sₖ = Σᵢ Wc⁽ⁱ⁾ (𝒵ₖ|ₖ₋₁⁽ⁱ⁾ - z̄ₖ)(𝒵ₖ|ₖ₋₁⁽ⁱ⁾ - z̄ₖ)ᵀ + R
    Pₓᵧ = Σᵢ Wc⁽ⁱ⁾ (𝒳ₖ|ₖ₋₁⁽ⁱ⁾ - x̄ₖ)(𝒵ₖ|ₖ₋₁⁽ⁱ⁾ - z̄ₖ)ᵀ
    Kₖ = Pₓᵧ Sₖ⁻¹
    xₖ = x̄ₖ + Kₖ (zₖ - z̄ₖ)
    Pₖ = P̄ₖ - Kₖ Sₖ Kₖᵀ
END FOR
```

### PF
```
INITIALIZE: {xₖ⁽ⁱ⁾, wₖ⁽ⁱ⁾}ᵢ₌₁ᴺ ~ N(x₀, P₀), wₖ⁽ⁱ⁾ = 1/N

FOR k = 1 TO N:
    # Predict
    FOR i = 1 TO N:
        xₖ⁽ⁱ⁾ = f(xₖ₋₁⁽ⁱ⁾, uₖ) + 𝒩(0, Q)
    END FOR
    
    # Update
    FOR i = 1 TO N:
        wₖ⁽ⁱ⁾ = wₖ₋₁⁽ⁱ⁾ · p(zₖ | xₖ⁽ⁱ⁾)
    END FOR
    wₖ = wₖ / Σᵢ wₖ⁽ⁱ⁾
    
    # Resample
    IF ESS < N/2:
        {xₖ⁽ⁱ⁾}ᵢ₌₁ᴺ = systematicResample({xₖ⁽ⁱ⁾, wₖ⁽ⁱ⁾})
        wₖ⁽ⁱ⁾ = 1/N ∀i
    END IF
    
    # Estimate
    xₖ = Σᵢ wₖ⁽ⁱ⁾ xₖ⁽ⁱ⁾
END FOR
```

---

## Appendix C: Minimal Example Code

**Complete EKF filter in ~20 lines:**

```python
from state_estimation import ExtendedKalmanFilter
from state_estimation.models import OmnidirectionalRobot
from state_estimation.common import make_residual_fn

# Initialize
robot = OmnidirectionalRobot(dt=0.01)
ekf = ExtendedKalmanFilter(dim_x=6, dim_z=4, dim_u=2)
ekf.x = np.array([0, 0, 0, 0, 0, 0])
ekf.P = np.diag([0.5, 0.5, 0.1, 0.2, 0.2, 0.05])
ekf.Q = np.diag([1e-4, 1e-4, 1e-5, 5e-3, 5e-3, 5e-4])
ekf.R = np.diag([6.72e-4, 6.72e-4, 1.3125e-2, 1.218e-3])
ekf.set_residual_fn(make_residual_fn(angle_indices=[2]))

# Main loop
for k in range(N - 1):
    ekf.predict(u=controls[k], f=robot.dynamics, F=robot.jacobian_F)
    ekf.update(z=measurements[k + 1], h=robot.measurement, H=robot.jacobian_H)
    estimates[k + 1] = ekf.x
```

---

**Author:** Maverick Sossa Tobón  
**Date:** January 2026  
**Version:** 1.0  
**Institution:** Universidad de Antioquia
