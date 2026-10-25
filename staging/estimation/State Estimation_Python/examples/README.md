# State Estimation Examples

Quick reference for running Bayesian filter examples with the omnidirectional robot.

## 📖 Complete Theory

For mathematical derivations, algorithms, and detailed explanations:
- **[Bayesian Filters Theory](../docs/BAYESIAN_FILTERS.md)** - EKF, UKF, PF complete methodology
- **[Q Matrix Computation](../docs/Q_MATRIX_COMPUTATION.md)** - Process noise determination

---

## 🚀 Quick Start

Run examples using the unified execution script:

```powershell
# From project root
.\run.ps1 ekf        # Extended Kalman Filter
.\run.ps1 ukf        # Unscented Kalman Filter
.\run.ps1 pf         # Particle Filter
.\run.ps1 compare    # Compare all filters
```

---

## 📋 Available Scripts

### Main Filters (Executable with `run.ps1`)

| Script | Filter | Purpose | Command |
|--------|--------|---------|---------|
| **`ekf_omnidirectional.py`** | EKF | Extended Kalman Filter with Jacobians | `.\run.ps1 ekf` |
| **`ukf_omnidirectional.py`** | UKF | Unscented Kalman Filter (sigma points) | `.\run.ps1 ukf` |
| **`pf_omnidirectional.py`** | PF | Particle Filter (N=2000, SIR algorithm) | `.\run.ps1 pf` |
| **`compare_filters.py`** | All | Side-by-side comparison of EKF/UKF/PF | `.\run.ps1 compare` |

### Utility Scripts

| Script | Purpose | Command |
|--------|---------|---------|
| **`example_Q_from_identification.py`** | Compute Q matrix from robot model | `.\run.ps1 compute_q` |
| `custom_model.py` | Example with differential drive robot | (manual execution) |
| `example_gp_correction.py` | GP-based model correction (experimental) | (manual execution) |
| `generate_embedded_data.py` | Export data for ESP32 deployment | (manual execution) |
| `trajectory_generators.py` | Trajectory generation utilities | (library module) |

---

## ⚙️ Configuration

Edit constants at the beginning of each script:

```python
# Data source
USE_EXPERIMENTAL_DATA = True      # True: real robot data, False: synthetic
EXPERIMENT_NUMBER = 2             # Experiment 1-10

# Simulation parameters
N_POINTS = 1000                   # Number of time steps
DT = 0.01                         # Sampling period (100 Hz)
```

### Data Files

**Experimental data:**
- **Sensors:** `data/sensors/expN.txt` (IMU + encoders)
- **Ground truth:** `data/processed/trajectories/traj_vid_N.csv` (video + IMU fusion)

**Format:**
- Frequency: 100 Hz
- Duration: 10 seconds (1000 samples)
- State: [x, y, φ, vx, vy, ω]
- Controls: [ax_body, ay_body]
- Measurements: [vx_body, vy_body, ω, φ]

---

## 📊 Output

Results saved to: `results/estimation/<filter_name>/`

**Generated files:**
- `<filter>_trayectoria_2d.png` - 2D trajectory with orientation arrows
- `<filter>_estados_temporales.png` - Time-series plots of all states
- `<filter>_metricas.csv` - Performance metrics (RMSE, MAE, NEES)

**Metrics:**
- **RMSE:** Root mean square error
- **MAE:** Mean absolute error
- **NEES:** Normalized estimation error squared (filter consistency)
- **NIS:** Normalized innovation squared

---

## 💡 Filter-Specific Notes

### Extended Kalman Filter (EKF)
- **Requires:** Jacobians F and H (provided by robot model)
- **Angle handling:** Automatic residual wrapping for ψ
- **Best for:** Weakly nonlinear systems, fast execution

### Unscented Kalman Filter (UKF)
- **Sigma points:** 13 points (2n+1 for n=6)
- **Parameters:** α=0.5, β=2.0, κ=0.0
- **Custom functions:** Circular mean and residual for angles
- **Best for:** Moderate nonlinearities, no Jacobian needed

### Particle Filter (PF)
- **Particles:** N=2000
- **Resampling:** Systematic (every iteration)
- **ESS monitoring:** Effective sample size tracking
- **Best for:** Severe nonlinearities, non-Gaussian noise

---

## 📚 Additional Documentation

### Python API
See [State Estimation_Python/README.md](../README.md) for API reference.

### C Implementation
See [State Estimation_C/README.md](../../State%20Estimation_C/README.md) for embedded deployment.

### Robot Model
See [Robot_Identification/README.md](../../Robot_Identification/README.md) for parameter identification.

---

## ✅ Example Execution

```powershell
# 1. Run UKF on experimental data from experiment 2
.\run.ps1 ukf

# Output:
# ========================================================
#   Unscented Kalman Filter
# ========================================================
# Description: UKF for omnidirectional robot state estimation
#
# Checking virtual environment...
# ✓ Virtual environment found: E:\Project\.venv
# ✓ Virtual environment already active
#
# Working Directory: State Estimation_Python
# Script:            examples\ukf_omnidirectional.py
# ========================================================
#   Executing...
# ========================================================
# [UKF] Running Unscented Kalman Filter...
# [UKF] Experiment: 2
# [UKF] Time steps: 1000 (10.00 s @ 100 Hz)
# ...
# [Results] RMSE_x: 0.0284 m
# [Results] RMSE_phi: 0.0512 rad
# [Results] NEES: 7.23 (valid range: [1.24, 14.45])
# ✓ Results saved to: results/estimation/ukf/
# ========================================================
#   ✓ EXECUTION COMPLETED SUCCESSFULLY
# ========================================================
```

---

**For complete mathematical theory, derivations, and implementation details:**
→ **[See docs/BAYESIAN_FILTERS.md](../docs/BAYESIAN_FILTERS.md)**

---

## Marco Teórico Común

### Modelo de Estado

**Vector de estado** (marco global):
$$
\mathbf{x} = \begin{bmatrix} x \\ y \\ \psi \\ v_x \\ v_y \\ \omega \end{bmatrix} \in \mathbb{R}^6
$$

Donde:
- $(x, y)$: Posición en marco global [m]
- $\psi$: Orientación (ángulo de rumbo) [rad]
- $(v_x, v_y)$: Velocidad en marco global [m/s]
- $\omega$: Velocidad angular [rad/s]

**Vector de control** (aceleraciones en marco del cuerpo):
$$
\mathbf{u} = \begin{bmatrix} a_{x,b} \\ a_{y,b} \end{bmatrix} \in \mathbb{R}^2
$$

**Vector de medición**:
$$
\mathbf{z} = \begin{bmatrix} v_{x,b} \\ v_{y,b} \\ \omega \\ \psi \end{bmatrix} \in \mathbb{R}^4
$$

Donde $(v_{x,b}, v_{y,b})$ son velocidades en marco del cuerpo derivadas de encoders mediante cinemática inversa.

### Dinámicas del Sistema

El modelo implementado en [`omnidirectional.py`](../state_estimation/models/omnidirectional.py#L90) realiza la transformación de aceleraciones del marco del cuerpo al marco global:

$$
\begin{aligned}
\mathbf{R}(\psi) &= \begin{bmatrix} \cos\psi & -\sin\psi \\ \sin\psi & \cos\psi \end{bmatrix} \\[0.5em]
\begin{bmatrix} a_x \\ a_y \end{bmatrix}_{\text{global}} &= \mathbf{R}(\psi) \begin{bmatrix} a_{x,b} \\ a_{y,b} \end{bmatrix}
\end{aligned}
$$

**Ecuaciones discretas de estado** (Euler con $\Delta t = 0.01$ s):

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

Donde $a_x = \cos\psi \cdot a_{x,b} - \sin\psi \cdot a_{y,b}$ y $a_y = \sin\psi \cdot a_{x,b} + \cos\psi \cdot a_{y,b}$.

**Nota:** La velocidad angular $\omega$ sigue un modelo de paseo aleatorio (proceso de Wiener), asumiendo cambios suaves entre pasos temporales.

### Modelo de Medición

Transformación de velocidades globales a marco del cuerpo:

$$
h(\mathbf{x}) = \begin{bmatrix}
\cos\psi \cdot v_x + \sin\psi \cdot v_y \\
-\sin\psi \cdot v_x + \cos\psi \cdot v_y \\
\omega \\
\psi
\end{bmatrix}
$$

Implementado en [`omnidirectional.py`](../state_estimation/models/omnidirectional.py#L152).

---

## 1. Extended Kalman Filter (EKF)

**Archivo de implementación:** [`filters/extended.py`](../state_estimation/filters/extended.py)  
**Script de ejecución:** [`ekf_omnidirectional.py`](ekf_omnidirectional.py)

### 1.1 Fundamento Matemático

El EKF linealiza localmente el sistema no lineal mediante expansiones de Taylor de primer orden alrededor del estado estimado, utilizando las matrices jacobianas $\mathbf{F}$ y $\mathbf{H}$.

#### Paso de Predicción

**Estado a priori:**
$$
\hat{\mathbf{x}}_{k|k-1} = f(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{u}_k)
$$

**Covarianza a priori:**
$$
\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^T + \mathbf{Q}_k
$$

#### Paso de Actualización

**Residual (innovación):**
$$
\mathbf{y}_k = \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1})
$$

**Covarianza de innovación:**
$$
\mathbf{S}_k = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^T + \mathbf{R}_k
$$

**Ganancia de Kalman:**
$$
\mathbf{K}_k = \mathbf{P}_{k|k-1} \mathbf{H}_k^T \mathbf{S}_k^{-1}
$$

**Estado a posteriori:**
$$
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k \mathbf{y}_k
$$

**Covarianza a posteriori** (forma de Joseph para estabilidad numérica):
$$
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_{k|k-1} (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T
$$

### 1.2 Cálculo de Jacobianos

#### Jacobiano de Dinámica: $\mathbf{F} = \frac{\partial f}{\partial \mathbf{x}}$

Implementado en [`omnidirectional.py`](../state_estimation/models/omnidirectional.py#L193):

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

Donde los términos de acoplamiento rotacional son:

$$
\begin{aligned}
\frac{\partial a_x}{\partial \psi} &= -\sin\psi \cdot a_{x,b} - \cos\psi \cdot a_{y,b} \\
\frac{\partial a_y}{\partial \psi} &= \cos\psi \cdot a_{x,b} - \sin\psi \cdot a_{y,b}
\end{aligned}
$$

**Interpretación física:** Estos términos capturan cómo cambios en la orientación afectan la aceleración en marco global, representando el acoplamiento cinemático entre rotación y traslación.

#### Jacobiano de Medición: $\mathbf{H} = \frac{\partial h}{\partial \mathbf{x}}$

Implementado en [`omnidirectional.py`](../state_estimation/models/omnidirectional.py#L216):

$$
\mathbf{H}_k = \begin{bmatrix}
0 & 0 & -\sin\psi \cdot v_x + \cos\psi \cdot v_y & \cos\psi & \sin\psi & 0 \\
0 & 0 & -\cos\psi \cdot v_x - \sin\psi \cdot v_y & -\sin\psi & \cos\psi & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 1 & 0 & 0 & 0
\end{bmatrix}
$$

**Interpretación física:** La primera y segunda fila representan la sensibilidad de las velocidades en marco del cuerpo respecto a velocidades globales y orientación. Los términos $\frac{\partial v_{x,b}}{\partial \psi}$ capturan la variación de la proyección de velocidad al rotar el marco de referencia.

### 1.3 Manejo de Ángulos

**Problema:** El ángulo $\psi \in [-\pi, \pi]$ presenta discontinuidades (e.g., $\pi$ y $-\pi$ representan la misma orientación, pero su diferencia aritmética es $2\pi$).

**Solución implementada:**

En [`ekf_omnidirectional.py`](ekf_omnidirectional.py#L111):
```python
from state_estimation.common import make_residual_fn

ekf.set_residual_fn(make_residual_fn(angle_indices=[2]))  # psi en índice 2
```

La función `make_residual_fn` ([`common/angles.py`](../state_estimation/common/angles.py)) envuelve la diferencia angular al intervalo $(-\pi, \pi]$:

$$
\text{residual}_{\psi}(a, b) = \text{atan2}(\sin(a - b), \cos(a - b))
$$

Esto garantiza que innovaciones angulares sean consistentes (e.g., $\psi_{\text{meas}} = -179°$ y $\psi_{\text{pred}} = 179°$ resultan en residual de $2°$, no $358°$).

### 1.4 Matrices de Covarianza

**Ruido de proceso** $\mathbf{Q}$ (calibrado experimentalmente):
$$
\mathbf{Q} = \text{diag}([1.23 \times 10^{-8}, 1.24 \times 10^{-8}, 1 \times 10^{-12}, 4.91 \times 10^{-4}, 4.97 \times 10^{-4}, 1 \times 10^{-12}])
$$

- Valores pequeños en posición ($\sim 10^{-8}$): Alta confianza en modelo de integración
- Valores mayores en velocidad ($\sim 10^{-4}$): Mayor incertidumbre en aceleraciones del IMU
- Valores mínimos en $\psi$ y $\omega$: Confianza en modelo de orientación

**Ruido de medición** $\mathbf{R}$:
$$
\mathbf{R} = \text{diag}([6.72 \times 10^{-4}, 6.72 \times 10^{-4}, 1.31 \times 10^{-2}, 4.06 \times 10^{-6}])
$$

Correspondiente a $[v_{x,b}, v_{y,b}, \omega, \psi]$. Los valores se derivan de caracterización experimental de encoders e IMU.

### 1.5 Configuración Experimental

**Inicialización** ([`ekf_omnidirectional.py`](ekf_omnidirectional.py#L95)):
```python
ekf.x = ground_truth[0]  # Estado inicial (sin error en experimentos controlados)
ekf.P = np.diag([0.5, 0.5, 0.1, 0.2, 0.2, 0.05])  # Covarianza inicial
```

**Frecuencia de actualización:** 100 Hz ($\Delta t = 0.01$ s)

**Flujo de ejecución:**
```python
for k in range(N - 1):
    ekf.predict(u=controls[k], f=robot.dynamics, F=robot.jacobian_F)
    ekf.update(z=measurements[k + 1], h=robot.measurement, H=robot.jacobian_H)
    estimates[k + 1] = ekf.x
```

---

## 2. Unscented Kalman Filter (UKF)

**Archivo de implementación:** [`filters/unscented.py`](../state_estimation/filters/unscented.py)  
**Script de ejecución:** [`ukf_omnidirectional.py`](ukf_omnidirectional.py)

### 2.1 Fundamento Matemático: Transformada Unscented

El UKF evita la linealización jacobiana del EKF mediante la **Transformada Unscented (UT)**, que propaga una distribución Gaussiana a través de transformaciones no lineales usando un conjunto determinístico de puntos sigma.

**Principio:** Es más fácil aproximar una distribución Gaussiana que aproximar funciones no lineales arbitrarias. La UT logra precisión de segundo orden (términos de Taylor hasta $\mathcal{O}(\Delta x^2)$) para cualquier no linealidad, mientras que el EKF solo es de primer orden.

#### Generación de Sigma Points

Para un estado $\mathbf{x} \sim \mathcal{N}(\bar{\mathbf{x}}, \mathbf{P})$ de dimensión $n$, se generan $2n+1$ sigma points:

$$
\mathcal{X}^{(i)} = \begin{cases}
\bar{\mathbf{x}} & i = 0 \\
\bar{\mathbf{x}} + \left(\sqrt{(n + \lambda)\mathbf{P}}\right)_i & i = 1, \ldots, n \\
\bar{\mathbf{x}} - \left(\sqrt{(n + \lambda)\mathbf{P}}\right)_{i-n} & i = n+1, \ldots, 2n
\end{cases}
$$

Donde $\sqrt{\mathbf{P}}$ es la descomposición de Cholesky: $\mathbf{L}\mathbf{L}^T = \mathbf{P}$.

**Implementación** ([`filters/unscented.py`](../state_estimation/filters/unscented.py#L54)):
```python
def sigma_points(self, x, P):
    U = cholesky((self.n + self._lambda) * P)  # scipy usa L^T
    sigmas = np.zeros((2*self.n + 1, self.n))
    sigmas[0] = x
    for k in range(self.n):
        sigmas[k + 1] = x + U[k]
        sigmas[n + k + 1] = x - U[k]
    return sigmas
```

**Nota:** `scipy.linalg.cholesky` retorna la matriz triangular superior $U$ donde $U^T U = A$. Se accede a las columnas como filas para construir las direcciones de los sigma points.

#### Parámetros de Escalamiento: $\alpha$, $\beta$, $\kappa$

**Parámetro de escala combinado:**
$$
\lambda = \alpha^2 (n + \kappa) - n
$$

**$\alpha \in (0, 1]$:** Controla la dispersión de sigma points alrededor de la media.
- Valores pequeños ($\alpha \approx 0.001$): Sigma points muy cercanos a $\bar{\mathbf{x}}$ → menor exploración, menor influencia de no linealidades
- Valores grandes ($\alpha \approx 1$): Mayor dispersión → captura mejor curvaturas de funciones no lineales
- **Valor usado:** $\alpha = 0.5$ (balance entre estabilidad y precisión)

**$\beta \geq 0$:** Incorpora conocimiento a priori de la distribución.
- $\beta = 2$: Óptimo para distribuciones Gaussianas (minimiza error de cuarto orden en curtosis)
- **Valor usado:** $\beta = 2.0$ (asumiendo Gaussianidad)

**$\kappa \in \mathbb{R}$:** Parámetro de ajuste secundario.
- $\kappa = 3 - n$: Garantiza matriz semidefinida positiva
- $\kappa = 0$: Simplificación común
- **Valor usado:** $\kappa = 0.0$

**Configuración experimental** ([`ukf_omnidirectional.py`](ukf_omnidirectional.py#L107)):
```python
points = MerweScaledSigmaPoints(n=6, alpha=0.5, beta=2.0, kappa=0.0)
```

Con $n=6$, resulta en $\lambda = 0.5^2 \cdot 6 - 6 = -4.5$.

#### Pesos para Media y Covarianza

**Peso para la media:**
$$
W_m^{(i)} = \begin{cases}
\frac{\lambda}{n + \lambda} & i = 0 \\
\frac{1}{2(n + \lambda)} & i = 1, \ldots, 2n
\end{cases}
$$

**Peso para la covarianza:**
$$
W_c^{(0)} = \frac{\lambda}{n + \lambda} + (1 - \alpha^2 + \beta), \quad W_c^{(i)} = W_m^{(i)} \text{ para } i > 0
$$

El término $(1 - \alpha^2 + \beta)$ en $W_c^{(0)}$ corrige efectos de orden superior en la covarianza.

**Implementación** ([`filters/unscented.py`](../state_estimation/filters/unscented.py#L45)):
```python
self.Wm = np.full(2*n + 1, 0.5 / (n + self._lambda))
self.Wc = np.copy(self.Wm)
self.Wm[0] = self._lambda / (n + self._lambda)
self.Wc[0] = self._lambda / (n + self._lambda) + (1 - alpha**2 + beta)
```

Con los valores usados:
- $W_m^{(0)} = -4.5 / 1.5 = -3.0$
- $W_m^{(i)} = 0.5 / 1.5 = 0.333$ para $i > 0$
- $W_c^{(0)} = -3.0 + (1 - 0.25 + 2) = -0.25$

**Nota:** Pesos negativos son válidos y surgen naturalmente del escalamiento de Merwe.

### 2.2 Predicción Unscented

**Algoritmo:**

1. **Generar sigma points** del estado a posteriori anterior:
   $$\mathcal{X}_{k-1|k-1}^{(i)} = \text{sigmaPoints}(\hat{\mathbf{x}}_{k-1|k-1}, \mathbf{P}_{k-1|k-1})$$

2. **Propagar** cada sigma point a través de la dinámica:
   $$\mathcal{X}_{k|k-1}^{(i)} = f(\mathcal{X}_{k-1|k-1}^{(i)}, \mathbf{u}_k)$$

3. **Calcular media a priori**:
   $$\hat{\mathbf{x}}_{k|k-1} = \sum_{i=0}^{2n} W_m^{(i)} \mathcal{X}_{k|k-1}^{(i)}$$

4. **Calcular covarianza a priori**:
   $$\mathbf{P}_{k|k-1} = \sum_{i=0}^{2n} W_c^{(i)} [\mathcal{X}_{k|k-1}^{(i)} - \hat{\mathbf{x}}_{k|k-1}][\mathcal{X}_{k|k-1}^{(i)} - \hat{\mathbf{x}}_{k|k-1}]^T + \mathbf{Q}_k$$

**Implementación** ([`filters/unscented.py`](../state_estimation/filters/unscented.py#L180)):
```python
def predict(self, u=None, f=None, Q=None):
    sigmas = self.points.sigma_points(self.x, self.P)
    for i, s in enumerate(sigmas):
        self.sigmas_f[i] = f(s, u)
    self.x = np.dot(self.points.Wm, self.sigmas_f)
    self.P = self._unscented_transform_cov(self.sigmas_f, self.x, 
                                            self.points.Wc, Q, self._residual_x_fn)
```

### 2.3 Actualización Unscented

**Algoritmo:**

1. **Propagar** sigma points predichos a través de la medición:
   $$\mathcal{Z}_{k|k-1}^{(i)} = h(\mathcal{X}_{k|k-1}^{(i)})$$

2. **Medición predicha**:
   $$\hat{\mathbf{z}}_{k|k-1} = \sum_{i=0}^{2n} W_m^{(i)} \mathcal{Z}_{k|k-1}^{(i)}$$

3. **Covarianza de innovación**:
   $$\mathbf{S}_k = \sum_{i=0}^{2n} W_c^{(i)} [\mathcal{Z}_{k|k-1}^{(i)} - \hat{\mathbf{z}}_{k|k-1}][\mathcal{Z}_{k|k-1}^{(i)} - \hat{\mathbf{z}}_{k|k-1}]^T + \mathbf{R}_k$$

4. **Covarianza cruzada** estado-medición:
   $$\mathbf{P}_{xz} = \sum_{i=0}^{2n} W_c^{(i)} [\mathcal{X}_{k|k-1}^{(i)} - \hat{\mathbf{x}}_{k|k-1}][\mathcal{Z}_{k|k-1}^{(i)} - \hat{\mathbf{z}}_{k|k-1}]^T$$

5. **Ganancia de Kalman**:
   $$\mathbf{K}_k = \mathbf{P}_{xz} \mathbf{S}_k^{-1}$$

6. **Actualización del estado**:
   $$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k (\mathbf{z}_k - \hat{\mathbf{z}}_{k|k-1})$$

7. **Actualización de covarianza**:
   $$\mathbf{P}_{k|k} = \mathbf{P}_{k|k-1} - \mathbf{K}_k \mathbf{S}_k \mathbf{K}_k^T$$

**Implementación** ([`filters/unscented.py`](../state_estimation/filters/unscented.py#L220)):
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

### 2.4 Funciones Personalizadas para Ángulos

#### Función de Media Circular

Para estados con componentes angulares, la media aritmética es inadecuada. Se usa **media circular**:

$$
\bar{\psi} = \text{atan2}\left(\sum_{i=0}^{2n} W_m^{(i)} \sin\psi^{(i)}, \sum_{i=0}^{2n} W_m^{(i)} \cos\psi^{(i)}\right)
$$

**Implementación** ([`ukf_omnidirectional.py`](ukf_omnidirectional.py#L47)):
```python
from state_estimation.common.angles import circular_mean

def state_mean_fn(sigmas, Wm):
    x = np.zeros(6)
    x[0] = np.dot(Wm, sigmas[:, 0])  # x (lineal)
    x[1] = np.dot(Wm, sigmas[:, 1])  # y (lineal)
    x[2] = circular_mean(sigmas[:, 2], Wm)  # psi (circular)
    x[3] = np.dot(Wm, sigmas[:, 3])  # vx (lineal)
    x[4] = np.dot(Wm, sigmas[:, 4])  # vy (lineal)
    x[5] = np.dot(Wm, sigmas[:, 5])  # omega (lineal)
    return x
```

La función `circular_mean` ([`common/angles.py`](../state_estimation/common/angles.py)) proyecta ángulos al círculo unitario, promedia vectores 2D y recupera el ángulo:

```python
def circular_mean(angles, weights):
    sin_sum = np.sum(weights * np.sin(angles))
    cos_sum = np.sum(weights * np.cos(angles))
    return np.arctan2(sin_sum, cos_sum)
```

**Justificación:** La media aritmética de $\psi = [179°, -179°]$ es $0°$ (incorrecto, debería ser $\pm 180°$). La media circular correctamente resulta en $180°$.

#### Función de Residual Angular

Para diferencias en covarianza, los ángulos deben normalizarse:

$$
\text{residual}(\psi_a, \psi_b) = \text{atan2}(\sin(\psi_a - \psi_b), \cos(\psi_a - \psi_b))
$$

**Implementación** ([`ukf_omnidirectional.py`](ukf_omnidirectional.py#L71)):
```python
from state_estimation.common.angles import normalize_angle

def residual_x(a, b):
    y = a - b
    y[2] = normalize_angle(y[2])  # psi
    return y

def residual_z(a, b):
    y = a - b
    y[3] = normalize_angle(y[3])  # psi en medición
    return y
```

**Configuración del UKF** ([`ukf_omnidirectional.py`](ukf_omnidirectional.py#L131)):
```python
ukf.set_mean_fn(x_mean_fn=state_mean_fn, z_mean_fn=z_mean_fn)
ukf.set_residual_fn(residual_x_fn=residual_x, residual_z_fn=residual_z)
```

### 2.5 Matrices de Covarianza

**Ruido de proceso:**
$$
\mathbf{Q} = \text{diag}([1 \times 10^{-4}, 1 \times 10^{-4}, 1 \times 10^{-5}, 5 \times 10^{-3}, 5 \times 10^{-3}, 5 \times 10^{-4}]) \times 10
$$

**Ruido de medición:**
$$
\mathbf{R} = \text{diag}([6.72 \times 10^{-4}, 6.72 \times 10^{-4}, 1.31 \times 10^{-2}, 1.22 \times 10^{-3}]) \times 10^3
$$

Escalados para mejorar condicionamiento numérico del UKF.

---

## 3. Particle Filter (PF)

**Archivo de implementación:** [`filters/particle.py`](../state_estimation/filters/particle.py)  
**Script de ejecución:** [`pf_omnidirectional.py`](pf_omnidirectional.py)

### 3.1 Fundamento: Filtrado Secuencial de Importancia

El Particle Filter representa la distribución posterior $p(\mathbf{x}_k | \mathbf{z}_{1:k}, \mathbf{u}_{1:k})$ mediante un conjunto de **partículas** (muestras Monte Carlo):

$$
p(\mathbf{x}_k | \mathbf{z}_{1:k}) \approx \sum_{i=1}^{N} w_k^{(i)} \delta(\mathbf{x}_k - \mathbf{x}_k^{(i)})
$$

Donde:
- $\mathbf{x}_k^{(i)}$: Estado de la partícula $i$
- $w_k^{(i)}$: Peso normalizado ($\sum_i w_k^{(i)} = 1$)
- $N$: Número de partículas (típicamente 1000-10000)

**Ventajas:**
- Maneja no linealidades arbitrarias y distribuciones multimodales
- No asume Gaussianidad
- Converge a la distribución verdadera cuando $N \to \infty$

**Desventajas:**
- Complejidad computacional $\mathcal{O}(N)$
- Degeneración de partículas requiere remuestreo

### 3.2 Algoritmo SIR (Sequential Importance Resampling)

Implementado en [`filters/particle.py`](../state_estimation/filters/particle.py).

#### Paso 1: Inicialización

Muestrear partículas de la distribución inicial:

$$
\mathbf{x}_0^{(i)} \sim \mathcal{N}(\mathbf{x}_0, \mathbf{P}_0), \quad w_0^{(i)} = \frac{1}{N}
$$

**Implementación** ([`pf_omnidirectional.py`](pf_omnidirectional.py#L70)):
```python
x0 = ground_truth[0]
P0 = np.diag([0.5, 0.5, 0.1, 0.2, 0.2, 0.05]) * 1e-5
pf.initialize_particles(x0=x0, P0=P0)
```

Genera 2000 partículas con dispersión inicial muy baja (alta confianza en estado inicial).

#### Paso 2: Predicción (Propagación)

Cada partícula se propaga a través de la dinámica con ruido de proceso:

$$
\mathbf{x}_k^{(i)} = f(\mathbf{x}_{k-1}^{(i)}, \mathbf{u}_k) + \mathbf{w}_k^{(i)}, \quad \mathbf{w}_k^{(i)} \sim \mathcal{N}(\mathbf{0}, \mathbf{Q})
$$

**Implementación** ([`filters/particle.py`](../state_estimation/filters/particle.py#L106)):
```python
def predict(self, u=None, f=None, process_noise_std=None):
    for i in range(self.N):
        self.particles[i] = f(self.particles[i], u)
    
    # Añadir ruido Gaussiano
    if process_noise_std is not None:
        noise = np.random.normal(0, process_noise_std, size=self.particles.shape)
        self.particles += noise
    
    self.x = np.average(self.particles, weights=self.weights, axis=0)
```

**Configuración experimental** ([`pf_omnidirectional.py`](pf_omnidirectional.py#L76)):
```python
process_std = np.array([0.01, 0.01, 0.01, 0.05, 0.05, 0.05])
```

Ruido mayor en velocidades que en posiciones/orientación.

#### Paso 3: Actualización (Repesado)

Los pesos se actualizan según la verosimilitud de la medición:

$$
w_k^{(i)} \propto w_{k-1}^{(i)} \cdot p(\mathbf{z}_k | \mathbf{x}_k^{(i)})
$$

Para mediciones Gaussianas:

$$
p(\mathbf{z}_k | \mathbf{x}_k^{(i)}) = \frac{1}{(2\pi)^{m/2}|\mathbf{R}|^{1/2}} \exp\left(-\frac{1}{2}[\mathbf{z}_k - h(\mathbf{x}_k^{(i)})]^T \mathbf{R}^{-1} [\mathbf{z}_k - h(\mathbf{x}_k^{(i)})]\right)
$$

**Implementación** ([`filters/particle.py`](../state_estimation/filters/particle.py#L148)):
```python
def update(self, z, h=None, R=None):
    likelihood_fn = self._make_gaussian_likelihood(h, R)
    
    for i in range(self.N):
        self.weights[i] *= likelihood_fn(z, self.particles[i])
    
    # Normalizar pesos
    self.weights /= np.sum(self.weights)
    
    self.x = np.average(self.particles, weights=self.weights, axis=0)
    self.P = self._compute_covariance()
```

La función `_make_gaussian_likelihood` ([`filters/particle.py`](../state_estimation/filters/particle.py#L364)) implementa la verosimilitud Gaussiana multivariada usando `scipy.stats.multivariate_normal`.

#### Paso 4: Remuestreo Sistemático

**Problema de degeneración:** Tras varias iteraciones, la mayoría de partículas tienen pesos despreciables ($w^{(i)} \approx 0$), concentrándose la masa probabilística en pocas partículas.

**Métrica:** Tamaño efectivo de muestra (ESS):

$$
N_{\text{eff}} = \frac{1}{\sum_{i=1}^{N} (w^{(i)})^2}
$$

- $N_{\text{eff}} = N$: Pesos uniformes (ideal)
- $N_{\text{eff}} = 1$: Un peso domina (degeneración total)

**Criterio de remuestreo:** Si $N_{\text{eff}} < N / 2$, aplicar remuestreo.

**Algoritmo de remuestreo sistemático** ([`filters/particle.py`](../state_estimation/filters/particle.py#L234)):

1. Generar $N$ posiciones equiespaciadas con offset aleatorio:
   $$u_i = \frac{i + \mathcal{U}(0,1)}{N}, \quad i = 0, \ldots, N-1$$

2. Calcular CDF de pesos: $c_j = \sum_{k=0}^{j} w^{(k)}$

3. Para cada $u_i$, encontrar $j$ tal que $c_{j-1} < u_i \leq c_j$ y duplicar partícula $j$

**Ventajas sobre multinomial:**
- Menor varianza (muestreo estratificado)
- Complejidad $\mathcal{O}(N)$ en lugar de $\mathcal{O}(N \log N)$

**Implementación:**
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

**Uso en el filtro** ([`pf_omnidirectional.py`](pf_omnidirectional.py#L87)):
```python
for k in range(N - 1):
    pf.predict(u=controls[k], f=robot.dynamics, process_noise_std=process_std)
    pf.update(z=measurements[k + 1], h=robot.measurement, R=R)
    pf.resample(scheme='systematic')  # Remuestreo en cada paso
```

**Nota:** En la implementación actual, se aplica remuestreo en cada iteración. Alternativamente, se puede condicionar al ESS:

```python
if pf.effective_sample_size() < pf.resample_threshold:
    pf.resample(scheme='systematic')
```

### 3.3 Estimación de Estado

**Media ponderada:**
$$
\hat{\mathbf{x}}_k = \sum_{i=1}^{N} w_k^{(i)} \mathbf{x}_k^{(i)}
$$

**Covarianza:**
$$
\mathbf{P}_k = \sum_{i=1}^{N} w_k^{(i)} (\mathbf{x}_k^{(i)} - \hat{\mathbf{x}}_k)(\mathbf{x}_k^{(i)} - \hat{\mathbf{x}}_k)^T
$$

**Implementación** ([`filters/particle.py`](../state_estimation/filters/particle.py#L343)):
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

### 3.4 Configuración Experimental

**Número de partículas:** $N = 2000$ (balance entre precisión y costo computacional)

**Matrices de ruido:**
```python
process_std = np.array([0.01, 0.01, 0.01, 0.05, 0.05, 0.05])
R = np.diag([6.72e-4, 6.72e-4, 1.3125e-2, 1.218e-3])
```

**Frecuencia:** 100 Hz (igual que EKF/UKF)

**Métricas de monitoreo:**
```python
if (k + 1) % 100 == 0:
    ess = pf.effective_sample_size()
    print(f"  Step {k+1}/{N-1}, ESS: {ess:.1f}")
```

Valores típicos de ESS: 800-1500 (de 2000), indicando distribución saludable de pesos.

---

## 4. Comparación Metodológica

### Tabla Comparativa

| Característica | EKF | UKF | PF |
|----------------|-----|-----|-----|
| **Linealización** | Jacobiana (1er orden) | Transformada Unscented (2do orden) | No requerida |
| **Distribución** | Gaussiana | Gaussiana | Arbitraria |
| **Complejidad** | $\mathcal{O}(n^3)$ | $\mathcal{O}(n^3)$ | $\mathcal{O}(N \cdot n)$ |
| **Jacobiano** | ✅ Requerido | ❌ No requerido | ❌ No requerido |
| **Precisión** | Baja (no lineal fuerte) | Alta (no lineal moderada) | Converge a óptimo |
| **Estabilidad numérica** | Sensible | Robusta | Muy robusta |
| **Multimodalidad** | ❌ | ❌ | ✅ |
| **Tiempo ejecución** | ~1 ms/iter | ~2 ms/iter | ~20 ms/iter |

### Escenarios de Aplicación

**EKF:**
- Dinámicas débilmente no lineales
- Recursos computacionales limitados
- Modelo analítico bien caracterizado

**UKF:**
- No linealidades moderadas
- Jacobiano difícil de derivar o costoso
- Mayor precisión sin incremento dramático de costo

**PF:**
- No linealidades severas (e.g., localización con ambigüedad)
- Distribuciones no Gaussianas (e.g., fallas intermitentes)
- Recursos computacionales abundantes

Para el robot omnidireccional con dinámica de cuerpo rígido, **EKF y UKF** suelen ser suficientes, siendo PF un caso de validación para distribuciones potencialmente multimodales en orientación.

---

## 5. Fuentes de Datos Experimentales

### 5.1 Estructura de Archivos

```
data/
├── sensors/              # Datos crudos de IMU + encoders
│   └── expN.txt         # N = 1-10
└── processed/
    └── trajectories/    # Trayectorias fusionadas (video + IMU)
        └── traj_vid_N.csv
```

### 5.2 Formato de Datos de Sensores

**Archivo:** `data/sensors/expN.txt`

```
t,ax,ay,alpha,w1,w2,w3,u1,u2,u3,vbx_sp,vby_sp,wb_sp
```

- `t`: Tiempo [s]
- `ax, ay`: Aceleración del IMU en marco del cuerpo [m/s²]
- `alpha`: Orientación del IMU [rad]
- `w1, w2, w3`: Velocidades angulares de ruedas [rad/s]
- `u1, u2, u3`: Señales PWM a motores [%]
- `*_sp`: Setpoints (no usados en estimación)

### 5.3 Formato de Trayectorias Ground Truth

**Archivo:** `data/processed/trajectories/traj_vid_N.csv`

```
time_s,x_m,y_m,phi_rad,vx_m_s,vy_m_s,omega_rad_s,u1_pwm,u2_pwm,u3_pwm
```

- Obtenido mediante flujo óptico + detección de color (ver [Robot_Identification/src/examples/README.md](../../Robot_Identification/src/examples/README.md))
- Frecuencia: 100 Hz (resampleado)
- Usado como referencia para métricas de error

### 5.4 Carga de Datos

Implementado en [`trajectory_generators.py`](trajectory_generators.py#L150):

```python
def load_experimental_data(sensors_path, trajectory_path, N=1000, dt=0.01):
    # Cargar sensores
    sensors_df = pd.read_csv(sensors_path)
    ax = sensors_df['ax'].values[:N]
    ay = sensors_df['ay'].values[:N]
    
    # Cargar ground truth
    traj_df = pd.read_csv(trajectory_path)
    x = traj_df['x_m'].values[:N]
    y = traj_df['y_m'].values[:N]
    phi = traj_df['phi_rad'].values[:N]
    
    # Construir mediciones: [vx_b, vy_b, omega, psi]
    measurements = compute_body_velocities(vx, vy, phi, omega)
    
    # Controles: [ax_b, ay_b]
    controls = np.column_stack([ax, ay])
    
    return {'time': t, 'controls': controls, 'measurements': measurements,
            'ground_truth': ground_truth, 'dt': dt}
```

---

## 6. Métricas de Evaluación

### 6.1 Errores de Estimación

**Root Mean Square Error (RMSE):**
$$
\text{RMSE}_i = \sqrt{\frac{1}{N} \sum_{k=1}^{N} (x_k^{(i)} - \hat{x}_k^{(i)})^2}
$$

Para cada componente del estado $i \in \{x, y, \psi, v_x, v_y, \omega\}$.

**Mean Absolute Error (MAE):**
$$
\text{MAE}_i = \frac{1}{N} \sum_{k=1}^{N} |x_k^{(i)} - \hat{x}_k^{(i)}|
$$

### 6.2 Consistencia del Filtro

**Normalized Estimation Error Squared (NEES):**
$$
\epsilon_k = (\mathbf{x}_k - \hat{\mathbf{x}}_k)^T \mathbf{P}_k^{-1} (\mathbf{x}_k - \hat{\mathbf{x}}_k)
$$

Para un filtro consistente, $\epsilon_k \sim \chi^2(n)$ con $n$ grados de libertad. Valor esperado: $\mathbb{E}[\epsilon_k] = n = 6$.

**Normalized Innovation Squared (NIS):**
$$
\nu_k = \mathbf{y}_k^T \mathbf{S}_k^{-1} \mathbf{y}_k
$$

Donde $\mathbf{y}_k$ es la innovación. Para filtro consistente, $\nu_k \sim \chi^2(m)$ con $m = 4$ (dimensión de medición).

**Implementación:** [`metrics.py`](../state_estimation/metrics.py)

### 6.3 Resultados Típicos

**EKF:**
- RMSE posición: 0.03-0.05 m
- RMSE orientación: 0.05-0.10 rad
- NEES promedio: 5-8 (ligeramente optimista)

**UKF:**
- RMSE posición: 0.025-0.04 m
- RMSE orientación: 0.04-0.08 rad
- NEES promedio: 6-9 (más consistente)

**PF (N=2000):**
- RMSE posición: 0.02-0.035 m
- RMSE orientación: 0.03-0.06 rad
- ESS promedio: 900-1400

---

## 7. Ejecución de Experimentos

### 7.1 Scripts Principales

**EKF:**
```bash
python ekf_omnidirectional.py
```

**UKF:**
```bash
python ukf_omnidirectional.py
```

**PF:**
```bash
python pf_omnidirectional.py
```

### 7.2 Configuración

Editar constantes al inicio de cada script:

```python
USE_EXPERIMENTAL_DATA = True   # True: datos reales, False: sintéticos
EXPERIMENT_NUMBER = 2          # Experimento 1-10
N_POINTS = 1000                # Número de pasos temporales
DT = 0.01                      # Periodo de muestreo (100 Hz)
```

### 7.3 Salidas

**Directorio de resultados:**
```
results/estimation/
├── ekf/
│   ├── ekf_trayectoria_2d.png
│   ├── ekf_estados_temporales.png
│   └── ekf_metricas.csv
├── ukf/
│   └── ...
└── pf/
    └── ...
```

**Formato CSV de métricas:**
```csv
Métrica,Valor
RMSE Total,0.0347
RMSE X [m],0.0234
RMSE Y [m],0.0221
RMSE Phi [rad],0.0587
NEES Promedio,6.823
```

---

## 8. Consideraciones de Implementación

### 8.1 Estabilidad Numérica

**Problema:** Matrices de covarianza pueden perder simetría o positividad definida por errores de punto flotante.

**Soluciones implementadas:**

1. **Forma de Joseph para EKF** ([`extended.py`](../state_estimation/filters/extended.py#L195)):
   $$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T$$
   
   Garantiza simetría y mejora condicionamiento numérico.

2. **Descomposición de Cholesky con manejo de errores** ([`unscented.py`](../state_estimation/filters/unscented.py#L76)):
   ```python
   try:
       U = cholesky((self.n + self._lambda) * P)
   except np.linalg.LinAlgError:
       # Si falla, forzar simetría y añadir regularización
       P = (P + P.T) / 2
       P += np.eye(self.n) * 1e-9
       U = cholesky((self.n + self._lambda) * P)
   ```

3. **Normalización de pesos en PF** ([`particle.py`](../state_estimation/filters/particle.py#L183)):
   ```python
   weight_sum = np.sum(self.weights)
   if weight_sum < 1e-10:  # Degeneración extrema
       self.weights = np.ones(self.N) / self.N  # Reiniciar uniformemente
   else:
       self.weights /= weight_sum
   ```

### 8.2 Eficiencia Computacional

**Vectorización de operaciones:**

- EKF/UKF: Operaciones matriciales con NumPy/SciPy (BLAS optimizado)
- PF: Bucles sobre partículas inevitables, pero operaciones internas vectorizadas

**Perfilado típico (Intel i7, 2.6 GHz):**
- EKF: ~0.8 ms/iteración
- UKF: ~1.5 ms/iteración
- PF (N=2000): ~18 ms/iteración

**Paralelización potencial:** El PF es embarazosamente paralelizable (cada partícula se procesa independientemente). Se puede usar `multiprocessing` o Numba JIT para acelerar.

### 8.3 Afinación de Parámetros

**Proceso de calibración de $\mathbf{Q}$ y $\mathbf{R}$:**

1. **Caracterización de sensores:**
   - Colocar robot estático, medir $\sigma_{\text{sensor}}$ durante 1 minuto
   - Diagonales de $\mathbf{R}$: varianzas empíricas

2. **Optimización de $\mathbf{Q}$:**
   - Iniciar con valores heurísticos: $\mathbf{Q} = 0.01 \cdot \mathbb{E}[\mathbf{x}\mathbf{x}^T]$
   - Ejecutar filtro, calcular NEES
   - Ajustar iterativamente buscando NEES $\approx n$
   - Herramientas: Grid search o CMA-ES

3. **Validación cruzada:**
   - Calibrar en experimentos 1-5
   - Validar en experimentos 6-10
   - Reportar métricas de ambos conjuntos

---

## Referencias Metodológicas

### Libros Fundamentales

1. **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics*. MIT Press.  
   Capítulos 3 (Recursive State Estimation), 7 (Kalman Filters), 8 (EKF), 9 (UKF), 4 (Particle Filters).

2. **Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001).** *Estimation with Applications to Tracking and Navigation*. Wiley.  
   Secciones sobre Jacobiano del EKF y validación NEES/NIS.

3. **Särkkä, S. (2013).** *Bayesian Filtering and Smoothing*. Cambridge University Press.  
   Derivaciones rigurosas de UKF y transformada Unscented.

### Artículos Específicos

4. **Julier, S. J., & Uhlmann, J. K. (1997).** "New Extension of the Kalman Filter to Nonlinear Systems". *Signal Processing, Sensor Fusion, and Target Recognition VI*, SPIE.  
   Introducción original del UKF.

5. **Van Der Merwe, R., & Wan, E. A. (2001).** "The Square-Root Unscented Kalman Filter for State and Parameter-Estimation". *IEEE International Conference on Acoustics, Speech, and Signal Processing*.  
   Algoritmo de sigma points escalados (usado en esta implementación).

6. **Arulampalam, M. S., et al. (2002).** "A Tutorial on Particle Filters for Online Nonlinear/Non-Gaussian Bayesian Tracking". *IEEE Transactions on Signal Processing*, 50(2), 174-188.  
   Tutorial exhaustivo de PF con análisis de métodos de remuestreo.

### Software y Bibliotecas

7. **FilterPy (Roger R. Labbe Jr.):** Biblioteca Python de referencia para filtros bayesianos.  
   Disponible en: https://github.com/rlabbe/filterpy  
   (Inspiración para la API de esta implementación)

---

## Apéndices

### A. Nomenclatura y Convenciones

| Símbolo | Descripción | Dimensión |
|---------|-------------|-----------|
| $\mathbf{x}_k$ | Vector de estado en tiempo $k$ | $6 \times 1$ |
| $\mathbf{u}_k$ | Vector de control | $2 \times 1$ |
| $\mathbf{z}_k$ | Vector de medición | $4 \times 1$ |
| $\mathbf{P}_k$ | Covarianza de estado | $6 \times 6$ |
| $\mathbf{Q}$ | Covarianza de ruido de proceso | $6 \times 6$ |
| $\mathbf{R}$ | Covarianza de ruido de medición | $4 \times 4$ |
| $\mathbf{F}_k$ | Jacobiano de dinámica | $6 \times 6$ |
| $\mathbf{H}_k$ | Jacobiano de medición | $4 \times 6$ |
| $\mathbf{K}_k$ | Ganancia de Kalman | $6 \times 4$ |
| $n$ | Dimensión del estado | 6 |
| $m$ | Dimensión de la medición | 4 |
| $N$ | Número de partículas (PF) | 1000-10000 |
| $\Delta t$ | Periodo de muestreo | 0.01 s |

### B. Pseudocódigo Completo

#### EKF
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

#### UKF
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

#### PF
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

### C. Código de Ejemplo Mínimo

**Filtro EKF completo en ~20 líneas:**

```python
from state_estimation import ExtendedKalmanFilter
from state_estimation.models import OmnidirectionalRobot
from state_estimation.common import make_residual_fn

# Inicializar
robot = OmnidirectionalRobot(dt=0.01)
ekf = ExtendedKalmanFilter(dim_x=6, dim_z=4, dim_u=2)
ekf.x = np.array([0, 0, 0, 0, 0, 0])
ekf.P = np.diag([0.5, 0.5, 0.1, 0.2, 0.2, 0.05])
ekf.Q = np.diag([1e-4, 1e-4, 1e-5, 5e-3, 5e-3, 5e-4])
ekf.R = np.diag([6.72e-4, 6.72e-4, 1.3125e-2, 1.218e-3])
ekf.set_residual_fn(make_residual_fn(angle_indices=[2]))

# Loop principal
for k in range(N - 1):
    ekf.predict(u=controls[k], f=robot.dynamics, F=robot.jacobian_F)
    ekf.update(z=measurements[k + 1], h=robot.measurement, H=robot.jacobian_H)
    estimates[k + 1] = ekf.x
```

---

**Autor:** Maverick Sossa Tobón 
**Fecha:** Enero 2026  
**Versión:** 1.0  
**Institución:** [Universidad de Antioquia]  
