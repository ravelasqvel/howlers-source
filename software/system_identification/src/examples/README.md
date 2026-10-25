# Metodología Experimental de Identificación Paramétrica en Tres Etapas

## Resumen Ejecutivo

Este documento describe la metodología experimental completa implementada para la identificación paramétrica de un robot omnidireccional de tres ruedas. Se emplea un enfoque de **identificación secuencial en tres etapas** que desacopla las dinámicas eléctricas, rotacionales y traslacionales del sistema, permitiendo una estimación robusta y bien condicionada de parámetros.

Cada etapa se compone de:
1. **Protocolo experimental** diseñado para maximizar la sensibilidad a parámetros específicos
2. **Preprocesamiento de datos** (filtrado, transformaciones cinemáticas)
3. **Optimización paramétrica** mediante minimización de error cuadrático medio

La implementación sigue estrictamente el marco teórico desarrollado en el [README.md principal](../../README.md) y se ejecuta a través de la clase `ThreeStageIdentification` en [three_stage.py](../identification/three_stage.py).

---

## Arquitectura del Sistema de Identificación

### Estructura de Datos Experimentales

```
data/
├── raw/
│   ├── motor_tests/       # Etapa 1: Pruebas de motor en estado estable
│   │   ├── stage1_1.txt
│   │   ├── stage1_2.txt
│   │   └── stage1_3.txt
│   └── rotation_tests/    # Etapa 2: Rotación pura
│       ├── stage2_1.txt
│       └── stage2_2.txt
└── processed_2/
    └── trajectories/      # Etapa 3: Trayectorias completas (video + IMU)
        ├── traj_vid_1.csv
        └── traj_vid_2.csv
```

### Formato de Datos Crudos

#### Archivos Stage 1 (motor_tests/stage1_X.txt)
```
t,ax,ay,alpha,w1,w2,w3,u1,u2,u3,vbx_sp,vby_sp
```
- `t`: tiempo [s]
- `ax, ay, alpha`: acelerómetro y orientación IMU (no usados en Stage 1)
- `w1, w2, w3`: velocidades angulares de ruedas [rad/s] (encoders)
- `u1, u2, u3`: señales PWM aplicadas [%] (0-100)
- `vbx_sp, vby_sp`: setpoints de velocidad (no usados)

#### Archivos Stage 2 (rotation_tests/stage2_X.txt)
```
t,ax,ay,alpha,w1,w2,w3,u1,u2,u3,vbx_sp,vby_sp,wb_sp
```
- Mismo formato que Stage 1 + `wb_sp`: setpoint de velocidad angular
- `alpha`: orientación crítica para rotación pura

#### Archivos Stage 3 (trajectories/traj_vid_X.csv)
```
time_s,x_m,y_m,phi_rad,vx_m_s,vy_m_s,omega_rad_s,u1_pwm,u2_pwm,u3_pwm
```
- Estados fusionados de video tracking + IMU
- 100 Hz, sincronizado con señales de control

---

## Etapa 1: Identificación de Parámetros Eléctricos del Motor

**Archivo:** [`stage1_experimental.py`](stage1_experimental.py)  
**Método de identificación:** [`ThreeStageIdentification.stage1_motor_parameters()`](../identification/three_stage.py#L48)  
**Base teórica:** [README.md § 2. Motor Electrical Model](../../README.md#2-motor-electrical-model)

### 1.1 Objetivo y Motivación Física

Estimar los parámetros eléctricos intrínsecos de cada motor DC:
- $R_a$ [Ω]: Resistencia del inducido
- $K$ [V/(rad/s)]: Constante de motor (también Nm/A por reciprocidad electromagnética)
- $b$ [Nm/(rad/s)]: Coeficiente de fricción viscosa (secundario)

**Motivación:** En estado estable ($\dot{\omega} = 0$), las ecuaciones eléctricas y mecánicas del motor se reducen a una relación algebraica entre voltaje aplicado y velocidad angular, eliminando la dependencia de dinámicas de orden superior (inducción, inercia del rotor).

### 1.2 Modelo Físico en Estado Estable

Del modelo DC estándar:

$$
\begin{aligned}
V &= R_a \cdot i + K \cdot \omega && \text{(Ley de voltajes de Kirchhoff)} \\
\tau_{\text{motor}} &= K \cdot i && \text{(Conversión electromecánica)} \\
\tau_{\text{motor}} &= \tau_{\text{fricción}} && \text{(Equilibrio mecánico en estado estable)}
\end{aligned}
$$

Asumiendo fricción viscosa $\tau_{\text{fricción}} = b \cdot \omega$:

$$
K \cdot i = b \cdot \omega \quad \Rightarrow \quad i = \frac{b \cdot \omega}{K}
$$

Sustituyendo en la ecuación de voltaje:

$$
V = R_a \cdot \frac{b \cdot \omega}{K} + K \cdot \omega = \left(\frac{R_a \cdot b}{K} + K\right) \cdot \omega
$$

Esta es una **relación lineal** entre $V$ y $\omega$ con pendiente:

$$
m = \frac{R_a \cdot b}{K} + K
$$

### 1.3 Protocolo Experimental

#### Diseño del Experimento
1. **Montaje:** Robot suspendido (ruedas libres) o motor en banco de pruebas
2. **Secuencia temporal:** 8 segmentos de 1.5 s cada uno
3. **Excitación:** PWM constante por segmento, variando en {-80%, -60%, ..., +60%, +80%}
4. **Medición:** Velocidad angular de encoders a 100 Hz

#### Criterios de Validez
- Tiempo de establecimiento: $t_{\text{settle}} \leq 0.5$ s (constante de tiempo mecánica $\tau_m = J/b \approx 0.2$ s)
- Ventana de estado estable: últimos 1.0 s de cada segmento
- Ruido de medición: $\sigma_{\omega} \approx 1.1$ rad/s (característica de encoders increméntales)

### 1.4 Pipeline de Procesamiento

#### Paso 1: Carga de Datos
```python
data = load_experimental_data(DATA_FILE)
t = data['t']        # (N,) tiempo
w_raw = data['w']    # (N, 3) velocidades angulares crudas
u_pwm = data['u']    # (N, 3) PWM en porcentaje
```

**Validación:**
- Verificar continuidad temporal: $\Delta t = t[i+1] - t[i]$ constante
- Duración total: $T_{\text{total}} = 8 \times 1.5 = 12$ s

#### Paso 2: Filtrado Kalman 1D

**Justificación:** Los encoders incrementales generan ruido de cuantización (resolución finita) y vibraciones mecánicas. Un filtro de Kalman de primer orden es óptimo para ruido Gaussiano blanco.

**Modelo de estado:**
$$
\begin{aligned}
\omega_{k+1} &= \omega_k + w_k, \quad w_k \sim \mathcal{N}(0, Q) && \text{(modelo de proceso)} \\
z_k &= \omega_k + v_k, \quad v_k \sim \mathcal{N}(0, R) && \text{(modelo de medición)}
\end{aligned}
$$

**Parámetros calibrados:**
- $Q = 0.001$: Baja varianza de proceso (velocidad casi constante en segmentos)
- $R = 1.12$: Varianza de medición (caracterizada experimentalmente)

**Implementación:**
```python
def filter_velocities(w_raw, Q=0.001, R=1.12):
    filters = [Kalman1D(Q, R) for _ in range(3)]
    w_filtered = np.zeros_like(w_raw)
    for i in range(len(w_raw)):
        for motor_idx in range(3):
            w_filtered[i, motor_idx] = filters[motor_idx].update(w_raw[i, motor_idx])
    return w_filtered
```

**Resultado esperado:** Reducción de ruido RMS > 70% sin introducir lag significativo (ganancia de Kalman adaptativa).

#### Paso 3: Conversión PWM → Voltaje

**Fundamento:** El controlador de motor usa modulación por ancho de pulso (PWM) con frecuencia $f_{\text{PWM}} \approx 20$ kHz. La constante de tiempo eléctrica del motor es:

$$
\tau_e = \frac{L_a}{R_a} \approx \frac{0.5 \text{ mH}}{1.5 \text{ Ω}} \approx 0.33 \text{ ms} \ll T_{\text{PWM}} = 50 \text{ μs}
$$

El motor actúa como **filtro pasa-bajos**, convirtiendo el PWM en voltaje DC equivalente:

$$
V_{\text{eff}} = \frac{D}{100} \cdot V_{\text{bat}}
$$

donde $D$ es el duty cycle en porcentaje.

**Implementación:**
```python
from src.utils.conversions import pwm_to_voltage

u_voltage = np.zeros_like(u_pwm)
for motor_idx in range(3):
    u_voltage[:, motor_idx] = pwm_to_voltage(u_pwm[:, motor_idx], V_BATTERY)
```

**Consideración crítica:** $V_{\text{bat}}$ debe medirse durante el experimento (batería LiPo 6S: 22.2 V nominal, varía 25.2 V → 18.0 V según estado de carga).

#### Paso 4: Extracción de Valores de Estado Estable

**Algoritmo:**
```python
def extract_steady_state_values(t, w_filtered, u_voltage, 
                                segment_duration=1.5, 
                                discard_time=0.5):
    dt = t[1] - t[0]
    discard_samples = int(discard_time / dt)
    segment_samples = int(segment_duration / dt)
    
    for seg in range(8):
        start_idx = seg * segment_samples
        steady_start = start_idx + discard_samples
        steady_end = start_idx + segment_samples
        
        # Promediar ventana de estado estable
        u_steady[seg, :] = np.mean(u_voltage[steady_start:steady_end, :], axis=0)
        w_steady[seg, :] = np.mean(w_filtered[steady_start:steady_end, :], axis=0)
    
    return u_steady, w_steady
```

**Resultado:** Arrays de $(8, 3)$ con pares $(V_i, \omega_i)$ para cada motor.

#### Paso 5: Llamada al Identificador

```python
identifier = ThreeStageIdentification(wheel_angles, d, r, M)

# Seleccionar motor (ejemplo: Motor 2)
motor_idx = 1  # 0-indexed
valid = np.abs(w_steady[:, motor_idx]) > 0.5  # Filtrar puntos inactivos
u_motor = u_steady[valid, motor_idx]
w_motor = w_steady[valid, motor_idx]

identifier.stage1_motor_parameters(
    u_motor, 
    w_motor, 
    motor_id=2, 
    results_dir=results_dir
)
```

### 1.5 Optimización Paramétrica (en `three_stage.py`)

**Modelo no lineal:**
$$
\omega_{\text{pred}} = \frac{K \cdot V}{R_a \cdot b + K^2}
$$

**Función objetivo:**
$$
\min_{R_a, K, b} \sum_{i=1}^{N} \left(\omega_{\text{meas},i} - \frac{K \cdot V_i}{R_a \cdot b + K^2}\right)^2
$$

**Restricciones físicas:**
- $R_a \in [0.1, 20.0]$ Ω (motores DC pequeños)
- $K \in [0.01, 5.0]$ V/(rad/s)
- $b \in [0, 0.1]$ Nm/(rad/s) (fricción baja en rodamientos de bolas)

**Método:** L-BFGS-B (gradiente cuasi-Newton con restricciones de caja)

**Inicialización:** $K_0 = \text{mean}(V/\omega)$ (estimación sin fricción)

### 1.6 Validación y Métricas

**Métricas de calidad:**
1. **RMSE:** $\sqrt{\frac{1}{N}\sum_i (\omega_i - \hat{\omega}_i)^2}$ [rad/s]
2. **$R^2$:** Coeficiente de determinación
   $$
   R^2 = 1 - \frac{\sum_i (y_i - \hat{y}_i)^2}{\sum_i (y_i - \bar{y})^2}
   $$

**Criterios de aceptación:**
- $R^2 > 0.9$: Excelente ajuste
- $R^2 > 0.8$: Aceptable
- $R^2 < 0.8$: Revisar calidad de datos o modelo de fricción

**Salidas:**
- Figura 1: Caracterización $\omega$ vs $V$ con curva ajustada
- Figura 2: Series temporales de velocidades filtradas y voltajes
- Figura 3: Puntos de estado estable para los 3 motores
- CSV: Métricas numéricas ($R_a$, $K$, $b$, RMSE, $R^2$)

---

## Etapa 2: Identificación de Momento de Inercia

**Archivo:** [`stage2_experimental.py`](stage2_experimental.py)  
**Método de identificación:** [`ThreeStageIdentification.stage2_inertia()`](../identification/three_stage.py#L228)  
**Base teórica:** [README.md § 3.4 Coriolis Terms](../../README.md#34-coriolis-terms)

### 2.1 Objetivo y Desacoplamiento Dinámico

Estimar el momento de inercia del robot respecto al eje vertical:

$$
I = \int\int\int_V \rho(x,y,z) \cdot r_{\perp}^2 \, dV \quad [\text{kg·m}^2]
$$

**Ventaja del desacoplamiento:** En rotación pura ($v_x \approx 0$, $v_y \approx 0$), la ecuación dinámica se reduce a:

$$
I \cdot \dot{\Omega} = \tau_z
$$

eliminando acoplamiento con masa $M$ y fuerzas de fricción traslacionales.

### 2.2 Protocolo Experimental

#### Diseño de la Maniobra
1. **Configuración:** Robot sobre superficie lisa (fricción mínima)
2. **Comando:** Velocidad angular variable: $\Omega_{\text{ref}}(t) = A \cdot \sin(\omega t)$ con $A = 2$ rad/s, $\omega = 2\pi/5$ rad/s
3. **Duración:** 10-15 segundos
4. **Restricción:** Mantener $\|v_{\text{linear}}\| < 0.1$ m/s (control de feedback)

#### Sensores Utilizados
- **IMU (MPU6050):** Orientación $\phi(t)$ a 100 Hz (integración de giroscopio con corrección de acelerómetro)
- **Encoders:** Velocidades de rueda $\omega_i(t)$
- **Controlador:** Señales PWM $u_i(t)$

### 2.3 Reconstrucción del Vector de Estado

**Desafío:** El sistema de medición no proporciona directamente $\mathbf{x} = [x, y, \phi, \dot{x}, \dot{y}, \dot{\phi}]^T$. Debe reconstruirse mediante fusión cinemática.

#### Paso 1: Carga y Unwrapping de Orientación

```python
data = load_stage2_data(DATA_FILE)
t = data['t']
alpha_raw = data['alpha']  # De IMU, con saltos ±2π

# Eliminar discontinuidades de ángulo
alpha_unwrapped = np.unwrap(alpha_raw)
```

**Justificación del unwrap:** La representación $\phi \in [-\pi, \pi]$ introduce saltos artificiales. El unwrapping reconstruye $\phi(t)$ continuo mediante:

$$
\phi_{\text{unwrap}}[k] = \phi[k] + 2\pi \cdot n_k
$$

donde $n_k$ se incrementa cuando $|\phi[k] - \phi[k-1]| > \pi$.

#### Paso 2: Filtrado Kalman de Velocidades de Rueda

Mismo procedimiento que Etapa 1:

```python
w_filtered = np.zeros_like(w_raw)
filters_enc = [Kalman1D(Q_ENC=0.001, R_ENC=1.12) for _ in range(3)]

for i in range(len(t)):
    for motor_idx in range(3):
        w_filtered[i, motor_idx] = filters_enc[motor_idx].update(w_raw[i, motor_idx])
```

#### Paso 3: Cinemática Inversa

**Objetivo:** Calcular velocidades del robot en marco del cuerpo desde velocidades de rueda.

**Matriz cinemática inversa:** Del [README principal](../../README.md#13-inverse-kinematics):

$$
\begin{bmatrix} v_x \\ v_y \\ \Omega \end{bmatrix}_{\text{body}} = \mathbf{H}^+ \cdot \begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \end{bmatrix}
$$

donde $\mathbf{H}^+$ es la pseudoinversa de Moore-Penrose:

$$
\mathbf{H}^+ = (\mathbf{H}^T \mathbf{H})^{-1} \mathbf{H}^T
$$

**Implementación:**
```python
from src.kinematics.validator import KinematicValidator

H = KinematicValidator.compute_H_matrix(wheel_angles, d, r)
H_inv = np.linalg.pinv(H)

def compute_body_velocities(w_wheels, H_inv):
    N = len(w_wheels)
    v_body = np.zeros((N, 3))
    for i in range(N):
        v_body[i, :] = H_inv @ w_wheels[i, :]
    return v_body

v_body = compute_body_velocities(w_filtered, H_inv)
```

**Resultado:** Arrays $(N, 3)$ con $[v_{x,\text{body}}, v_{y,\text{body}}, \Omega]$.

#### Paso 4: Transformación a Marco Global

**Matriz de rotación:**
$$
\mathbf{R}(\phi) = \begin{bmatrix} \cos\phi & -\sin\phi & 0 \\ \sin\phi & \cos\phi & 0 \\ 0 & 0 & 1 \end{bmatrix}
$$

**Transformación de velocidades:**
$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\phi} \end{bmatrix}_{\text{global}} = \mathbf{R}(\phi) \cdot \begin{bmatrix} v_x \\ v_y \\ \Omega \end{bmatrix}_{\text{body}}
$$

**Implementación:**
```python
def body_to_global_velocities(v_body, phi):
    N = len(v_body)
    v_global = np.zeros((N, 3))
    for i in range(N):
        c, s = np.cos(phi[i]), np.sin(phi[i])
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        v_global[i, :] = R @ v_body[i, :]
    return v_global
```

#### Paso 5: Construcción del Vector de Estado Completo

**Posiciones sintéticas:** En rotación pura, el robot permanece casi estacionario. Añadimos ruido Gaussiano para estabilidad numérica:

```python
def reconstruct_full_state(alpha, w_filtered, H_inv, noise_std=0.01):
    N = len(alpha)
    y_measured = np.zeros((N, 6))
    
    # Posiciones: ruido Gaussiano (robot casi estático)
    y_measured[:, 0] = np.random.normal(0, noise_std, N)  # x
    y_measured[:, 1] = np.random.normal(0, noise_std, N)  # y
    
    # Orientación: de IMU (referenciada a cero inicial)
    y_measured[:, 2] = alpha - alpha[0]  # phi
    
    # Velocidades: de cinemática
    v_body = compute_body_velocities(w_filtered, H_inv)
    v_global = body_to_global_velocities(v_body, y_measured[:, 2])
    
    y_measured[:, 3:6] = v_global
    
    return y_measured
```

**Validación estadística:**
```python
omega_rms = np.sqrt(np.mean(y_measured[:, 5]**2))
vx_rms = np.sqrt(np.mean(y_measured[:, 3]**2))
vy_rms = np.sqrt(np.mean(y_measured[:, 4]**2))

assert omega_rms > 0.1, "Velocidad angular insuficiente"
assert vx_rms < 0.2 and vy_rms < 0.2, "Traslación excesiva (no rotación pura)"
```

### 2.4 Optimización Paramétrica (en `three_stage.py`)

**Función de costo ponderada:**

$$
J(I) = \sum_{k=1}^{N} \left[ w_\phi \cdot (\phi_k - \hat{\phi}_k)^2 + w_\Omega \cdot (\Omega_k - \hat{\Omega}_k)^2 \right]
$$

**Pesos calibrados:**
- $w_\phi = 10.0$: Tracking de orientación (integra velocidad angular → error acumulativo)
- $w_\Omega = 5.0$: Tracking de velocidad angular (medición directa)
- $w_{\text{trans}} = 0$: Estados traslacionales ignorados

**Simulación del modelo:**
- Integrador: `scipy.integrate.solve_ivp` con método RK45 (Runge-Kutta adaptativo)
- Modelo dinámico: Ecuaciones de estado del robot con $I$ variable

**Optimización:**
```python
result = minimize(
    cost_full,
    x0=[0.02],                    # Inicialización: I típico para robot 3 kg
    bounds=[(0.001, 0.5)],        # Límites físicos
    method='L-BFGS-B',
    options={'maxiter': 50, 'ftol': 1e-4}
)
I = result.x[0]
```

### 2.5 Validación y Diagnóstico

**Comparación visual:**
- Gráfica 1: $\phi(t)$ medido vs simulado
- Gráfica 2: $\Omega(t)$ medido vs simulado

**Métricas:**
- RMSE en $\phi$: $< 0.1$ rad (≈ 5.7°)
- RMSE en $\Omega$: $< 0.2$ rad/s

**Valores de referencia:**
- Robot pequeño (< 3 kg): $I \in [0.005, 0.02]$ kg·m²
- Robot mediano (3-5 kg): $I \in [0.02, 0.08]$ kg·m²

---

## Etapa 3: Identificación de Factores de Compensación

**Archivo:** [`stage3_experimental_vid.py`](stage3_experimental_vid.py)  
**Método de identificación:** [`ThreeStageIdentification.stage3_compensation()`](../identification/three_stage.py#L435)  
**Base teórica:** [README.md § Stage 3: Motor Compensation Factors](../../README.md#stage-3-motor-compensation-factors-c_u_1-c_u_2-c_u_3)

### 3.1 Objetivo y Fuentes de Asimetría

**Objetivo:** Compensar variaciones motor-a-motor que causan tracking asimétrico:

$$
u_{\text{real},i} = C_{u,i} \cdot u_{\text{comando},i}, \quad i \in \{1,2,3\}
$$

**Fuentes físicas de asimetría:**
1. **Variaciones de fabricación:** Resistencia de bobinado $R_a$ ± 5%
2. **Desgaste de rodamientos:** Fricción asimétrica
3. **Tolerancias mecánicas:** Desalineación de ejes (< 0.5°)
4. **Magnetización no uniforme:** Densidad de flujo magnético variante

**Ejemplo práctico:** Si $C_{u,1} = 0.95$, el motor 1 requiere solo 95% del voltaje comandado para igualar el par de los otros motores.

### 3.2 Protocolo Experimental

#### Diseño de Trayectoria Rica
**Criterios de excitación persistente:**
1. Traslaciones en $\pm X$, $\pm Y$
2. Rotaciones $> 360°$ totales
3. Movimientos diagonales (acopla todos los motores)
4. Cambios de velocidad: aceleración → crucero → desaceleración

**Trayectoria implementada:** Cuadrado de 0.5 m × 0.5 m con rotaciones de 90° en esquinas:
- Lado 1: Traslación $+X$ (0.5 m)
- Esquina 1: Rotación $+90°$
- Lado 2: Traslación $+Y$ (0.5 m)
- ...

**Duración:** 15-20 segundos a velocidad nominal de 0.3 m/s

#### Sistema de Medición Multi-Sensor

**Visión por computadora (flujo óptico + detección de color):**
- Cámara cenital: 1920×1080 @ 30 Hz
- Método de tracking: Flujo óptico Lucas-Kanade (100 puntos en grid)
- Posición $(x, y)$: Centroide de puntos rastreados → transformación de perspectiva → coordenadas métricas
- Orientación $\phi$: Detección de batería rectangular azul/morada oscura mediante segmentación HSV + `cv2.minAreaRect()`
- Procesamiento: `track_simple_robust.py` (ver [`Video_Robot_Tracking/`](../../../Video_Robot_Tracking/))
- Precisión: $\sigma_x \approx 0.005$ m, $\sigma_\phi \approx 0.02$ rad

**IMU (MPU6050):**
- Frecuencia: 100 Hz
- Mediciones: Acelerómetro $\mathbf{a}$ [m/s²], giroscopio $\boldsymbol{\omega}$ [rad/s]
- Orientación: Filtro complementario (95% giroscopio integrado + 5% acelerómetro)

**Controlador embebido:**
- Señales PWM: $u_i(t)$ registradas a 100 Hz
- Sincronización: Timestamp común desde inicio de experimento

**Fusión de datos:** Realizada por [`process_imu_data.py`](../../../Video_Robot_Tracking/process_imu_data.py):
1. **Video tracking** (`track_simple_robust.py`):
   - Flujo óptico Lucas-Kanade para posición $(x, y)$ a ~30 Hz
   - Detección de batería azul para orientación preliminar
   - Calibración de perspectiva (transformación homográfica basada en baldosas de 0.30 m)
   - Remuestreo a 100 Hz por interpolación lineal
2. **Sincronización temporal:** Alineación IMU-Video mediante correlación cruzada de velocidad angular
3. **Fusión de estados:** $x, y$ (video) + $\phi$ (IMU, más preciso) + $u_i$ (controlador embebido)
4. **Diferenciación numérica:** Velocidades calculadas con diferencias centradas: $\dot{x}[k] = (x[k+1] - x[k-1])/(2\Delta t)$

### 3.3 Pipeline de Procesamiento

#### Paso 1: Carga de Datos Fusionados

```python
def load_stage3_video_data(filepath):
    df = pd.read_csv(filepath)
    
    y_measured = np.column_stack([
        df['x_m'].values,          # Posición X de video
        df['y_m'].values,          # Posición Y de video
        df['phi_rad'].values,      # Orientación de IMU
        df['vx_m_s'].values,       # Velocidad X (diferenciada)
        df['vy_m_s'].values,       # Velocidad Y (diferenciada)
        df['omega_rad_s'].values   # Velocidad angular de IMU
    ])
    
    u_pwm = np.column_stack([
        df['u1_pwm'].values,
        df['u2_pwm'].values,
        df['u3_pwm'].values
    ])
    
    t = df['time_s'].values
    
    return {'t': t, 'y_measured': y_measured, 'u_pwm': u_pwm}
```

**Verificación de calidad:**
```python
x_range = y_measured[:, 0].max() - y_measured[:, 0].min()
y_range = y_measured[:, 1].max() - y_measured[:, 1].min()
phi_range = np.rad2deg(y_measured[:, 2].max() - y_measured[:, 2].min())

assert x_range > 0.3, "Desplazamiento X insuficiente"
assert y_range > 0.3, "Desplazamiento Y insuficiente"
assert abs(phi_range) > 90, "Rotación insuficiente para identificabilidad"
```

#### Paso 2: Conversión PWM → Voltaje

Mismo procedimiento que etapas anteriores:

```python
def pwm_to_voltage_batch(u_pwm, V_battery):
    return (u_pwm / 100.0) * V_battery

u_voltage = pwm_to_voltage_batch(u_pwm, V_BATTERY)
```

#### Paso 3: Corrección de Referencia Angular

**Problema:** El sistema de coordenadas de video usa $\phi = 0$ apuntando a $+X$, pero el controlador del robot usa $\phi = 0$ apuntando a $+Y$ (norte magnético).

**Corrección:**
```python
y_measured[:, 2] -= np.pi / 2  # Rotar -90° el marco de referencia
```

#### Paso 4: Llamada al Identificador

```python
identifier = ThreeStageIdentification(wheel_angles, d, r, M)

identifier.stage3_compensation(
    t=t,
    u=u_voltage,
    y_measured=y_measured,
    Ra=Ra,    # De Etapa 1
    K=K,      # De Etapa 1
    I=I,      # De Etapa 2
    plot=True,
    results_dir=results_dir
)
```

### 3.4 Optimización Paramétrica (en `three_stage.py`)

**Función de costo multi-objetivo:**

$$
J(C_{u,1}, C_{u,2}, C_{u,3}) = \sum_{k=1}^{N} \left[ w_x \cdot e_x^2 + w_y \cdot e_y^2 + w_\phi \cdot e_\phi^2 + w_{\dot{x}} \cdot e_{\dot{x}}^2 + w_{\dot{y}} \cdot e_{\dot{y}}^2 + w_{\Omega} \cdot e_{\Omega}^2 \right]_k
$$

**Pesos calibrados:**
- Posición: $w_x = w_y = 100.0$ (tracking preciso del trayecto)
- Orientación: $w_\phi = 50.0$
- Velocidades: $w_{\dot{x}} = w_{\dot{y}} = w_\Omega = 10.0$

**Modelo de simulación:**
- Sistema: ODE completo de 6 estados con modelo de motor compensado
- Integrador: RK45 con $\Delta t_{\text{max}} = 0.01$ s
- Inputs: $u_{\text{comp},i}(t) = C_{u,i} \cdot u_i(t)$

**Optimización:**
```python
result = minimize(
    cost_function,
    x0=[1.0, 1.0, 1.0],           # Inicialización: motores idénticos
    bounds=[(0.7, 1.3)] * 3,      # ±30% de variación permitida
    method='L-BFGS-B',
    options={'maxiter': 100, 'ftol': 1e-5}
)
Cu1, Cu2, Cu3 = result.x
```

### 3.5 Validación y Análisis de Resultados

**Gráficas comparativas:**
1. **Trayectoria 2D:** $y(x)$ medido vs simulado
2. **Series temporales:** $x(t)$, $y(t)$, $\phi(t)$, $\dot{x}(t)$, $\dot{y}(t)$, $\Omega(t)$

**Métricas de error:**
- RMSE posición: $< 0.05$ m
- RMSE orientación: $< 0.1$ rad
- Error máximo: $< 2\sigma$ del ruido de medición

**Interpretación de factores:**
- $|C_{u,i} - 1| < 0.05$: Motores bien emparejados (variación < 5%)
- $|C_{u,i} - 1| \in [0.05, 0.15]$: Asimetría moderada (compensar en controlador)
- $|C_{u,i} - 1| > 0.15$: Asimetría severa (revisar ensamblaje mecánico o reemplazar motor)

**Ejemplo de salida:**
```
Identified Compensation Factors:
  Cu1 = 0.9823  → Motor 1 requiere 98.2% del voltaje nominal
  Cu2 = 1.0456  → Motor 2 requiere 104.6% del voltaje nominal
  Cu3 = 0.9921  → Motor 3 requiere 99.2% del voltaje nominal
```

---

## Conexión con el Marco Teórico

### Correspondencia entre Teoría y Experimentación

| Concepto Teórico (README.md) | Implementación Experimental |
|-------------------------------|----------------------------|
| **§2.1 Voltage Equation** | `pwm_to_voltage()`: Conversión PWM → $V$ |
| **§2.3 Combined Motor Model** | `stage1_motor_parameters()`: Optimización de $R_a$, $K$ |
| **§3.2 Force Transformation** | `compute_body_velocities()`: $\mathbf{H}^+ \boldsymbol{\omega}$ |
| **§3.3 Coordinate Transformation** | `body_to_global_velocities()`: $\mathbf{R}(\phi)$ |
| **§3.4 Coriolis Terms** | `OmnidirectionalRobot.dynamics()`: $\Omega \times \mathbf{v}$ |
| **§4.2 State-Space Equations** | `solve_ivp()`: Integración numérica de $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$ |

### Flujo de Información entre Etapas

```
Etapa 1 (stage1_experimental.py)
    ↓ Ra, K, b
Etapa 2 (stage2_experimental.py)
    ↓ I (usando Ra, K de Etapa 1)
Etapa 3 (stage3_experimental_vid.py)
    ↓ Cu1, Cu2, Cu3 (usando Ra, K, I de etapas previas)
Modelo completo calibrado en OmnidirectionalRobot
```

### Propagación de Incertidumbre

**Análisis de sensibilidad simplificado:**

1. **Etapa 1 → Etapa 2:**
   - Error en $K$ afecta torque motor: $\tau = K \cdot i$
   - Propagación a $I$: $\Delta I / I \approx 2 \cdot \Delta K / K$

2. **Etapa 2 → Etapa 3:**
   - Error en $I$ afecta dinámica rotacional
   - Compensación parcial por factores $C_{u,i}$ (absorben errores residuales)

**Estrategia de mitigación:**
- Ejecutar cada etapa múltiples veces (3-5 repeticiones)
- Promediar parámetros identificados
- Validar con datos de prueba independientes

---

## Consideraciones Prácticas

### Calibración del Sistema de Medición

**Encoders:**
- Resolución: 1000 PPR (pulsos por revolución)
- Resolución angular: $2\pi / 1000 = 0.00628$ rad
- Calibración: Medir RPM con tacómetro óptico → validar constante de conversión

**IMU:**
- Calibración de acelerómetro: Promediar 1000 muestras en reposo → offset
- Calibración de giroscopio: Integrar en superficie nivelada → deriva ($< 0.1°$/s)

**Video:**
- Calibración de perspectiva: Selección manual de 4 esquinas de baldosa (0.30 m) → matriz de transformación homográfica
- Calibración de escala: Automática a partir del tamaño de baldosa conocido → píxeles/metro
- Segmentación de color: Umbrales HSV para detección de batería [H: 100-150, S: 80-255, V: 30-180]

### Requisitos de Hardware

**Computacional:**
- CPU: > 2 GHz (optimización numérica)
- RAM: > 4 GB (almacenamiento de series temporales)
- Python: 3.8+ con NumPy, SciPy, Matplotlib

**Experimental:**
- Superficie plana: desnivel < 1 mm/m
- Iluminación: uniforme, sin sombras móviles (para tracking de video)
- Batería: monitorear voltaje cada 5 minutos (usar multímetro)

### Troubleshooting Común

| Síntoma | Causa Probable | Solución |
|---------|----------------|----------|
| $R^2 < 0.8$ en Etapa 1 | Fricción estática (stick-slip) | Aplicar prelubricación, usar PWM > 30% |
| Divergencia en Etapa 2 | Traslación excesiva ($v_{\text{RMS}} > 0.2$) | Rehacer experimento, verificar PID de posición |
| RMSE alto en Etapa 3 | Deslizamiento de ruedas | Reducir aceleración, verificar presión de ruedas |
| Factores $C_{u,i} > 1.3$ | Error en calibración de batería | Medir $V_{\text{bat}}$ real durante experimento |

---

## Referencias Metodológicas

### Algoritmos de Procesamiento

1. **Kalman Filter:** Welch, G., & Bishop, G. (2006). *An Introduction to the Kalman Filter*. UNC Chapel Hill.
2. **Unwrapping de ángulos:** `numpy.unwrap()` - implementación de algoritmo de Itoh-Fukushima
3. **Pseudoinversa:** Moore-Penrose, calculada vía SVD (`numpy.linalg.pinv`)

### Optimización Numérica

4. **L-BFGS-B:** Byrd, R. H., et al. (1995). "A Limited Memory Algorithm for Bound Constrained Optimization". *SIAM J. Scientific Computing*.
5. **RK45:** Dormand-Prince method (4th order embedded in 5th order, control adaptativo de paso)

### Teoría de Identificación

6. **Excitación persistente:** Ljung, L. (1999). *System Identification: Theory for the User*. Prentice Hall.
7. **Identificabilidad estructural:** Bellman, R., & Åström, K. J. (1970). "On Structural Identifiability". *Mathematical Biosciences*.

---

## Apéndices

### A. Estructura de Archivos de Resultados

```
results/identification/
├── stage1/
│   ├── stage1_caracterizacion_motor2.png
│   ├── stage1_metricas_motor2.csv
│   ├── stage1_velocidades_filtradas_voltajes_aplicados.png
│   └── stage1_voltaje_vs_velocidad_estado_estable_todos_motores.png
├── stage2/
│   ├── stage2_phi_tracking.png
│   ├── stage2_omega_tracking.png
│   └── stage2_metricas.csv
└── stage3/
    ├── stage3_trajectory_2d.png
    ├── stage3_states_comparison.png
    └── stage3_metricas.csv
```

### B. Formato de Archivos CSV de Métricas

**stage1_metricas_motorX.csv:**
```csv
Parameter,Value,Unit,Description
Ra,1.5100,Ω,Armature Resistance
K,0.4990,V/(rad/s),Motor Constant
b,0.0023,Nm/(rad/s),Friction Coefficient
RMSE,2.1500,rad/s,Root Mean Square Error
R2,0.9320,-,Coefficient of Determination
```

**stage2_metricas.csv:**
```csv
Parameter,Value,Unit
I,0.0189,kg·m²
RMSE_phi,0.0823,rad
RMSE_omega,0.1452,rad/s
```

**stage3_metricas.csv:**
```csv
Parameter,Value,Interpretation
Cu1,0.9823,Motor 1: 98.2% efficiency
Cu2,1.0456,Motor 2: 104.6% efficiency
Cu3,0.9921,Motor 3: 99.2% efficiency
RMSE_position,0.0347,m
RMSE_orientation,0.0912,rad
```

### C. Checklist Pre-Experimento

**Etapa 1:**
- [ ] Robot suspendido/motor en banco
- [ ] Batería cargada (> 22.0 V para 6S)
- [ ] Voltímetro conectado para monitoreo en tiempo real
- [ ] Encoders calibrados (verificar conteo en ambas direcciones)

**Etapa 2:**
- [ ] Superficie plana nivelada (verificar con nivel de burbuja)
- [ ] IMU calibrado (reposo → offset cero)
- [ ] Espacio libre de 1 m radio
- [ ] Controlador PID de posición activo

**Etapa 3:**
- [ ] Cámara montada en cenital (verificar que capture toda el área de movimiento)
- [ ] Baldosas del piso visibles (0.30 m × 0.30 m para calibración de perspectiva)
- [ ] Batería azul/morada visible en robot (contraste con fondo)
- [ ] Iluminación estable sin sombras móviles ni reflejos especulares
- [ ] Sincronización temporal configurada (timestamps comunes IMU-Video)
- [ ] Trayectoria de referencia cargada en controlador

---

## Conclusiones Metodológicas

Este pipeline de identificación en tres etapas proporciona una caracterización completa y robusta de robots omnidireccionales mediante:

1. **Desacoplamiento dinámico:** Cada etapa maximiza sensibilidad a parámetros objetivo minimizando interferencia de otros efectos
2. **Fusión multi-sensor:** Combinación óptima de encoders (alta frecuencia, baja deriva) + IMU (orientación absoluta) + video (posición global)
3. **Validación cruzada:** Métricas estadísticas ($R^2$, RMSE) y visualizaciones comparativas en cada etapa
4. **Reproducibilidad:** Protocolos experimentales estandarizados con criterios de aceptación cuantitativos

La metodología es aplicable a cualquier robot omnidireccional con cinemática inversa conocida, requiriendo solo ajustes menores en parámetros geométricos ($\theta_i$, $d$, $r$) y rangos de optimización según especificaciones del hardware.

---

**Autor:** [Tu Nombre]  
**Fecha:** Enero 2026  
**Versión:** 1.0  
**Licencia:** MIT
