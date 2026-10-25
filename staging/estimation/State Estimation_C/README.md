# Control and State Estimation in C

Complete implementation of **generalized** state estimation algorithms in C, including Unscented Kalman Filter (UKF), Particle Filter (PF), and Extended Kalman Filter (EKF).

---

## Overview

This project provides production-ready, **fully generalized** implementations of various state estimation filters used in robotics, navigation, and control systems. The API follows the design philosophy of the Python FilterPy library, allowing users to define custom dynamics and measurement models without modifying filter internals.

### Key Features

- ✅ **Generalized API**: Works with any user-defined dynamics `f(x, u)` and measurement `h(x)`
- ✅ **Control Input Support**: All filters support optional control inputs via `dim_u` parameter
- ✅ **No Hard-coded Parameters**: Time step `dt` and other model-specific parameters are embedded in user functions
- ✅ **Flexible User Data**: Particle Filter supports arbitrary user data (e.g., landmarks, maps) via `void*` pointer

### Available Filters

| Filter | Status | Description | Use Case |
|--------|--------|-------------|----------|
| **UKF** | ✅ Complete | Unscented Kalman Filter | Nonlinear systems with Gaussian noise |
| **PF** | ✅ Complete | Particle Filter (SIR) | Highly nonlinear, non-Gaussian, multimodal |
| **EKF** | ✅ Complete | Extended Kalman Filter | Mildly nonlinear systems with known Jacobians |

---

## Generalized API Design

All filters follow a consistent, generalized API inspired by Python's FilterPy:

### Common Principles

1. **User-Defined Dynamics**: `f(x, u)` - You define how your system evolves
2. **User-Defined Measurements**: `h(x)` - You define what you measure
3. **Control Input Support**: Optional `u` vector (set `dim_u = 0` if not needed)
4. **No Hard-coded dt**: Time discretization is embedded in your dynamics function
5. **Consistent Initialization**: All filters use `(dim_x, dim_z, dim_u, ...)` pattern

### Function Signatures

**EKF & UKF:**
```c
void f(const float *x, const float *u, float *x_out);     // Dynamics
void h(const float *x, float *z_out);                      // Measurement
void F(const float *x, const float *u, float *F_out);     // Jacobian (EKF only)
void H(const float *x, float *H_out);                      // Jacobian (EKF only)
```

**Particle Filter:**
```c
void f(const float *x, const float *u, const float *noise, float *x_out);
float likelihood(const float *x, const float *z, const float *R, const void *user_data);
```

---

## Angle Handling (Angular States)

### The Problem

Angles are **periodic** quantities in the range [-π, π]. Standard arithmetic fails:

```
Problem: avg(179°, -179°) = 0°  ❌ INCORRECT
Correct: avg(179°, -179°) = ±180° ✓
```

### Available Functions

```c
#include "../LinearAlgebra/angle_utils.h"

float normalize_angle(float angle);  // Normalize to [-π, π]
float angle_diff(float a, float b);  // Shortest angular difference
float circular_mean(const float *angles, const float *weights, size_t n);
```

### Usage in Filters

**EKF**: In update step (`ekf_update`)
- Innovation: `y = angle_diff(z, z_pred)` for angular measurements
- Post-update: `normalize_angle(x[i])` for angular states

**UKF**: In unscented transform (`ukf_unscented_transform`) - **CRITICAL**
- Sigma point averaging: `circular_mean()` instead of arithmetic mean
- Covariance: `angle_diff()` for angular differences

**PF**: In estimation (`pf_estimate`)
- Particle mean: `circular_mean()` for angular components

### Example: Robot with Orientation

```c
// State: [x, y, θ, v]  where θ is angle
bool state_is_angle[] = {false, false, true, false};  // Only θ is angle
bool meas_is_angle[] = {true};  // Bearing measurement is angle

// For EKF
ekf_set_angle_states(&ekf, state_is_angle, 4, meas_is_angle, 1);

// For UKF
ukf_set_angle_states(&ukf, state_is_angle, 4, meas_is_angle, 1);

// For PF
pf_set_angle_states(&pf, state_is_angle, 4);
```

See individual filter documentation for more details.

---

## References

### Available Filters

| Filter | Status | Description | Use Case |
|--------|--------|-------------|----------|
| **UKF** | ✅ Complete | Unscented Kalman Filter | Nonlinear systems with Gaussian noise |
| **PF** | ✅ Complete | Particle Filter (SIR) | Highly nonlinear, non-Gaussian, multimodal |
| **EKF** | ✅ Complete | Extended Kalman Filter | Mildly nonlinear systems with known Jacobians |

---

## Quick Start

### Prerequisites

- **Desktop/Test Development**: GCC compiler (MinGW on Windows, GCC on Linux/macOS)
- **ESP32 Deployment**: ESP-IDF v5.0+ (see [esp32s3_test/README.md](esp32s3_test/README.md))
- Standard C math library

### Build Options (Desktop/Test)

```powershell
# Build all filters (recommended - creates executables for testing)
.\build.ps1 all

# Build individual filters
.\build.ps1 ekf    # Extended Kalman Filter → ekf_demo.exe
.\build.ps1 ukf    # Unscented Kalman Filter → ukf_demo.exe
.\build.ps1 pf     # Particle Filter → pf_demo.exe

# Build unified menu (interactive selection)
.\build.ps1 menu   # Creates menu.exe

# Get help
.\build.ps1 help
```

### Run Examples

**Option A: Run Individual Filter Tests**

```powershell
# Build and run specific filter
.\build.ps1 ekf
.\ekf_demo.exe     # EKF - 2D constant velocity tracking

.\build.ps1 ukf
.\ukf_demo.exe     # UKF - 2D constant velocity tracking

.\build.ps1 pf
.\pf_demo.exe      # PF - Robot localization with landmarks
```

**Option B: Use Interactive Menu**

```powershell
.\build.ps1 menu
.\menu.exe

# Then select from menu:
# [1] UKF - Constant Velocity 2D Tracking
# [2] PF  - Robot Localization with Landmarks
# [3] EKF - Constant Velocity 2D Tracking
# [0] Exit
```

**Option C: Deploy to ESP32-S3 (Production)**

```powershell
cd esp32s3_test
idf.py build          # Build firmware
idf.py -p COM3 flash  # Flash to device
idf.py -p COM3 monitor  # Monitor output
```

---

## Project Structure

```
State Estimation_C/
├── build.ps1                      # Unified build script (all, ekf, ukf, pf, menu)
├── clean.ps1                      # Clean build artifacts
├── main.c                         # Interactive menu system
├── Makefile                       # Make configuration
├── README.md                      # This file
│
├── Headers/
│   └── headers.h                  # Common types and definitions
│
├── LinearAlgebra/                 # Matrix operations library
│   ├── linearalgebra.h
│   ├── angle_utils.c/h            # Angle wrapping, circular mean
│   ├── chol.c, inv.c, mul.c, ...
│   └── CLapack/                   # LAPACK routines
│
├── EKF/                           # Extended Kalman Filter (library module)
│   ├── ekf.h                      # EKF API
│   └── ekf.c                      # EKF implementation
│
├── UKF/                           # Unscented Kalman Filter (library module)
│   ├── ukf.h                      # UKF API
│   └── ukf.c                      # UKF implementation
│
├── PF/                            # Particle Filter (library module)
│   ├── pf.h                       # PF API
│   └── pf.c                       # PF implementation
│
├── Examples/                      # Desktop test applications
│   ├── ekf_example.c              # EKF: 2D constant velocity
│   ├── ukf_example.c              # UKF: 2D constant velocity
│   └── pf_example.c               # PF: Robot localization
│
└── esp32s3_test/                  # ESP32-S3 deployment (production)
    ├── CMakeLists.txt
    ├── sdkconfig
    ├── components/                # ESP-IDF components
    └── main/                      # ESP32 application code
        ├── compare_filters_esp32.c  # Unified filter comparison
        ├── ekf.c/h, ukf.c/h, pf.c/h
        ├── robot_model.c/h        # Omnidirectional robot dynamics
        └── robot_data.h           # Experimental data arrays
```

---

## Build System

The project uses a unified PowerShell build script (`build.ps1`) that supports multiple build targets:

### Build Targets

| Target | Output | Description |
|--------|--------|-------------|
| `menu` | `menu.exe` | Interactive menu with all filters (recommended) |
| `ukf` | `ukf_demo.exe` | Standalone UKF example |
| `pf` | `pf_demo.exe` | Standalone Particle Filter example |
| `ekf` | `ekf_demo.exe` | Standalone Extended Kalman Filter example |
| `all` | Multiple `.exe` | Build all standalone examples |
| `help` | - | Show build system help |

### Build Details

**Menu Build:**
- Compiles `main.c` + all example files + filter implementations
- Single executable with interactive selection
- Uses `-DBUILD_WITH_MENU` flag to handle multiple main functions

**Standalone Builds:**
- Each filter example compiled separately
- Independent executables for testing/benchmarking
- No menu overhead, direct execution

---

## Documentation

### Python Implementation & Theory

For complete mathematical theory, algorithm derivations, and Python implementations:
- **[Bayesian Filters Theory](../State%20Estimation_Python/docs/BAYESIAN_FILTERS.md)** - Complete EKF, UKF, PF mathematical framework
- **[State Estimation_Python/README.md](../State%20Estimation_Python/README.md)** - Python API and examples

### C Implementation Details

The C implementations follow the theoretical framework documented in the Python section. Each subdirectory (EKF/, UKF/, PF/) contains library modules with header files defining the API:
- **EKF/ekf.h** - Extended Kalman Filter API (linearization via Jacobians)
- **UKF/ukf.h** - Unscented Kalman Filter API (sigma-point transform)
- **PF/pf.h** - Particle Filter API (Sequential Importance Resampling)

### ESP32 Deployment

The **esp32s3_test/** directory contains ESP-IDF project for embedded deployment:
- Complete filter comparison on ESP32-S3
- Hardware-optimized matrix operations
- Real-time performance benchmarks
- UART communication for sensor data

Build instructions:
```powershell
cd "esp32s3_test"
idf.py build
idf.py -p COM3 flash monitor
```

---

## Usage Examples

### Unscented Kalman Filter

```c
#include "./UKF/ukf.h"

// 1. Define your system functions (dt embedded in model)
void my_dynamics(const float* x, const float* u, float* x_out) {
    const float dt = 0.01f;  // Embedded time step
    x_out[0] = x[0] + x[1] * dt;  // position
    x_out[1] = x[1];               // velocity
}

void my_measurement(const float* x, float* z_out) {
    z_out[0] = x[0];  // measure position
}

// 2. Initialize UKF (note: no dt parameter)
UKF_State ukf;
size_t dim_x = 2, dim_z = 1, dim_u = 0;  // No control input
ukf_init(&ukf, dim_x, dim_z, dim_u, 0.1f, 2.0f, 1.0f, 
         my_dynamics, my_measurement);

// 3. Set initial state and noise
float x0[2] = {0.0f, 0.0f};
float P0[4] = {100.0f, 0.0f, 0.0f, 100.0f};
ukf_set_state(&ukf, x0, P0);

float Q[4] = {0.01f, 0.0f, 0.0f, 0.01f};
float R[1] = {0.1f};
ukf_set_noise(&ukf, Q, R);

// 4. Run filter loop
for (int i = 0; i < num_steps; i++) {
    ukf_predict(&ukf, NULL);  // NULL = no control input
    float z[1] = {measurement};
    ukf_update(&ukf, z);
    // Use ukf.x for state estimate
}

// 5. Cleanup
ukf_free(&ukf);
```

### Particle Filter

```c
#include "./PF/pf.h"

// 1. Define your system functions
void robot_motion(const float *particle, const float *u,
                  const float *noise, float *particle_out) {
    const float dt = 0.01f;  // Embedded in model
    // Define motion model (dt embedded, not a parameter)
}

// User data structure for landmarks (or any other data)
typedef struct {
    const float *landmarks;
    size_t num_landmarks;
} LandmarkData;

float measurement_likelihood(const float *particle,
                            const float *z, const float *R,
                            const void *user_data) {
    const LandmarkData *lm = (const LandmarkData*)user_data;
    // Compute likelihood using landmarks from user_data
    // Return p(z|particle)
}

// 2. Initialize PF (no dt parameter)
PF_State pf;
size_t dim_x = 3, dim_z = 4, dim_u = 2;
pf_init(&pf, 5000, dim_x, dim_u, dim_z, robot_motion, 
        measurement_likelihood, PF_RESAMPLE_SYSTEMATIC);

// 3. Set noise and create particles
float process_std[3] = {0.2f, 0.05f, 0.0f};
pf_set_process_noise(&pf, process_std);

float mean[3] = {0.0f, 0.0f, 0.0f};
float std[3] = {5.0f, 5.0f, 1.0f};
pf_create_gaussian_particles(&pf, mean, std);

// 4. Prepare user data and measurement noise
LandmarkData lm_data = {.landmarks = my_landmarks, .num_landmarks = 4};
float R[16];  // Measurement covariance matrix
// ... initialize R ...

// 5. Run filter loop
for (int i = 0; i < num_steps; i++) {
    pf_predict(&pf, control_input);
    pf_update(&pf, measurements, R, &lm_data);  // Pass user_data
    pf_resample_if_needed(&pf);
    pf_estimate(&pf);
    // Use pf.state_mean for estimate
}

// 6. Cleanup
pf_free(&pf);
```

### Extended Kalman Filter

```c
#include "./EKF/ekf.h"

// 1. Define your system functions and Jacobians (dt embedded)
void my_dynamics(const float* x, const float* u, float* x_out) {
    const float dt = 0.01f;  // Embedded time step
    x_out[0] = x[0] + x[1] * dt;  // position
    x_out[1] = x[1];               // velocity
}

void my_measurement(const float* x, float* z_out) {
    z_out[0] = x[0];  // measure position
}

void jacobian_f(const float* x, const float* u, float* F) {
    const float dt = 0.01f;
    // F = [1  dt]
    //     [0   1]
    F[0] = 1.0f;  F[1] = dt;
    F[2] = 0.0f;  F[3] = 1.0f;
}

void jacobian_h(const float* x, float* H) {
    // H = [1  0]
    H[0] = 1.0f;  H[1] = 0.0f;
}

// 2. Initialize EKF (note: no dt parameter)
EKF_State ekf;
size_t dim_x = 2, dim_z = 1, dim_u = 0;  // No control input
ekf_init(&ekf, dim_x, dim_z, dim_u, my_dynamics, my_measurement, 
         jacobian_f, jacobian_h);

// 3. Set initial state and noise
float x0[2] = {0.0f, 0.0f};
float P0[4] = {100.0f, 0.0f, 0.0f, 100.0f};
ekf_set_state(&ekf, x0, P0);

float Q[4] = {0.1f, 0.0f, 0.0f, 0.1f};
float R[1] = {1.0f};
ekf_set_noise(&ekf, Q, R);

// 4. Run filter loop
for (int i = 0; i < num_steps; i++) {
    ekf_predict(&ekf, NULL);  // NULL = no control input
    float z[1] = {measurement};
    ekf_update(&ekf, z);
    // Use ekf.x for state estimate
}

// 5. Cleanup
ekf_free(&ekf);
```

---

## 🔧 Build System

### PowerShell Build Script

```powershell
.\build.ps1 <target>

Targets:
  ukf  - Build UKF example
  pf   - Build Particle Filter example
  ekf  - Build Extended Kalman Filter example
  all  - Build all examples
  help - Show help message
```

### Manual Compilation

**UKF Example:**
```bash
gcc -o ukf_demo.exe Examples/ukf_example.c UKF/ukf.c \
    LinearAlgebra/chol.c LinearAlgebra/inv.c LinearAlgebra/mul.c \
    LinearAlgebra/tran.c LinearAlgebra/lup.c LinearAlgebra/linsolve_lup.c \
    LinearAlgebra/linsolve_lower_triangular.c \
    LinearAlgebra/linsolve_upper_triangular.c LinearAlgebra/dot.c \
    -I. -IHeaders -ILinearAlgebra -lm
```

**PF Example:**
```bash
gcc -o pf_demo.exe Examples/pf_example.c PF/pf.c \
    -I. -IHeaders -lm
```

**EKF Example:**
```bash
gcc -o ekf_demo.exe Examples/ekf_example.c EKF/ekf.c \
    LinearAlgebra/mul.c LinearAlgebra/inv.c LinearAlgebra/tran.c \
    LinearAlgebra/lup.c LinearAlgebra/linsolve_lup.c \
    LinearAlgebra/linsolve_lower_triangular.c \
    LinearAlgebra/linsolve_upper_triangular.c \
    -I. -IHeaders -ILinearAlgebra -lm
```

---

## 🎯 Filter Comparison

| Feature | UKF | PF | EKF |
|---------|-----|----|----|
| **Linearity** | Moderate nonlinearity | Any nonlinearity | Mild nonlinearity |
| **Noise** | Gaussian | Any distribution | Gaussian |
| **Multimodal** | No | Yes | No |
| **Computational Cost** | Medium | High | Low |
| **Memory Usage** | Low | High | Low |
| **Accuracy** | High (if assumptions met) | High | Medium |

**When to use each:**
- **UKF**: Moderate nonlinearity, Gaussian noise, single hypothesis
- **PF**: Highly nonlinear, non-Gaussian noise, multiple hypotheses
- **EKF**: Mild nonlinearity, Gaussian noise, computational constraints

---

## 📊 Example Results

### UKF - Constant Velocity Tracking
- Position Error: ~0.13-0.17 meters
- Velocity Error: ~0.04-0.05 m/s
- Convergence: Within 5 iterations

### PF - Robot Localization  
- Position Error: ~0.1-0.3 meters
- Convergence: Within 5-10 iterations
- Effective N: 60-80% of particles

### EKF - Constant Velocity Tracking
- Position Error: ~0.5-1.5 meters
- Velocity Error: ~0.05-0.15 m/s
- Convergence: Within 3-5 iterations

---

## 📖 References

### UKF
1. Van der Merwe, R. (2004). "Sigma-Point Kalman Filters"
2. Julier, S.J. & Uhlmann, J.K. (2004). "Unscented Filtering"

### Particle Filter
1. Arulampalam, M. S., et al. (2002). "A tutorial on particle filters"
2. Ristic, B., et al. (2004). "Beyond the Kalman filter"

---

## 🛠️ Development

### Adding New Examples

1. Create example file in `Examples/` directory
2. Add build target in `build.ps1`
3. Update documentation

### Code Style

- Follow existing code structure
- Add comprehensive comments
- Include example usage in headers
- Document all public functions

---

## 📝 License

MIT License - See individual file headers for details.

---

## 👤 Author

**MaverickST**  
Version: 1.0  
Date: December 2025

---

## 🎉 Acknowledgments

- LinearAlgebra library based on CControl project
- Algorithms from Kalman-and-Bayesian-Filters-in-Python notebooks
- Van der Merwe's UKF formulation
- SIR Particle Filter implementation

---

**For detailed documentation on each filter, see the README.md files in their respective directories.**
