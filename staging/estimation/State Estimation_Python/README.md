# State Estimation Python Library

A professional, generalized state estimation library for robotics applications. Implements Extended Kalman Filter (EKF), Unscented Kalman Filter (UKF), and Particle Filter (PF) with a FilterPy-inspired API.

## Features

- **Generalized Filter Implementations**: Works with any user-defined dynamics `f(x, u)` and measurement functions `h(x)`
- **Multiple Filters**: EKF, UKF, and PF implementations
- **Professional API**: Clean, consistent interface inspired by FilterPy
- **Comprehensive Tools**: 
  - Performance metrics (RMSE, MAE, NEES, NIS)
  - Visualization utilities (trajectories, uncertainty ellipses)
  - Robot models (omnidirectional, extensible to others)
- **Well-Documented**: Extensive docstrings and examples
- **100% English**: Code, comments, and documentation in English

## Installation

### From Source

```bash
cd "State Estimation_Python"
pip install -e .
```

### Dependencies

```bash
pip install -r requirements.txt
```

Core dependencies:
- `numpy >= 1.20.0`
- `scipy >= 1.7.0`
- `matplotlib >= 3.3.0`

## Quick Start

### Extended Kalman Filter (EKF)

```python
import numpy as np
from state_estimation import ExtendedKalmanFilter
from state_estimation.models import OmnidirectionalRobot

# Initialize robot model
robot = OmnidirectionalRobot(dt=0.01)

# Create EKF
ekf = ExtendedKalmanFilter(dim_x=6, dim_z=4, dim_u=3)
ekf.x = np.zeros(6)  # Initial state [x, y, psi, vx_b, vy_b, omega]
ekf.P = np.eye(6) * 0.1  # Initial covariance
ekf.Q = np.eye(6) * 0.01  # Process noise
ekf.R = np.eye(4) * 0.1  # Measurement noise

# Run filter
for k in range(N):
    # Predict
    ekf.predict(u=control_inputs[k], 
                f=robot.dynamics, 
                F=robot.jacobian_F)
    
    # Update
    ekf.update(z=measurements[k], 
               h=robot.measurement, 
               H=robot.jacobian_H)
    
    # Store estimate
    estimates[k] = ekf.x
```

### Unscented Kalman Filter (UKF)

```python
from state_estimation.filters import UnscentedKalmanFilter, MerweScaledSigmaPoints

# Create sigma points
points = MerweScaledSigmaPoints(n=6, alpha=0.1, beta=2.0, kappa=0.0)

# Create UKF
ukf = UnscentedKalmanFilter(dim_x=6, dim_z=4, dim_u=3, points=points)
ukf.x = np.zeros(6)
ukf.P = np.eye(6) * 0.1
ukf.Q = np.eye(6) * 0.01
ukf.R = np.eye(4) * 0.1

# Run filter
for k in range(N):
    ukf.predict(u=control_inputs[k], f=robot.dynamics)
    ukf.update(z=measurements[k], h=robot.measurement)
    estimates[k] = ukf.x
```

### Particle Filter (PF)

```python
from state_estimation import ParticleFilter

# Create PF with 1000 particles
pf = ParticleFilter(dim_x=6, dim_z=4, N=1000, dim_u=3)
pf.initialize_particles(x0=np.zeros(6), P0=np.eye(6))

# Run filter
for k in range(N):
    pf.predict(u=control_inputs[k], f=robot.dynamics, process_noise_std=0.1)
    pf.update(z=measurements[k], h=robot.measurement, R=np.eye(4)*0.1)
    pf.resample()  # Resample if needed
    estimates[k] = pf.x
```

## Project Structure

```
State Estimation_Python/
├── state_estimation/          # Main library
│   ├── filters/               # Filter implementations
│   │   ├── extended.py        # EKF
│   │   ├── unscented.py       # UKF  
│   │   └── particle.py        # PF
│   ├── common/                # Utilities
│   │   ├── angles.py          # Angle normalization
│   │   ├── discretization.py  # Integration methods
│   │   └── residuals.py       # Residual functions
│   ├── models/                # Robot models
│   │   └── omnidirectional.py # Omniwheel robot
│   ├── metrics/               # Performance metrics
│   │   └── performance.py     # RMSE, MAE, NEES, NIS
│   └── visualization/         # Plotting tools
│       ├── trajectories.py    # Trajectory plots
│       └── covariances.py     # Uncertainty plots
├── examples/                  # Usage examples
├── notebooks/                 # Jupyter tutorials
├── tests/                     # Unit tests
├── results/                   # Output directory
│   ├── figures/               # Saved plots
│   ├── metrics/               # Performance metrics
│   └── data/                  # Processed results
├── data/                      # Shared data folder (project root)
│   ├── raw/                   # Raw experimental data
│   ├── processed/             # Processed data
│   └── synthetic/             # Synthetic data
├── README.md
├── requirements.txt
└── setup.py
```

## Examples

Five complete examples are provided in the `examples/` directory:

1. **`ekf_omnidirectional.py`** - EKF with omniwheel robot
2. **`ukf_omnidirectional.py`** - UKF with omniwheel robot
3. **`pf_omnidirectional.py`** - Particle Filter with omniwheel robot
4. **`compare_filters.py`** - Compare all three filters
5. **`custom_model.py`** - Use custom dynamics and measurement models

Run an example:
```bash
cd examples
python ekf_omnidirectional.py
```

## Custom Models

The library is designed to work with any robot model. Simply provide:

1. **Dynamics function**: `f(x, u) -> x_next`
2. **Measurement function**: `h(x) -> z`
3. **Jacobians** (for EKF): `F(x, u)` and `H(x)`

Example:

```python
def my_dynamics(x, u):
    """Custom dynamics: x_next = f(x, u)"""
    # Your dynamics here
    return x_next

def my_measurement(x):
    """Custom measurement: z = h(x)"""
    # Your measurement model here
    return z

def my_jacobian_F(x, u):
    """Jacobian ∂f/∂x"""
    # Your Jacobian here
    return F

def my_jacobian_H(x):
    """Jacobian ∂h/∂x"""
    # Your Jacobian here
    return H

# Use with EKF
ekf.predict(u=u, f=my_dynamics, F=my_jacobian_F)
ekf.update(z=z, h=my_measurement, H=my_jacobian_H)
```

## Handling Angular States

For states/measurements with angles, use residual and mean functions:

```python
from state_estimation.common import make_residual_fn
from state_estimation.common.angles import circular_mean

# For EKF
residual_fn = make_residual_fn(angle_indices=[2])  # psi is at index 2
ekf.set_residual_fn(residual_fn)

# For UKF
def state_mean(sigmas, Wm):
    x = np.zeros(6)
    x[0] = np.dot(Wm, sigmas[:, 0])  # x (linear)
    x[1] = np.dot(Wm, sigmas[:, 1])  # y (linear)
    x[2] = circular_mean(sigmas[:, 2], Wm)  # psi (circular)
    # ... rest of states
    return x

ukf.set_mean_fn(x_mean_fn=state_mean)
```

## Performance Metrics

```python
from state_estimation.metrics import compute_all_metrics, print_metrics

# Compute metrics
metrics = compute_all_metrics(
    estimates=estimates,
    ground_truth=true_states,
    covariances=covariances  # Optional
)

# Print results
print_metrics(metrics, filter_name="EKF")
```

Available metrics:
- **RMSE**: Root Mean Square Error
- **MAE**: Mean Absolute Error  
- **NEES**: Normalized Estimation Error Squared (consistency)
- **NIS**: Normalized Innovation Squared (consistency)

## Visualization

```python
from state_estimation.visualization import plot_trajectory, plot_states, plot_comparison

# Plot trajectory
plot_trajectory(estimates, ground_truth=true_states, 
                title="EKF Trajectory", save_path="results/figures/ekf_trajectory.png")

# Plot all states
plot_states(time, estimates, ground_truth=true_states,
            state_names=['x', 'y', 'psi', 'vx', 'vy', 'omega'])

# Compare filters
plot_comparison(time, {'EKF': ekf_states, 'UKF': ukf_states, 'PF': pf_states},
                ground_truth=true_states, state_idx=0, state_name='x')
```

## API Reference

### ExtendedKalmanFilter

```python
ExtendedKalmanFilter(dim_x, dim_z, dim_u=0)
```

**Methods:**
- `predict(u, f, F, Q=None)` - Prediction step
- `update(z, h, H, R=None)` - Update step
- `set_residual_fn(residual_fn)` - Set custom residual function

**Attributes:**
- `x` - State estimate
- `P` - State covariance
- `Q` - Process noise covariance
- `R` - Measurement noise covariance

### UnscentedKalmanFilter

```python
UnscentedKalmanFilter(dim_x, dim_z, dim_u=0, points=None)
```

**Methods:**
- `predict(u, f, Q=None)` - Prediction step
- `update(z, h, R=None)` - Update step
- `set_mean_fn(x_mean_fn, z_mean_fn)` - Set custom mean functions
- `set_residual_fn(residual_x_fn, residual_z_fn)` - Set custom residuals

### ParticleFilter

```python
ParticleFilter(dim_x, dim_z, N=1000, dim_u=0)
```

**Methods:**
- `initialize_particles(x0, P0)` - Initialize particle cloud
- `predict(u, f, process_noise_std=None, Q=None)` - Prediction step
- `update(z, likelihood_fn=None, h=None, R=None)` - Update step
- `resample(scheme='systematic')` - Resample particles
- `effective_sample_size()` - Get ESS

## Testing

```bash
pytest tests/
```

## Contributing

Contributions are welcome! Please ensure:
- Code is in English
- Follows PEP 8 style guide
- Includes docstrings
- Includes tests for new features

## License

MIT License

## Citation

If you use this library in your research, please cite:

```bibtex
@software{state_estimation_library,
  title = {State Estimation Library},
  author = {State Estimation Team},
  year = {2025},
  url = {}
}
```

## References

1. Kalman, R. E. (1960). "A New Approach to Linear Filtering and Prediction Problems"
2. Julier, S. J., & Uhlmann, J. K. (2004). "Unscented Filtering and Nonlinear Estimation"
3. Arulampalam, M. S., et al. (2002). "A Tutorial on Particle Filters"
4. Labbe, R. (2015). "Kalman and Bayesian Filters in Python" (FilterPy)

## Contact

For questions or issues, please open an issue on the repository.
